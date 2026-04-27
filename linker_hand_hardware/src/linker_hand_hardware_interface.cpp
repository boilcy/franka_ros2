#include "linker_hand_hardware/linker_hand_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <exception>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "LinkerHandApi.h"
#include "pluginlib/class_list_macros.hpp"

namespace linker_hand_hardware {
namespace {

constexpr std::size_t kL10JointCount = 10;
constexpr std::size_t kTwentyJointCount = 20;
constexpr std::size_t kG20TactileStateCount = 25;
constexpr std::uint8_t kG20TouchMatrixCode = 0xC6;

const std::vector<double> kL10LeftMin{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.26, -0.26, -0.52};
const std::vector<double> kL10LeftMax{1.45, 1.43, 1.62, 1.62, 1.62, 1.62, 0.26, 0.0, 0.0, 1.01};
const std::vector<int> kL10LeftDirection{-1, -1, -1, -1, -1, -1, 0, -1, -1, -1};
const std::vector<double> kL10RightMin{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.52};
const std::vector<double> kL10RightMax{0.75, 1.43, 1.62, 1.62, 1.62, 1.62, 0.0, 0.13, 0.26, 1.01};
const std::vector<int> kL10RightDirection{-1, -1, -1, -1, -1, -1, -1, 0, 0, -1};

const std::vector<double> kL20LeftMin{0.0, 0.0, 0.0, 0.0, 0.0, -0.297, -0.26, -0.26, -0.26, -0.26,
                                      0.122, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> kL20LeftMax{0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26,
                                      1.78, 0.0, 0.0, 0.0, 0.0, 1.29, 1.08, 1.08, 1.08, 1.08};
const std::vector<int> kL20LeftDirection{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                         -1, 0, 0, 0, 0, -1, -1, -1, -1, -1};
const std::vector<double> kL20RightMin{0.0, 0.0, 0.0, 0.0, 0.0, -0.297, -0.26, -0.26, -0.26, -0.26,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> kL20RightMax{0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26,
                                       1.78, 0.0, 0.0, 0.0, 0.0, 1.29, 1.08, 1.08, 1.08, 1.08};
const std::vector<int> kL20RightDirection{-1, -1, -1, -1, -1, -1, 0, 0, 0, 0,
                                          -1, 0, 0, 0, 0, -1, -1, -1, -1, -1};

bool isReservedTwentyJointIndex(std::size_t index) {
  return index >= 11 && index <= 14;
}

std::uint8_t rowSelectorToIndex(std::uint8_t selector) {
  return selector % 16 == 0 && selector <= 176 ? static_cast<std::uint8_t>(selector / 16) : 255;
}

}  // namespace

LinkerHandHardwareInterface::~LinkerHandHardwareInterface() {
  stopIoThread();
  deactivateSdk();
  closeCanSocket();
}

CallbackReturn LinkerHandHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (const auto parameter = info_.hardware_parameters.find("hand_model");
      parameter != info_.hardware_parameters.end()) {
    hand_model_ = uppercase(parameter->second);
  }
  if (const auto parameter = info_.hardware_parameters.find("hand_side");
      parameter != info_.hardware_parameters.end()) {
    hand_side_ = parameter->second;
  }
  if (const auto parameter = info_.hardware_parameters.find("can_interface");
      parameter != info_.hardware_parameters.end()) {
    can_interface_ = parameter->second;
  }
  if (const auto parameter = info_.hardware_parameters.find("can_baudrate");
      parameter != info_.hardware_parameters.end()) {
    try {
      can_baudrate_ = std::stoi(parameter->second);
    } catch (const std::exception& exception) {
      RCLCPP_FATAL(logger_, "Invalid can_baudrate '%s': %s", parameter->second.c_str(),
                   exception.what());
      return CallbackReturn::ERROR;
    }
  }
  if (const auto parameter = info_.hardware_parameters.find("state_timeout_ms");
      parameter != info_.hardware_parameters.end()) {
    try {
      const auto timeout_ms = std::stoll(parameter->second);
      if (timeout_ms <= 0) {
        RCLCPP_FATAL(logger_, "state_timeout_ms must be positive, got '%s'.", parameter->second.c_str());
        return CallbackReturn::ERROR;
      }
      state_timeout_ = std::chrono::milliseconds(timeout_ms);
    } catch (const std::exception& exception) {
      RCLCPP_FATAL(logger_, "Invalid state_timeout_ms '%s': %s", parameter->second.c_str(),
                   exception.what());
      return CallbackReturn::ERROR;
    }
  }

  if (hand_side_ == "left") {
    can_id_ = 0x28;
  } else if (hand_side_ == "right") {
    can_id_ = 0x27;
  } else {
    RCLCPP_FATAL(logger_, "Invalid hand_side '%s'. Expected 'left' or 'right'.", hand_side_.c_str());
    return CallbackReturn::ERROR;
  }

  if (!configureModel()) {
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  for (const auto& joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }
  if (joint_names_.size() != command_joint_to_sdk_index_.size()) {
    RCLCPP_FATAL(logger_, "Model %s expects %zu command joints, got %zu.", hand_model_.c_str(),
                 command_joint_to_sdk_index_.size(), joint_names_.size());
    return CallbackReturn::ERROR;
  }

  hw_positions_.assign(joint_names_.size(), 0.0);
  hw_position_commands_.assign(joint_names_.size(), 0.0);
  pending_position_commands_.assign(joint_names_.size(), 0.0);
  last_published_positions_.assign(joint_names_.size(), 0.0);
  sdk_positions_.assign(min_values_.size(), 0.0);
  sdk_position_commands_.assign(min_values_.size(), 0.0);

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto& joint = info_.joints[joint_index];
    for (const auto& state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION &&
          !state_interface.initial_value.empty()) {
        double parsed_value = 0.0;
        try {
          parsed_value = std::stod(state_interface.initial_value);
        } catch (const std::exception& exception) {
          RCLCPP_FATAL(logger_, "Invalid initial_value '%s' for joint '%s': %s",
                       state_interface.initial_value.c_str(), joint.name.c_str(), exception.what());
          return CallbackReturn::ERROR;
        }
        hw_positions_[joint_index] = parsed_value;
        hw_position_commands_[joint_index] = parsed_value;
        pending_position_commands_[joint_index] = parsed_value;
        last_published_positions_[joint_index] = parsed_value;
        const auto sdk_index = command_joint_to_sdk_index_[joint_index];
        sdk_positions_[sdk_index] = parsed_value;
        sdk_position_commands_[sdk_index] = parsed_value;
      }
    }
  }

  tactile_interface_names_.clear();
  tactile_states_.clear();
  if (hand_model_ == "G20") {
    tactile_interface_names_ = {
        "normal_force_thumb", "normal_force_index", "normal_force_middle", "normal_force_ring",
        "normal_force_little", "tangential_force_thumb", "tangential_force_index",
        "tangential_force_middle", "tangential_force_ring", "tangential_force_little",
        "tangential_direction_thumb", "tangential_direction_index", "tangential_direction_middle",
        "tangential_direction_ring", "tangential_direction_little", "approach_thumb", "approach_index",
        "approach_middle", "approach_ring", "approach_little", "thumb_matrix_mass",
        "index_matrix_mass", "middle_matrix_mass", "ring_matrix_mass", "little_matrix_mass"};
    tactile_states_.assign(kG20TactileStateCount, 0.0);
  }

  RCLCPP_INFO(logger_, "Configured Linker Hand hardware model=%s side=%s interface=%s backend=%s",
              hand_model_.c_str(), hand_side_.c_str(), can_interface_.c_str(),
              backend_ == Backend::kCppSdk ? "linkerhand-cpp-sdk" : "g20-socketcan");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinkerHandHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    received_state_ = false;
    activation_time_ = std::chrono::steady_clock::now();
    last_state_time_ = activation_time_;
    last_g20_tactile_poll_ = activation_time_ - std::chrono::milliseconds(100);
    std::fill(g20_finger_state_received_.begin(), g20_finger_state_received_.end(), false);
  }

  if (backend_ == Backend::kCppSdk) {
    if (!activateSdk()) {
      return CallbackReturn::ERROR;
    }
  } else if (!openCanSocket()) {
    return CallbackReturn::ERROR;
  }

  startIoThread();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinkerHandHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  stopIoThread();
  deactivateSdk();
  closeCanSocket();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LinkerHandHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() + tactile_interface_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    state_interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION,
                                  &hw_positions_[index]);
  }
  for (std::size_t index = 0; index < tactile_interface_names_.size(); ++index) {
    state_interfaces.emplace_back("g20_tactile", tactile_interface_names_[index], &tactile_states_[index]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LinkerHandHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    command_interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION,
                                    &hw_position_commands_[index]);
  }
  return command_interfaces;
}

hardware_interface::return_type LinkerHandHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/) {
  if (backend_ == Backend::kCppSdk && !readSdkState()) {
    return hardware_interface::return_type::ERROR;
  }

  std::scoped_lock<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  if (!received_state_) {
    if (now - activation_time_ > state_timeout_) {
      RCLCPP_ERROR(logger_, "No Linker Hand %s state received within %ld ms on %s.",
                   hand_model_.c_str(), state_timeout_.count(), can_interface_.c_str());
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  if (now - last_state_time_ > state_timeout_) {
    RCLCPP_ERROR(logger_, "Linker Hand %s state is stale for more than %ld ms on %s.",
                 hand_model_.c_str(), state_timeout_.count(), can_interface_.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LinkerHandHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/) {
  if (!std::all_of(hw_position_commands_.begin(), hw_position_commands_.end(), [](double command) {
        return std::isfinite(command);
      })) {
    return hardware_interface::return_type::ERROR;
  }

  if (!changed(hw_position_commands_, last_published_positions_)) {
    return hardware_interface::return_type::OK;
  }

  if (backend_ == Backend::kCppSdk) {
    try {
      auto sdk_command = sdk_position_commands_;
      for (std::size_t index = 0; index < hw_position_commands_.size(); ++index) {
        sdk_command[command_joint_to_sdk_index_[index]] = hw_position_commands_[index];
      }
      sdk_hand_->fingerMoveArc(sdk_command);
      sdk_position_commands_ = sdk_command;
      last_published_positions_ = hw_position_commands_;
      return hardware_interface::return_type::OK;
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(logger_, "Failed to command Linker Hand %s through C++ SDK: %s",
                   hand_model_.c_str(), exception.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  {
    std::scoped_lock<std::mutex> lock(command_mutex_);
    pending_position_commands_ = hw_position_commands_;
    for (std::size_t index = 0; index < hw_position_commands_.size(); ++index) {
      sdk_position_commands_[command_joint_to_sdk_index_[index]] = hw_position_commands_[index];
    }
    command_dirty_ = true;
  }
  last_published_positions_ = hw_position_commands_;
  return hardware_interface::return_type::OK;
}

bool LinkerHandHardwareInterface::configureModel() {
  if (hand_model_ == "L10") {
    backend_ = Backend::kCppSdk;
    min_values_ = hand_side_ == "left" ? kL10LeftMin : kL10RightMin;
    max_values_ = hand_side_ == "left" ? kL10LeftMax : kL10RightMax;
    directions_ = hand_side_ == "left" ? kL10LeftDirection : kL10RightDirection;
    command_joint_to_sdk_index_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  } else if (hand_model_ == "L20") {
    backend_ = Backend::kCppSdk;
    min_values_ = hand_side_ == "left" ? kL20LeftMin : kL20RightMin;
    max_values_ = hand_side_ == "left" ? kL20LeftMax : kL20RightMax;
    directions_ = hand_side_ == "left" ? kL20LeftDirection : kL20RightDirection;
    command_joint_to_sdk_index_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 16, 17, 18, 19};
  } else if (hand_model_ == "G20") {
    backend_ = Backend::kG20SocketCan;
    min_values_ = hand_side_ == "left" ? kL20LeftMin : kL20RightMin;
    max_values_ = hand_side_ == "left" ? kL20LeftMax : kL20RightMax;
    directions_ = hand_side_ == "left" ? kL20LeftDirection : kL20RightDirection;
    command_joint_to_sdk_index_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 16, 17, 18, 19};
  } else {
    RCLCPP_FATAL(logger_, "Unsupported Linker Hand model '%s'. Supported models: L10, L20, G20.",
                 hand_model_.c_str());
    return false;
  }
  return true;
}

bool LinkerHandHardwareInterface::activateSdk() {
  try {
    const auto sdk_model = hand_model_ == "L10" ? L10 : L20;
    const auto sdk_side = hand_side_ == "left" ? LEFT : RIGHT;
    sdk_hand_ = std::make_unique<LinkerHandApi>(sdk_model, sdk_side, can_interface_, can_baudrate_);
    return true;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "Failed to activate linkerhand-cpp-sdk for %s on %s: %s",
                 hand_model_.c_str(), can_interface_.c_str(), exception.what());
    return false;
  }
}

void LinkerHandHardwareInterface::deactivateSdk() {
  sdk_hand_.reset();
}

bool LinkerHandHardwareInterface::readSdkState() {
  if (!sdk_hand_) {
    return false;
  }
  try {
    const auto state = sdk_hand_->getStateArc();
    if (state.size() != min_values_.size()) {
      if (state.empty()) {
        return true;
      }
      RCLCPP_ERROR(logger_, "C++ SDK returned %zu states for %s, expected %zu.", state.size(),
                   hand_model_.c_str(), min_values_.size());
      return false;
    }
    std::scoped_lock<std::mutex> lock(state_mutex_);
    sdk_positions_ = state;
    for (std::size_t index = 0; index < hw_positions_.size(); ++index) {
      hw_positions_[index] = sdk_positions_[command_joint_to_sdk_index_[index]];
    }
    received_state_ = true;
    last_state_time_ = std::chrono::steady_clock::now();
    return true;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "Failed to read Linker Hand %s through C++ SDK: %s", hand_model_.c_str(),
                 exception.what());
    return false;
  }
}

bool LinkerHandHardwareInterface::openCanSocket() {
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_FATAL(logger_, "Failed to create SocketCAN socket: %s", std::strerror(errno));
    return false;
  }

  ifreq interface_request{};
  const auto max_name_length = sizeof(interface_request.ifr_name) - 1;
  if (can_interface_.size() > max_name_length) {
    RCLCPP_FATAL(logger_, "CAN interface name '%s' is too long.", can_interface_.c_str());
    closeCanSocket();
    return false;
  }
  std::strncpy(interface_request.ifr_name, can_interface_.c_str(), max_name_length);
  if (ioctl(can_socket_, SIOCGIFINDEX, &interface_request) < 0) {
    RCLCPP_FATAL(logger_, "Failed to resolve CAN interface '%s': %s.", can_interface_.c_str(),
                 std::strerror(errno));
    closeCanSocket();
    return false;
  }

  can_filter filter{};
  filter.can_id = can_id_;
  filter.can_mask = CAN_SFF_MASK;
  if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    RCLCPP_FATAL(logger_, "Failed to set CAN filter for id 0x%X: %s", can_id_, std::strerror(errno));
    closeCanSocket();
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(can_socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    RCLCPP_FATAL(logger_, "Failed to bind CAN socket to '%s': %s", can_interface_.c_str(),
                 std::strerror(errno));
    closeCanSocket();
    return false;
  }

  const int flags = fcntl(can_socket_, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
  }
  return true;
}

void LinkerHandHardwareInterface::closeCanSocket() {
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
  }
}

void LinkerHandHardwareInterface::startIoThread() {
  if (io_running_.exchange(true)) {
    return;
  }
  io_thread_ = std::thread([this]() { ioLoop(); });
}

void LinkerHandHardwareInterface::stopIoThread() {
  io_running_.store(false);
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void LinkerHandHardwareInterface::ioLoop() {
  if (backend_ == Backend::kCppSdk) {
    while (io_running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return;
  }

  auto last_poll = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
  while (io_running_.load()) {
    std::vector<double> command_snapshot;
    bool should_send_command = false;
    {
      std::scoped_lock<std::mutex> lock(command_mutex_);
      if (command_dirty_) {
        command_snapshot = pending_position_commands_;
        command_dirty_ = false;
        should_send_command = true;
      }
    }

    if (should_send_command) {
      sendG20Command(command_snapshot);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_poll >= std::chrono::milliseconds(40)) {
      pollG20State();
      last_poll = now;
    }

    receiveAvailableFrames(5);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool LinkerHandHardwareInterface::sendCanFrame(std::uint8_t command,
                                               const std::vector<std::uint8_t>& payload) {
  if (can_socket_ < 0 || payload.size() > 7) {
    return false;
  }

  can_frame frame{};
  frame.can_id = can_id_;
  frame.can_dlc = static_cast<__u8>(payload.size() + 1);
  frame.data[0] = command;
  std::copy(payload.begin(), payload.end(), frame.data + 1);

  const auto written = ::write(can_socket_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_WARN(logger_, "Failed to send CAN frame 0x%02X on %s: %s", command,
                can_interface_.c_str(), std::strerror(errno));
    return false;
  }
  return true;
}

void LinkerHandHardwareInterface::sendG20Command(const std::vector<double>& positions) {
  auto sdk_command = sdk_position_commands_;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    sdk_command[command_joint_to_sdk_index_[index]] = positions[index];
  }
  const auto range = radiansToDeviceRange(sdk_command);
  sendCanFrame(0x41, {range[5], range[10], range[0], 0, 0, range[15]});
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  sendCanFrame(0x42, {range[6], 0, range[1], 0, 0, range[16]});
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  sendCanFrame(0x43, {range[7], 0, range[2], 0, 0, range[17]});
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  sendCanFrame(0x44, {range[8], 0, range[3], 0, 0, range[18]});
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  sendCanFrame(0x45, {range[9], 0, range[4], 0, 0, range[19]});
}

void LinkerHandHardwareInterface::pollG20State() {
  for (const auto command : {0x41, 0x42, 0x43, 0x44, 0x45}) {
    sendCanFrame(static_cast<std::uint8_t>(command), {});
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  for (const auto command : {0x90, 0x91, 0x92, 0x93}) {
    sendCanFrame(static_cast<std::uint8_t>(command), {});
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_g20_tactile_poll_ >= std::chrono::milliseconds(100)) {
    for (const auto command : {0xB1, 0xB2, 0xB3, 0xB4, 0xB5}) {
      sendCanFrame(static_cast<std::uint8_t>(command), {kG20TouchMatrixCode});
      std::this_thread::sleep_for(std::chrono::milliseconds(7));
    }
    last_g20_tactile_poll_ = now;
  }
}

void LinkerHandHardwareInterface::receiveAvailableFrames(int timeout_ms) {
  if (can_socket_ < 0) {
    return;
  }

  pollfd descriptor{};
  descriptor.fd = can_socket_;
  descriptor.events = POLLIN;

  while (io_running_.load()) {
    const int poll_result = poll(&descriptor, 1, timeout_ms);
    if (poll_result <= 0) {
      return;
    }

    can_frame frame{};
    const auto received = ::read(can_socket_, &frame, sizeof(frame));
    if (received != static_cast<ssize_t>(sizeof(frame))) {
      return;
    }

    std::array<std::uint8_t, 8> data{};
    std::copy(std::begin(frame.data), std::end(frame.data), data.begin());
    processCanFrame(frame.can_id & CAN_SFF_MASK, data, frame.can_dlc);
    timeout_ms = 0;
  }
}

void LinkerHandHardwareInterface::processCanFrame(std::uint32_t can_id,
                                                  const std::array<std::uint8_t, 8>& data,
                                                  std::uint8_t length) {
  if (can_id != can_id_ || length < 2) {
    return;
  }
  const std::uint8_t command = data[0];
  if (command >= 0x41 && command <= 0x45) {
    processG20FingerFrame(command, data, length);
  } else if ((command >= 0x90 && command <= 0x93) || (command >= 0xB1 && command <= 0xB5)) {
    processG20TactileFrame(command, data, length);
  }
}

void LinkerHandHardwareInterface::processG20FingerFrame(std::uint8_t command,
                                                        const std::array<std::uint8_t, 8>& data,
                                                        std::uint8_t length) {
  if (length < 7) {
    return;
  }
  std::vector<std::uint8_t> raw(kTwentyJointCount, 0U);
  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    raw = radiansToDeviceRange(sdk_positions_);
  }

  const std::size_t finger = command - 0x41;
  const auto d = data.begin() + 1;
  if (finger == 0) {
    raw[5] = d[0];
    raw[10] = d[1];
    raw[0] = d[2];
    raw[15] = d[5];
  } else {
    raw[finger] = d[2];
    raw[5 + finger] = d[0];
    raw[15 + finger] = d[5];
  }

  const auto mapped_positions = deviceRangeToRadians(raw);
  std::scoped_lock<std::mutex> lock(state_mutex_);
  sdk_positions_ = mapped_positions;
  for (std::size_t index = 0; index < hw_positions_.size(); ++index) {
    hw_positions_[index] = sdk_positions_[command_joint_to_sdk_index_[index]];
  }
  g20_finger_state_received_[finger] = true;
  if (std::all_of(g20_finger_state_received_.begin(), g20_finger_state_received_.end(), [](bool value) {
        return value;
      })) {
    received_state_ = true;
    last_state_time_ = std::chrono::steady_clock::now();
  }
}

void LinkerHandHardwareInterface::processG20TactileFrame(std::uint8_t command,
                                                         const std::array<std::uint8_t, 8>& data,
                                                         std::uint8_t length) {
  std::scoped_lock<std::mutex> lock(state_mutex_);
  if (command >= 0x90 && command <= 0x93 && length >= 6) {
    const std::size_t offset = (command - 0x90) * 5;
    for (std::size_t index = 0; index < 5; ++index) {
      tactile_states_[offset + index] = static_cast<double>(data[index + 1]);
    }
    return;
  }

  if (command >= 0xB1 && command <= 0xB5 && length == 8) {
    const auto row_index = rowSelectorToIndex(data[1]);
    if (row_index == 255) {
      return;
    }
    const std::size_t finger = command - 0xB1;
    double row_mass = 0.0;
    for (std::size_t col = 0; col < 6; ++col) {
      row_mass += static_cast<double>(data[col + 2]);
    }
    g20_tactile_mass_[finger] += row_mass;
    if (row_index == 11) {
      tactile_states_[20 + finger] = g20_tactile_mass_[finger];
      g20_tactile_mass_[finger] = 0.0;
    }
  }
}

std::vector<std::uint8_t> LinkerHandHardwareInterface::radiansToDeviceRange(
    const std::vector<double>& positions) const {
  std::vector<std::uint8_t> result(min_values_.size(), 0U);
  for (std::size_t index = 0; index < min_values_.size(); ++index) {
    if (min_values_.size() == kTwentyJointCount && isReservedTwentyJointIndex(index)) {
      continue;
    }
    const double clamped = clamp(positions[index], min_values_[index], max_values_[index]);
    const double mapped = directions_[index] == -1
                              ? scale(clamped, min_values_[index], max_values_[index], 255.0, 0.0)
                              : scale(clamped, min_values_[index], max_values_[index], 0.0, 255.0);
    result[index] = static_cast<std::uint8_t>(clamp(std::round(mapped), 0.0, 255.0));
  }
  return result;
}

std::vector<double> LinkerHandHardwareInterface::deviceRangeToRadians(
    const std::vector<std::uint8_t>& positions) const {
  std::vector<double> result(min_values_.size(), 0.0);
  for (std::size_t index = 0; index < min_values_.size(); ++index) {
    if (min_values_.size() == kTwentyJointCount && isReservedTwentyJointIndex(index)) {
      continue;
    }
    const double value = clamp(static_cast<double>(positions[index]), 0.0, 255.0);
    result[index] = directions_[index] == -1
                        ? scale(value, 0.0, 255.0, max_values_[index], min_values_[index])
                        : scale(value, 0.0, 255.0, min_values_[index], max_values_[index]);
  }
  return result;
}

double LinkerHandHardwareInterface::scale(double value,
                                          double from_min,
                                          double from_max,
                                          double to_min,
                                          double to_max) {
  if (std::fabs(from_max - from_min) < 1e-12) {
    return to_min;
  }
  return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

double LinkerHandHardwareInterface::clamp(double value, double min_value, double max_value) {
  return std::min(max_value, std::max(min_value, value));
}

bool LinkerHandHardwareInterface::changed(const std::vector<double>& lhs,
                                          const std::vector<double>& rhs) {
  if (lhs.size() != rhs.size()) {
    return true;
  }
  constexpr double kEpsilon = 1e-6;
  const auto mismatch = std::mismatch(lhs.begin(), lhs.end(), rhs.begin(),
                                      [](double left, double right) {
                                        return std::fabs(left - right) <= kEpsilon;
                                      });
  return mismatch.first != lhs.end();
}

std::string LinkerHandHardwareInterface::uppercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

}  // namespace linker_hand_hardware

PLUGINLIB_EXPORT_CLASS(linker_hand_hardware::LinkerHandHardwareInterface,
                       hardware_interface::SystemInterface)
