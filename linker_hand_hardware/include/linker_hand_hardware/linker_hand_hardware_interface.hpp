#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

class LinkerHandApi;

namespace linker_hand_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LinkerHandHardwareInterface : public hardware_interface::SystemInterface {
 public:
  LinkerHandHardwareInterface() = default;
  ~LinkerHandHardwareInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  enum class Backend { kCppSdk, kG20SocketCan };

  bool configureModel();
  bool openCanSocket();
  void closeCanSocket();
  bool activateSdk();
  void deactivateSdk();
  void startIoThread();
  void stopIoThread();
  void ioLoop();
  bool readSdkState();
  bool sendCanFrame(std::uint8_t command, const std::vector<std::uint8_t>& payload);
  void receiveAvailableFrames(int timeout_ms);
  void processCanFrame(std::uint32_t can_id,
                       const std::array<std::uint8_t, 8>& data,
                       std::uint8_t length);
  void processG20FingerFrame(std::uint8_t command, const std::array<std::uint8_t, 8>& data,
                             std::uint8_t length);
  void processG20TactileFrame(std::uint8_t command, const std::array<std::uint8_t, 8>& data,
                              std::uint8_t length);
  void sendG20Command(const std::vector<double>& positions);
  void pollG20State();
  std::vector<std::uint8_t> radiansToDeviceRange(const std::vector<double>& positions) const;
  std::vector<double> deviceRangeToRadians(const std::vector<std::uint8_t>& positions) const;
  static double scale(double value, double from_min, double from_max, double to_min, double to_max);
  static double clamp(double value, double min_value, double max_value);
  static bool changed(const std::vector<double>& lhs, const std::vector<double>& rhs);
  static std::string uppercase(std::string value);

  rclcpp::Logger logger_{rclcpp::get_logger("LinkerHandHardwareInterface")};
  std::thread io_thread_;
  std::atomic<bool> io_running_{false};

  std::mutex state_mutex_;
  std::mutex command_mutex_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> tactile_interface_names_;

  std::vector<double> hw_positions_;
  std::vector<double> tactile_states_;

  std::vector<double> hw_position_commands_;
  std::vector<double> last_published_positions_;
  std::vector<double> pending_position_commands_;
  std::vector<double> sdk_positions_;
  std::vector<double> sdk_position_commands_;

  std::vector<double> min_values_;
  std::vector<double> max_values_;
  std::vector<int> directions_;
  std::vector<std::size_t> command_joint_to_sdk_index_;

  std::array<bool, 5> g20_finger_state_received_{false, false, false, false, false};
  std::array<double, 5> g20_tactile_mass_{0.0, 0.0, 0.0, 0.0, 0.0};

  std::unique_ptr<LinkerHandApi> sdk_hand_;
  Backend backend_{Backend::kCppSdk};
  std::string hand_model_{"L20"};
  std::string hand_side_{"left"};
  std::string can_interface_{"can0"};
  std::uint32_t can_id_{0x28};
  int can_socket_{-1};
  int can_baudrate_{1000000};
  std::chrono::steady_clock::time_point activation_time_{};
  std::chrono::steady_clock::time_point last_state_time_{};
  std::chrono::steady_clock::time_point last_g20_tactile_poll_{};
  std::chrono::milliseconds state_timeout_{1000};
  bool command_dirty_{false};
  bool received_state_{false};
};

}  // namespace linker_hand_hardware
