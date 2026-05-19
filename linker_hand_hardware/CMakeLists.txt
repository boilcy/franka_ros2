cmake_minimum_required(VERSION 3.8)
project(linker_hand_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(LINKERHAND_CPP_SDK_LIB_SUBDIR "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(LINKERHAND_CPP_SDK_LIB_SUBDIR "aarch64")
else()
  message(WARNING "Unknown processor '${CMAKE_SYSTEM_PROCESSOR}', defaulting linkerhand-cpp-sdk to x86_64")
  set(LINKERHAND_CPP_SDK_LIB_SUBDIR "x86_64")
endif()

set(LINKERHAND_CPP_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../linkerhand-cpp-sdk" CACHE PATH
  "Path to the local linkerhand-cpp-sdk checkout")
set(LINKERHAND_CPP_SDK_INCLUDE_DIR "${LINKERHAND_CPP_SDK_ROOT}/include")
set(LINKERHAND_CPP_SDK_LIB_DIR "${LINKERHAND_CPP_SDK_ROOT}/lib/${LINKERHAND_CPP_SDK_LIB_SUBDIR}")
find_library(LINKERHAND_CPP_SDK_LIBRARY
  NAMES linkerhand_cpp_sdk
  PATHS "${LINKERHAND_CPP_SDK_LIB_DIR}"
  NO_DEFAULT_PATH
)
if(NOT LINKERHAND_CPP_SDK_LIBRARY)
  message(FATAL_ERROR "linkerhand_cpp_sdk library not found in ${LINKERHAND_CPP_SDK_LIB_DIR}")
endif()

add_library(linker_hand_hardware SHARED
  src/linker_hand_hardware_interface.cpp
)

target_include_directories(linker_hand_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${LINKERHAND_CPP_SDK_INCLUDE_DIR}
)

ament_target_dependencies(linker_hand_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(linker_hand_hardware ${LINKERHAND_CPP_SDK_LIBRARY} Threads::Threads)
set_target_properties(linker_hand_hardware PROPERTIES
  BUILD_RPATH "${LINKERHAND_CPP_SDK_LIB_DIR}"
  INSTALL_RPATH "${LINKERHAND_CPP_SDK_LIB_DIR}"
)

pluginlib_export_plugin_description_file(hardware_interface linker_hand_hardware.xml)

install(TARGETS linker_hand_hardware DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(linker_hand_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
