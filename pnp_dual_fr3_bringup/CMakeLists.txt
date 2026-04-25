cmake_minimum_required(VERSION 3.8)
project(pnp_dual_fr3_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
