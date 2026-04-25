cmake_minimum_required(VERSION 3.8)
project(pnp_dual_fr3_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
