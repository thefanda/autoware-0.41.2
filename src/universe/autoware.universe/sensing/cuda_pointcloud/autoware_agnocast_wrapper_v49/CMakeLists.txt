cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper_v49)

find_package(autoware_cmake_v49 REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(agnocastlib_v49 REQUIRED)

include_directories(
  include
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  agnocastlib_v49
)

install(
  FILES cmake/agnocast_wrapper_config_extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_auto_package(
  CONFIG_EXTRAS "cmake/agnocast_wrapper_config_extras.cmake"
)
