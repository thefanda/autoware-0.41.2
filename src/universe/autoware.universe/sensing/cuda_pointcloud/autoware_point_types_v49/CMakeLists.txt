cmake_minimum_required(VERSION 3.14)
project(autoware_point_types_v49)

find_package(autoware_cmake_v49 REQUIRED)
autoware_package()

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_autoware_point_types_v49
    test/test_point_types.cpp
  )
  target_include_directories(test_autoware_point_types_v49
    PRIVATE include
  )
  ament_target_dependencies(test_autoware_point_types_v49
    point_cloud_msg_wrapper
  )
endif()

autoware_ament_auto_package()
