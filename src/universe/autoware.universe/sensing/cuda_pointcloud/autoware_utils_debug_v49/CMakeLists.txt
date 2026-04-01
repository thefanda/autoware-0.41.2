cmake_minimum_required(VERSION 3.14)
project(autoware_utils_debug_v49)

find_package(autoware_cmake_v49 REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/time_keeper.cpp"
)

target_link_libraries(${PROJECT_NAME}
  fmt::fmt
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
