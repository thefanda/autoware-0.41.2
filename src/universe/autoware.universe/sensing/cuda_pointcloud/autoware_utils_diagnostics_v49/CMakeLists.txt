cmake_minimum_required(VERSION 3.14)
project(autoware_utils_diagnostics_v49)

find_package(autoware_cmake_v49 REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/diagnostics_interface.cpp"
  "src/timeout_diagnostics.cpp"
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    "test/main.cpp"
    "test/cases/diagnostics_interface.cpp"
    "test/cases/timeout_diagnostics.cpp"
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
