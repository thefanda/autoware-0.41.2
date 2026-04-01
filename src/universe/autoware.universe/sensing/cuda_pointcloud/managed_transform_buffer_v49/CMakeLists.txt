cmake_minimum_required(VERSION 3.14)
project(managed_transform_buffer_v49)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE -D__GLIBCXX_USE_CLOCK_MONOTONIC=1 -D__GLIBCXX_USE_CLOCK_REALTIME=1")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/managed_transform_buffer.cpp
  src/managed_transform_buffer_provider.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test test/test_managed_transform_buffer.cpp)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()

if(BUILD_EXAMPLES)
  message(STATUS "Building examples")
  add_executable(example_managed_transform_buffer_v49 examples/example_managed_transform_buffer_v49.cpp)
  target_link_libraries(example_managed_transform_buffer_v49 ${PROJECT_NAME})
  install(TARGETS example_managed_transform_buffer_v49
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_auto_package()
