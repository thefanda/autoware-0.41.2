cmake_minimum_required(VERSION 3.5)
project(infinite_loop_cpp)

# 建议开启 C++14 (Autoware 标准)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# 1. 寻找依赖
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dfcv_mining_msgs REQUIRED)

# Node 1 需要的 API
find_package(autoware_adapi_v1_msgs REQUIRED)

# Node 2 需要的地图库 (完全替代 tinyxml2)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_lanelet2_extension REQUIRED)
find_package(lanelet2_core REQUIRED)

add_executable(infinite_loop_node src/infinite_loop_node.cpp)
ament_target_dependencies(infinite_loop_node
  rclcpp
  autoware_adapi_v1_msgs
  geometry_msgs
  nav_msgs
)
add_executable(map_parser_node src/map_parser_node.cpp)

ament_target_dependencies(map_parser_node
  rclcpp
  geometry_msgs
  std_msgs
  dfcv_mining_msgs
  autoware_map_msgs
  autoware_lanelet2_extension
  lanelet2_core
)

# ============================================
# 4. 安装配置
# ============================================
install(TARGETS 
  infinite_loop_node
  map_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config 文件夹
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
