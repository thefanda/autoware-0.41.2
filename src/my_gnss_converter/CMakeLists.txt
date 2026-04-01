cmake_minimum_required(VERSION 3.8)
project(my_gnss_converter)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
add_definitions(-DPCL_NO_PRECOMPILE)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)  # 添加 pcl_conversions 查找
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

find_package(dfcv_mining_msgs REQUIRED)  # 你的自定义消息包
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)  # 新增

# pointcloud_converter 可执行文件
add_executable(localization_compare_node
  src/localization_compare_node.cpp
)
add_executable(pointcloud_converter src/pointcloud_converter.cpp)
target_link_libraries(pointcloud_converter
  ${PCL_LIBRARIES}
  pcl_common  # 添加 PCL 基础库
)

ament_target_dependencies(
  localization_compare_node
  rclcpp
  message_filters
  nav_msgs
  geometry_msgs
  dfcv_mining_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
ament_target_dependencies(pointcloud_converter 
  rclcpp 
  sensor_msgs
  dfcv_mining_msgs
  pcl_conversions
  autoware_vehicle_msgs  # 新增
)


# gnss_converter_node 可执行文件
add_executable(gnss_converter_node src/gnss_converter_node.cpp)
ament_target_dependencies(gnss_converter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ublox_msgs
  dfcv_mining_msgs
  tf2
  tf2_ros
  pcl_conversions
)

# 安装目标
  
install(TARGETS
  pointcloud_converter
  gnss_converter_node
  localization_compare_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp 
  sensor_msgs 
  pcl_conversions
  ublox_msgs
  dfcv_mining_msgs
  tf2
  tf2_ros
  autoware_vehicle_msgs
  message_filters
  nav_msgs
  geometry_msgs
  # 新增
)

ament_package()
