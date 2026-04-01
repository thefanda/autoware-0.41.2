cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor_v49)

find_package(autoware_cmake_v49 REQUIRED)
autoware_package()

# Suppress CGAL warning
set(CGAL_DATA_DIR ".")
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pointcloud_preprocessor_filter_base_v49_ SHARED
  src/filter.cpp
  src/utility/memory.cpp
)

target_include_directories(pointcloud_preprocessor_filter_base_v49_ PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${autoware_point_types_v49_INCLUDE_DIRS}
)

ament_target_dependencies(pointcloud_preprocessor_filter_base_v49_
  SYSTEM
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  autoware_utils_v49
  pcl_ros
  autoware_point_types_v49
  managed_transform_buffer_v49
)

add_library(faster_voxel_grid_downsample_filter_v49 SHARED
  src/downsample_filter/faster_voxel_grid_downsample_filter.cpp
)

target_include_directories(faster_voxel_grid_downsample_filter_v49 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(faster_voxel_grid_downsample_filter_v49
  SYSTEM
  pcl_conversions
  rclcpp
  sensor_msgs
)

add_library(concatenate_data_v49 SHARED
  src/concatenate_data/combine_cloud_handler_base.cpp
  src/concatenate_data/combine_cloud_handler.cpp
  src/concatenate_data/cloud_collector.cpp
  src/concatenate_data/collector_matching_strategy.cpp
  src/concatenate_data/concatenate_pointclouds.cpp
)

target_include_directories(concatenate_data_v49 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(concatenate_data_v49
  SYSTEM
  autoware_point_types_v49
  autoware_sensing_msgs_v49
  autoware_utils_v49
  autoware_vehicle_msgs_v49
  managed_transform_buffer_v49
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_ros
  rclcpp_components
)

ament_auto_add_library(pointcloud_preprocessor_filter_v49_ SHARED
  src/concatenate_data/concatenate_and_time_sync_node.cpp
  src/crop_box_filter/crop_box_filter_node.cpp
  src/time_synchronizer/time_synchronizer_node.cpp
  src/downsample_filter/voxel_grid_downsample_filter_node.cpp
  src/downsample_filter/random_downsample_filter_node.cpp
  src/downsample_filter/approximate_downsample_filter_node.cpp
  src/downsample_filter/pickup_based_voxel_grid_downsample_filter_node.cpp
  src/outlier_filter/ring_outlier_filter_node.cpp
  src/outlier_filter/radius_search_2d_outlier_filter_node.cpp
  src/outlier_filter/voxel_grid_outlier_filter_node.cpp
  src/outlier_filter/polar_voxel_outlier_filter_node.cpp
  src/outlier_filter/dual_return_outlier_filter_node.cpp
  src/passthrough_filter/passthrough_filter_node.cpp
  src/passthrough_filter/passthrough_filter_uint16_node.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/pointcloud_accumulator/pointcloud_accumulator_node.cpp
  src/vector_map_filter/lanelet2_map_filter_node.cpp
  src/distortion_corrector/distortion_corrector.cpp
  src/distortion_corrector/distortion_corrector_node.cpp
  src/blockage_diag/blockage_diag_node.cpp
  src/polygon_remover/polygon_remover.cpp
  src/vector_map_filter/vector_map_inside_area_filter_node.cpp
  src/utility/geometry.cpp
  src/pointcloud_densifier/pointcloud_densifier_node.cpp
  src/pointcloud_densifier/occupancy_grid.cpp
)

target_link_libraries(pointcloud_preprocessor_filter_v49_
  pointcloud_preprocessor_filter_base_v49_
  faster_voxel_grid_downsample_filter_v49
  concatenate_data_v49
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf2_sensor_msgs_LIBRARIES}
)

# ========== Time synchronizer ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PointCloudDataSynchronizerComponent"
  EXECUTABLE time_synchronizer_node_v49)

# ========== Concatenate data ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PointCloudConcatenationComponent"
  EXECUTABLE concatenate_pointclouds_node_v49)

# ========== Concatenate and Sync data ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PointCloudConcatenateDataSynchronizerComponent"
  EXECUTABLE concatenate_and_time_sync_node_v49)

# ========== CropBox ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::CropBoxFilterComponent"
  EXECUTABLE crop_box_filter_node_v49)

# ========== Down Sampler Filter ==========
# -- Voxel Grid Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::VoxelGridDownsampleFilterComponent"
  EXECUTABLE voxel_grid_downsample_filter_node_v49)

# -- Pickup Based Voxel Grid Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PickupBasedVoxelGridDownsampleFilterComponent"
  EXECUTABLE pickup_based_voxel_grid_downsample_filter_node_v49)

# -- Random Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::RandomDownsampleFilterComponent"
  EXECUTABLE random_downsample_filter_node_v49)

rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::ApproximateDownsampleFilterComponent"
  EXECUTABLE approximate_downsample_filter_node_v49)

# ========== Outlier Filter ==========
# -- Ring Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::RingOutlierFilterComponent"
  EXECUTABLE ring_outlier_filter_node_v49)

# -- Voxel Grid Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::VoxelGridOutlierFilterComponent"
  EXECUTABLE voxel_grid_outlier_filter_node_v49)

# -- Polar Voxel Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PolarVoxelOutlierFilterComponent"
  EXECUTABLE polar_voxel_outlier_filter_node_v49)

# -- Radius Search 2D Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::RadiusSearch2DOutlierFilterComponent"
  EXECUTABLE radius_search_2d_outlier_filter_node_v49)

# -- DualReturn Outlier Filter--
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::DualReturnOutlierFilterComponent"
  EXECUTABLE dual_return_outlier_filter_node_v49)

# ========== Passthrough Filter ==========
# -- Passthrough Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PassThroughFilterComponent"
  EXECUTABLE passthrough_filter_node_v49)

# -- Passthrough Filter Uint16 --
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PassThroughFilterUInt16Component"
  EXECUTABLE passthrough_filter_uint16_node_v49)

# ========== Pointcloud Accumulator Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PointcloudAccumulatorComponent"
  EXECUTABLE pointcloud_accumulator_node_v49)

# ========== Vector Map Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::Lanelet2MapFilterComponent"
  EXECUTABLE vector_map_filter_node_v49)

# ========== Distortion Corrector ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::DistortionCorrectorComponent"
  EXECUTABLE distortion_corrector_node_v49)

# ========== Blockage Diagnostics ===========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::BlockageDiagComponent"
  EXECUTABLE blockage_diag_node_v49)

# ========== PolygonRemover ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PolygonRemoverComponent"
  EXECUTABLE polygon_remover_node_v49)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(polygon_remover_node_v49 gmp CGAL CGAL::CGAL CGAL::CGAL_Core)

# ========== Vector Map Inside Area Filter ===========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::VectorMapInsideAreaFilterComponent"
  EXECUTABLE vector_map_inside_area_filter_node_v49)

# ========== Pointcloud Densifier ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter_v49_
  PLUGIN "autoware::pointcloud_preprocessor_v49::PointCloudDensifierNode"
  EXECUTABLE pointcloud_densifier_node_v49)

# Make sure launch directory is installed
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS pointcloud_preprocessor_filter_base_v49_ EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS faster_voxel_grid_downsample_filter_v49 EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS concatenate_data_v49 EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_utilities
    test/test_utilities.cpp
  )

  ament_add_gtest(test_distortion_corrector_node_v49
    test/test_distortion_corrector_node.cpp
  )

  ament_add_gtest(test_concatenate_node_unit
    test/test_concatenate_node_unit.cpp
  )

  ament_add_gtest(test_pickup_based_voxel_grid_downsample_filter_node_v49
    test/test_pickup_based_voxel_grid_downsample_filter_node.cpp
  )

  ament_add_gtest(test_concatenation_info
    test/test_concatenation_info.cpp
  )

  ament_add_gtest(test_polar_voxel_outlier_filter_node_v49
    test/test_polar_voxel_outlier_filter_node.cpp
  )

  target_link_libraries(test_utilities pointcloud_preprocessor_filter_v49_)
  target_link_libraries(test_distortion_corrector_node_v49 pointcloud_preprocessor_filter_v49_)
  target_link_libraries(test_concatenate_node_unit pointcloud_preprocessor_filter_v49_)
  target_link_libraries(test_pickup_based_voxel_grid_downsample_filter_node_v49 pointcloud_preprocessor_filter_v49_)
  target_link_libraries(test_concatenation_info concatenate_data_v49)
  target_link_libraries(test_polar_voxel_outlier_filter_node_v49 pointcloud_preprocessor_filter_v49_)

  add_ros_test(
    test/test_concatenate_node_component.py
    TIMEOUT "50"
  )

endif()
