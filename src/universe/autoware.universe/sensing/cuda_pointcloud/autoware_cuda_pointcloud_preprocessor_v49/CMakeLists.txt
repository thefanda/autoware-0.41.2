cmake_minimum_required(VERSION 3.14)
project(autoware_cuda_pointcloud_preprocessor_v49)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)
find_package(agnocastlib_v49)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found, so the autoware_cuda_pointcloud_preprocessor_v49 package will not be built.")
  return()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g -G")
endif()

if("$ENV{ENABLE_AGNOCAST}" AND NOT agnocastlib_v49_FOUND)
  message(FATAL_ERROR "agnocastlib_v49 is required when Agnocast is enabled")
endif()

ament_auto_find_build_dependencies()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Ensure NVCC also uses C++17 (otherwise std::optional / inline variables fail on CUDA 11.x)
if(NOT CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 17)
endif()
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if("$ENV{ENABLE_AGNOCAST}")
    message("Agnocast support is enabled")
    add_definitions(-DUSE_AGNOCAST_ENABLED)
endif()

if(BUILD_TESTING)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

# cSpell: ignore expt gencode
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
# Force C++17 for FindCUDA-based builds (CUDA 11.x often defaults to C++14).
list(APPEND CUDA_NVCC_FLAGS "-std=c++17")
# ================= CUDA ARCH =================
# Jetson (aarch64) only supports its native arch
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # Jetson Orin = compute_87 (JetPack / CUDA 11.x)
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
else()
  # x86 desktop GPUs
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
endif()
# TODO(knzo25): enable when the driver supports it
# list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")

# __host__ or __device__ annotation on lambda requires --extended-lambda nvcc flag
list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")

# cSpell: ignore fmad
# NOTE(manato): adding `--fmad=false` can reduce numerical discrepancies between CPU implementations
# and GPU implementations of the filters. But a slight performance impact can also be introduced
# To prioritize performance, the flag is disabled
#
# list(APPEND CUDA_NVCC_FLAGS "--fmad=false")

################################################################################
### cuda_pointcloud_preprocessor ###
cuda_add_library(cuda_pointcloud_preprocessor_lib_v49 SHARED
  src/cuda_concatenate_data/cuda_combine_cloud_handler.cpp
  src/cuda_concatenate_data/cuda_combine_cloud_handler_kernel.cu
  src/cuda_concatenate_data/cuda_cloud_collector.cpp
  src/cuda_concatenate_data/cuda_collector_matching_strategy.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  src/cuda_pointcloud_preprocessor/common_kernels.cu
  src/cuda_pointcloud_preprocessor/organize_kernels.cu
  src/cuda_pointcloud_preprocessor/outlier_kernels.cu
  src/cuda_pointcloud_preprocessor/undistort_kernels.cu
  src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter.cu
  src/cuda_downsample_filter/cuda_random_downsample_filter.cu
  src/cuda_outlier_filter/cuda_polar_voxel_outlier_filter.cu
)

target_link_libraries(cuda_pointcloud_preprocessor_lib_v49
  ${autoware_pointcloud_preprocessor_v49_TARGETS}
)

target_include_directories(cuda_pointcloud_preprocessor_lib_v49 SYSTEM PRIVATE
  ${autoware_pointcloud_preprocessor_v49_INCLUDE_DIRS}
  ${autoware_point_types_v49_INCLUDE_DIRS}
  ${autoware_sensing_msgs_v49_INCLUDE_DIRS}
  ${cuda_blackboard_v49_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${autoware_cuda_utils_v49_INCLUDE_DIRS}
)

if("$ENV{ENABLE_AGNOCAST}")
    target_include_directories(cuda_pointcloud_preprocessor_lib_v49 SYSTEM PRIVATE
        ${autoware_agnocast_wrapper_v49_INCLUDE_DIRS}
        ${agnocastlib_v49_INCLUDE_DIRS}
    )
    target_link_libraries(cuda_pointcloud_preprocessor_lib_v49
        ${agnocastlib_v49_LIBRARIES}
    )
endif()


# Targets
ament_auto_add_library(cuda_pointcloud_preprocessor_v49 SHARED
  src/cuda_concatenate_data/cuda_concatenate_and_time_sync_node.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_crop_only_node.cpp
  src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.cpp
  src/cuda_downsample_filter/cuda_random_downsample_filter_node.cpp
  src/cuda_outlier_filter/cuda_polar_voxel_outlier_filter_node.cpp
)

target_link_libraries(cuda_pointcloud_preprocessor_v49
  ${CUDA_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  cuda_pointcloud_preprocessor_lib_v49
)

# ========== Concatenate and Sync data ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaPointCloudConcatenateDataSynchronizerComponent"
  EXECUTABLE cuda_concatenate_and_time_sync_node_v49)

# ========== Pointcloud preprocessor ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaPointcloudPreprocessorNode"
  EXECUTABLE cuda_pointcloud_preprocessor_node_v49
)

# ========== Pointcloud preprocessor crop only ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaPointcloudPreprocessorCropOnlyNode"
  EXECUTABLE cuda_pointcloud_preprocessor_crop_only_node_v49
)

# ========== VoxelGrid downsample filter ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaVoxelGridDownsampleFilterNode"
  EXECUTABLE cuda_voxel_grid_downsample_filter_node_v49
)

# ========== PolarVoxel outlier filter ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaPolarVoxelOutlierFilterNode"
  EXECUTABLE cuda_polar_voxel_outlier_filter_node_v49
)

# ========== Random downsample filter ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode"
  EXECUTABLE cuda_random_downsample_filter_node_v49
)

################################################################################
# Install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    cuda_pointcloud_preprocessor_v49
    cuda_pointcloud_preprocessor_lib_v49
  LIBRARY DESTINATION lib
)

ament_auto_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
