from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autoware_cuda_pointcloud_preprocessor_v49')

    # cpu_container = ComposableNodeContainer(
    #     name='cpu_pointcloud_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     output='screen',
    #     composable_node_descriptions=[

            # ==================================================
            # 1. CropBox filter (CPU)
            # ==================================================
            # ComposableNode(
            #     package='autoware_pointcloud_preprocessor_v49',
            #     plugin='autoware::pointcloud_preprocessor_v49::CropBoxFilterComponent',
            #     name='crop_box_filter_measurement_range',
            #     parameters=[
            #         os.path.join(pkg_share, 'config', 'crop_box_filter_node.param.yaml')
            #     ],
            #     remappings=[
            #         ('input', '/sensing/lidar/concatenated/pointcloud'),
            #         ('output', 'measurement_range/pointcloud'),
            #     ],
            #     extra_arguments=[{
            #         'use_intra_process_comms': True
            #     }],
            # ),
            # ==================================================
            # 2. VoxelGrid downsample (CPU)
            # ==================================================
            # ComposableNode(
            #     package='autoware_pointcloud_preprocessor',
            #     plugin='autoware::pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
            #     name='voxel_grid_downsample_filter_v49',
            #     parameters=[
            #         os.path.join(pkg_share, 'config', 'voxel_grid_downsample_filter_node.param.yaml')
            #     ],
            #     remappings=[
            #         ('input', 'measurement_range/pointcloud'),
            #         ('output', 'voxel_grid_downsample/pointcloud'),
            #     ],
            #     extra_arguments=[{
            #         'use_intra_process_comms': True
            #     }],
            # ),
            # ==================================================
            # 3. Random downsample filter (CPU) - commented, using CUDA version in GPU container
            # ==================================================
            # ComposableNode(
            #     package='autoware_pointcloud_preprocessor_v49',
            #     plugin='autoware::pointcloud_preprocessor_v49::RandomDownsampleFilterComponent',
            #     name='random_downsample_filter_v49',
            #     parameters=[
            #         os.path.join(pkg_share, 'config', 'random_downsample_filter_node.param.yaml')
            #     ],
            #     remappings=[
            #         ('input', 'voxel_grid_downsample/pointcloud'),
            #         ('output', '/localization/util/downsample/pointcloud'),
            #     ],
            #     extra_arguments=[{
            #         'use_intra_process_comms': True
            #     }],
            # ),
    #     ],
    # )

    gpu_container = ComposableNodeContainer(
        name='gpu_pointcloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[

            # ==================================================
            # 1. CropBox filter (GPU)
            # ==================================================
            ComposableNode(
                package='autoware_cuda_pointcloud_preprocessor_v49',
                plugin='autoware::cuda_pointcloud_preprocessor_v49::CudaPointcloudPreprocessorCropOnlyNode',
                name='crop_box_keep_inside_40m_v49',
                parameters=[
                    os.path.join(pkg_share, 'config', 'crop_box_keep_inside_40m.param.yaml')
                ],
                remappings=[
                    ('~/input/pointcloud', '/sensing/lidar/concatenated/pointcloud'),
                    ('~/output/pointcloud', 'measurement_range/pointcloud'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': False
                }],
            ),
            # ==================================================
            # 1.2 Remove ego-vehicle box (GPU)
            # ==================================================
            # ComposableNode(
            #     package='autoware_cuda_pointcloud_preprocessor_v49',
            #     plugin='autoware::cuda_pointcloud_preprocessor_v49::CudaPointcloudPreprocessorCropOnlyNode',
            #     name='crop_box_remove_ego_vehicle_v49',
            #     parameters=[
            #         os.path.join(pkg_share, 'config', 'crop_box_remove_ego_vehicle.param.yaml')
            #     ],
            #     remappings=[
            #         ('~/input/pointcloud', 'range40/pointcloud'),
            #         ('~/output/pointcloud', 'measurement_range/pointcloud'),
            #     ],
            #     extra_arguments=[{
            #         'use_intra_process_comms': False
            #     }],
            # ),
            # ==================================================
            # 2. VoxelGrid downsample (CUDA)
            # ==================================================
            ComposableNode(
                package='autoware_cuda_pointcloud_preprocessor_v49',
                plugin='autoware::cuda_pointcloud_preprocessor_v49::CudaVoxelGridDownsampleFilterNode',
                name='voxel_grid_downsample_filter_v49',
                parameters=[
                    os.path.join(pkg_share, 'config', 'cuda_voxel_grid_downsample_filter.param.yaml')
                ],
                remappings=[
                    # CUDA 节点必须 remap wrapper 话题
                    ('~/input/pointcloud', 'measurement_range/pointcloud'),
                    ('~/output/pointcloud', 'voxel_grid_downsample/pointcloud'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': False
                }],
            ),
            # ==================================================
            # 3. Random downsample (CUDA)
            # # ==================================================
            ComposableNode(
                package='autoware_cuda_pointcloud_preprocessor_v49',
                plugin='autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode',
                name='random_downsample_filter_v49',
                parameters=[
                    os.path.join(pkg_share, 'config', 'cuda_random_downsample_filter.param.yaml')
                ],
                remappings=[
                    ('~/input/pointcloud', 'voxel_grid_downsample/pointcloud'),
                    ('~/output/pointcloud', '/localization/util/downsample/pointcloud'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': False
                }],
            ),
        ],
    )
    return LaunchDescription([
        # cpu_container,
        gpu_container,
    ])
