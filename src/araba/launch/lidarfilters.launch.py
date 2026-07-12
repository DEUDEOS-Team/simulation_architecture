import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. KATMAN: İskelet (Statik TF) Bağlantısı
        # Arabanın şasisi (base_link) ile LiDAR sensörünün (odom_lidar) konumunu birleştirir.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            output='screen',
            # 0 0 0 (X, Y, Z metre) | 0 0 0 (Roll, Pitch, Yaw radyan)
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'odom_lidar']
        ),

        # 2. KATMAN: Voxel Grid Filtresi (DEOS Veri Seyreltme)
        Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name='voxel_grid_filter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'filter_field_name': 'z',
                'filter_limit_min': -0.5,
                'filter_limit_max': 2.0,
                'filter_limit_negative': False,
                'leaf_size': 0.2
            }],
            remappings=[
                ('input', '/lidar/scan/points'),
                ('output', '/deos/sensors/lidar/points_downsampled')
            ]
        ),

        # 3. KATMAN: KISS-ICP Odometri Düğümü (Kısa Vadeli Hafıza)
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'odom',
                'child_frame': 'base_link',
                'publish_odom_tf': False,
                'publish_aliases_tf': False
            }],
            remappings=[
                ('pointcloud_topic', '/deos/sensors/lidar/points_downsampled'),
                ('odometry', '/kiss/odometry')
            ]
        )
    ])