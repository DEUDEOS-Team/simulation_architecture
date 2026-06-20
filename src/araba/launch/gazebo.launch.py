import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('araba')
    gz_resource_path = ':'.join([
        os.path.dirname(pkg_share),
        '/home/aaltindas/sim_ws/otonom_projem/models',
    ])

    world_path = os.path.join(pkg_share, 'worlds', 'benim_dunyam.sdf')
    urdf_path = PathJoinSubstitution([FindPackageShare('araba'), 'urdf', 'araba.urdf'])
    rviz_config = os.path.join(pkg_share, 'config', 'lidar_view.rviz')
    detection_model_path = os.path.join(pkg_share, 'models', 'onnx', 'detection.onnx')
    segmentation_model_path = os.path.join(pkg_share, 'models', 'onnx', 'model.onnx')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true ise Gazebo GUI olmadan başlar'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        # Gazebo (orjinal ros_gz_sim launch'ı)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments={
                'gz_args': f'-r {world_path}',
                'on_exit_shutdown': 'True',
            }.items(),
        ),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'araba', '-topic', 'robot_description',
                        '-x', '44.498', '-y', '-69.858', '-z', '0.28', '-Y', '1.58']),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}]),

        TimerAction(period=5.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 remappings=[('/model/araba/joint_state', '/joint_states')],
                 arguments=[
                     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                     '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     '/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                     '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                     '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                     '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                     '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                     '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     '/model/araba/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                 ]),
        ]),

        TimerAction(period=6.0, actions=[
            Node(package='araba', executable='camera_perception_node', name='camera_perception_node',
                 output='screen', parameters=[{'use_sim_time': True, 'model_path': detection_model_path,
                                               'show_window': False}]),
        ]),

        TimerAction(period=6.5, actions=[
            Node(package='araba', executable='lane_perception_node', name='lane_perception_node',
                 output='screen', parameters=[{'use_sim_time': True, 'model_path': segmentation_model_path,
                                               'show_window': False}]),
        ]),

        TimerAction(period=7.0, actions=[
            Node(package='araba', executable='perception_pipeline_node', name='perception_pipeline_node',
                 output='screen', parameters=[{'use_sim_time': True, 'show_dashboard': True}]),
        ]),

        # TODO: Algoritma kontrolü aktif edilince yorumdan çıkar
        # TimerAction(period=7.5, actions=[
        #     Node(package='araba', executable='speed_controller_node', name='speed_controller_node',
        #          output='screen', parameters=[{'use_sim_time': True}]),
        # ]),

        # Manuel WASD kontrol — doğrudan /cmd_vel'e yayın yapar
        TimerAction(period=8.0, actions=[
            Node(package='araba', executable='keyboard_teleop', name='keyboard_teleop',
                 output='screen'),
        ]),
    ])
