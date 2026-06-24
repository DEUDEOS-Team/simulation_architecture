"""
Veri toplama launch'ı (data_collect).

Amaç: MODEL ÇALIŞTIRMADAN haritayı dolaşıp ham kamera karesi toplamak. Sadece:
  Gazebo + araç + kamera köprüsü + WASD teleop
Algı node'ları (lane_test / lane_yolo / kamera tespiti) BİLEREK yok — kayıt edilen
/camera/image overlay'siz, ham olsun diye.

Kullanım:
  # 1. terminal:
  ros2 launch araba data_collect.launch.py
  # 2. terminal (ham kareleri kaydeder):
  python3 src/araba/scripts/dataset_recorder.py

WASD ile sür; recorder her ~0.5 m'de bir kareyi PNG kaydeder.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('araba')
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    gz_resource_path = ':'.join([
        os.path.dirname(pkg_share),
        os.path.join(ws_root, 'otonom_projem', 'models'),
    ])

    world_path = os.path.join(pkg_share, 'worlds', 'benim_dunyam.sdf')
    urdf_path = PathJoinSubstitution([FindPackageShare('araba'), 'urdf', 'araba.urdf'])
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

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
                        '-x', '45.31', '-y', '16', '-z', '0.51', '-Y', '1.58']),

        # Kamera + cmd_vel + odom köprüsü (algı node'u YOK)
        TimerAction(period=5.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 remappings=[('/model/araba/joint_state', '/joint_states')],
                 arguments=[
                     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                     '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                     '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     '/model/araba/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                 ]),
        ]),

        # WASD ile manuel sürüş
        TimerAction(period=6.0, actions=[
            Node(package='araba', executable='keyboard_teleop', name='keyboard_teleop',
                 output='screen'),
        ]),
    ])
