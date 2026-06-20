#!/usr/bin/env python3
"""
Gazebo Harmonic simülasyonu için araba_v2 launch dosyası.
ros_gz_sim ile boş dünya başlatır, robotu spawn eder, ros_gz_bridge ile
ROS 2 <-> Gazebo mesaj köprüsünü kurar.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('araba_v2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'araba_v2.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Gazebo Harmonic boş dünya
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Robotu Gazebo'ya spawn et
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_araba_v2',
        output='screen',
        arguments=[
            '-name', 'araba_v2',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ]
    )

    # ros_gz_bridge: ROS 2 <-> Gazebo mesaj köprüsü
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states
            '/world/empty/model/araba_v2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Tekerlek hız komutları
            '/model/araba_v2/joint/on_sag_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/on_sol_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/arka_sol_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/arka_sag_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            # Direksiyon pozisyon komutları
            '/model/araba_v2/joint/on_sag_yatay_direksiyon_hareket/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/on_sol_yatay_direksiyon_hareket/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/empty/model/araba_v2/joint_state', '/joint_states'),
        ]
    )

    # Statik TF: base_footprint -> base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        gazebo,
        TimerAction(period=3.0, actions=[spawn_robot]),
        robot_state_pub,
        static_tf,
        TimerAction(period=4.0, actions=[bridge]),
    ])
