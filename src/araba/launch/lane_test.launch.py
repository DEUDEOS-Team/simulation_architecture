"""
Şerit modeli DENEME launch'ı (lane_test).

Amaç: kullanıcının attığı şerit modelini/kodunu (lane_test_node) izole bir ortamda
hızlıca denemek. Mevcut gazebo.launch.py'deki ağır algı node'ları (kamera tespiti,
şerit, dashboard) BURADA yok — sadece:
  Gazebo + araç + kamera köprüsü + lane_test_node (OpenCV penceresi) + WASD kontrol

Kullanım:
  ros2 launch araba lane_test.launch.py
  ros2 launch araba lane_test.launch.py show_window:=false   # pencereyi kapat
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('araba')
    # otonom_projem/models yolu workspace kökünden türetilir (kullanıcı adından bağımsız)
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    gz_resource_path = ':'.join([
        os.path.dirname(pkg_share),
        os.path.join(ws_root, 'otonom_projem', 'models'),
    ])

    world_path = os.path.join(pkg_share, 'worlds', 'benim_dunyam.sdf')
    urdf_path = PathJoinSubstitution([FindPackageShare('araba'), 'urdf', 'araba.urdf'])
    lane_model_path = os.path.join(pkg_share, 'models', 'onnx', 'lane_seg.onnx')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    show_window = LaunchConfiguration('show_window')

    # onnxruntime-gpu için CUDA kütüphane yolları (pip nvidia-*-cu12 paketleri).
    # Terminalde .bashrc source edilmemiş olsa bile GPU bulunsun diye node ortamına eklenir.
    import sys
    nvidia_base = os.path.join(
        os.path.expanduser('~'), '.local', 'lib',
        f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages', 'nvidia',
    )
    ld_dirs = []
    if os.path.isdir(nvidia_base):
        for d in sorted(os.listdir(nvidia_base)):
            libdir = os.path.join(nvidia_base, d, 'lib')
            if os.path.isdir(libdir):
                ld_dirs.append(libdir)
    ld_library_path = ':'.join(ld_dirs + [os.environ.get('LD_LIBRARY_PATH', '')]).rstrip(':')

    return LaunchDescription([
        DeclareLaunchArgument('show_window', default_value='true',
                              description='true ise OpenCV şerit penceresi açılır'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),

        # Gazebo (GUI açık — kamera sensörünün render etmesi için)
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

        # Sadece deneme için gerekli köprüler (lidar/imu/gps yok)
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

        # Şerit deneme node'u — OpenCV penceresinde çıktıyı gösterir
        TimerAction(period=6.0, actions=[
            Node(package='araba', executable='lane_test_node.py', name='lane_test_node',
                 output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'camera_topic': '/camera/image',
                     'model_path': lane_model_path,
                     'show_window': show_window,
                 }]),
        ]),

        # Otonom şerit-takip kontrolü — /perception/center_pts dinler, /cmd_vel yayınlar
        TimerAction(period=7.0, actions=[
            Node(package='araba', executable='autonomous_control_node.py', name='autonomous_control_node',
                 output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'cam_width': 1280.0,
                     'speed_scale': 4.0,
                 }]),
        ]),
    ])
