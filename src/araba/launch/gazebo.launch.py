import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Aracın dünya çerçevesindeki spawn pozu. final_odom_node bu değerlerle Gazebo'nun
# spawn-göreli /odom ve /imu/data'sını dünya (ENU) koordinatına çevirir — create ile senkron kalmalı.
SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_YAW = 45.31, 16.0, 0.51, 1.58
# GPS datumu — benim_dunyam.sdf içindeki <spherical_coordinates> ile senkron kalmalı.
# 2026-07-07: Gerçek yarışma sahasının merkezine taşındı (missions/map.geojson —
# arkadaşın sahadan aldığı yol ağı — merkezi ~40.7898904, 29.5088876).
DATUM_LAT, DATUM_LON = 40.7899, 29.5089


def generate_launch_description():
    pkg_share = get_package_share_directory('araba')
    # otonom_projem/models yolu workspace kökünden türetilir (kullanıcı adından bağımsız).
    # pkg_share = <ws>/install/share/araba -> dört üst dizin = workspace kökü
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    gz_resource_path = ':'.join([
        os.path.dirname(pkg_share),
        os.path.join(ws_root, 'otonom_projem', 'models'),
    ])

    # world:=saha_dunyasi.sdf ile gerçek saha zemininde başlar (worlds/ altındaki dosya adı)
    world_path = PathJoinSubstitution([FindPackageShare('araba'), 'worlds',
                                       LaunchConfiguration('world')])
    urdf_path = PathJoinSubstitution([FindPackageShare('araba'), 'urdf', 'araba.urdf'])
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'lidar_view.rviz')
    detection_model_path = os.path.join(pkg_share, 'models', 'onnx', 'detection.onnx')
    # 2026-07-11: yeni gazeboset yolov8s-seg modeli (model.onnx); eski model lane_seg.onnx'te yedek
    lane_model_path = os.path.join(pkg_share, 'models', 'onnx', 'model.onnx')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    headless = LaunchConfiguration('headless')
    manuel = LaunchConfiguration('manuel')
    show_window = LaunchConfiguration('show_window')
    localization = LaunchConfiguration('localization')
    # ekf: robot_localization (navsat+EKF) + KISS-ICP zinciri (arda'nın hattı)
    # final_odom: bizim GPS+IMU+odom füzyon node'umuz (7 turda kanıtlanmış yedek)
    use_ekf = IfCondition(PythonExpression(["'", localization, "' == 'ekf'"]))
    use_final_odom = IfCondition(PythonExpression(["'", localization, "' != 'ekf'"]))
    mission_file = LaunchConfiguration('mission_file')
    centerlines_file = LaunchConfiguration('centerlines_file')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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
        DeclareLaunchArgument('headless', default_value='false',
                              description='true ise Gazebo GUI olmadan başlar'),
        DeclareLaunchArgument('manuel', default_value='false',
                              description='true ise otonom kontrol yerine WASD klavye kontrolü'),
        DeclareLaunchArgument('show_window', default_value='true',
                              description='true ise şerit OpenCV penceresi açılır'),
        # Görev/harita dosyaları değiştirilebilir; ör. gerçek saha verisiyle plan testi:
        #   mission_file:=.../missions/saha_deneme_gorev.geojson
        #   centerlines_file:=.../missions/map_islenmis.geojson
        DeclareLaunchArgument('mission_file',
                              default_value=os.path.join(pkg_share, 'missions', 'teknofest_gorev.geojson'),
                              description='mission_planning görev GeoJSON yolu'),
        DeclareLaunchArgument('centerlines_file',
                              default_value=os.path.join(pkg_share, 'missions', 'teknofest_centerlines.geojson'),
                              description='mission_planning yol ağı GeoJSON yolu'),
        DeclareLaunchArgument('world', default_value='benim_dunyam.sdf',
                              description='worlds/ altındaki dünya dosyası (ör. saha_dunyasi.sdf)'),
        DeclareLaunchArgument('localization', default_value='ekf',
                              description="ekf: robot_localization+KISS-ICP | "
                                          "final_odom: eski GPS+IMU füzyon node'u (yedek)"),
        # Spawn pozu; saha dünyası için start noktası: -38.53 -0.91 yaw=-0.85
        DeclareLaunchArgument('spawn_x', default_value=str(SPAWN_X)),
        DeclareLaunchArgument('spawn_y', default_value=str(SPAWN_Y)),
        DeclareLaunchArgument('spawn_z', default_value=str(SPAWN_Z)),
        DeclareLaunchArgument('spawn_yaw', default_value=str(SPAWN_YAW)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),

        # Gazebo (orjinal ros_gz_sim launch'ı)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments={
                # headless:=true → '-s' ile sadece sunucu (GUI yok); GUI çökmeleri
                # (ör. OpenGL bağlamı açılamayan ortamlar) tüm stack'i düşürmesin.
                'gz_args': ['-r ', PythonExpression(
                    ["'-s ' if '", headless, "'.lower() == 'true' else ''"]),
                    world_path],
                'on_exit_shutdown': 'True',
            }.items(),
        ),

        # LiDAR ön-işleme + KISS-ICP odometrisi (/kiss/odometry) — sadece EKF hattında gerekli
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('araba'), 'launch', 'lidarfilters.launch.py')
            ),
            condition=use_ekf,
        ),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'araba', '-topic', 'robot_description',
                        '-x', spawn_x, '-y', spawn_y,
                        '-z', spawn_z, '-Y', spawn_yaw]),

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

        # Şerit segmentasyonu — /perception/center_pts yayınlar (otonom kontrolün girdisi)
        TimerAction(period=6.5, actions=[
            Node(package='araba', executable='lane_test_node.py', name='lane_test_node',
                 output='screen', parameters=[{'use_sim_time': True,
                                               'camera_topic': '/camera/image',
                                               'model_path': lane_model_path,
                                               'show_window': show_window}]),
        ]),

        # LiDAR engel tespiti — /lidar/scan/points → /perception/lidar_obstacles (JSON)
        # (mimarideki sensor_fusion/lidar_obstacle_node'un sim uyarlaması)
        # centerlines_file = yol maskesi: çizili yolun dışındaki nesneler engel sayılmaz
        TimerAction(period=6.8, actions=[
            Node(package='araba', executable='lidar_obstacle_node.py', name='lidar_obstacle_node',
                 output='screen', parameters=[{'use_sim_time': True,
                                               'centerlines_file': centerlines_file}]),
        ]),

        # ============ LOKALİZASYON (localization:=ekf | final_odom) ============
        # Her iki hat da /localization/odom/final üretir (mission_planning'in girdisi).

        # [ekf] GPS verisini EKF'nin anlayacağı metrik odometriye çeviren düğüm.
        # yaw_offset launch'taki spawn_yaw'dan gelir: Gazebo IMU'su spawn yönünü 0 kabul
        # eder; ENU yaw = IMU yaw + spawn_yaw (ekf.yaml'daki sabit değeri ezer, böylece
        # rotanın ortasından spawn'lı testlerde de pusula doğru başlar).
        TimerAction(period=6.9, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                condition=use_ekf,
                parameters=[ekf_config_path,
                            {'yaw_offset': ParameterValue(spawn_yaw, value_type=float)}],
                remappings=[
                    ('gps/fix', '/gps/fix'),
                    ('imu', '/imu/data'),
                    # EKF'nin ürettiği güncel konumu dinleyip datum'u düzeltir
                    ('odometry/filtered', '/localization/odom/final'),
                    # EKF'ye odom0 olarak gidecek metrik GPS çıktısı
                    ('odometry/gps', '/odometry/gps')
                ]
            ),
        ]),

        # [ekf] Genişletilmiş Kalman Filtresi (GPS pozisyon + KISS-ICP hız + IMU yaw)
        TimerAction(period=7.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                condition=use_ekf,
                parameters=[ekf_config_path],
                remappings=[
                    # EKF'nin çıktısını, mission_planning_node'un beklediği topice yönlendiriyoruz
                    ('odometry/filtered', '/localization/odom/final')
                ]
            ),
        ]),

        # [final_odom] Eski GPS(pozisyon)+IMU(yön)+odom(hız) füzyon node'umuz — yedek
        TimerAction(period=6.9, actions=[
            Node(package='araba', executable='final_odom_node.py', name='final_odom_node',
                 output='screen', condition=use_final_odom,
                 parameters=[{'use_sim_time': True,
                              'spawn_x': ParameterValue(spawn_x, value_type=float),
                              'spawn_y': ParameterValue(spawn_y, value_type=float),
                              'spawn_z': ParameterValue(spawn_z, value_type=float),
                              'spawn_yaw': ParameterValue(spawn_yaw, value_type=float),
                              'datum_lat': DATUM_LAT, 'datum_lon': DATUM_LON}]),
        ]),

        TimerAction(period=7.0, actions=[
            Node(package='araba', executable='perception_pipeline_node', name='perception_pipeline_node',
                 output='screen', parameters=[{'use_sim_time': True, 'show_dashboard': True}]),
        ]),

        # Görev planlama — missions/*.geojson + /localization/odom/final →
        # /planning/steering_ref, /planning/speed_limit, /planning/current_task ...
        # (mimarideki planning/mission_planning node'unun sim uyarlaması;
        #  perception/turn_permissions + decision_debug + park_complete
        #  perception_pipeline_node'dan gelir, planning/park_mode oraya döner)
        TimerAction(period=7.2, actions=[
            Node(package='araba', executable='mission_planning_node.py', name='mission_planning_node',
                 output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'mission_file': mission_file,
                     'centerlines_file': centerlines_file,
                     'require_go_signal': False,          # sim'de UMS-2 Go onayı yok
                     'heading_source': 'final_odom',
                     'heading_is_enu_yaw': True,          # final_odom ENU yaw yayınlar → pusulaya çevir
                     'final_odom_topic': '/localization/odom/final',
                     'gps_fix_topic': '/gps/fix',
                     'imu_topic': '/imu/data',
                     'perception_turn_permissions_topic': '/perception/turn_permissions',
                     'perception_decision_debug_topic': '/perception/decision_debug',
                     'perception_park_complete_topic': '/perception/park_complete',
                     'planning_steering_topic': '/planning/steering_ref',
                     'planning_speed_topic': '/planning/speed_limit',
                     'planning_current_task_topic': '/planning/current_task',
                     'planning_arrived_topic': '/planning/arrived',
                     'planning_park_mode_topic': '/planning/park_mode',
                     'planning_park_remaining_topic': '/planning/park_remaining_s',
                 }]),
        ]),

        # Otonom şerit takibi — /perception/center_pts dinler, komutunu /cmd_vel_lane'e
        # yazar; vehicle_controller şerit/rota arbitrasyonuyla /cmd_vel_raw üretir,
        # hız güvenliği speed_controller_node üzerinden /cmd_vel'e ulaşır.
        # (manuel:=true verilirse başlamaz; onun yerine keyboard_teleop çalışır)
        TimerAction(period=7.5, actions=[
            Node(package='araba', executable='autonomous_control_node.py', name='autonomous_control_node',
                 output='screen', condition=UnlessCondition(manuel),
                 remappings=[('/cmd_vel', '/cmd_vel_lane')],
                 parameters=[{'use_sim_time': True,
                              'cam_width': 1280.0,
                              # throttle 0.5 -> 2.5 m/s (9 km/h seyir), 0.35 -> 1.75 (6.3 km/h viraj)
                              # (arda 10.0 denedi; dönüş/durma ayarlarımız 5.0'a göre kanıtlı)
                              'speed_scale': 5.0}]),
        ]),

        # Araç kontrol arbiter'ı (mimari vehicle_controller'ın sim uyarlaması, 5. adım):
        # şerit tazeyse /cmd_vel_lane geçer, şerit kaybolunca /planning/steering_ref
        # ile yavaş kavşak sürünmesi (işaret çevirisi node içinde) → /cmd_vel_raw.
        TimerAction(period=7.6, actions=[
            Node(package='araba', executable='vehicle_controller_node.py', name='vehicle_controller_node',
                 output='screen', condition=UnlessCondition(manuel),
                 parameters=[{'use_sim_time': True}]),
        ]),

        # Hız güvenlik katmanı — /cmd_vel_raw + algı durumları → /cmd_vel
        # Kırmızı ışıkta durur, levha hız sınırı uygular, engelde acil fren yapar.
        TimerAction(period=7.8, actions=[
            Node(package='araba', executable='speed_controller_node', name='speed_controller_node',
                 output='screen', parameters=[{'use_sim_time': True}]),
        ]),

        # Manuel WASD kontrol — sadece manuel:=true iken. Komut yine /cmd_vel_raw'a
        # gider; böylece manuel modda da ışık/levha/engel güvenliği devrededir.
        TimerAction(period=8.0, actions=[
            Node(package='araba', executable='keyboard_teleop', name='keyboard_teleop',
                 output='screen', condition=IfCondition(manuel),
                 remappings=[('/cmd_vel', '/cmd_vel_raw')]),
        ]),
    ])
