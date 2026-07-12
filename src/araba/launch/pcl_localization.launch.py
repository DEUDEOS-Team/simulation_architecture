import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Kendi araba paketimizin içindeki maps klasörünün yolunu dinamik olarak buluyoruz
    pkg_share = get_package_share_directory('araba')
    map_file_path = os.path.join(pkg_share, 'maps', 'saha_haritasi.pcd')

    return LaunchDescription([
        Node(
            package='pcl_localization_ros2',
            executable='pcl_localization_node',
            name='pcl_localization_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,                  # Simülasyon saati kullanımı ZORUNLU
                'map_file': map_file_path,             # Dinamik olarak bulduğumuz pcd harita yolu
                'map_frame_id': 'map',                 # Global harita ekseni
                'odom_frame_id': 'odom',               # EKF'nin başlangıç ekseni
                'base_frame_id': 'base_link',          # Aracın şasi ekseni
                
                # Voxel filtreden ve KISS-ICP'den geçen aynı temizlenmiş veriyi dinliyoruz
                'scan_topic': '/deos/sensors/lidar/points_downsampled', 
                
                'use_pcd_map': True,
                'resolution': 1.0,                     # NDT eşleştirme çözünürlüğü (metre)
                
                # Aracın Gazebo'daki başlangıç (spawn) koordinatları.
                # Eğer araç sıfır noktasında başlamıyorsa bunları güncellemelisin.
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_yaw': 0.0
            }]
        )
    ])