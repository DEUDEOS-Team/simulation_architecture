# 🚀 ROS 2 & Gazebo Harmonic Entegrasyon Rehberi — araba_v2

> **Oluşturulma:** 2026-05-31
> **Kaynak:** `sw2urdf` (SolidWorks to URDF Exporter v1.6) çıktısı
> **Hedef:** ROS 2 (Humble/Iron/Jazzy) + Gazebo Harmonic (Ignition Gazebo 8+)

---

## 📋 Mevcut Durum Analizi

### Robot Yapısı

| Link | Eklemlenme | Tip | Eksen | Görev |
|------|-----------|-----|-------|-------|
| `base_link` | — | — | — | Ana şasi (100.8 kg) — motor, diferansiyel, aks, süspansiyon, koltuk vb. |
| `on_sag_yatay_direksiyon_hareket_link` | `base_link` → | `revolute` | Z | Sağ ön direksiyon mafsalı |
| `on_sag_ileri_donus_hareket` | `on_sag_yatay_direksiyon_hareket_link` → | `continuous` | X | Sağ ön teker dönüşü |
| `on_sol_yatay_direksiyon_hareket_link` | `base_link` → | `revolute` | Z | Sol ön direksiyon mafsalı |
| `on_sol_ileri_donus_hareket` | `on_sol_yatay_direksiyon_hareket_link` → | `continuous` | X | Sol ön teker dönüşü |
| `arka_sol_ileri_donus_hareket_link` | `base_link` → | `continuous` | X | Sol arka teker dönüşü |
| `arka_sag_ileri_donus_hareket_link` | `base_link` → | `continuous` | X | Sağ arka teker dönüşü |
| `imu_sensor_link` | `base_link` → | `fixed` | — | IMU sensörü |
| `gps_sensor_link` | `base_link` → | `fixed` | — | GPS sensörü |
| `lidar_sensor_link` | `base_link` → | `fixed` | — | LiDAR sensörü |
| `cam_sensor_link` | `base_link` → | `fixed` | — | Kamera |

### ❗ Tespit Edilen Sorunlar

1. **Direksiyon joint limitleri sıfır:** `on_sag_yatay_direksiyon_hareket` ve `on_sol_yatay_direksiyon_hareket` jointlerinde `lower="0" upper="0"` — direksiyon hiç dönemez! ±30° (≈0.52 rad) civarı bir değer atanmalı.
2. **CSV dosyasında Türkçe ondalık ayracı:** `,` (virgül) kullanılmış, URDF'te `.` (nokta) kullanılmış — URDF doğru, CSV sadece referans.
3. **Meshlar STL formatında:** Gazebo Harmonic STL destekler, ancak collision mesh'leri için optimize edilmesi gerekebilir.
4. **ROS 1 launch dosyaları:** Tamamen ROS 2 Python launch formatına çevrilmeli.

---

## 🔧 Adım Adım Migrasyon

### Adım 1: ROS 2 Çalışma Alanını Oluşturma

```bash
# ROS 2 çalışma alanı oluştur (örnek: ~/ros2_ws)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Mevcut araba_v2 klasörünü buraya kopyala
cp -r /path/to/araba_v2 ~/ros2_ws/src/

# Alternatif: sembolik link
# ln -s /path/to/araba_v2 ~/ros2_ws/src/araba_v2
```

### Adım 2: `package.xml` Güncelleme (ROS 2 formatı)

Mevcut `package.xml` (`format="2"`, `catkin` tabanlı) aşağıdaki gibi değiştirilmeli:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>araba_v2</name>
  <version>1.0.0</version>
  <description>
    <p>URDF Description package for araba_v2 — 4-wheeled ground vehicle</p>
    <p>Migrated from ROS 1 sw2urdf export. Contains meshes, URDF, and launch files
    for ROS 2 + Gazebo Harmonic simulation.</p>
  </description>

  <maintainer email="TODO@email.com">TODO</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- ROS 2 core dependencies -->
  <depend>rclcpp</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>rviz2</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>

  <!-- Gazebo Harmonic (Ignition) -->
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <depend>gz_ros2_control</depend>

  <!-- Controller dependencies -->
  <depend>controller_manager</depend>
  <depend>ros2_control</depend>
  <depend>ros2_controllers</depend>
  <depend>joint_state_broadcaster</depend>
  <depend>velocity_controllers</depend>
  <depend>position_controllers</depend>
  <depend>effort_controllers</depend>

  <!-- TF -->
  <depend>tf2_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <architecture_independent/>
  </export>
</package>
```

### Adım 3: `CMakeLists.txt` Güncelleme (ament_cmake)

```cmake
cmake_minimum_required(VERSION 3.8)
project(araba_v2)

# Ament CMake
find_package(ament_cmake REQUIRED)

# URDF ve mesh dosyalarını kur
install(DIRECTORY config launch meshes textures urdf
  DESTINATION share/${PROJECT_NAME}
)

# RVIz konfigürasyonu varsa
# install(DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### Adım 4: URDF Düzeltmeleri

#### 4a. Direksiyon Joint Limitlerini Açma

`urdf/araba_v2.urdf` dosyasında iki direksiyon joint'inin limitlerini düzeltin:

**`on_sag_yatay_direksiyon_hareket` jointi (satır 99-103):**
```xml
<limit
  lower="-0.5236"   <!-- -30° radyan -->
  upper="0.5236"    <!-- +30° radyan -->
  effort="10"
  velocity="5" />
```

**`on_sol_yatay_direksiyon_hareket` jointi (satır 210-214):**
```xml
<limit
  lower="-0.5236"   <!-- -30° radyan -->
  upper="0.5236"    <!-- +30° radyan -->
  effort="10"
  velocity="5" />
```

#### 4b. Tekerlek Jointlerine Sürtünme/Damping Ekleme

Tüm `continuous` tip jointlere sürtünme parametresi eklenerek gerçekçilik artırılabilir:
```xml
<joint name="on_sag_ileri_donus_hareket" type="continuous">
  ...
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

#### 4c. Gazebo Eklentileri için `<gazebo>` Etiketleri Ekleme

URDF içine Gazebo Harmonic için `<gazebo>` etiketleri eklenmeli. **Not:** ROS 2 / Gazebo Harmonic'te bu etiketler `ros_gz_sim` tarafından işlenir.

```xml
<!-- URDF sonuna, </robot> öncesine ekleyin -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <pose>0.675 -0.95107 0.0265 1.5708 0 1.5708</pose>
  </sensor>
</gazebo>

<gazebo reference="lidar_sensor_link">
  <sensor name="lidar_sensor" type="gpu_lidar">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
  </sensor>
</gazebo>

<gazebo>
  <plugin name="araba_v2_joint_state" filename="ignition-gazebo-joint-state-publisher-system">
    <joint_name>on_sag_yatay_direksiyon_hareket</joint_name>
    <joint_name>on_sag_ileri_donus_hareket</joint_name>
    <joint_name>on_sol_yatay_direksiyon_hareket</joint_name>
    <joint_name>on_sol_ileri_donus_hareket</joint_name>
    <joint_name>arka_sol_ileri_donus_hareket</joint_name>
    <joint_name>arka_sag_ileri_donus_hareket</joint_name>
  </plugin>
</gazebo>
```

### Adım 5: ROS 2 Launch Dosyalarını Oluşturma

ROS 2'de launch dosyaları Python API ile yazılır. `launch/` klasöründeki eski XML dosyalarını silip yerine Python dosyaları oluşturacağız.

#### `launch/display.launch.py` — RVIz2 ile Görselleştirme

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_dir = get_package_share_directory('araba_v2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'araba_v2.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'araba_v2.rviz')]
        ),
    ])
```

#### `launch/gazebo.launch.py` — Gazebo Harmonic Simülasyonu

```python
#!/usr/bin/env python3
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

    # Gazebo Harmonic (Ignition) simülasyonunu başlat
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

    # Robotu Gazebo'ya spawn et (URDF → SDF dönüşümü otomatik)
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

    # ros_gz_bridge — ROS 2 ↔ Gazebo mesaj köprüsü
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
            # Tekerlek komutları (hız kontrolü)
            '/model/araba_v2/joint/on_sag_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/on_sol_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/arka_sol_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/arka_sag_ileri_donus_hareket/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            # Direksiyon komutları (pozisyon kontrolü)
            '/model/araba_v2/joint/on_sag_yatay_direksiyon_hareket/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/araba_v2/joint/on_sol_yatay_direksiyon_hareket/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Sensörler (IMU)
            '/world/empty/model/araba_v2/link/imu_sensor_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # LiDAR
            '/world/empty/model/araba_v2/link/lidar_sensor_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # TF
            '/model/araba_v2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/empty/model/araba_v2/joint_state', '/joint_states'),
        ]
    )

    # Statik TF: base_footprint → base_link
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
        # Robotu spawn etmek için Gazebo'nun tamamen başlamasını bekle
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
        robot_state_pub,
        static_tf,
        TimerAction(
            period=4.0,
            actions=[bridge]
        ),
    ])
```

### Adım 6: ros2_control Entegrasyonu (Gelişmiş Kontrol)

Daha gerçekçi bir simülasyon için `ros2_control` + `gz_ros2_control` kullanılmalı. Bunun için ayrı bir URDF dosyası oluşturun:

#### `urdf/araba_v2.gazebo.xacro` — ros2_control ile

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="araba_v2">
  <!-- Orijinal URDF'i dahil et -->
  <xacro:include filename="araba_v2.urdf"/>

  <!-- Gazebo ros2_control eklentisi -->
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find araba_v2)/config/gz_ros2_control.yaml</parameters>
      <ros>
        <namespace>/araba_v2</namespace>
        <remapping>/araba_v2/cmd_vel:=/cmd_vel</remapping>
      </ros>
    </plugin>
  </gazebo>
</robot>
```

#### `config/gz_ros2_control.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    # Kontrolcü listesi
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    steering_controller:
      type: position_controllers/JointGroupPositionController

# Tekerlek hız kontrolcüsü
velocity_controller:
  ros__parameters:
    joints:
      - on_sag_ileri_donus_hareket
      - on_sol_ileri_donus_hareket
      - arka_sol_ileri_donus_hareket
      - arka_sag_ileri_donus_hareket
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity

# Direksiyon pozisyon kontrolcüsü
steering_controller:
  ros__parameters:
    joints:
      - on_sag_yatay_direksiyon_hareket
      - on_sol_yatay_direksiyon_hareket
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Adım 7: Build ve Test

```bash
# ROS 2 çalışma alanına git
cd ~/ros2_ws

# Gerekli bağımlılıkları kur (Ubuntu)
sudo apt install ros-${ROS_DISTRO}-ros-gz-sim \
                 ros-${ROS_DISTRO}-ros-gz-bridge \
                 ros-${ROS_DISTRO}-gz-ros2-control \
                 ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2

# Build
colcon build --packages-select araba_v2 --symlink-install

# Ortamı kaynakla
source install/setup.bash

# Önce URDF'i RVIz2'de görüntüle (simülasyon olmadan)
ros2 launch araba_v2 display.launch.py

# Gazebo Harmonic simülasyonunu başlat
ros2 launch araba_v2 gazebo.launch.py
```

---

## 📊 Özet: ROS 1 → ROS 2 / Gazebo Harmonic Dönüşüm Tablosu

| Bileşen | ROS 1 (Mevcut) | ROS 2 + Gazebo Harmonic (Hedef) |
|---------|---------------|----------------------------------|
| **Build sistemi** | `catkin` | `ament_cmake` |
| **package.xml** | `format="2"` | `format="3"` |
| **Launch** | XML `.launch` | Python `.launch.py` |
| **Robot tanımı** | URDF (`.urdf`) | URDF veya Xacro (`.urdf` / `.xacro`) |
| **Simülatör** | Gazebo Classic 11 | Gazebo Harmonic (Ignition 8+) |
| **Sim paketi** | `gazebo_ros` | `ros_gz_sim` |
| **Spawn** | `spawn_model` | `ros_gz_sim create` |
| **Mesaj köprüsü** | Otomatik (`gazebo_ros`) | `ros_gz_bridge` (manuel) |
| **Kontrol** | `ros_control` | `ros2_control` + `gz_ros2_control` |
| **RVIz** | `rviz` | `rviz2` |
| **TF** | `tf` (static_transform_publisher) | `tf2_ros` |
| **Dünya dosyası** | `.world` (SDF 1.6) | `.sdf` (SDF 1.10+) |

---

## ⚠️ Dikkat Edilmesi Gerekenler

1. **Mesh dosyaları:** STL formatı her iki ortamda da çalışır. Ancak collision hesaplamaları için karmaşık STL mesh'ler yerine basitleştirilmiş geometriler (kutu, silindir) kullanmanız performansı artırır. Özellikle `base_link` çok fazla SolidWorks parçasından oluştuğu için collision mesh'i ağır olabilir.

2. **Direksiyon simetrisi:** `on_sag_yatay_direksiyon_hareket` ve `on_sol_yatay_direksiyon_hareket` aynı komutla ters yönlerde hareket etmeli (Ackermann geometrisi). Bunu ROS 2 tarafında bir düğümle sağlamanız gerekir.

3. **Gazebo Harmonic'te URDF vs SDF:** Gazebo Harmonic natively SDF kullanır. `ros_gz_sim create` URDF'i otomatik olarak SDF'e dönüştürür, ancak bazı gelişmiş özellikler (örn. kapalı kinematik zincirler) kaybolabilir. Karmaşık simülasyonlar için doğrudan SDF yazmayı düşünebilirsiniz.

4. **`package://` çözümlemesi:** Hem ROS 1 hem ROS 2'de `package://araba_v2/meshes/...` yolu `ament_index_python` / `rospack` tarafından çözümlenir, değişiklik gerekmez.

5. **Sensörler:** URDF'te sensörler sadece görsel geometri olarak tanımlanmış. Gazebo'da çalışması için `<gazebo>` etiketleri ile gerçek sensör eklentileri eklenmelidir (IMU, LiDAR, Kamera).

6. **`export.log` dosyası:** 2 MB'lık bu dosya sw2urdf export günlüğüdür. Paketle birlikte kurulması gerekmez. `.gitignore`'a ekleyin.

---

## 📁 Hedeflenen Nihai Klasör Yapısı

```
araba_v2/
├── CMakeLists.txt                    # ament_cmake formatında
├── package.xml                       # format="3"
├── config/
│   ├── joint_names_araba_v2.yaml     # Joint listesi (güncellenecek)
│   └── gz_ros2_control.yaml          # YENİ: ros2_control konfigürasyonu
├── launch/
│   ├── display.launch.py             # YENİ: RVIz2 görselleştirme
│   └── gazebo.launch.py              # YENİ: Gazebo Harmonic simülasyon
├── meshes/
│   ├── base_link.STL
│   ├── on_sag_*.STL
│   ├── on_sol_*.STL
│   ├── arka_*.STL
│   ├── imu_sensor_link.STL
│   ├── gps_sensor_link.STL
│   ├── lidar_sensor_link.STL
│   └── cam_sensor_link.STL
├── rviz/                             # YENİ: RVIz2 konfigürasyonu
│   └── araba_v2.rviz
├── urdf/
│   ├── araba_v2.csv                  # Referans (değişmeyecek)
│   ├── araba_v2.urdf                 # Düzeltilmiş joint limitleri
│   └── araba_v2.gazebo.xacro         # YENİ: ros2_control + Gazebo eklentili
└── textures/                         # Boş, gerekirse doku dosyaları
```

---

## 🧪 Hızlı Test Komutları

```bash
# URDF syntax kontrolü
check_urdf <(xacro urdf/araba_v2.urdf)

# ROS 2 topic listesi (simülasyon çalışırken)
ros2 topic list

# Joint state'leri izleme
ros2 topic echo /joint_states

# Tekerleklere komut gönderme (test)
ros2 topic pub /model/araba_v2/joint/arka_sol_ileri_donus_hareket/cmd_vel std_msgs/msg/Float64 "data: 10.0"

# Direksiyona komut gönderme (test)
ros2 topic pub /model/araba_v2/joint/on_sag_yatay_direksiyon_hareket/cmd_pos std_msgs/msg/Float64 "data: 0.3"
```

---

## 🔗 Faydalı Kaynaklar

- [ROS 2 Migration Guide](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1.html)
- [Gazebo Harmonic (Ignition) + ROS 2](https://gazebosim.org/docs/harmonic/ros2_integration)
- [ros_gz_sim Documentation](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim)
- [gz_ros2_control Documentation](https://github.com/ros-controls/gz_ros2_control)
- [URDF → SDF Conversion](https://gazebosim.org/api/sim/8/migrationurdf.html)

---

*Rehber sonu. Her adımda karşılaştığınız hataları bana iletirseniz çözüm üretebilirim.*
