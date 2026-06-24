# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Ortam

- **ROS 2**: Jazzy
- **Gazebo**: Harmonic (`gz-sim8`)
- **Paket**: `araba` (`src/araba/`) — `ament_cmake` tabanlı tek ROS paketi
- **GUI**: WSLg üzerinden çalışır (Gazebo penceresi + RViz için ekstra kurulum gerekmez)

> **Not:** `~/.bashrc` zaten hem `/opt/ros/jazzy/setup.bash`'i hem de (derlenmişse)
> `~/sim2_ws/install/setup.bash`'i otomatik yükler. Yeni terminalde elle `source`
> yapmaya gerek yoktur; aşağıdaki `source` satırları yalnızca referans içindir.

## Temel Komutlar

```bash
# (Gerekirse) ROS 2 + workspace kaynaklarını yükle — .bashrc bunu zaten yapar
source /opt/ros/jazzy/setup.bash
source ~/sim2_ws/install/setup.bash

# Workspace'i derle (symlink-install: Python/launch değişiklikleri yeniden derleme gerektirmez)
cd ~/sim2_ws
colcon build --symlink-install

# Gazebo simülasyonunu başlat (tam stack: Gazebo + RViz + algı node'ları + bridge)
ros2 launch araba gazebo.launch.py

# Sadece RViz ile URDF görüntüleme (Gazebo olmadan)
ros2 launch araba display.launch.py

# Klavye ile araç kontrolü (gazebo.launch.py içinde de otomatik başlar)
ros2 run araba keyboard_teleop

# Trafik ışığı eklentisini derle ve kur (ilk kurulumda veya .cc değişikliği sonrası)
cd src/araba/models/traffic_light
./build_and_install.sh
```

### Bağımlılıklar (yeni bir makineye kurulum)

```bash
sudo apt install python3-colcon-common-extensions ros-jazzy-joint-state-publisher-gui python3-pynput
```

Python algı/kontrol node'ları ayrıca şunları kullanır: `cv2` (opencv), `cv_bridge`,
`numpy`, `onnxruntime`, `PIL` (pillow), `tkinter`.

## Mimari Genel Bakış

### Paket Yapısı (`src/araba/`)

- **`urdf/araba.urdf`** — SolidWorks'ten ihraç edilen araç modeli. Ackermann direksiyon sistemi: ön tekerlekler ayrı direksiyon (`_direksiyon_link`) ve dönüş (`_donus_link`) eklemlere sahip. Arka tekerlekler sadece dönüş eklemli. Kütlesi 148 kg.
- **`worlds/benim_dunyam.sdf`** — Teknofest temalı simülasyon dünyası. Yürüyen yaya aktörü, trafik ışığı modeli, tünel, döner kavşak, durak ve çeşitli Türkçe trafik levhaları içerir. Model referansları `file://` mutlak yolu kullanır (`/home/<user>/sim2_ws/otonom_projem/...`).
- **`models/`** — Gazebo model tanımları: `traffic_light` (C++ eklentili), `traffic_cone`, `red_barrier`, `white_barrier`, `walking_actor`, `tunel`, `doner_kavsak`, `durak` ve çeşitli Türkçe trafik levhaları. (Bu dizin `colcon` ile install'e **kopyalanmaz**; modeller `otonom_projem/models/` üzerinden çözülür — bkz. Kaynak Yolları.)
- **`launch/gazebo.launch.py`** — Ana başlatma dosyası (aşağıdaki node listesine bakın).
- **`scripts/`** — Tüm Python node'ları ve algoritma modülleri (aşağıya bakın).

### `gazebo.launch.py` Başlattığı Düğümler

Zamanlanmış (`TimerAction`) sırayla başlar:

1. **Gazebo** (`ros_gz_sim`/`gz_sim.launch.py`) — `benim_dunyam.sdf` dünyasıyla
2. **`robot_state_publisher`** — `xacro` ile URDF'ten `robot_description`
3. **`ros_gz_sim/create`** — aracı dünyaya spawn eder (başlangıç konumu sabit)
4. **`rviz2`** — `config/lidar_view.rviz` ile
5. (5s) **`ros_gz_bridge/parameter_bridge`** — konu köprüsü (aşağıdaki tablo)
6. (6s) **`camera_perception_node`** — nesne tespiti, `models/onnx/detection.onnx`
7. (6.5s) **`lane_perception_node`** — şerit segmentasyonu, `models/onnx/model.onnx`
8. (7s) **`perception_pipeline_node`** — algı füzyonu + dashboard
9. (8s) **`keyboard_teleop`** — WASD ile manuel `/cmd_vel`

> `speed_controller_node` launch içinde yorum satırıdır (algoritma kontrolü aktif edilince açılacak — `TODO`).

### Python Node ve Modülleri (`scripts/`)

- **ROS node'ları** (`ros2 run araba <ad>`): `camera_perception_node`, `lane_perception_node`, `perception_pipeline_node`, `speed_controller_node`, `keyboard_teleop`
- **Algoritma modülleri** (import edilebilir saf Python): `safety_logic`, `obstacle_logic`, `traffic_light_logic`, `traffic_sign_logic`, `parking_logic`, `slalom_logic`, `decision_arbiter`, `perception_fusion`, `lane_violation`, `geojson_mission_reader`, `route_graph`, `route_planner`, `waypoint_manager`, `mission_manager`
- **`scripts/onnx/`** — ONNX/PyTorch model dosyaları (`detection.onnx`, `model.onnx`, `smallmodel.pt`); install'e `models/onnx/` altına symlink edilir
- **`scripts/deos_algorithms/`, `scripts/sensors/`** — ek algoritma ve sensör yardımcıları
- Geliştirme notları: `scripts/INTEGRATION_ROADMAP.md`, `scripts/README_ALGORITHMS.md`

### ROS-Gazebo Köprüsü (`ros_gz_bridge`)

| Konu | Mesaj Türü | Yön |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gz→ROS |
| `/lidar/scan` | `sensor_msgs/LaserScan` | Gz→ROS |
| `/lidar/scan/points` | `sensor_msgs/PointCloud2` | Gz→ROS |
| `/camera/image` | `sensor_msgs/Image` | Gz→ROS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gz→ROS |
| `/imu/data` | `sensor_msgs/Imu` | Gz→ROS |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gz→ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→Gz |
| `/odom` | `nav_msgs/Odometry` | Gz→ROS |
| `/model/araba/joint_state` → `/joint_states` | `sensor_msgs/JointState` | Gz→ROS |

### Trafik Işığı Eklentisi (`models/traffic_light/TrafficLightPlugin.cc`)

Gerçekçi 4 fazlı döngü uygular: **GREEN (15s) → YELLOW (3s) → RED (10s) → RED_YELLOW (3s)**. Her faz değişiminde `/traffic_light/state` konusuna `gz.msgs.StringMsg` yayınlar (`"GREEN"`, `"YELLOW"`, `"RED"`, `"RED_YELLOW"`). `build_and_install.sh` eklentiyi derleyip `~/.gz/sim/plugins/libTrafficLightPlugin.so` konumuna kurar. (`.bashrc` içindeki `GZ_SIM_SYSTEM_PLUGIN_PATH` bu dizini kapsar.)

### Kaynak Yolları

`gazebo.launch.py`, `GZ_SIM_RESOURCE_PATH`'i çalışma anında **kullanıcı adından bağımsız** olarak hesaplar (`pkg_share`'den workspace kökü türetilir) ve iki dizini kapsar:
1. `install/araba/share` üst dizini — `model://araba/...` referansları için
2. `<workspace>/otonom_projem/models/` — `model://traffic_light/...` gibi paylaşılan model referansları için

> Elle `gz sim` çalıştırırken `.bashrc` içindeki `GZ_SIM_RESOURCE_PATH` (`~/sim2_ws/otonom_projem/models`) kullanılır.

### `otonom_projem/` ve `yedek_arabalar/`

- **`otonom_projem/`** — Paket dışında tutulan paylaşılan model ve dünya dosyaları. `otonom_projem/teknofest_haritası/` içinde Teknofest haritası ham dosyaları (SDF, PNG, STL) bulunur. `benim_dunyam.sdf` bu dizindeki modelleri `file://` yoluyla referans eder.
- **`yedek_arabalar/`** — Eski araç paket sürümleri (`araba_v2` ament, `araba_v3` catkin/ROS1).

> Bu iki dizinde **`COLCON_IGNORE`** dosyası vardır; böylece `colcon build` yalnızca
> asıl `araba` paketini derler. (Aksi halde colcon eski/yan paketleri de derlemeye
> çalışır ve `araba_v3` catkin paketi hata verir.)
