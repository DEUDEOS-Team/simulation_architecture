# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Ortam

- **ROS 2**: Jazzy
- **Gazebo**: Harmonic (`gz-sim8`)
- **Paket**: `araba` (`src/araba/`) — `ament_cmake` tabanlı tek paket

## Temel Komutlar

```bash
# ROS 2 kaynaklarını yükle
source /opt/ros/jazzy/setup.bash

# Workspace'i derle
cd ~/sim_ws
colcon build

# Kurulumu kaynak olarak ekle
source install/setup.bash

# Gazebo simülasyonunu başlat (tam stack: Gazebo + RViz + kamera görüntüsü + bridge)
ros2 launch araba gazebo.launch.py

# Sadece RViz ile URDF görüntüleme (Gazebo olmadan)
ros2 launch araba display.launch.py

# Klavye ile araç kontrolü
ros2 run araba keyboard_teleop

# Trafik ışığı eklentisini derle ve kur (ilk kurulumda veya değişiklik sonrası)
cd src/araba/models/traffic_light
./build_and_install.sh
```

## Mimari Genel Bakış

### Paket Yapısı (`src/araba/`)

- **`urdf/araba.urdf`** — SolidWorks'ten ihraç edilen araç modeli. Ackermann direksiyon sistemi: ön tekerlekler ayrı direksiyon (`_direksiyon_link`) ve dönüş (`_donus_link`) eklemlere sahip. Arka tekerlekler sadece dönüş eklemli. Kütlesi 148 kg.
- **`worlds/benim_dunyam.sdf`** — Teknofest temalı simülasyon dünyası. Yürüyen yaya aktörü, trafik ışığı modeli ve Türkçe trafik levhaları içerir.
- **`models/`** — Gazebo model tanımları: `traffic_light` (C++ eklentili), `traffic_cone`, `red_barrier`, `white_barrier`, `walking_actor`, `tunel`, `doner_kavsak`, `durak` ve çeşitli Türkçe trafik levhaları.
- **`launch/gazebo.launch.py`** — Ana başlatma dosyası; Gazebo, `robot_state_publisher`, `ros_gz_bridge`, RViz ve `rqt_image_view`'ı birlikte başlatır.
- **`scripts/keyboard_teleop.py`** — `pynput` kullanan klavye kontrol düğümü; W/A/S/D ile `/cmd_vel` yayınlar.

### ROS-Gazebo Köprüsü (`ros_gz_bridge`)

`gazebo.launch.py` içindeki köprü şu konuları çevirir:

| Konu | Mesaj Türü | Yön |
|---|---|---|
| `/lidar/scan` | `sensor_msgs/LaserScan` | Gz→ROS |
| `/lidar/scan/points` | `sensor_msgs/PointCloud2` | Gz→ROS |
| `/camera/image` | `sensor_msgs/Image` | Gz→ROS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gz→ROS |
| `/imu/data` | `sensor_msgs/Imu` | Gz→ROS |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gz→ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→Gz |
| `/odom` | `nav_msgs/Odometry` | Gz→ROS |

### Trafik Işığı Eklentisi (`models/traffic_light/TrafficLightPlugin.cc`)

Gerçekçi 4 fazlı döngü uygular: **GREEN (15s) → YELLOW (3s) → RED (10s) → RED_YELLOW (3s)**. Her faz değişiminde `/traffic_light/state` konusuna `gz.msgs.StringMsg` yayınlar (`"GREEN"`, `"YELLOW"`, `"RED"`, `"RED_YELLOW"`). Eklenti `~/.gz/sim/plugins/libTrafficLightPlugin.so` konumuna kurulur.

### Kaynak Yolları

`GZ_SIM_RESOURCE_PATH` iki dizini kapsar (otomatik olarak `gazebo.launch.py` tarafından ayarlanır):
1. `install/share` üst dizini — `model://araba/...` referansları için
2. `otonom_projem/models/` — `model://traffic_light/...` gibi paylaşılan model referansları için

### `otonom_projem/`

Paket dışında tutulan alternatif model ve dünya dosyaları. `otonom_projem/teknofest_haritası/` içinde Teknofest haritası ham dosyaları (SDF, PNG, STL) bulunur. Bu dizindeki modeller `src/araba/models/` ile aynı içeriğe sahiptir; `gazebo.launch.py` kaynak yolu sayesinde her ikisine de erişir.
