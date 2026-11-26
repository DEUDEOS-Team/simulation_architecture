# DEOS Otonom Araç Simülasyon Mimarisi — Kapsamlı Kurulum ve Dosya Rehberi

Bu rehber, çalışma dizinindeki tüm kaynak kodlar, launch dosyaları, yapılandırmalar, URDF/SDF modelleri, haritalar, yardımcı araçlar ve testler sıfırdan incelenerek hazırlanmıştır. Projenin **nasıl kurulacağını**, **nasıl çalıştırılacağını** ve **her klasör/dosyanın tam olarak ne işe yaradığını** açıklar.

---

## 1. Proje Özeti ve Sistem Mimarisi

Bu depo, **ROS 2 (Jazzy)** ve **Gazebo Harmonic (gz-sim 8)** üzerinde çalışan, TEKNOFEST Robotaksi / otonom sürüş parkuru için geliştirilmiş uçtan uca bir otonom araç simülasyon paketidir (`araba`).

### Uçtan Uca Veri Akışı (`gazebo.launch.py`)

```text
[Gazebo Harmonic (benim_dunyam.sdf / saha_dunyasi.sdf)]
   │  AckermannSteering + SICK LiDAR + RealSense D415 + IMU + GPS
   ▼
[ros_gz_bridge (parameter_bridge)]
   ├── /camera/image, /camera/camera_info ──► camera_perception_node (detection.onnx, 29 sınıf)
   │                                           └──► /perception/detections
   │                                        ──► lane_test_node.py (model.onnx, YOLO-seg)
   │                                           └──► /perception/center_pts, /perception/lane_mask
   ├── /lidar/scan/points ──────────────────► lidar_obstacle_node.py (yol + şerit maskeli kümeleme)
   │                                           ├──► /perception/lidar_obstacles (JSON)
   │                                           └──► /perception/pole_candidates (ışık direkleri)
   ├── /gps/fix + /imu/data + /odom ────────► final_odom_node.py (varsayılan, 50 Hz ENU)
   │                                           [veya localization:=ekf → lidarfilters + navsat + EKF]
   │                                           └──► /localization/odom/final
   ▼
[Üst Düzey Algı & Planlama]
   ├── perception_pipeline_node  ◄── detections + lidar_obstacles + pole_candidates + imu + odom
   │     (TrafficLightLogic, TrafficSignLogic, ObstacleLogic, ParkingLogic, DecisionArbiter)
   │     └──► /perception/traffic_light_state, traffic_sign_state, obstacle_state,
   │          turn_permissions, decision_debug, park_complete
   ├── avoidance_node.py         ◄── lidar_obstacles + center_pts + final_odom (gap_avoid_logic)
   │     └──► /planning/lateral_offset (+ = SOL) & kaçınma direksiyon düzeltmesi
   └── mission_planning_node.py  ◄── missions/*.geojson + final_odom + turn_permissions
         └──► /planning/steering_ref, speed_limit, turn_active, maneuver_active, park_mode
   ▼
[Kontrol Zinciri]
   autonomous_control_node.py (/perception/center_pts + lateral_offset → /cmd_vel_lane)
         │
         ▼
   vehicle_controller_node.py (Öncelik: OVERRIDE > MANEUVER > TURN > LANE > PLAN > STOP)
         │  [veya manuel:=true → keyboard_teleop]
         ▼ (/cmd_vel_raw)
   speed_controller_node (DecisionArbiter + eğrilik hız tavanı + acil fren → /cmd_vel)
         │
         ▼
   [Gazebo AckermannSteering Plugin]
```

---

## 2. Sıfırdan Tam Kurulum Rehberi

### 2.1. İşletim Sistemi Hazırlığı (Windows için WSL2 / Doğrudan Ubuntu 24.04)

Bu proje **Ubuntu 24.04 (Noble)** ve **ROS 2 Jazzy** üzerinde çalışır.

- **Bilgisayarında doğrudan Ubuntu 24.04 kurulu olanlar:** Bu bölümdeki WSL adımlarını **atlayıp** doğrudan **[Bölüm 2.2](#22-sistem-ve-ros-2-paketleri-ubuntu-2404--ros-2-jazzy)** ile devam edebilirler. Başlamadan önce terminalde `nvidia-smi` yazarak NVIDIA ekran kartı sürücüsünün aktif olduğunu doğrulamanız yeterlidir.
- **Windows 10 / 11 kullananlar:** Aşağıdaki adımlarla **WSL2 + Ubuntu 24.04** kurulumunu yapın:

1. **WSL2 ve Ubuntu 24.04 kurulumu** (PowerShell'i **Yönetici olarak** açın):
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
   Kurulum tamamlandıktan sonra bilgisayarı yeniden başlatın. Açılan Ubuntu terminalinde yeni bir Linux kullanıcı adı ve şifre oluşturun.

2. **WSL sürümünü doğrulama** (PowerShell'de):
   ```powershell
   wsl -l -v
   ```
   `Ubuntu-24.04` satırında `VERSION` değerinin **`2`** olduğundan emin olun. (Gerekirse `wsl --set-version Ubuntu-24.04 2` ile yükseltin.)

3. **GPU ve GUI (WSLg) doğrulaması:**
   - Windows tarafında güncel **NVIDIA Game Ready / Studio** sürücüsünün kurulu olması yeterlidir.
   - **Önemli:** WSL2 içindeki Ubuntu'ya ayrıca Linux NVIDIA sürücüsü **kurmayın** (Windows sürücüsü GPU'yu WSL2'ye otomatik aktarır).
   - Ubuntu terminalinde `nvidia-smi` komutunu çalıştırın; ekran kartınızı görüyorsanız CUDA/GPU erişimi hazırdır. Windows 11 (ve güncel Windows 10) ile gelen WSLg sayesinde Gazebo, RViz2 ve OpenCV pencereleri ek X-server kurmadan açılır.

4. **Proje konumu uyarısı:**
   - Depoyu `/mnt/c/...` (Windows diski) yerine mutlaka Linux dosya sistemi altına (örneğin `~/simulation_architecture`) klonlayın; `/mnt/c` altında `colcon build` ve simülasyon performansı çok düşer:
     ```bash
     cd ~
     git clone https://github.com/DEUDEOS-Team/simulation_architecture.git
     cd simulation_architecture
     ```

### 2.2. Sistem ve ROS 2 Paketleri (Ubuntu 24.04 + ROS 2 Jazzy)

Kod tabanındaki `package.xml`, launch dosyaları, CMake ve yamalara göre gereken sistem/ROS 2 paketleri:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-robot-localization \
  python3-colcon-common-extensions \
  python3-tk \
  libgz-sim8-dev \
  cmake build-essential
```

> **Not:** `python3-tk`, `perception_pipeline_node` içindeki opsiyonel Tkinter dashboard için gereklidir. `libgz-sim8-dev`, Gazebo trafik ışığı C++ eklentisini derlemek için gerekir.

### 2.3. Python ve CUDA / ONNX Runtime GPU Bağımlılıkları (Zorunlu)

Şerit segmentasyonu (`lane_test_node.py`) ve algı hattının akıcı çalışması için **NVIDIA GPU + CUDA 12 + `onnxruntime-gpu` kurulumu zorunludur**.

```bash
# 1) Temel Python paketleri
pip install --break-system-packages \
  numpy \
  opencv-python \
  pillow \
  pynput

# 2) ONNX Runtime GPU (CPU sürümü varsa çakışmaması için önce kaldırın)
pip uninstall -y --break-system-packages onnxruntime
pip install --break-system-packages \
  onnxruntime-gpu \
  nvidia-cuda-runtime-cu12 \
  nvidia-cublas-cu12 \
  nvidia-cudnn-cu12 \
  nvidia-cufft-cu12 \
  nvidia-curand-cu12 \
  nvidia-cusolver-cu12 \
  nvidia-cusparse-cu12 \
  nvidia-nvjitlink-cu12
```

> **Not:** `gazebo.launch.py`, `lane_test.launch.py` ve `lane_yolo.launch.py` dosyaları `~/.local/lib/python3.X/site-packages/nvidia/*/lib` dizinlerini otomatik olarak `LD_LIBRARY_PATH`'e ekler; bu sayede `pip` ile kurulan `nvidia-*-cu12` CUDA/cuDNN kütüphaneleri ek ayar gerektirmeden `onnxruntime-gpu` tarafından bulunur. Kurulum sonrası `CUDAExecutionProvider` doğrulaması için:
> ```bash
> python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
> ```

- **YOLO / Model dönüştürme (isteğe bağlı):** Yalnızca `lane_yolo_node.py` (`lane_yolo.launch.py`) ve `scripts/onnx/convert_to_onnx.py` için gerekir (CUDA 12 uyumlu PyTorch ile kullanılmalıdır; `cu13` torch, `onnxruntime-gpu`'nun `cu12` cuDNN sürümüyle çakışır):
  ```bash
  pip install --break-system-packages ultralytics
  ```

### 2.4. Git Submodule'lar ve ROS 2 Jazzy Yaması

`.gitmodules` dosyasında iki harici ROS 2 paketi tanımlıdır (yalnızca `localization:=ekf` veya `pcl_localization.launch.py` kullanılacaksa gereklidir; varsayılan `localization:=final_odom` bunlarsız da çalışır):

1. `src/pcl_localization_ros2` (`https://github.com/adawarp/pcl_localization_ros2.git`) — NDT tabanlı 3B PCD harita lokalizasyonu
2. `src/kiss-icp` (`https://github.com/PRBonn/kiss-icp.git`) — LiDAR odometrisi (`/kiss/odometry`)

Submodule'ları çekmek ve `pcl_localization_ros2` için ROS 2 Jazzy header yamasını (`.h` → `.hpp`) uygulamak için:

```bash
git submodule update --init --recursive
cd src/pcl_localization_ros2
git apply ../../patches/pcl_localization_ros2_jazzy_includes.patch
cd ../..
```

### 2.5. Gazebo Trafik Işığı C++ Eklentisinin Kurulumu

Parkurdaki trafik ışığı modelinin (`traffic_light`) renk değiştirebilmesi için özel C++ Gazebo eklentisi derlenip `~/.gz/sim/plugins/` dizinine kopyalanmalıdır:

```bash
cd src/araba/models/traffic_light
chmod +x build_and_install.sh
./build_and_install.sh
cd ../../../..
```

Bu script `CMakeLists.txt` üzerinden `TrafficLightPlugin.cc` dosyasını `gz-sim8` ile derler ve oluşan `libTrafficLightPlugin.so` dosyasını `~/.gz/sim/plugins/` klasörüne kurar.

### 2.6. Çalışma Alanının Derlenmesi (Colcon)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Neden `--symlink-install` önemlidir?**  
> `src/araba/CMakeLists.txt` içinde `.py` uzantısıyla (RENAME yapılmadan) kurulan düğümler (`lane_test_node.py`, `autonomous_control_node.py`, `lidar_obstacle_node.py`, `avoidance_node.py`, `final_odom_node.py`, `mission_planning_node.py`, `vehicle_controller_node.py` vb.) `--symlink-install` sayesinde doğrudan kaynak dosyalara sembolik bağlanır. Böylece Python kodlarında yapılan değişiklikler yeniden `colcon build` almadan anında geçerli olur.

---

## 3. Çalıştırma Komutları (Launch Dosyaları)

Her yeni terminalde önce `source /opt/ros/jazzy/setup.bash && source install/setup.bash` çalıştırılmalıdır.

### 3.1. Ana Simülasyon (`gazebo.launch.py`)

```bash
# Varsayılan: benim_dunyam.sdf + tam otonom sürüş + final_odom lokalizasyonu
ros2 launch araba gazebo.launch.py

# Manuel WASD sürüş (ışık/levha/engel güvenlik katmanı aktif kalır)
ros2 launch araba gazebo.launch.py manuel:=true

# Headless mod (Gazebo GUI olmadan, sunucu modunda)
ros2 launch araba gazebo.launch.py headless:=true show_window:=false

# EKF + KISS-ICP lokalizasyon hattı ile (not: launch yorumuna göre simde navsat datum sorunu mevcut)
ros2 launch araba gazebo.launch.py localization:=ekf
```

**Gerçek Yarışma Sahası Dünyasında (`saha_dunyasi.sdf`) Başlatma:**
```bash
ros2 launch araba gazebo.launch.py \
  world:=saha_dunyasi.sdf \
  mission_file:=$(ros2 pkg prefix araba)/share/araba/missions/saha_deneme_gorev.geojson \
  centerlines_file:=$(ros2 pkg prefix araba)/share/araba/missions/map_islenmis.geojson \
  spawn_x:=-38.53 spawn_y:=-0.91 spawn_yaw:=-0.85
```

**Desteklenen `gazebo.launch.py` argümanları:**
| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `headless` | `false` | `true` ise Gazebo `-s` (sadece sunucu) modunda açılır |
| `manuel` | `false` | `true` ise `autonomous_control_node` + `vehicle_controller_node` yerine `keyboard_teleop` başlar |
| `show_window` | `true` | `lane_test_node` OpenCV şerit önizleme penceresini açar/kapatır |
| `localization` | `final_odom` | `final_odom` (GPS+IMU+odom füzyonu, simde çalışan) veya `ekf` (`robot_localization` + KISS-ICP) |
| `world` | `benim_dunyam.sdf` | `worlds/` altındaki SDF dünya dosyası (`benim_dunyam.sdf` veya `saha_dunyasi.sdf`) |
| `mission_file` | `missions/teknofest_gorev.geojson` | Görev noktaları GeoJSON yolu |
| `centerlines_file` | `missions/teknofest_centerlines.geojson` | Yol ağı (şerit eksenleri) GeoJSON yolu |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | `45.31`, `16.0`, `0.51`, `1.58` | Aracın dünyadaki başlangıç konumu ve yönelimi |

### 3.2. Diğer Launch Dosyaları

| Launch Dosyası | Amaç ve Kullanım |
|---|---|
| `display.launch.py` | Sadece URDF modelini `joint_state_publisher_gui` + `rviz2` ile incelemek için: `ros2 launch araba display.launch.py` |
| `lane_test.launch.py` | İzole şerit segmentasyon testi (`lane_seg.onnx` + `lane_test_node.py` + `autonomous_control_node.py`, ağır algı düğümleri yok) |
| `lane_yolo.launch.py` | Alternatif şerit testi (`2206_2model.onnx` + `ultralytics` tabanlı `lane_yolo_node.py`). `device:=cpu` veya `device:=0` destekler |
| `data_collect.launch.py` | Model çalıştırmadan ham `/camera/image` veri toplama ortamı (Gazebo + WASD teleop). İkinci terminalde `python3 src/araba/scripts/dataset_recorder.py` çalıştırılır |
| `lidarfilters.launch.py` | Statik TF (`base_link -> odom_lidar`) + `pcl_ros` Voxel Grid filtresi (`leaf_size=0.2`) + `kiss_icp_node` (`/kiss/odometry`). `localization:=ekf` tarafından dahil edilir |
| `pcl_localization.launch.py` | `pcl_localization_ros2` ile PCD haritası (`maps/benim_dunyam.pcd` veya `map_file:=...`) üzerinde NDT eşleştirme tabanlı global lokalizasyon |

---

## 4. Proje Dizin Yapısı ve Tüm Dosyaların Görevleri

### 4.1. Kök Dizin ve Yardımcı Klasörler

- **`.gitignore`**: Colcon çıktıları (`/build/`, `/install/`, `/log/`), büyük veri/kayıt klasörleri (`/dataset/`, `/rapor/`, `*.mcap`, `*.mp4`), Windows `*:Zone.Identifier` artıkları ve yerel araç ayarlarını dışlar.
- **`.gitmodules`**: `src/pcl_localization_ros2` ve `src/kiss-icp` git alt modüllerini tanımlar.
- **`patches/pcl_localization_ros2_jazzy_includes.patch`**: `pcl_localization_ros2` içindeki eski `.h` header include'larını ROS 2 Jazzy uyumlu `.hpp` sürümüne çeviren yama.
- **`otonom_projem/`**: Eski/yedek Gazebo model ve harita deposu.
  - `COLCON_IGNORE`: Colcon'un bu klasörü paket olarak taramasını engeller.
  - `models/`: `src/araba/models/` ile neredeyse aynı 28 adet Gazebo modeli (sadece `teknofest_haritası` yok). Launch dosyaları geriye dönük uyumluluk için bu yolu da `GZ_SIM_RESOURCE_PATH`'e ekler.
  - `teknofest_haritası/`: `benim_dunyam.sdf` (eski kopyası), harita görselleri (`map.jpeg`, `map1.png`, `map_clean.png`) ve `tunnel.STL`.

---

### 4.2. ROS 2 Paketi Temel Dosyaları (`src/araba/`)

- **`package.xml`**: `araba` paketinin (`ament_cmake`) meta verileri ve çalışma zamanı bağımlılıkları (`robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `ros_gz_sim`, `ros_gz_bridge`, `xacro`).
- **`CMakeLists.txt`**: `config`, `launch`, `maps`, `meshes`, `missions`, `models`, `urdf`, `worlds` dizinlerini `share/araba/` altına; `scripts/onnx` dizinini `share/araba/models/onnx/` altına; Python düğümlerini ve saf Python algoritma modüllerini `lib/araba/` altına kurar.

---

### 4.3. Yapılandırma Dosyaları (`src/araba/config/`)

| Dosya | Görevi |
|---|---|
| `ekf.yaml` | `robot_localization` paketi için `ekf_filter_node` (30 Hz, 2D mod; `/odometry/gps` pozisyon + `/kiss/odometry` hız/yaw-rate + `/imu/data` yaw) ve `navsat_transform` (datum `[40.7899, 29.5089, 0.0]`) parametreleri |
| `gz_ros2_control.yaml` | `ros2_control` için alternatif kontrolcü yapılandırması (100 Hz; 4 teker hız + 2 ön direksiyon pozisyon kontrolcüsü). Mevcut `araba.urdf` doğrudan Gazebo `AckermannSteering` eklentisini kullandığı için şu an aktif değildir |
| `joint_names_araba.yaml` & `joint_names_araba_v2.yaml` | SolidWorks-to-URDF dışa aktarımından gelen eklem isim listeleri |
| `lidar_view.rviz` | `gazebo.launch.py` ve `display.launch.py` tarafından açılan RViz2 görselleştirme profili |

---

### 4.4. Araç Modeli, Mekanik ve Sensörler (`src/araba/urdf/` & `src/araba/meshes/`)

- **`urdf/araba.urdf`**: SolidWorks'ten aktarılan ve Gazebo Harmonic eklentileriyle donatılan araç modeli (`base_footprint` → `base_link`, 2 ön direksiyon eklemi `±0.5236 rad / ±30°`, 4 tekerlek eklemi):
  - **Sürüş Eklentisi (`gz-sim-ackermann-steering-system`)**: `wheel_base=1.675 m`, `wheel_separation=1.486 m`, `wheel_radius=0.25 m`, `max_velocity=5.56 m/s (20 km/h)`, `min_velocity=-2.78 m/s`. `/cmd_vel` dinler, `/odom` yayınlar.
  - **Joint State Publisher (`gz-sim-joint-state-publisher-system`)**: `/model/araba/joint_state` yayınlar (köprü `/joint_states`'e çevirir).
  - **LiDAR (`lidar_sensor_link`)**: SICK multiScan100 benzetimi — `gpu_lidar`, 16 dikey kanal (`±15°`), 2118 yatay örnek (`360°`), 0.1–100 m menzil, 20 Hz (`/lidar/scan`, `/lidar/scan/points`).
  - **Kamera (`cam_sensor_link`)**: Intel RealSense D415 benzetimi — 1280×720 RGB, 69.4° FOV (`1.2113 rad`), 30 Hz (`/camera/image`, `/camera/camera_info`).
  - **IMU (`imu_sensor_link`)**: 100 Hz, Gauss gürültülü (`/imu/data`).
  - **GPS (`gps_sensor_link`)**: NavSat sensörü, 10 Hz (`/gps/fix`).
- **`urdf/araba.csv`**: SolidWorks URDF dışa aktarımının kütle, atalet momenti ve eklem tablosu (referans).
- **`meshes/*.STL`**: Şasi, 4 tekerlek, 2 ön direksiyon mafsalı ve 4 sensörün 3B görsel/çarpışma STL modelleri.

---

### 4.5. Dünyalar, Haritalar, Görevler ve Gazebo Modelleri

- **`src/araba/worlds/`**:
  - `benim_dunyam.sdf`: Ana TEKNOFEST simülasyon parkuru. `map.jpeg` (97×120 m) zemini, yürüyen yaya aktörü (`walking_actor`), trafik levhaları, ışıklar, koniler, bariyerler ve tünel içerir. GPS datumu: `40.7899, 29.5089` (ENU).
  - `saha_dunyasi.sdf`: Gerçek yarışma sahasının yol ağından (`map_islenmis.geojson`) üretilen `saha_zemin.png` dokusunu kullanan dünya. Aynı GPS datumunu paylaşır.
  - `benim_dunyam.sdf.save`: Yedek SDF dosyası.
  - `map.jpeg`, `map1.png`, `map_clean.png`, `saha_zemin.png`, `tunnel.STL`: Dünya zemin dokuları ve tünel 3B modeli.
- **`src/araba/maps/`**:
  - `benim_dunyam.pcd` (~2.7 MB): `build_pcd_map.py` ile simülasyon parkurundan üretilen 3B nokta bulutu haritası (NDT lokalizasyon için).
  - `saha_haritasi.pcd` (~124 KB): Gerçek saha 3B PCD haritası.
- **`src/araba/missions/`**:
  - `teknofest_centerlines.geojson` & `teknofest_gorev.geojson`: `benim_dunyam.sdf` için çift şeritli, tek yönlü (`oneway: true`), döner kavşaklı ve tünelli yol ağı ile görev noktaları.
  - `test_gorev_kavsak.geojson`: Kavşak dönüşlerini test etmek için görev dosyası.
  - `map.geojson`: Gerçek sahadan QGIS ile çizilmiş ham yol ağı.
  - `map_islenmis.geojson`: `prepare_saha_geojson.py` ile köşe birleştirme, tek yön ve tünel etiketlemesi yapılmış saha yol ağı.
  - `saha_deneme_gorev.geojson`: Gerçek saha haritası için deneme görev noktaları.
  - `rota_onizleme.png`, `rota_onizleme_node10.png`, `saha_onizleme.png`: Önizleme araçlarıyla üretilmiş rota görselleri.
- **`src/araba/models/`**: 29 adet Gazebo modeli — tüm TEKNOFEST trafik levhaları (`dur`, `durak`, `girilmez`, `doner_kavsak`, `park_yeri`, `park_yapilmaz`, `tunel`, `yaya_gecidi`, mecburi/yasak yön levhaları vb.), `traffic_cone`, `red_barrier`, `white_barrier`, `walking_actor`, `trafik_isik`, `teknofest_haritası` ve C++ eklentili **`traffic_light`** (`TrafficLightPlugin.cc`, `CMakeLists.txt`, `build_and_install.sh`, `controller.sh` — harici yeşil/sarı/kırmızı döngü kontrolcüsü).

---

### 4.6. ROS 2 Düğümleri (`src/araba/scripts/`)

#### Aktif Simülasyon Düğümleri (`gazebo.launch.py` içinde çalışanlar)

| Dosya | Yayın/Abone Özeti ve Görevi |
|---|---|
| `camera_perception_node.py` | `/camera/image` → `detection.onnx` (CPU, her 3 karede 1, 29 Türkçe sınıf) → `/perception/detections` (`Float32MultiArray`) ve `/perception/sign_image`. |
| `lane_test_node.py` | `/camera/image` → `model.onnx` (YOLO-seg, ONNX Runtime) → ROI + alt-bant bağlı bileşen filtresi + kayan pencere + EMA yumuşatma → `/perception/center_pts` ve `/perception/lane_mask`. |
| `lidar_obstacle_node.py` | `/lidar/scan/points` → vektörize XYZ açma, zemin/tavan yükseklik filtresi (`min_z=-1.35`, `max_z=-0.5`), tampon ofseti, Öklid kümeleme, çift kapı (harita yol maskesi + kamera şerit maskesi) → `/perception/lidar_obstacles` (JSON) ve trafik ışığı direkleri için `/perception/pole_candidates`. |
| `avoidance_node.py` | `gap_avoid_logic` sarmalayıcısı. `/perception/lidar_obstacles` üzerinde çoğunluk filtresi (8 kareden ≥5'i) uygular, şerit içi konumu harita/kameradan ölçerek `/planning/lateral_offset` (+ = SOL) ve direksiyon düzeltmesi yayınlar. |
| `final_odom_node.py` | Varsayılan lokalizasyon düğümü (50 Hz). `/gps/fix` (WGS84→ENU pozisyon) + `/imu/data` (`spawn_yaw` ile birleşmiş driftsiz yönelim) + `/odom` (hız) verilerini birleştirip `/localization/odom/final` yayınlar. |
| `perception_pipeline_node.py` | Merkezi algı orkestratörü (`use_sim_time=False`). Kamera tespitleri + LiDAR engelleri + direk adayları + IMU + odom verilerini `perception_fusion.fuse()` ile sınıflandırır; `TrafficLightLogic`, `TrafficSignLogic`, `ObstacleLogic`, `ParkingLogic` ve `DecisionArbiter` çalıştırır. Opsiyonel 4 panelli Tkinter dashboard içerir (`show_dashboard:=false` varsayılan). |
| `mission_planning_node.py` | GeoJSON görev + yol grafı planlayıcısı (`RouteGraph`, `route_planner`, `MissionManager`, `WaypointManager`). ENU yaw → pusula çevrimi yapar; kavşak dönüş yayları (`turn_active`), döner kavşak halka takibi, levha bazlı dönüş kısıtı/yeniden planlama ve yön-tabanlı durak cebi S-manevrası (`maneuver_active`) üretir. |
| `autonomous_control_node.py` | `/perception/center_pts` + `/planning/lateral_offset` dinleyen P kontrolcü + Ackermann dönüşümü. `/cmd_vel_lane` (tek başına çalıştırıldığında `/cmd_vel`) yayınlar. |
| `vehicle_controller_node.py` | Kontrol katmanı hakemi (arbiter). Öncelik sırası: `OVERRIDE > MANEUVER > TURN > LANE > PLAN (şerit kaybında sürünme) > STOP`. Kaçınma düzeltmesini moddan bağımsız ekler; `/cmd_vel_raw` yayınlar. |
| `speed_controller_node.py` | 20 Hz hız güvenlik katmanı (`use_sim_time=False`). `/cmd_vel_raw` + ışık/levha/engel durumlarını `DecisionArbiter` ile birleştirir; acil fren, kırmızı ışık/STOP duruşu, eğriliğe (direksiyon açısına) orantılı hız tavanı uygulayarak nihai `/cmd_vel` komutunu Gazebo'ya gönderir. |
| `keyboard_teleop.py` | `manuel:=true` veya `data_collect.launch.py` ile çalışan `pynput` tabanlı WASD + Space + ESC/Q klavye kontrolcüsü. |

#### Alternatif, Deneme ve Gerçek Araç Düğümleri

| Dosya | Görevi |
|---|---|
| `lane_perception_node.py` | `model.onnx` için polinomial eğri uydurmalı alternatif/eski şerit algılama ROS 2 düğümü (yerini `lane_test_node.py` almıştır). |
| `lane_yolo_node.py` | `ultralytics.YOLO(..., task='segment')` ve `masks.xy` poligonlarıyla çalışan alternatif şerit takip düğümü (`lane_yolo.launch.py`). |
| `perception_fusion_node.py` | Gerçek araç (`/deos/...` topic ağacı, `hardware_motion_enable`, `autonomy_enable`, `deos_logging`) için yazılmış üretim sürümü algı füzyon düğümü. Simülasyonda yerine `perception_pipeline_node.py` kullanılır. |
| `dataset_recorder.py` | `data_collect.launch.py` açıkken ayrı terminalde çalıştırılır; araç her ~0.5 m ilerlediğinde `/camera/image` üzerinden ham PNG kareler kaydeder (`REC_OUT`, `REC_MIN_DIST`, `REC_TOPIC` ortam değişkenleriyle ayarlanabilir). |

---

### 4.7. Saf Python Algoritma Modülleri ve Köprüler (`src/araba/scripts/`)

Bu modüller ROS bağımsızdır (`rclpy` kullanmaz) ve hem simülasyonda hem birim testlerinde doğrudan import edilir:

- **Köprü ve Ortak Tipler:**
  - `deos_algorithms/__init__.py`: `from deos_algorithms.xxx import ...` çağrılarını symlink olmadan üst dizindeki gerçek modüllere yönlendiren `sys.meta_path` sanal paket köprüsü.
  - `sensors/types.py`: `StereoBbox`, `LidarObstacle`, `ImuSample` frozen veri sınıfları.
  - `ros_topic_layout.py`: Gerçek araç için `/deos/...` standart ROS 2 topic hiyerarşisini üreten `build_deos_topics()`.
  - `geo_utils.py`: Paylaşımlı `haversine_m()` GPS mesafe hesabı.
- **Görev ve Rota Katmanı:**
  - `geojson_mission_reader.py`: GeoJSON görev dosyasını okuyup `MissionPlan` / `MissionPoint` (`START`, `CHECKPOINT`, `STOP`, `PARK`, `PICKUP`, `DROPOFF` vb.) üretir.
  - `route_graph.py`: Centerline GeoJSON'undan yönlü/tünelli yol grafı (`RouteGraph`) kurar; Dijkstra ve zorunlu tünel rotası (`dijkstra_mandatory_tunnel`) hesaplar.
  - `route_planner.py`: Görev noktalarını yol grafına bağlayıp en kısa rotayı ve anlık yeniden planlamayı (replanning) üretir.
  - `waypoint_manager.py`: GPS + pusula yönüne göre sıradaki waypoint için `steering_ref`, cross-track error (XTE) ve varış kontrolü üretir.
  - `mission_manager.py`: `PICKUP`/`DROPOFF` bekleme, `PARK` modu ve zaman aşımı gibi görev davranışlarını yönetir.
- **Algı Füzyonu, Karar ve Kaçınma Katmanı:**
  - `perception_fusion.py`: Ham kamera/LiDAR/IMU verilerini ışık, levha, engel ve park adaylarına sınıflandırır (`fuse()`).
  - `safety_logic.py`: Koridor modeli, pinhole mesafe kestirimi, `SimpleTracker` ve 3 seviyeli çarpışma tehdit analizi (`EMERGENCY` / `HARD_SOFT` / `SOFT_SLOW`).
  - `obstacle_logic.py`: `SafetyLogic` üzerinde yaya (dinamik bekleme/kaçınma) ve koni/bariyer (statik kaçınma) davranış makinesi.
  - `gap_avoid_logic.py`: Boşluk-takibi (VFH benzeri) engelden kaçınma; asimetrik şerit bandı (`band_left_m=3.0`, `band_right_m=1.30`) içinde yanal ofset üretir.
  - `traffic_light_logic.py`: Çoklu kare onaylı, bağlam duyarlı (kırmızı→sarı vs yeşil→sarı) trafik ışığı hız/duruş mantığı.
  - `traffic_sign_logic.py`: 26+ Türkçe trafik levhası için hız sınırı, zorunlu STOP duruşu ve `TurnPermissions` dönüş kısıtları.
  - `slalom_logic.py`: Koni parkuru için S-weave slalom durum makinesi (`gazebo.launch.py`'de varsayılan `enable_slalom=False`).
  - `parking_logic.py`: 4 fazlı (yaklaşma → hizalanma → manevra → park edildi) dik/paralel park durum makinesi.
  - `lane_violation.py`: 10 saniyelik dilimlerde şerit ihlali sayacı (`LaneViolationTracker`).
  - `decision_arbiter.py`: Tüm modüllerin aday kararlarını öncelik sırasına göre tek bir `FinalDecision` çıktısında birleştirir.
- **`README_ALGORITHMS.md`**: Algoritma modüllerinin mimari özet dokümanı.

---

### 4.8. Yapay Zeka Modelleri (`src/araba/scripts/onnx/`)

| Dosya | Boyut | Kullanım Yeri |
|---|---|---|
| `model.onnx` | ~46 MB | **Aktif** YOLOv8s-seg şerit segmentasyon modeli (`gazebo.launch.py` → `lane_test_node.py`) |
| `detection.onnx` | ~43 MB | **Aktif** 29 sınıflı trafik levhası / ışık / koni / bariyer / yaya tespit modeli (`camera_perception_node`) |
| `lane_seg.onnx` | ~13 MB | Eski/hafif şerit segmentasyon modeli (`lane_test.launch.py` varsayılanı) |
| `2206_2model.onnx` | ~13 MB | `lane_yolo.launch.py` (`lane_yolo_node.py`) için kullanılan segmentasyon modeli |
| `2206model.onnx` | ~13 MB | Deneysel/yedek şerit segmentasyon modeli |
| `smallmodel.pt` | ~46 MB | PyTorch kaynak ağırlık dosyası |
| `convert_to_onnx.py` | — | `smallmodel.pt` dosyasını `opset=17`, `imgsz=640` ile `model.onnx` formatına dönüştüren script |

---

### 4.9. Yardımcı Araçlar (`src/araba/scripts/tools/`)

| Araç | Görevi ve Kullanımı |
|---|---|
| `build_pcd_map.py` | Aracı `teknofest_centerlines.geojson` boyunca ~3 m aralıklarla Gazebo'da ışınlayıp LiDAR taramalarını birleştirir ve `maps/benim_dunyam.pcd` haritasını üretir (`gz-transport` kullanır). |
| `generate_teknofest_geojson.py` | `benim_dunyam.sdf` parkur geometrisinden çift şeritli `teknofest_centerlines.geojson` ve `teknofest_gorev.geojson` dosyalarını üretir. |
| `prepare_saha_geojson.py` | QGIS'ten gelen ham `map.geojson` dosyasını temizler: T-kavşak köşelerini birleştirir, sağdan trafik tek yön (`oneway: true`) atar, tünel kenarlarını (`tunnel: true`) işaretler ve `map_islenmis.geojson` üretir. |
| `generate_saha_ground.py` | `map_islenmis.geojson` yol ağından `worlds/saha_zemin.png` Gazebo zemin dokusunu çizer ve `saha_dunyasi.sdf` için kutu boyut/poz değerlerini basar. |
| `render_route_preview.py` | `map.jpeg` üzerine yol grafını, tek/çift yön oklarını, tüneli ve `route_planner` rotasını çizip `missions/rota_onizleme.png` üretir. |
| `render_saha_preview.py` | Gerçek saha yol ağını uydu görüntüsü üzerine çizer (`missions/saha_onizleme.png`) ve `missions/saha_deneme_gorev.geojson` üretir. |
| `sim_report_recorder.py` | Simülasyon koşusu sırasında Gazebo ground-truth + ROS topic'lerini dinleyip verilen klasöre `lokalizasyon.csv`, `kontrol.csv`, `planlama.csv`, `kararlar.jsonl` ve `olaylar.log` kaydeder (`python3 src/araba/scripts/tools/sim_report_recorder.py rapor/kosu1`). |

---

### 4.10. Birim ve Regresyon Testleri (`src/araba/scripts/tools/tests/`)

Algoritma ve düğüm mantığını simülasyon açmadan doğrulamak için 13 adet test scripti bulunur:

| Test Dosyası | Doğruladığı Davranış |
|---|---|
| `test_avoid_bias.py` | `avoidance_node` direksiyon düzeltme kuralları |
| `test_column_gate.py` | Kolon testi ve koni çarpma regresyon korumaları |
| `test_durak_maneuver.py` | Yön-tabanlı durak cebi S-manevrası |
| `test_gap_avoid.py` | `gap_avoid_logic` boşluk takibi ve asimetrik şerit bandı |
| `test_intersection_scope.py` | Kavşak geçişi sonrası yapışkan dönüş kısıtının temizlenmesi |
| `test_light_commit.py` | Kırmızı ışık commit bandı ve kilit mekanizması |
| `test_majority_filter.py` | `avoidance_node` çoğunluk filtresi (titreyen tabela eleme) |
| `test_obstacle_unknown_avoid.py` | `unknown` LiDAR kümesinin statik kaçınmaya girmesi ve acil fren kilitlenmemesi |
| `test_pole_distance.py` | LiDAR direk adayı çıkarımı ve kamera trafik ışığı bbox eşleştirmesi |
| `test_roundabout_follow.py` | Döner kavşak halka takip ve alternatif rota seçimi |
| `test_route_unstable.py` | Rota yeniden hesaplanırken dönüş tetiklemesinin bastırılması |
| `test_turn_late_gain.py` | Geç tetiklenen kavşak dönüşlerinde yakınlık bazlı direksiyon kazancı |
| `test_uturn_penalty.py` | Sola-dönülmez / bloklu kenarda U dönüşü cezası |

Testleri çalıştırmak için (pytest kuruluysa `pytest` veya doğrudan `python3` ile):
```bash
for t in src/araba/scripts/tools/tests/test_*.py; do python3 "$t"; done
```

---

## 5. Kritik Teknik Notlar ve Senkronizasyon Kuralları

1. **GPS Datum Senkronizasyonu (`40.7899, 29.5089`):** `benim_dunyam.sdf`, `saha_dunyasi.sdf`, `gazebo.launch.py` (`DATUM_LAT/LON`), `ekf.yaml`, `final_odom_node.py` ve `missions/*.geojson` dosyalarının tamamı aynı WGS84 datumunu paylaşmalıdır.
2. **Koordinat ve İşaret Konvansiyonları:**
   - `final_odom_node` standart **ENU yaw** (`0°=Doğu`, saat yönü tersi) yayınlar; `mission_planning_node` bunu `heading_is_enu_yaw:=True` ile pusulaya (`0°=Kuzey`, saat yönü) çevirir.
   - Planlama `steering_ref` ve füzyon `steer_override` değerleri `[-1, 1]` aralığında **+ = SAĞA**; ROS `Twist.angular.z` ve `/planning/lateral_offset` ise **+ = SOLA** konvansiyonundadır. Çevrim `vehicle_controller_node` ve `autonomous_control_node` içinde yapılır.
3. **`use_sim_time` Ayrımı:** `perception_pipeline_node` ve `speed_controller_node` bilinçli olarak `use_sim_time:=False` ile çalışır; çünkü düşük RTF (~0.2) altında iç zaman aşımı kontrolleri duvar saati (`time.monotonic()`) kullanır. `avoidance_node` ve diğer sürüş/algı düğümleri ise simülasyon saati (`use_sim_time:=True`) kullanır.
4. **Lokalizasyon Seçimi:** `gazebo.launch.py` varsayılanı `localization:=final_odom`'dur. `localization:=ekf` seçeneği `navsat_transform` + `ekf_node` + `lidarfilters.launch.py` (KISS-ICP) hattını başlatır ancak simülasyonda `navsat_transform` datum davranışı nedeniyle konumun `(0, 0)`'da sabitlenme sorunu launch içinde belgelenmiştir.
