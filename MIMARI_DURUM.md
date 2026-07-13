# Simülasyon — Mimari Durum ve Kullanım Kılavuzu

> **Tarih:** 12 Temmuz 2026 | **Branch:** `bekir`
> **ROS 2:** Jazzy | **Gazebo:** Harmonic (`gz-sim8`) | **Paket:** `araba`
> **Workspace:** `~/sim2_ws` | **GUI:** WSLg (ek kurulum gerekmez)
>
> Bu doküman eski "Detaylı Durum Özeti"nin (5 Temmuz) yerine geçer. O tarihten bu yana
> görev sistemi, lokalizasyon, kavşak dönüşü, park ve güvenlik katmanları tamamen
> entegre edildi — aşağıdaki her şey canlı stack'i anlatır.

---

## 1. Hızlı Başlangıç

```bash
# Derleme (ilk kurulumda veya CMakeLists/RENAME'li node değişince)
cd ~/sim2_ws
colcon build --symlink-install

# TAM OTONOM TUR (varsayılan): Gazebo + RViz + tüm algı/planlama/kontrol
ros2 launch araba gazebo.launch.py

# Manuel sürüş (WASD) — güvenlik katmanı yine devrede
ros2 launch araba gazebo.launch.py manuel:=true

# Başsız test koşusu (GUI ve OpenCV pencereleri kapalı)
ros2 launch araba gazebo.launch.py headless:=true show_window:=false
```

> `.bashrc` hem ROS'u hem workspace'i otomatik source eder; yeni terminalde elle
> `source` gerekmez. Python node'ları ve launch **symlink** kurulur → kod değişikliği
> yeniden derleme istemez (node'u yeniden başlatmak yeter).

### Launch Argümanları

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `headless` | `false` | `true` → Gazebo GUI'siz (sadece sunucu) |
| `manuel` | `false` | `true` → otonom kontrol yerine WASD teleop |
| `show_window` | `true` | Şerit segmentasyonu OpenCV penceresi |
| `mission_file` | `missions/teknofest_gorev.geojson` | Görev noktaları (GeoJSON) |
| `centerlines_file` | `missions/teknofest_centerlines.geojson` | Yol ağı / centerline'lar |
| `world` | `benim_dunyam.sdf` | `worlds/` altındaki dünya dosyası |
| `spawn_x/y/z/yaw` | `45.31 / 16.0 / 0.51 / 1.58` | Araç başlangıç pozu (dünya/ENU) |
| `localization` | `ekf` | `ekf`: robot_localization+KISS-ICP | `final_odom`: eski füzyon node'u (yedek) |

**Rotanın ortasından test koşusu** (ör. kavşak+park bölümü):

```bash
ros2 launch araba gazebo.launch.py \
  spawn_x:=-36.1 spawn_y:=18.0 spawn_yaw:=-1.5708 \
  mission_file:=/yol/kisa_gorev.geojson
```

> Kısa görev dosyasının **ilk hedefi spawn noktasına yakın olmalı** — ilk rota,
> görev dosyasındaki 0. hedeften planlanır. `spawn_*` değerleri lokalizasyona da
> parametre gider (final_odom'un spawn dönüşümü / navsat'ın `yaw_offset`'i);
> hepsi launch içinde otomatik senkron kalır.

### Lokalizasyon (`localization:=ekf | final_odom`)

İki hat da `/localization/odom/final` üretir (mission_planning'in girdisi):

- **`ekf` (varsayılan, arda'nın hattı):** `lidarfilters.launch.py` LiDAR'ı voxel
  filtreden geçirip **KISS-ICP** ile LiDAR odometrisi üretir (`/kiss/odometry`);
  `robot_localization/navsat_transform` GPS'i metrik odometriye çevirir; **EKF**
  GPS pozisyonu + KISS-ICP hızı + IMU yaw'ı kaynaştırır (`config/ekf.yaml`).
  Gazebo IMU'su spawn yönünü 0 kabul ettiği için navsat'ın `yaw_offset`'i launch'ta
  `spawn_yaw`'dan otomatik verilir.
- **`final_odom` (yedek):** Eski GPS(poz)+IMU(yön)+odom(hız) füzyon node'umuz —
  7 canlı turda kanıtlanmış; EKF hattında sorun görülürse
  `localization:=final_odom` ile anında geri dönülür.

Ayrıca **PCD harita lokalizasyonu** (opsiyonel, ayrı launch): `maps/benim_dunyam.pcd`
(parkurun ground-truth nokta haritası; `scripts/tools/build_pcd_map.py` ile yeniden
üretilebilir) üzerinde NDT eşleştirme:

```bash
ros2 launch araba pcl_localization.launch.py            # varsayılan: benim_dunyam.pcd
ros2 launch araba pcl_localization.launch.py map_file:=.../maps/saha_haritasi.pcd
```

**EKF hattının bağımlılıkları** (bir defalık kurulum):

```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libpcl-dev
git submodule update --init          # src/kiss-icp + src/pcl_localization_ros2
colcon build --symlink-install       # kiss_icp ve pcl_localization_ros2 derlenir
```

### GPS Datumu

Dünya ↔ enlem/boylam dönüşümünün merkezi: **40.7899, 29.5089** (gerçek yarışma
sahası merkezi). `benim_dunyam.sdf` içindeki `<spherical_coordinates>` ile
`gazebo.launch.py`'deki `DATUM_LAT/LON` senkron tutulmalıdır.

---

## 2. Mimari — Veri Akışı

```
Gazebo sensörleri (ros_gz_bridge, 10 topic)
 │
 ├─ /camera/image ─► camera_perception_node (detection.onnx, 29 sınıf)
 │                     └─► /perception/detections
 ├─ /camera/image ─► lane_test_node (model.onnx, şerit segmentasyonu)
 │                     └─► /perception/center_pts
 ├─ /lidar/scan/points ─► lidar_obstacle_node (kümeleme + yol maskesi)
 │                     └─► /perception/lidar_obstacles
 ├─ LOKALİZASYON (bkz. §1) ─► /localization/odom/final (dünya/ENU poz + yön)
 │    ekf: /lidar/scan/points ─► voxel ─► KISS-ICP ─► /kiss/odometry ─┐
 │         /gps/fix ─► navsat_transform ─► /odometry/gps ─► EKF ◄─────┘◄─ /imu/data
 │    final_odom: /gps/fix + /imu/data + /odom ─► final_odom_node (yedek)
 │
 ├─ detections + lidar ─► perception_pipeline_node (füzyon + dashboard)
 │       ├─► /perception/traffic_light_state / traffic_sign_state / obstacle_state
 │       ├─► /perception/turn_permissions, decision_debug, park_complete
 │       ◄── /planning/park_mode  (park görüş modunu planlama açar)
 │
 └─ final_odom + görev GeoJSON ─► mission_planning_node
         └─► /planning/steering_ref, speed_limit, current_task,
             arrived, park_mode, park_remaining_s

KONTROL ZİNCİRİ:
 center_pts ─► autonomous_control_node ─► /cmd_vel_lane
 /cmd_vel_lane + /planning/* ─► vehicle_controller_node ─► /cmd_vel_raw
     (öncelik: OVERRIDE > TURN > LANE > PLAN > STOP —
      şerit tazeyse şerit sürer; kavşak dönüşünde planlamanın yay takibi devralır)
 /cmd_vel_raw + algı durumları ─► speed_controller_node ─► /cmd_vel ─► Gazebo
     (kırmızı ışıkta durdurur, levha sınırı uygular, engelde acil fren)

 manuel:=true → keyboard_teleop /cmd_vel_raw'a yazar (güvenlik katmanı yine aktif)
```

### Launch Sırası (`gazebo.launch.py`)

| Süre | Node | Görevi |
|---|---|---|
| 0s | Gazebo + robot_state_publisher + create + rviz2 | Dünya, model, spawn |
| 5s | ros_gz_bridge | 10 topic köprüsü |
| 6.0s | `camera_perception_node` | Nesne tespiti (`detection.onnx`) |
| 6.5s | `lane_test_node` | Şerit segmentasyonu (`model.onnx`) |
| 6.8s | `lidar_obstacle_node` | LiDAR engel kümeleme (yol maskeli) |
| 0s (ekf) | `lidarfilters` (voxel + KISS-ICP) | LiDAR odometrisi `/kiss/odometry` |
| 6.9-7.0s | lokalizasyon: `navsat`+`ekf_node` **veya** `final_odom_node` | `/localization/odom/final` |
| 7.0s | `perception_pipeline_node` | Algı füzyonu + tkinter dashboard |
| 7.2s | `mission_planning_node` | Görev/rota planlama + kavşak dönüşü |
| 7.5s | `autonomous_control_node` | Şerit takibi P-kontrolcü (manuel'de kapalı) |
| 7.6s | `vehicle_controller_node` | Şerit/rota arbitrasyonu |
| 7.8s | `speed_controller_node` | Hız güvenlik katmanı |
| 8.0s | `keyboard_teleop` | Sadece `manuel:=true` iken |

---

## 3. Görev Sistemi (mission_planning)

- **Girdi:** `missions/*.geojson` görev noktaları (durak, park_giris, park_yeri,
  kavşak vb. görev tipleri) + centerlines yol ağı + `/localization/odom/final`.
- **Rota:** `route_graph` (centerline → graf) + `route_planner` (Dijkstra);
  engel nedeniyle yol kapanırsa REPLAN.
- **Takip:** `waypoint_manager` GPS waypoint takibi; varış yarıçapı görev
  noktası başına ayarlı.
- **Kavşak dönüşü (yay takibi):** Dönüş algılanınca giriş/çıkış bacakları arasına
  **teğet dairesel yay** (r=4 m) kurulur; 12 m hazırlık + 8 m çıkış koşusu ile
  ~1 m aralıkla örneklenir ve **pure pursuit** (öngörü 2.5 m) ile sürülür.
  Şeride basmamak için yayın apeksi köşe düğümünün en fazla **0.7 m** içinden
  geçer; kalan derinlik dışa-kaydırma ile karşılanır ve bu kayma yaklaşma boyunca
  **rampalı** biner (ani direksiyon yok). Dönüş çıkışı: yeni bacağa hizalanma
  (≤25°) **veya** yay ilerlemesinin tamamlanması — hangisi önce gelirse.
- **Park akışı:** `park_giris` noktasına varınca **3 dk şartname sayacı** başlar
  ve araç waypoint'lerle **cebin hizasına kadar koordinat sürüşüne devam eder**;
  `park_yeri` waypoint'ine varınca `/planning/park_mode` açılır ve görüş tabanlı
  park (`parking_logic`) devralır. Park tamamlanınca `perception/park_complete`
  ile görev kapanır.

## 4. Güvenlik / Davranış Mantıkları

- **Trafik ışığı** (`traffic_light_logic`): 2 kare onay + 1.5 s bellek.
  Kırmızı görülünce **kilitlenir**, ancak onaylı yeşille çözülür (45 s failsafe).
  **Bayat-yeşil kuralı:** onaylı yeşil görüşten düşerse 6 s boyunca hız tavanı
  %50 — kavşak kör pencerede sürünerek geçilir; kırmızı kilidini asla gevşetmez.
- **Engel** (`obstacle_logic` + LiDAR): kümeler **sadece çapa göre** sınıflanır
  (<0.4 m koni, <1.5 m bilinmeyen, ≥1.5 m bariyer); güvenlik koridoru ±1.2 m,
  bakış 20 m. **road_blocked:** yalnız ≤8 m'deki ≥2 bariyer **ve** aralarında
  ≥1.8 m geçilebilir boşluk yoksa yol kapalı sayılır → tünel ağzı geçit,
  gerçek barikat kapalı (tünelde REPLAN fırtınası bununla çözüldü).
- **Park** (`parking_logic`): tabela **stereo mesafesi** varsa onu kullanır
  (bbox kestirimi yedek; kamera sabitleri gerçek kameraya göre: 1280×720,
  fx=917.4, yükseklik 0.948 m). **Hedef kilidi:** seçilen cep kareler arası
  takip edilir (200 px eşleşme, 10 kare kayıp toleransı) — araç artık her karede
  "daha iyi" tabelaya atlamaz. Park-yasak tabelası asla hedef olmaz.

## 5. ONNX Modelleri (`scripts/onnx/` → install'de `models/onnx/`)

| Model | Kullanan | Durum |
|---|---|---|
| `detection.onnx` (44.8 MB) | `camera_perception_node` | ✅ AKTİF — 29 sınıf |
| **`model.onnx` (47.3 MB)** | `lane_test_node` | ✅ **AKTİF — YENİ şerit modeli** (11 Tem: gazeboset yolov8s-seg, tünel görüşü için eğitildi) |
| `lane_seg.onnx` (13.3 MB) | — | 💤 önceki şerit modeli (yedek) |
| `2206*.onnx`, `smallmodel.pt` | — | 💤 eski/deneysel |

> Şerit modeli sınıfları `{0: yol, 1: alternatif}` — `lane_test_node` yol sınıfını
> 0 sayar; bu modelde de sıra aynıdır. Farklı model takarken sınıf sırasına dikkat.

## 6. Görev/Harita Dosyaları (`src/araba/missions/`)

| Dosya | İçerik |
|---|---|
| `teknofest_gorev.geojson` | Tam görev (varsayılan) |
| `teknofest_centerlines.geojson` | Sim parkurunun yol ağı (varsayılan) |
| `saha_deneme_gorev.geojson` + `map_islenmis.geojson` | Gerçek saha verisiyle plan testi |
| `map.geojson` | Sahadan alınan ham yol ağı |
| `rota_onizleme.png`, `saha_onizleme.png` | Rota görselleştirmeleri |

---

## 7a. Son Değişiklikler (13 Temmuz — arda lokalizasyon entegrasyonu)

`arda` branch'i merge edildi ve şu şekilde entegre edildi:

- **EKF lokalizasyon hattı varsayılan oldu** (`localization:=ekf`, bkz. §1):
  navsat_transform + EKF + KISS-ICP LiDAR odometrisi. Eski `final_odom_node`
  **yedek olarak duruyor** (`localization:=final_odom`); iki hat da aynı topic'i
  üretir, gerisi hiçbir node değişmez. `ekf.yaml`'daki sabit `yaw_offset` launch'ta
  `spawn_yaw`'a bağlandı (rotanın ortasından spawn'lı testler bozulmasın).
- **Engel tarafı bilinçli olarak alınmadı:** arda'nın commit'i `obstacle_logic`'teki
  road_blocked yakınlık+boşluk kuralımızı `barrier_count > 1`'e geri döndürüyordu
  (tünel REPLAN fırtınası geri gelirdi) ve `decision_arbiter` kaçınma davranışını
  değiştiriyordu — ikisi de bizim sürümde bırakıldı. (Not: arda'nın decision_arbiter
  işaret-yönü düzeltmesi mimari repoda değerlendirilmeye değer.)
- `speed_scale` 10.0 → **5.0'a geri** (dönüş/durma ayarlarımız 5.0'la kanıtlı);
  dünya SDF'indeki `/home/arda/...` yolları bizim workspace'e geri çevrildi.
- **PCD haritası üretildi:** `maps/benim_dunyam.pcd` (116k nokta, ground-truth
  teleport-tarama yöntemi, üretici: `scripts/tools/build_pcd_map.py`);
  `pcl_localization.launch.py` varsayılanı buna çevrildi (`map_file` argümanlı).
- Submodule'lar eklendi: `src/kiss-icp`, `src/pcl_localization_ros2`
  (`.gitmodules` path'leri düzeltildi); `.gitignore` temizlendi; CMake artık
  `maps/`'i kuruyor.
- ⚠️ **EKF hattı henüz canlı turda doğrulanmadı** — ilk koşuda `/localization/odom/final`
  akışı ve rota takibi izlenmeli; sorun olursa `localization:=final_odom` ile devam.

## 7. Önceki Değişiklikler (11–12 Temmuz — 7. tur sonrası paket)

7\. canlı tur rotanın ~%70'ini tamamladı (4 kavşak dönüşü, kırmızıda dur/kalk,
tünel temiz, park girişine varış). Kalan sorunlara yapılan düzeltmeler:

1. **Dönüşlerde şerit değme:** yay apeksinin köşe kesmesi 1.66 m → **0.7 m** ile
   sınırlandı; dışa-kaydırma yaklaşma boyunca rampalı (bkz. §3). Park girişindeki
   tek-çizgi ihlalini de bu düzeltme kapsıyor.
2. **Işık kör penceresi:** bayat-yeşil sürünme kuralı eklendi (bkz. §4) — dönüş
   sırasında ışık görüşten çıktığında kavşak %50 hızla geçilir.
3. **Park mimarisi yeniden yazıldı:** görüş kontrolü artık park girişinden değil
   cep hizasından başlar; kamera sabitleri düzeltildi (eskisi mesafeyi ~3× yanlış
   kestiriyordu); stereo mesafe tercihi + hedef kilidi eklendi (bkz. §3–4).
4. **Tünel REPLAN fırtınası:** road_blocked yakınlık+boşluk kuralı (bkz. §4).
5. **Yeni şerit modeli:** gazeboset yolov8s-seg → `model.onnx`; launch artık bunu
   yükler (tünel içi görüş sorunu için eğitildi).
6. **Dönüş çıkışı erkene alındı:** hizalanma beklemeden yay tamamlanınca kontrol
   şerit takibine devredilir (6. turda geç devir gözlemi).

**Doğrulama durumu:** 1–3 ve 5 numaralı düzeltmeler birim testli (41 kontrol)
ama **henüz canlı turda doğrulanmadı**; 4 ve 6 ile yay dönüşünün kendisi 7. turda
canlı kanıtlandı. Sıradaki iş: kısa görevle park-odaklı canlı koşu, sonra tam tur.

## 8. Bilinen Notlar / Tuzaklar

- `--symlink-install`'a rağmen CMakeLists'te **RENAME ile kurulan** node'lar
  (`camera_perception_node`, `perception_pipeline_node`, `speed_controller_node`,
  `keyboard_teleop`) **kopya** kurulur → bu dosyalarda kod değişikliği
  `colcon build` ister. `.py` uzantılı node'lar ve tüm algoritma modülleri
  gerçek symlink'tir.
- `scripts/deos_algorithms/` import'ları `__init__.py` içindeki meta-path finder
  ile çözülür; **repoya yeni symlink koymayın** (git'te düz metne dönüşüyor).
- Süreç temizliğinde `pkill -f` kendi terminalini de öldürebilir — PID listesiyle
  öldürüp `pgrep` ile doğrulayın.
- Trafik ışığı eklentisi: GREEN 15s → YELLOW 3s → RED 10s → RED_YELLOW 3s;
  `/traffic_light/state` Gazebo-içi topic'tir, ROS'a köprülenmez — araç ışığı
  **kameradan** algılar. Eklenti kurulumu: `models/traffic_light/build_and_install.sh`.
- WSL'de algı FPS düşükse sorun render değil **fizik adımıdır** (`max_step_size`).

## 9. Bağımlılıklar

```bash
sudo apt install python3-colcon-common-extensions ros-jazzy-joint-state-publisher-gui python3-pynput
# EKF lokalizasyon hattı için (bkz. §1):
sudo apt install ros-jazzy-robot-localization ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libpcl-dev
git submodule update --init   # kiss-icp + pcl_localization_ros2
# pcl_localization_ros2 Jazzy'de eski tf2 include'ları yüzünden derlenmez — yamayı uygula:
git -C src/pcl_localization_ros2 apply ../../patches/pcl_localization_ros2_jazzy_includes.patch
# Python: opencv (cv2), cv_bridge, numpy(<2), onnxruntime, pillow, tkinter
# ONNX export gerekirse: pip install --user --break-system-packages "onnx>=1.12,<2" ultralytics
```

> **Ağ notu:** kiss_icp derlemesi bağımlılıklarını (Sophus, oneTBB, robin-map)
> codeload.github.com'dan tarball olarak indirir; bazı ağlarda (ör. bu WSL kurulumu)
> bu host asılı kalıyor ama `git clone` çalışıyor. Takılırsa bağımlılıkları git ile
> indirip cmake'e yerel dizin olarak verin:
>
> ```bash
> D=~/.cache/kiss_icp_deps; mkdir -p $D; cd $D
> git clone --depth 1 --branch 1.24.6    https://github.com/strasdat/Sophus.git
> git clone --depth 1 --branch v1.4.0    https://github.com/Tessil/robin-map.git
> git clone --depth 1 --branch v2022.1.0 https://github.com/uxlfoundation/oneTBB.git
> cd ~/sim2_ws && colcon build --symlink-install --packages-select kiss_icp pcl_localization_ros2 \
>   --cmake-args -DFETCHCONTENT_SOURCE_DIR_SOPHUS=$D/Sophus \
>                -DFETCHCONTENT_SOURCE_DIR_TESSIL=$D/robin-map \
>                -DFETCHCONTENT_SOURCE_DIR_TBB=$D/oneTBB
> ```
