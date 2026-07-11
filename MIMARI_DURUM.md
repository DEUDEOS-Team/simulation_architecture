# Simülasyon Mimarisi — Detaylı Durum Özeti

> **Tarih:** 5 Temmuz 2026  
> **ROS 2:** Jazzy | **Gazebo:** Harmonic (`gz-sim8`) | **Paket:** `araba`  
> **Workspace:** `~/simulation_architecture` (`sim2_ws` değil!)

---

## 1. Launch Sırası ve Aktif Node'lar (`gazebo.launch.py`)

| # | Zaman | Node | Durum | Açıklama |
|---|-------|------|-------|----------|
| 1 | 0s | **Gazebo** (`ros_gz_sim`) | ✅ AKTİF | `benim_dunyam.sdf` dünyası, `-r` ile otomatik başlar |
| 2 | 0s | **robot_state_publisher** | ✅ AKTİF | URDF'ten `robot_description` yayını |
| 3 | 0s | **ros_gz_sim/create** | ✅ AKTİF | Aracı spawn eder (x=45.31, y=16, z=0.51, Y=1.58) |
| 4 | 0s | **rviz2** | ✅ AKTİF | `lidar_view.rviz` konfigürasyonu ile |
| 5 | 5s | **ros_gz_bridge** | ✅ AKTİF | 10 topic köprüsü (aşağıdaki tabloda) |
| 6 | 6s | **camera_perception_node** | ✅ AKTİF | `detection.onnx` ile 29 sınıf nesne tespiti, `show_window=True` |
| 7 | 6.5s | **lane_test_node** | ✅ AKTİF | `lane_seg.onnx` ile şerit segmentasyonu, `show_window=True` |
| 8 | 7s | **perception_pipeline_node** | ✅ AKTİF | Algı füzyonu + tkinter dashboard, `show_dashboard=True` |
| 9 | 7.5s | **autonomous_control_node** | ✅ AKTİF | Şerit takibi P-kontrolcü → `/cmd_vel` |

### Launch'ta Yorum Satırı Olan (Devre Dışı)

| Node | Durum | Açıklama |
|------|-------|----------|
| **speed_controller_node** | ❌ YORUM SATIRI | Trafik ışığı/levha/engel → hız sınırlama. Launch'ta yok, manuel `ros2 run` ile başlatılabilir |
| **keyboard_teleop** | ❌ KALDIRILDI | Eski manuel kontrol, artık launch'ta yok |

---

## 2. ROS ↔ Gazebo Köprüsü (10 Topic)

| Topic | Mesaj Türü | Yön | Köprü Durumu |
|-------|-----------|-----|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Gz→ROS | ✅ |
| `/lidar/scan` | `sensor_msgs/LaserScan` | Gz→ROS | ✅ |
| `/lidar/scan/points` | `sensor_msgs/PointCloud2` | Gz→ROS | ✅ |
| `/camera/image` | `sensor_msgs/Image` | Gz→ROS | ✅ |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gz→ROS | ✅ |
| `/imu/data` | `sensor_msgs/Imu` | Gz→ROS | ✅ |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gz→ROS | ✅ |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→Gz | ✅ |
| `/odom` | `nav_msgs/Odometry` | Gz→ROS | ✅ |
| `/model/araba/joint_state` → `/joint_states` | `sensor_msgs/JointState` | Gz→ROS | ✅ |

---

## 3. Ana Veri Akışı (Şu Anda Çalışan)

```
┌─────────────────────────────────────────────────────────────────────┐
│ Gazebo Kamera (1280x720)                                            │
│   │                                                                 │
│   ├──► /camera/image ──► camera_perception_node                     │
│   │     detection.onnx (29 sınıf)                                   │
│   │     │                                                           │
│   │     └──► /perception/detections (Float32MultiArray)             │
│   │           [x1,y1,x2,y2,score,cls_id]*N                          │
│   │           │                                                     │
│   │           └──► perception_pipeline_node                         │
│   │                 perception_fusion.fuse()                        │
│   │                 TrafikLightLogic / TrafficSignLogic /           │
│   │                 ObstacleLogic güncellemesi                      │
│   │                 │                                               │
│   │                 ├──► /perception/traffic_light_state (String)   │
│   │                 ├──► /perception/traffic_sign_state (String)    │
│   │                 ├──► /perception/obstacle_state (String)        │
│   │                 └──► tkinter Dashboard (GUI)                    │
│   │                                                                 │
│   └──► /camera/image ──► lane_test_node                             │
│         lane_seg.onnx (şerit segmentasyonu)                         │
│         │                                                           │
│         └──► /perception/center_pts (Float32MultiArray)             │
│               [(cx,y), (cx,y), ...] sıralı orta noktalar            │
│               │                                                     │
│               └──► autonomous_control_node                          │
│                     P-kontrolcü (KP=0.8, max_angle=30°)             │
│                     │                                               │
│                     └──► /cmd_vel (Twist) ──► Gazebo Ackermann      │
└─────────────────────────────────────────────────────────────────────┘
```

### ⚠️ Kritik Kopukluk

**perception_pipeline_node'un çıktıları (`/perception/traffic_light_state`, `/perception/traffic_sign_state`, `/perception/obstacle_state`) hiçbir node tarafından TÜKETİLMİYOR.**

`speed_controller_node` bu topic'leri dinleyip `/cmd_vel`'e hız sınırlaması uygulamak üzere yazılmış, ancak launch'ta **yorum satırında**. Bu yüzden:

- Trafik ışığı **algılanıyor** ama aracı **durdurmuyor** 🚦❌
- Trafik levhaları **algılanıyor** ama hıza **etki etmiyor** 🛑❌
- Engeller **algılanıyor** ama acil duruş **yok** 🚧❌
- Otonom kontrol **sadece şerit takibi** yapıyor

---

## 4. ONNX Model Envanteri

| Model | Boyut | Kullanan Node | Durum |
|-------|-------|---------------|-------|
| `detection.onnx` | 44.8 MB | `camera_perception_node` | ✅ AKTİF — 29 sınıf nesne tespiti |
| `lane_seg.onnx` | 13.3 MB | `lane_test_node` | ✅ AKTİF — şerit segmentasyonu |
| `2206_2model.onnx` | 13.3 MB | `lane_yolo_node` | 💤 YEDEK — ultralytics YOLO alternatifi |
| `2206model.onnx` | 13.3 MB | (kullanılmıyor) | 💤 YEDEK — eski model |
| `model.onnx` | 47.4 MB | (kullanılmıyor) | 💤 YEDEK — eski şerit modeli |
| `smallmodel.pt` | 47.5 MB | (kullanılmıyor) | 💤 PyTorch model, ONNX değil |

> **Not:** ONNX modeller `scripts/onnx/` altındadır. CMakeLists.txt bunları `share/araba/models/onnx/` altına symlink olarak kurar. `models/onnx/` fiziksel dizini boştur — her şey symlink ile çalışır.

---

## 5. Algoritma Modülleri (16 Saf Python Modülü)

### ROS Entegre Edilmiş (Aktif)

| Modül | Kim Kullanıyor | Görev |
|-------|---------------|-------|
| `perception_fusion.py` | `camera_perception_node` | ONNX çıktısını `StereoBbox` listesine dönüştürme |
| `obstacle_logic.py` | `perception_pipeline_node` | Engel davranış analizi (emergency stop, kaçınma) |
| `traffic_light_logic.py` | `perception_pipeline_node` | Trafik ışığı durum yorumlama |
| `traffic_sign_logic.py` | `perception_pipeline_node` | Trafik levhası hız sınırlaması |
| `decision_arbiter.py` | `speed_controller_node` | Karar birleştirme (acil duruş önceliği) |
| `sensors/types.py` | `camera_perception_node`, `perception_pipeline_node` | `StereoBbox`, `ImuSample` veri tipleri |

### ROS Entegre EDİLMEMİŞ (Boşta)

| Modül | Tahmini Görev | Eksik Entegrasyon |
|-------|--------------|-------------------|
| `safety_logic.py` | Tehdit seviyesi analizi, mesafe takibi | GPS/IMU verisi bağlı değil |
| `parking_logic.py` | Park manevrası (fazlar, yönlendirme) | Görev sistemi bağlı değil |
| `slalom_logic.py` | Slalom geçişi (S-weave, merkez) | Görev sistemi bağlı değil |
| `lane_violation.py` | Şerit ihlali tespiti | Şerit verisi bağlı değil |
| `geojson_mission_reader.py` | GeoJSON görev dosyası okuma | Dosya yolu parametresi yok |
| `route_graph.py` | Yol grafı (centerline → Dijkstra) | Centerline GeoJSON'u yok |
| `route_planner.py` | Görev planı → rota | Graph + MissionPlan bağlı değil |
| `waypoint_manager.py` | GPS waypoint takibi | GPS topic'i bağlı değil |
| `mission_manager.py` | Görev akış yöneticisi | GPS + MissionPlan bağlı değil |

### ⚠️ `deos_algorithms` İçe Aktarma Sorunu

Tüm algoritma modülleri `from deos_algorithms.xxx import ...` şeklinde birbirine referans verir. Bu isimle gerçek bir Python paketi yoktur; `scripts/deos_algorithms/` dizini symlink'ler barındırır. ROS wrapper node'ları **kendi başlarına** çalışmak için `sys.path` düzenlemesi yapar:

```python
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)
```

Bu geçici bir çözümdür. `deos_algorithms` gerçek bir paket olarak kurulursa daha temiz olur.

---

## 6. Node Detayları

### 6.1 `camera_perception_node.py` — Nesne Tespiti ✅

| Özellik | Değer |
|----------|-------|
| Girdi | `/camera/image` (`sensor_msgs/Image`) |
| Model | `detection.onnx` (ONNX Runtime, CPU) |
| Çıktı | `/perception/detections` (`Float32MultiArray`) |
| Parametreler | `conf_threshold=0.40`, `process_every_n=3`, `show_window=True` |
| Altyapı | Her 3 karede bir inference, aralarda son tespiti yayınlar |
| Sınıflar | 29 sınıf: trafik levhaları, ışıklar, koni, bariyer, yaya, tünel vb. |

### 6.2 `lane_test_node.py` — Şerit Segmentasyonu ✅

| Özellik | Değer |
|----------|-------|
| Girdi | `/camera/image` |
| Model | `lane_seg.onnx` (ONNX Runtime, CUDA dene→CPU fallback) |
| Çıktı | `/perception/center_pts` (`Float32MultiArray`) — (x,y) çiftleri |
| Yöntem | ROI maskeleme, alt-bileşen filtreleme, satır-medyanı + EMA yumuşatma |
| Parametreler | `ROI=0.6-0.88`, `SMOOTH_ALPHA=0.35`, `proc_width=0` (orijinal) |
| HUD | Gerçek FPS + infer/post/draw süreleri + GPU/CPU rozeti |

### 6.3 `lane_yolo_node.py` — Alternatif Şerit (ultralytics YOLO) 💤

| Özellik | Değer |
|----------|-------|
| Durum | **Launch'ta YOK** — manuel başlatılabilir |
| Girdi | `/camera/image` |
| Model | `2206_2model.onnx` (ultralytics YOLO, task='segment') |
| Çıktı | `/perception/center_pts` (aynı topic — `lane_test_node` ile değiştirilebilir) |
| Avantaj | `masks.xy` poligonları orijinal koordinatlarda gelir, letterbox kayması yok |
| Dezavantaj | CPU'da çalışır, GPU için cu12-uyumlu torch gerekir (cu13 çakışıyor) |
| Kullanım | `ros2 run araba lane_yolo_node.py` veya ayrı launch dosyası |

### 6.4 `perception_pipeline_node.py` — Algı Füzyonu + Dashboard ✅

| Özellik | Değer |
|----------|-------|
| Girdi | `/perception/detections`, `/camera/image` (önizleme için) |
| Çıktı | `/perception/traffic_light_state`, `/perception/traffic_sign_state`, `/perception/obstacle_state` (String, KV format) |
| Dashboard | tkinter, koyu tema, 4 bölümlü canlı panel (10Hz yenileme) |
| Parametreler | `show_dashboard=True`, `state_publish_hz=5.0` |

### 6.5 `autonomous_control_node.py` — Otonom Şerit Takibi ✅

| Özellik | Değer |
|----------|-------|
| Girdi | `/perception/center_pts` |
| Çıktı | `/cmd_vel` (`Twist`), `/control/target_angle` (`Float32`), `/control/throttle` (`Float32`) |
| Kontrolcü | P-kontrolcü (KP=0.8), EMA yumuşatma (α=0.3) |
| Parametreler | `cam_width=1280`, `speed_scale=4.0`, `wheel_base=1.675`, `steer_limit=30°` |
| Hız profili | Düz yolda ~2.0 m/s (throttle=0.5×4.0), virajda ~1.4 m/s (throttle=0.35×4.0) |
| Şerit kaybı | 1.0s tolerans, sonra durur |

### 6.6 `speed_controller_node.py` — Hız Kontrolü ❌ DEVRE DIŞI

| Özellik | Değer |
|----------|-------|
| Durum | **Launch'ta YOK** — `gazebo.launch.py` içinde yorum satırı |
| Girdi | `/cmd_vel_raw` (kullanıcı), `/perception/traffic_light_state`, `/perception/traffic_sign_state`, `/perception/obstacle_state` |
| Çıktı | `/cmd_vel` (hız sınırlı) |
| Algoritma | `DecisionArbiter.arbitrate()` ile acil duruş + hız sınırlama |
| Not | Launch'a eklendiğinde, `autonomous_control_node`'un `/cmd_vel` yayınıyla **çakışır** — topic remapping gerekir |

### 6.7 `keyboard_teleop` — Manuel Kontrol ❌ KALDIRILDI

Launch'tan tamamen çıkarıldı. Otonom sürüş tek kontrol kaynağı.

---

## 7. Dosya ve Dizin Yapısı

```
simulation_architecture/
├── src/araba/                          # Tek ROS paketi
│   ├── CMakeLists.txt                  # ✅ Temiz, ölü referanslar temizlenmiş
│   ├── package.xml
│   ├── urdf/araba.urdf                 # ✅ 148 kg, Ackermann direksiyon
│   ├── worlds/benim_dunyam.sdf         # ✅ Teknofest temalı dünya
│   ├── config/
│   │   ├── lidar_view.rviz             # ✅ RViz konfigürasyonu
│   │   └── gz_ros2_control.yaml        # ⚠️ ros2_control (kullanılıyor mu?)
│   ├── launch/
│   │   ├── gazebo.launch.py            # ✅ ANA launch (ROS 2 Python)
│   │   ├── gazebo.launch               # 💀 ROS 1 XML — ÖLÜ
│   │   ├── display.launch.py           # ✅ URDF görüntüleme (ROS 2)
│   │   └── display.launch              # 💀 ROS 1 XML — ÖLÜ
│   ├── models/                         # Gazebo model dizinleri (29 adet)
│   │   ├── traffic_light/              # ✅ C++ eklentili
│   │   ├── traffic_cone/               # ✅ Statik
│   │   ├── walking_actor/              # ✅ Yürüyen yaya aktörü
│   │   └── ... (levhalar, bariyerler)
│   └── scripts/
│       ├── camera_perception_node.py   # ✅ ROS wrapper
│       ├── lane_test_node.py           # ✅ ROS wrapper (DENEME — .py uzantılı)
│       ├── lane_yolo_node.py           # 💤 Alternatif (DENEME — .py uzantılı)
│       ├── perception_pipeline_node.py # ✅ ROS wrapper
│       ├── autonomous_control_node.py  # ✅ ROS wrapper (DENEME — .py uzantılı)
│       ├── speed_controller_node.py    # ❌ ROS wrapper (launch'ta yok)
│       ├── safety_logic.py             # 💤 Saf algoritma (ROS bağlı değil)
│       ├── obstacle_logic.py           # ✅ perception_pipeline'da kullanılıyor
│       ├── traffic_light_logic.py      # ✅ perception_pipeline'da kullanılıyor
│       ├── traffic_sign_logic.py       # ✅ perception_pipeline'da kullanılıyor
│       ├── parking_logic.py            # 💤 Saf algoritma (ROS bağlı değil)
│       ├── slalom_logic.py             # 💤 Saf algoritma (ROS bağlı değil)
│       ├── decision_arbiter.py         # ⚠️ speed_controller'da kullanılıyor
│       ├── perception_fusion.py        # ✅ camera_perception + pipeline'da
│       ├── lane_violation.py           # 💤 Saf algoritma (ROS bağlı değil)
│       ├── geojson_mission_reader.py   # 💤 Saf algoritma (ROS bağlı değil)
│       ├── route_graph.py              # 💤 Saf algoritma (ROS bağlı değil)
│       ├── route_planner.py            # 💤 Saf algoritma (ROS bağlı değil)
│       ├── waypoint_manager.py         # 💤 Saf algoritma (ROS bağlı değil)
│       ├── mission_manager.py          # 💤 Saf algoritma (ROS bağlı değil)
│       ├── deos_algorithms/            # Symlink çiftliği (paket taklidi)
│       ├── sensors/types.py            # Veri tipleri
│       ├── onnx/                       # ONNX modeller (orijinal konum)
│       ├── INTEGRATION_ROADMAP.md      # Entegrasyon yol haritası
│       └── README_ALGORITHMS.md        # Algoritma dokümantasyonu
├── otonom_projem/                      # Paylaşılan modeller (COLCON_IGNORE)
│   └── models/                         # traffic_light vb. modeller
├── yedek_arabalar/                     # Eski sürümler (COLCON_IGNORE)
│   ├── araba_v2/                       # ament (ROS 2)
│   └── araba_v3/                       # catkin (ROS 1) — derleme hatası verir
└── install/                            # Colcon build çıktısı
```

---

## 8. Trafik Işığı Eklentisi

| Özellik | Değer |
|----------|-------|
| Durum | ✅ ÇALIŞIYOR |
| Dosya | `models/traffic_light/TrafficLightPlugin.cc` |
| Döngü | GREEN (15s) → YELLOW (3s) → RED (10s) → RED_YELLOW (3s) |
| Yayın | `/traffic_light/state` (`gz.msgs.StringMsg`) — Gazebo içi, ROS'a bridge edilmez |
| Kurulum | `models/traffic_light/build_and_install.sh` → `~/.gz/sim/plugins/` |

> ⚠️ `/traffic_light/state` şu anda ROS köprüsüne ekli DEĞİL. `camera_perception_node` ışığı kameradan görerek algılıyor, Gazebo eklentisinden doğrudan okumuyor.

---

## 9. Çalışan / Çalışmayan Özeti

### ✅ Çalışanlar (Tam Entegre)

| Bileşen | Durum |
|---------|-------|
| Gazebo simülasyonu (dünya + fizik) | ✅ |
| ROS-Gazebo köprüsü (10 topic) | ✅ |
| Kamera → ONNX nesne tespiti (29 sınıf) | ✅ |
| Kamera → ONNX şerit segmentasyonu | ✅ |
| Algı füzyonu (perception pipeline) | ✅ |
| Trafik ışığı/levha/engel durum yayını | ✅ |
| tkinter Dashboard (canlı gösterge paneli) | ✅ |
| Şerit takibi P-kontrolcü → `/cmd_vel` | ✅ |
| Araç sürüşü (Ackermann) | ✅ |
| Trafik ışığı C++ eklentisi (Gazebo içi) | ✅ |
| RViz2 (LiDAR görüntüleme) | ✅ |

### ❌ Eksik / Kopuk

| Bileşen | Eksiklik | Etki |
|---------|---------|------|
| **Hız kontrolü** | `speed_controller_node` launch'ta yok | Trafik ışığı, levha ve engeller aracı durdurmuyor |
| **GPS entegrasyonu** | `/gps/fix` bridge'de var ama dinleyen node yok | Waypoint takibi ve görev sistemi çalışmıyor |
| **IMU entegrasyonu** | `/imu/data` bridge'de var ama dinleyen node yok | Heading bilgisi eksik |
| **LiDAR engel** | `/lidar/scan` bridge'de var ama dinleyen node yok | LiDAR tabanlı engel kaçınma yok |
| **Görev sistemi** | 5 modül (mission, route, waypoint) tamamen bağlantısız | Tech özellikler (PARK, SLALOM, PICKUP) çalışmıyor |
| **`/cmd_vel` çakışması** | `speed_controller_node` aktif edilirse, `autonomous_control_node` ile aynı topic'i yayınlar | Topic remapping şart |

### 💤 Alternatif / Yedek

| Bileşen | Açıklama |
|---------|----------|
| `lane_yolo_node.py` | ultralytics YOLO tabanlı alternatif şerit takip, `lane_test_node` yerine kullanılabilir |
| `2206model.onnx` | Eski şerit modeli |
| `model.onnx` | Eski ONNX modeli (47 MB) |
| `smallmodel.pt` | PyTorch formatında model |

---

## 10. Entegrasyon Yol Haritası (INTEGRATION_ROADMAP.md'den)

Altyapının tamamlanması için eksik ROS wrapper node'ları:

| Öncelik | Node | Görev | Tahmini Süre |
|---------|------|-------|-------------|
| 🔴 P0 | — | `deos_algorithms` import yolunu kalıcı çöz | 10 dk |
| 🔴 P1 | `gps_bridge_node.py` | GPS/IMU/Odom → `GpsPosition` | 15 dk |
| 🔴 P1 | `control_bridge_node.py` | FinalDecision + şerit → `/cmd_vel` (birleşik) | 20 dk |
| 🟡 P2 | `decision_node.py` | Tüm algı → `DecisionArbiter.arbitrate()` | 25 dk |
| 🟡 P2 | `mission_bridge_node.py` | GPS → WaypointManager + MissionManager | 25 dk |
| 🟡 P3 | `gazebo.launch.py` güncelle | Yeni node'ları launch'a ekle + topic remapping | 15 dk |
| 🟠 P4 | `obstacle_lidar_node.py` | LiDAR → engel tespiti | 30 dk |

---

## 11. Workspace Notları

- **Aktif workspace:** `~/simulation_architecture`
- `.bashrc` otomatik olarak `~/simulation_architecture/install/setup.bash`'i source eder
- Eski workspace `~/sim_ws` artık kullanılmıyor
- `--symlink-install` sayesinde Python/launch değişiklikleri yeniden derleme gerektirmez
- `yedek_arabalar/araba_v2` ve `araba_v3` `COLCON_IGNORE` ile colcon'dan hariç tutulur
- `otonom_projem/` de `COLCON_IGNORE` içerir
