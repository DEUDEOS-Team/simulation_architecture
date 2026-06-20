# Simülasyon-Algoritma Entegrasyon Yol Haritası

Bu belge, `scripts/` altındaki saf Python algoritma modülleri ile Gazebo Harmony simülasyonu arasındaki boşlukları kapatmak için yapılması gerekenleri sıralar.

---

## Durum Özeti

| Katman | Durum | Açıklama |
|--------|-------|----------|
| Algoritma modülleri | ✅ 16 saf Python modül | ROS bağımsız, `deos_algorithms` paketi altında |
| ROS wrapper nodeları | ❌ Hiç yok | Algoritmaları ROS'a bağlayacak düğüm yok |
| CMakeLists.txt | 🔴 KIRIK | Var olmayan 4 scripte referans veriyor |
| Launch dosyası | 🔴 KIRIK | Var olmayan nodeları başlatmaya çalışıyor |
| ONNX inference | ❌ Eksik | ONNX modelleri var ama çalıştıran ROS düğümü yok |
| Sensör köprüleri | ❌ Eksik | Gazebo sensörleri → algoritma formatına dönüşüm yok |
| Kontrol çıkışı | ❌ Eksik | FinalDecision → `/cmd_vel` dönüşümü yok |

---

## Faz 1: Derleme Altyapısını Onar (Kritik)

### 1.1 CMakeLists.txt — Ölü referansları temizle, yeni hedefleri ekle

Şu an CMakeLists.txt'de var olmayan 4 script hedefi var:
- `keyboard_teleop.py` → silindi
- `detection_onnx.py` → silindi
- `lane_visualizer.py` → silindi
- `sign_detection.py` → silindi

**Yapılacak**: Bu satırları kaldır. Yeni ROS wrapper nodeları eklendikçe CMakeLists.txt'ye install hedefi eklenecek.

Ayrıca algoritma modüllerinin kurulumu için:
```cmake
# Python algoritma modüllerini lib/araba/ altına kopyala
install(DIRECTORY
  scripts/sensors
  DESTINATION lib/${PROJECT_NAME}/sensors
)
install(PROGRAMS
  scripts/safety_logic.py
  scripts/obstacle_logic.py
  scripts/traffic_light_logic.py
  scripts/traffic_sign_logic.py
  scripts/parking_logic.py
  scripts/slalom_logic.py
  scripts/decision_arbiter.py
  scripts/perception_fusion.py
  scripts/lane_violation.py
  scripts/geojson_mission_reader.py
  scripts/route_graph.py
  scripts/route_planner.py
  scripts/waypoint_manager.py
  scripts/mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### 1.2 `deos_algorithms` paket yolu sorununu çöz

Tüm modüller `from deos_algorithms.xxx import ...` kullanıyor. İki çözüm var:

**Seçenek A — sys.path (hızlı, geçici)**:
```python
import sys, os
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)
```
Her ROS wrapper nodunda bu satırları ekleyerek `import deos_algorithms.safety_logic` yerine `import safety_logic` kullanılabilir.

**Seçenek B — Paket yapısı (doğru, kalıcı)**:
`scripts/` altına gerçek bir `deos_algorithms/` paketi kur. `__init__.py` zaten var, dizin yapısı uygun. `sys.path` düzenlemesi yeterli.

---

## Faz 2: ROS Wrapper Nodeları (Zorunlu)

Her wrapper node **sadece altyapı görevi görür** — ROS konularını dinler, veriyi dönüştürür, algoritma modülünü çağırır, sonucu yayınlar. Algoritma mantığına dokunmaz.

### 2.1 `camera_perception_node.py` — Kamera → Detection

| Özellik | Detay |
|----------|-------|
| **Abone** | `/camera/image` (`sensor_msgs/Image`) |
| **İş** | ONNX modeli (`detection.onnx`) ile inference → `StereoBbox` listesi |
| **Kullanır** | `perception_fusion.fuse()` (sadece stereo kısmı) |
| **Yayın** | `/perception/sign_dets`, `/perception/light_dets`, `/perception/obstacle_dets` |
| **Referans** | Eski `sign_detection.py` — yeniden yazılacak, `perception_fusion` API'sine uyarlanacak |

### 2.2 `lane_perception_node.py` — Kamera → Şerit

| Özellik | Detay |
|----------|-------|
| **Abone** | `/camera/image` (`sensor_msgs/Image`) |
| **İş** | ONNX modeli (`model.onnx`) ile şerit segmentasyonu → şerit eğrisi noktaları |
| **Yayın** | `/perception/center_pts` (`Float32MultiArray`) |
| **Referans** | Eski `detection_onnx.py` — yeniden yazılacak |

### 2.3 `gps_bridge_node.py` — GPS/IMU/Odom → GpsPosition

| Özellik | Detay |
|----------|-------|
| **Abone** | `/gps/fix` + `/imu/data` + `/odom` |
| **İş** | GPS'ten lat/lon, IMU'dan heading, birleştirip `GpsPosition` üretir |
| **Yayın** | `/localization/position` (özel mesaj veya `Float32MultiArray`) |
| **Kullanılır** | `waypoint_manager.py`, `mission_manager.py` tarafından |

### 2.4 `obstacle_lidar_node.py` — LiDAR → LidarObstacle (opsiyonel)

| Özellik | Detay |
|----------|-------|
| **Abone** | `/lidar/scan` veya `/lidar/scan/points` |
| **İş** | LiDAR nokta bulutundan engel çıkarımı → `LidarObstacle` listesi |
| **Kullanır** | `perception_fusion.fuse()` (lidar kısmı) |
| **Not** | Eğer kamera tespiti yeterliyse ertelenebilir |

### 2.5 `decision_node.py` — Tüm algı → FinalDecision

| Özellik | Detay |
|----------|-------|
| **Abone** | `/perception/sign_dets`, `/perception/light_dets`, `/perception/obstacle_dets`, `/localization/position` |
| **İş** | Tüm karar modüllerini çalıştırır: SafetyLogic, ObstacleLogic, TrafficLightLogic, TrafficSignLogic, SlalomLogic, ParkingLogic → DecisionArbiter.arbitrate() |
| **Yayın** | `/decision/final` (`Float32MultiArray`: [emergency_stop, speed_cap, has_steer, steer_override]) |

### 2.6 `control_bridge_node.py` — FinalDecision → /cmd_vel

| Özellik | Detay |
|----------|-------|
| **Abone** | `/decision/final` + opsiyonel `/perception/center_pts` (şerit takibi için) |
| **İş** | FinalDecision'dan `/cmd_vel` (Twist) üretir. Ackermann formülü: `angular.z = v * tan(δ) / wheel_base` |
| **Yayın** | `/cmd_vel` (`geometry_msgs/Twist`) |

### 2.7 `mission_bridge_node.py` — WaypointManager + MissionManager → yönlendirme

| Özellik | Detay |
|----------|-------|
| **Parametre** | GeoJSON görev dosyası yolu, centerline dosyası yolu |
| **Abone** | `/localization/position` |
| **İş** | `WaypointManager.update()` + `MissionManager.update()` → waypoint takibi |
| **Yayın** | `/mission/waypoint_state` (`Float32MultiArray`) |

---

## Faz 3: Launch Entegrasyonu

### 3.1 `gazebo.launch.py` güncellemesi

Mevcut launch dosyasındaki ölü node referanslarını temizle, yeni nodeları ekle:

```python
# Kaldırılacak:
# - perception_node (detection_onnx) → yerine camera_perception_node + lane_perception_node
# - lane_visualizer → yerine opsiyonel debug node'u

# Eklenecek (TimerAction sıralamasıyla):
# 5.0s → bridge (mevcut)
# 6.0s → camera_perception_node
# 6.5s → lane_perception_node
# 7.0s → gps_bridge_node
# 7.5s → decision_node
# 8.0s → control_bridge_node
# 8.0s → camera_republish (mevcut)
# 10.0s → camera_view (mevcut)
```

### 3.2 `display.launch.py` — değişiklik yok

Sadece URDF görselleştirme için kullanılır, algoritmalarla ilgisi yok.

---

## Faz 4: Test ve Doğrulama

### 4.1 Birim testler (Python unittest/pytest)

Her algoritma modülü ROS bağımsız olduğu için doğrudan test edilebilir:
```bash
cd src/araba/scripts
python3 -c "
from safety_logic import SafetyLogic, Detection
logic = SafetyLogic()
result = logic.analyze([])
print('Safety OK:', result.decision.speed_cap_ratio)
"
```

### 4.2 ROS entegrasyon testi

```bash
# Terminal 1: Simülasyonu başlat
ros2 launch araba gazebo.launch.py

# Terminal 2: Topic'leri kontrol et
ros2 topic list | grep -E "perception|decision|mission|localization"
ros2 topic echo /decision/final

# Terminal 3: Manuel kontrolle algoritma tepkilerini gözlemle
ros2 run araba keyboard_teleop  # (yeniden yazıldıktan sonra)
```

---

## Özet: Öncelik Sıralaması

| Öncelik | Görev | Tahmini Süre | Bağımlılık |
|---------|-------|-------------|------------|
| 🔴 P0 | CMakeLists.txt ölü referansları temizle | 5dk | Yok |
| 🔴 P0 | `deos_algorithms` import yolunu çöz | 10dk | Yok |
| 🔴 P1 | `camera_perception_node.py` yaz | 30dk | P0 |
| 🔴 P1 | `lane_perception_node.py` yaz | 20dk | P0 |
| 🔴 P1 | `gps_bridge_node.py` yaz | 15dk | P0 |
| 🟡 P2 | `decision_node.py` yaz | 25dk | P1 |
| 🟡 P2 | `control_bridge_node.py` yaz | 20dk | P2 |
| 🟡 P2 | `mission_bridge_node.py` yaz | 25dk | P1 |
| 🟡 P3 | `gazebo.launch.py` güncelle | 15dk | P2 |
| 🟠 P4 | `obstacle_lidar_node.py` yaz | 30dk | P1 |
| 🟠 P4 | Debug görselleştirme node'u | 20dk | P1 |
| 🟠 P5 | Birim testler | 30dk | P0 |

---

## Altyapı — Algoritma Ayrımı

Bu yol haritasındaki **tüm yeni kodlar sadece altyapıdır**. Aşağıdaki konulara **kesinlikle dokunulmaz**:

- `safety_logic.py` — tehdit seviyeleri, mesafe eşikleri, tracker mantığı
- `traffic_light_logic.py` — ışık durum yorumlama, sarı bağlamı
- `traffic_sign_logic.py` — levha sınıfları, hız katsayıları, STOP mantığı
- `obstacle_logic.py` — dinamik/statik kaçınma stratejisi
- `slalom_logic.py` — S-weave, merkez weave, geçiş tespiti
- `parking_logic.py` — park fazları, manevra mantığı
- `decision_arbiter.py` — öncelik sıralaması, birleştirme mantığı
- `perception_fusion.py` — sınıflandırma eşlemeleri
- `route_graph.py`, `route_planner.py` — Dijkstra, graph mantığı
- `waypoint_manager.py` — waypoint takip mantığı
- `mission_manager.py` — görev akış mantığı
