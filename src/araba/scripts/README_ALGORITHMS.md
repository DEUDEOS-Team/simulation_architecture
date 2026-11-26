# Algoritma Modülleri — DEOS Otonom Sürüş Sistemi

Bu dizin, **gerçek araç üzerinde kullanılmak üzere** yazılmış saf Python algoritma modüllerini içerir. Tüm modüller **ROS bağımsızdır** — hiçbir Publisher, Subscriber veya Node içermez. Bu sayede birim testine ve farklı platformlara (simülasyon / gerçek araç) kolayca taşınabilir.

> ⚠️ **Önemli**: Bu modüller `deos_algorithms` paket adı altında birbirine `from deos_algorithms.xxx import ...` şeklinde referans verir. Simülasyon ortamında çalışmak için `sys.path` düzenlemesi veya uygun bir kurulum gerekir.

---

## Modül Haritası

```
┌─────────────────────────────────────────────────────────────────┐
│                      GÖREV & ROTA KATMANI                        │
│  geojson_mission_reader.py  ←→  route_graph.py                  │
│  route_planner.py           ←→  waypoint_manager.py             │
│  mission_manager.py                                              │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────────┐
│                      ALGI KATMANI                                │
│  perception_fusion.py     ←→  sensors/types.py                  │
│  (stereo/lidar/imu → Detection listelerine dönüşüm)             │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────────┐
│                      KARAR KATMANI                               │
│  safety_logic.py          →  obstacle_logic.py                  │
│  traffic_light_logic.py   →  slalom_logic.py                    │
│  traffic_sign_logic.py    →  parking_logic.py                   │
│  lane_violation.py        →  decision_arbiter.py  (birleştirici)│
└─────────────────────────────────────────────────────────────────┘
```

---

## Modül Bazında Detaylı Açıklama

### 1. `geojson_mission_reader.py` — Görev Dosyası Okuyucu
| Özellik | Değer |
|----------|-------|
| **Amaç** | GeoJSON FeatureCollection formatındaki görev dosyasını okur, `MissionPlan` üretir |
| **Ana sınıf** | `GeoJsonMissionReader`, `MissionPlan`, `MissionPoint` |
| **Görev tipleri** | `START`, `CHECKPOINT`, `STOP`, `PARK`, `PARK_ENTRY`, `PICKUP`, `DROPOFF` |
| **Girdi** | GeoJSON dosya yolu veya string |
| **Çıktı** | `MissionPlan` (sıralı `MissionPoint` listesi + metadata) |
| **Özellik** | Türkçe/İngilizce eş anlamlı görev isimleri, `name` alanından `task` çıkarımı |

### 2. `route_graph.py` — Yol Grafı ve Rota Bulma
| Özellik | Değer |
|----------|-------|
| **Amaç** | QGIS centerline GeoJSON'undan yol grafı oluşturur, Dijkstra ile en kısa yol bulur |
| **Ana fonksiyonlar** | `build_graph_from_centerlines_geojson()`, `dijkstra()`, `dijkstra_mandatory_tunnel()` |
| **Girdi** | Centerline GeoJSON (LineString'ler), başlangıç/hedef düğüm ID'leri |
| **Çıktı** | `RouteGraph`, düğüm ID yolu (list[int]) |
| **Özellik** | Haversine mesafe hesabı, tünel zorunlu rota, `oneway` desteği, `blocked` kenar filtreleme |

### 3. `route_planner.py` — Görev Planına Göre Rota Üretimi
| Özellik | Değer |
|----------|-------|
| **Amaç** | MissionPlan noktalarını graph'a snap edip Dijkstra ile rota üretir |
| **Ana fonksiyonlar** | `route_mission_plan_via_graph()`, `route_mission_plan_without_graph()`, `route_remaining_mission_via_graph()` |
| **Girdi** | `MissionPlan` + `RouteGraph` (opsiyonel) |
| **Çıktı** | Yeni `MissionPlan` (graph koordinatlarıyla zenginleştirilmiş) |
| **Özellik** | Graph olmadan en yakın komşu sıralama, tünel tercihi, replanning desteği |

### 4. `waypoint_manager.py` — GPS Tabanlı Waypoint Takibi
| Özellik | Değer |
|----------|-------|
| **Amaç** | Araç GPS konumuna göre sıradaki waypoint'e yönlendirme üretir |
| **Ana sınıf** | `WaypointManager`, `WaypointState`, `GpsPosition` |
| **Girdi** | `GpsPosition` (lat, lon, heading_deg) |
| **Çıktı** | `WaypointState` (steering_ref, distance_to_wp, cross_track_error, arrived, speed_limit_ratio) |
| **Özellik** | Haversine mesafe, forward azimuth, cross-track error (XTE), otomatik ilerleme |

### 5. `mission_manager.py` — Görev Davranış Yöneticisi
| Özellik | Değer |
|----------|-------|
| **Amaç** | Görev-temelli davranışları yönetir: PICKUP/DROPOFF bekleme, PARK modu, ilerleme |
| **Ana sınıf** | `MissionManager`, `MissionDecision` |
| **Girdi** | `GpsPosition` + `now_s` zaman damgası |
| **Çıktı** | `WaypointState` + `MissionDecision` (hold, park_mode, park_remaining_s) |
| **Özellik** | 15sn PICKUP/DROPOFF duraklama, 3dk park timeout, park giriş tetikleyici |

### 6. `safety_logic.py` — Güvenlik / Engel Analizi
| Özellik | Değer |
|----------|-------|
| **Amaç** | BBox tespitlerinden çarpışma riski analizi yapar |
| **Ana sınıf** | `SafetyLogic`, `Detection`, `SafetyDecision`, `ThreatLevel`, `SimpleTracker` |
| **Girdi** | `Detection` listesi (x1,y1,x2,y2, class_name, confidence) |
| **Çıktı** | `SafetyAnalysis` → `SafetyDecision` (emergency_stop, speed_cap_ratio, threat_level) |
| **Özellik** | Koridor modeli (1.2m yarı genişlik, 20m ileri), mesafe kestirimi (pinhole), tracker (confirm/forget), 3 seviye tehdit (EMERGENCY/HARD_SOFT/SOFT_SLOW) |

### 7. `obstacle_logic.py` — Engel Davranış Katmanı
| Özellik | Değer |
|----------|-------|
| **Amaç** | `SafetyLogic` üstüne inşa edilmiş, dinamik/statik engel ayırımı yapan davranış katmanı |
| **Ana sınıf** | `ObstacleLogic`, `ObstacleState`, `ObstacleDetection` |
| **Girdi** | `ObstacleDetection` listesi (kind, confidence, bbox_px, mesafe/lateral opsiyonel) |
| **Çıktı** | `ObstacleState` (emergency_stop, speed_cap, avoidance_direction, behavior_mode) |
| **Özellik** | Yaya → DYNAMIC_WAIT (dur/bekle/kaçın), Koni/Bariyer → STATIC_AVOID (yön commit mekanizması ile zigzag azaltma), iki aşamalı dinamik kaçınma |

### 8. `traffic_light_logic.py` — Trafik Işığı Kısıtları
| Özellik | Değer |
|----------|-------|
| **Amaç** | Trafik ışığı tespitlerini hız kısıtlarına dönüştürür |
| **Ana sınıf** | `TrafficLightLogic`, `LightDetection`, `LightColor`, `LightState` |
| **Girdi** | `LightDetection` listesi (color, confidence, bbox_px) |
| **Çıktı** | `LightState` (must_stop, speed_cap_ratio, active_color, reason) |
| **Özellik** | Bellek tabanlı çoklu kare onayı, sarı ışık bağlam duyarlı (kırmızıdan sonra sarı ≠ yeşilden sonra sarı), durma tespiti (vehicle_speed_mps) |

### 9. `traffic_sign_logic.py` — Trafik Levhası Kısıtları
| Özellik | Değer |
|----------|-------|
| **Amaç** | Trafik levhası tespitlerini hız/yön kısıtlarına dönüştürür |
| **Ana sınıf** | `TrafficSignLogic`, `SignDetection`, `SignClass`, `SignState` |
| **Girdi** | `SignDetection` listesi (class_name, confidence, bbox_px) |
| **Çıktı** | `SignState` (must_stop_soon, speed_cap_ratio, turn_permissions, reasons) |
| **Özellik** | 26+ Türkçe levha sınıfı, STOP'ta 5sn zorunlu duruş, TUNNEL %70 hız, YAYA GEÇİDİ %50 hız, `TurnPermissions` yön kısıtlamaları |

### 10. `slalom_logic.py` — Koni Slalom Manevrası
| Özellik | Değer |
|----------|-------|
| **Amaç** | Koni engelleri arasında slalom manevrasını yönetir |
| **Ana sınıf** | `SlalomLogic`, `SlalomState` |
| **Girdi** | `ObstacleDetection` listesi (yalnızca `ObstacleKind.CONE` olanlar) |
| **Çıktı** | `SlalomState` (aktif, steering [-1..1], hiz_katsayisi, gecilen_koni, faz) |
| **Özellik** | Başlangıç kapısı tespiti, S-weave (tek taraflı konilerde zigzag), merkez weave, 15 frame koni kaybı sonrası otomatik bitiş, 3 kademe hız (uzak/orta/yakın) |

### 11. `parking_logic.py` — Park Manevrası
| Özellik | Değer |
|----------|-------|
| **Amaç** | Park yeri tespitlerinden park manevrası üretir |
| **Ana sınıf** | `ParkingLogic`, `ParkingDetection`, `ParkState` |
| **Girdi** | `ParkingDetection` listesi (bbox_px, confidence, parking_allowed, park_type) |
| **Çıktı** | `ParkState` (phase, steering, speed_ratio, reverse, complete) |
| **Özellik** | 4 fazlı durum makinesi (YAKLAŞMA → HİZALANMA → MANEVRA → PARK_EDİLDİ), park yasak tabelası filtreleme, arama modu (tarama steer), dik park geri geri manevra |

### 12. `lane_violation.py` — Şerit İhlali Takibi
| Özellik | Değer |
|----------|-------|
| **Amaç** | Şerit dışına çıkma ihlallerini sayar |
| **Ana sınıf** | `LaneViolationTracker` |
| **Girdi** | `LaneBounds` + `now_s` + `outside` bool + opsiyonel `speed_mps` |
| **Çıktı** | İhlal sayacı (`violation_count`) |
| **Özellik** | 10'ar saniyelik bucket sayımı, dururken tek ihlal kuralı, `wheels_outside_lane()` geometrik kontrol |

### 13. `perception_fusion.py` — Algı Füzyon Katmanı
| Özellik | Değer |
|----------|-------|
| **Amaç** | Stereo/LiDAR/IMU sensör çıktılarını algoritma modüllerinin beklediği Detection formatına dönüştürür |
| **Ana fonksiyon** | `fuse()`, `parking_detections_from_signs()` |
| **Girdi** | `StereoBbox[]` + `LidarObstacle[]` + `ImuSample` |
| **Çıktı** | `PerceptionFrame` (sign_dets, light_dets, obstacle_dets, imu) |
| **Özellik** | `classify_color()` ve `classify_obstacle()` ile otomatik sınıflandırma, tabela→park adayı dönüşümü |

### 14. `decision_arbiter.py` — Nihai Karar Birleştirici
| Özellik | Değer |
|----------|-------|
| **Amaç** | Tüm karar modüllerinin çıktılarını öncelik sırasına göre tek bir kararda birleştirir |
| **Ana sınıf** | `DecisionArbiter`, `Candidate`, `FinalDecision`, `LaneBounds` |
| **Girdi** | `Candidate[]` (her modülden bir aday) + opsiyonel `LaneBounds` |
| **Çıktı** | `FinalDecision` (emergency_stop, speed_cap, steer_override, reasons) |
| **Öncelik** | 1)Acil durdurma → 2)Çarpışma önleme → 3)Şerit → 4)Işık → 5)Levha → 6)Park → 7)Slalom |

### 15. `sensors/types.py` — Sensör Veri Tipleri
| Özellik | Değer |
|----------|-------|
| **Amaç** | Algı boru hattının kullandığı ortak veri yapıları |
| **Sınıflar** | `StereoBbox` (kamera tespiti), `LidarObstacle` (LiDAR tespiti), `ImuSample` (IMU verisi) |
| **Özellik** | Tümü frozen dataclass, opsiyonel mesafe/lateral alanları |

### 16. `onnx/` — ONNX Model Dosyaları
| Dosya | Amaç |
|-------|------|
| `model.onnx` (~47MB) | Şerit segmentasyonu + obje tespiti YOLO modeli |
| `detection.onnx` (~45MB) | Trafik levhası/ışığı/engel tespit modeli (29 sınıf) |
| `smallmodel.pt` (~47MB) | PyTorch kaynak checkpoint |
| `convert_to_onnx.py` | `.pt` → `.onnx` dönüşüm scripti |

---

## Modüller Arası Bağımlılık Grafiği

```
geojson_mission_reader.py  (bağımsız)
route_graph.py             (bağımsız)
sensors/types.py           (bağımsız)

route_planner.py           → geojson_mission_reader, route_graph
waypoint_manager.py        → geojson_mission_reader
mission_manager.py         → geojson_mission_reader, waypoint_manager

safety_logic.py            (bağımsız — numpy)
traffic_light_logic.py     (bağımsız)
traffic_sign_logic.py      (bağımsız)

obstacle_logic.py          → safety_logic
slalom_logic.py            → obstacle_logic, safety_logic
parking_logic.py           (bağımsız)
lane_violation.py          → decision_arbiter (LaneBounds)

perception_fusion.py       → obstacle_logic, parking_logic, sensors/types,
                              traffic_light_logic, traffic_sign_logic

decision_arbiter.py        (bağımsız — tüm modüllerin çıktısını birleştirir)
```

---

## Veri Akışı (Gerçek Araç Hedeflenen)

```
[Kamera] ──StereoBbox──→ perception_fusion.py ──┐
[LiDAR]  ──LidarObstacle→ perception_fusion.py ──┤
[IMU]    ──ImuSample───→ perception_fusion.py ──┤
[GPS]    ──GpsPosition──→ waypoint_manager.py ───┤
                                                  ▼
            ┌──────────────────────────────────────────┐
            │  Detection listeleri (sign/light/obstacle)│
            │  + GpsPosition                            │
            └────────────┬─────────────────────────────┘
                         │
    ┌────────────────────┼────────────────────────┐
    ▼                    ▼                         ▼
safety_logic      traffic_light_logic      traffic_sign_logic
obstacle_logic                             parking_logic
slalom_logic                               lane_violation
    │                    │                         │
    └────────────────────┼─────────────────────────┘
                         ▼
                 decision_arbiter.py
                         │
                         ▼
                 FinalDecision
              (emergency_stop, speed_cap,
               steer_override, reasons)
                         │
                         ▼
                   [Kontrolcü]
                   (/cmd_vel)
```

---

## Önemli Notlar

1. **`deos_algorithms` paket yolu**: Tüm import'lar `deos_algorithms.xxx` formatındadır. Simülasyonda çalıştırmak için `sys.path`'e bu dizin eklenmeli veya uygun bir symlink/package yapısı kurulmalıdır.

2. **ROS bağımsızlık**: Bu modüllerin hiçbiri `rclpy` import etmez. Hepsi saf Python'dır ve birim test edilebilir.

3. **Algoritma mantığına dokunma kuralı**: Bu dosyalardaki algoritma mantığı (mesafe kestirimi, threat seviyeleri, hız katsayıları, durum makineleri) değiştirilmemelidir. Sadece altyapı (topic bağlantıları, ROS wrapper'lar, launch entegrasyonu) üzerinde çalışılmalıdır.
