# DEĞİŞİKLİKLER — ilk halden şu anki hale

> Amaç: Projeyi ilk paylaşılan halinden bu yana yapılan değişiklikleri tek yerde
> toplamak ki entegre eden arkadaş zorlanmasın. (Repo git geçmişi olmadığı için bu
> liste konuşma/çalışma kaydından elle derlendi.)

Tarih: 2026-06-24

## ⭐ NİHAİ DURUM (kullanılan kurulum)

- **Şerit takibi:** `lane_test` (saf onnxruntime, model `lane_seg.onnx`) → `ros2 launch araba lane_test.launch.py`
- **Harita dokusu:** `map.jpeg` (modelin en iyi çalıştığı görünüm; `map_clean.png`/`map1.png` denendi, geri `map.jpeg`'e dönüldü).
- `lane_yolo` (ultralytics) **alternatif/deney** olarak duruyor; ana hat değil.

---

## 1. YENİ EKLENEN DOSYALAR (kod)

| Dosya | Ne işe yarar | Kaynağı |
|---|---|---|
| `src/araba/scripts/lane_test_node.py` | Şerit/gidilebilir-alan segmentasyonu (raw **onnxruntime**), ROI + satır-medyanı ile **orta nokta** çıkarımı, kareler-arası EMA yumuşatma, OpenCV debug penceresi. `/perception/center_pts` yayınlar. | `ros2deneme.py` (artık `arsiv_kullanilmayan/`) |
| `src/araba/scripts/autonomous_control_node.py` | `/perception/center_pts` dinler → P kontrolcü ile direksiyon/gaz → `/cmd_vel` (+ `/control/target_angle`, `/control/throttle`). | `control_wout_kavsak.py` (arşivde) |
| `src/araba/scripts/lane_yolo_node.py` | **Alternatif** şerit node'u: maskelemeyi **ultralytics YOLO** ile yapar (`results.masks.xy` poligonları, letterbox kayması yok). Orta-nokta/kontrol mantığı lane_test ile aynı. Yol sınıfını modelin `names`'inden **dinamik** bulur. | `test_onnx.py` / `test (1).py` (arşivde) |
| `src/araba/launch/lane_test.launch.py` | İzole deneme launch'ı: Gazebo + araç + kamera köprüsü + `lane_test_node` + `autonomous_control_node`. | yeni |
| `src/araba/launch/lane_yolo.launch.py` | `lane_test.launch.py` ikizi; node = `lane_yolo_node`, model = `2206_2model.onnx`, `device` parametresi (cpu/gpu). | yeni |
| `src/araba/scripts/dataset_recorder.py` | **Modelsiz** ham kamera karesi toplayıcı (eğitim/fine-tune verisi için). `/odom` mesafesine göre PNG kaydeder. `python3` ile çalıştırılır. | yeni |
| `src/araba/launch/data_collect.launch.py` | Model ÇALIŞTIRMADAN Gazebo + kamera köprüsü + WASD teleop (veri toplama için). | yeni |

---

## 2. DEĞİŞEN DOSYALAR

### `src/araba/CMakeLists.txt`
- Yeni node'lar **RENAME'siz** `install(PROGRAMS)` bloğuna eklendi (`lane_test_node.py`,
  `lane_yolo_node.py`, `autonomous_control_node.py`) → `--symlink-install` ile gerçek
  symlink kurulur, **kod değişikliği yeniden derleme istemez** (node yine de yeniden başlatılmalı).
- `scripts/onnx` dizini `share/araba/models/onnx`'e kurulur.

### `src/araba/worlds/benim_dunyam.sdf`
- **Zemin** `100×100` → **`1000×1000`** (dönüşte gökyüzünün ROI'ye girmesini önlemek için
  kara parçası büyütüldü; parkur boyutu DEĞİŞMEDİ).
- Zemin materyali **koyu yeşil** yapıldı (gri-benzeri renk kaldırıldı).
- **4 çevre duvarı** eklendi: `cevre_duvar_kuzey/guney/dogu/bati` (statik, yeşil, yükseklik 25)
  — uzakta gökyüzü/boşluk görünmesin diye.
- Gökyüzü: `<scene><background>` açık mavi (`0.3 0.6 0.9`).
- **Fizik**: `max_step_size=0.004`, `real_time_update_rate=250` (WSL'de RTF/algı FPS için).
- Araç spawn pozisyonu güncellendi (aşağıdaki tabloya bak).

### Araç spawn pozisyonları (launch başına)
| Launch | x | y | z | Yaw |
|---|---|---|---|---|
| `gazebo.launch.py` | 45.31 | 16 | 0.51 | 1.58 |
| `lane_test.launch.py` | 45.31 | 16 | 0.51 | 1.58 |
| `lane_yolo.launch.py` | 45.31 | **-1** | 0.51 | 1.58 |

> Not: Oturum boyunca denenen diğer konumlar: `44.498, -69.858, z=0.28` ve `45.31, 11.48`.

### ONNX modelleri — `src/araba/scripts/onnx/`
| Dosya | Sınıf sırası | Kullanan |
|---|---|---|
| `lane_seg.onnx` | `{0: road, 1: alternative}` | **lane_test** (varsayılan şerit modeli) |
| `2206_2model.onnx` | `{0: alternative, 1: road}` | **lane_yolo** |
| `2206model.onnx` | `{0: alternative, 1: road}` | (şu an kullanılmıyor; alternatif) |
| `detection.onnx`, `model.onnx`, `smallmodel.pt` | — | tam algı pipeline'ı (`gazebo.launch.py`) |

> ⚠️ **ÖNEMLİ — sınıf indeksi tuzağı:** Yol (gidilebilir) sınıfı `lane_seg.onnx`'te **0**,
> `2206*` modellerinde **1**. `lane_test_node` yolu sabit `cls 0` sayar; `lane_yolo_node`
> ise modelin `names`'inden dinamik bulur. Farklı model takarken buna dikkat.

---

### `src/araba/scripts/lane_yolo_node.py` (maskeleme iyileştirildi)
- Yol poligonları artık **union** (birleşim) maskesinde toplanıyor (eskiden ayrı maske
  listesi → en büyük tek parça seçiliyordu, gerisi atılıyordu).
- Union maskeye **CLOSE+OPEN morfoloji** + `keep_bottom_component` uygulanıyor (lane_test
  ile aynı hat) → boşluk/gürültü temizlenir, aracın altına bağlı yol seçilir.
- **Tanı logu** eklendi: her ~30 karede sınıf/conf/alan dökümü + yol ROI kaplaması (%).

### `src/araba/scripts/autonomous_control_node.py` (parametreleştirildi)
- `kp`, `max_angle`, `steer_alpha`, `steer_limit` artık launch parametresi.
  **Varsayılanlar orijinalle aynı** (Kp 0.8, açı 30°, alpha 0.3) → davranış değişmedi,
  sadece ayarlanabilir hale geldi.

## 3. YENİ BAĞIMLILIKLAR (sadece `lane_yolo_node` için)

`lane_yolo_node` ultralytics kullanıyor. Yeni bir makinede:

```bash
pip3 install --user --break-system-packages ultralytics
# torch'u CPU build'e sabitle (GPU/cu13 torch, onnxruntime-gpu'nun cu12 cuDNN'iyle ÇAKIŞIR):
pip3 install --user --break-system-packages --force-reinstall --no-deps \
  torch torchvision --index-url https://download.pytorch.org/whl/cpu
# numpy'ı ROS Jazzy ile uyumlu tut (ultralytics 2.x'e yükseltirse cv_bridge bozulur):
pip3 install --user --break-system-packages "numpy<2"
```

- **`lane_test_node` bu bağımlılıkları İSTEMEZ** (saf `onnxruntime`).
- `lane_test` GPU'da (onnxruntime CUDAExecutionProvider) çalışır; `lane_yolo` CPU'da
  (~18 FPS, sim kamerası ~8.5 FPS olduğu için darboğaz değil).

---

## 4. HARİTA (PARKUR DOKUSU)

- **Çalışan dünya** = `src/araba/worlds/benim_dunyam.sdf` (launch bunu açar). Parkur
  dokusu `<pbr><metal><albedo_map>` ile veriliyor.
- **Aktif doku = `map.jpeg`** (SDF satır ~208: `<albedo_map>map.jpeg</albedo_map>`).
  `map_clean.png` (yüksek çözünürlük) denendi ama model dönüşlerde yolu kaybetti
  (domain shift); `map.jpeg` modelin en iyi çalıştığı görünüm olduğu için ona dönüldü.
- `albedo_map` yolu SDF'e **göreli** olduğu için harita PNG'si `worlds/` klasöründe
  durmak zorunda. Harita değiştirilecekse: yeni PNG'yi `src/araba/worlds/`'e koy +
  SDF'teki `albedo_map` adını güncelle + `colcon build` (symlink için).

> Not: `worlds/` içinde `map_clean.png` ve `map1.png` de duruyor (alternatif/yedek)
> ama aktif değil. Farklı bir haritada eğitim verisi toplanacaksa o haritaya geçilip
> oradan veri alınmalı (model o görünüme göre eğitilmeli).

---

## 5. ARŞİVE TAŞINANLAR — `arsiv_kullanilmayan/`

Çalışmada kullanılmayan ama klasörde duran dosyalar buraya alındı (karışmasın):
`deneme.py`, `ros2deneme.py`, `control_wout_kavsak (1).py`, `2206_2model.onnx`,
`2206model.onnx`, `model1.onnx`, `model.onnx`, `Ortanokta.png`.
Ayrıca Windows indirme artığı `*:Zone.Identifier` dosyaları silindi.

> Bu dosyaların İÇERİĞİ kayıp değil: kod mantıkları `src/araba/scripts/` altındaki
> node'lara entegre edildi; onnx modellerinin kullanılan kopyaları `scripts/onnx/`'te.
