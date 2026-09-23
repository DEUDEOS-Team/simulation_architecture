# KURULUM — Projeyi sıfırdan bir bilgisayarda çalıştırma

Hedef ortam: **Ubuntu 24.04** (native veya WSL2) + **ROS 2 Jazzy** + **Gazebo Harmonic**.
Aşağıdaki adımları sırayla uygula; sonunda `ros2 launch araba gazebo.launch.py`
ile simülasyon (araç, tabelalar, trafik ışığı, harita, algı node'ları) açılır.

> Workspace'in **`~/sim2_ws`** konumunda olduğu varsayılır. Başka yere klonlarsan
> Adım 4'teki `$HOME/sim2_ws` kısmını kendi yolunla değiştir.

---

## 1. ROS 2 Jazzy kurulumu

```bash
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

## 2. Gazebo Harmonic + projenin ROS bağımlılıkları

`ros-jazzy-ros-gz` Gazebo Harmonic'i de beraberinde kurar.

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-cv-bridge \
  ros-jazzy-robot-localization \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-eigen \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-geometry-msgs \
  libeigen3-dev libtbb-dev cmake build-essential \
  python3-colcon-common-extensions python3-pip python3-tk python3-pynput
```

## 3. Python paketleri

`numpy` 2.x sürümü ROS'un `cv_bridge`'i ile uyumsuz, bu yüzden `<2` sabitlenir.

```bash
pip install --break-system-packages "numpy<2" onnxruntime opencv-python pillow scipy
# (İsteğe bağlı) yalnızca YOLO eğitim/araç script'leri için:
# pip install --break-system-packages ultralytics
```

## 4. Repoyu indir

`--recursive` şart: `kiss-icp` ve `pcl_localization_ros2` alt modül olarak gelir.

```bash
git clone --recursive -b bekir https://github.com/DEUDEOS-Team/simulation_architecture.git ~/sim2_ws
cd ~/sim2_ws
```

---

## 5. Repoyu aldıktan sonra yapılması GEREKEN değişiklikler

### 5.1 Sabit `/home/bekir` yollarını kendi yoluna çevir

Dünya dosyası (`benim_dunyam.sdf`) tabela, bariyer, trafik ışığı ve tünel modellerini
`file:///home/bekir/sim2_ws/...` mutlak yoluyla yükler. Kullanıcı adın `bekir` değilse
bu yollar bulunamaz ve **dünya boş açılır**. Tek komutla düzelt:

```bash
cd ~/sim2_ws
grep -rl "/home/bekir" src otonom_projem | xargs sed -i "s|/home/bekir|$HOME|g"
```

Etkilenen dosyalar: `src/araba/worlds/benim_dunyam.sdf`,
`otonom_projem/teknofest_haritası/benim_dunyam.sdf` ve `src/araba/scripts/tools/tests/`
altındaki test dosyaları.

Kontrol (çıktı boş olmalı):

```bash
grep -rn "/home/bekir" src/araba/worlds otonom_projem --include=*.sdf
```

### 5.2 `pcl_localization_ros2` için Jazzy yamasını uygula

Alt modül Jazzy'de header isimleri yüzünden derlenmez; yama repoda hazır:

```bash
cd ~/sim2_ws/src/pcl_localization_ros2
git apply ../../patches/pcl_localization_ros2_jazzy_includes.patch
cd ~/sim2_ws
```

### 5.3 Trafik ışığı eklentisini derle ve kur

Eklenti (`TrafficLightPlugin`) kaynak olarak repoda, derlenmiş `.so` yok:

```bash
cd ~/sim2_ws/otonom_projem/models/traffic_light
chmod +x build_and_install.sh
source /opt/ros/jazzy/setup.bash
./build_and_install.sh          # ~/.gz/sim/plugins/libTrafficLightPlugin.so oluşturur
cd ~/sim2_ws
```

### 5.4 `~/.bashrc` ayarları

```bash
cat >> ~/.bashrc <<'EOF'

# --- sim2_ws ---
source /opt/ros/jazzy/setup.bash
if [ -f ~/sim2_ws/install/setup.bash ]; then source ~/sim2_ws/install/setup.bash; fi
export GZ_SIM_RESOURCE_PATH=$HOME/sim2_ws/otonom_projem/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/.gz/sim/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
EOF
source ~/.bashrc
```

> **WSL2 + NVIDIA** kullanıyorsan Gazebo'nun GPU ile çizmesi için şunları da ekleyebilirsin:
> `export GALLIUM_DRIVER=d3d12` ve `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`

---

## 6. Derle

```bash
cd ~/sim2_ws
rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`kiss-icp` ilk derlemede birkaç dakika sürebilir (bazı bağımlılıkları internetten indirir).

## 7. Çalıştır

```bash
ros2 launch araba gazebo.launch.py
```

Gazebo + RViz açılır, birkaç saniye içinde algı node'ları ve klavye kontrolü başlar
(WASD ile sür). Sadece URDF'i RViz'de görmek için: `ros2 launch araba display.launch.py`.

---

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| Dünya açılıyor ama tabelalar/bariyerler/tünel yok | Adım 5.1 yapılmamış |
| Trafik ışığı yanmıyor, `/traffic_light/state` yayını yok | Adım 5.3 + 5.4 (`GZ_SIM_SYSTEM_PLUGIN_PATH`) |
| `pcl_localization_ros2` derlenirken `tf2_eigen.h` hatası | Adım 5.2 yapılmamış |
| `kiss-icp` / `pcl_localization_ros2` klasörleri boş | `git submodule update --init --recursive` |
| `cv_bridge` / `numpy` ile ilgili `ImportError` | `pip install --break-system-packages "numpy<2"` |
| Kaynak değişikliği sonrası ONNX modeli bulunamıyor | `colcon build --symlink-install` tekrar çalıştır |

Mimari ve node açıklamaları için: `CLAUDE.md`, `MIMARI_DURUM.md`.
