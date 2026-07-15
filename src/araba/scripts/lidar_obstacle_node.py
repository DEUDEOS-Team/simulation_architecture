#!/usr/bin/env python3
"""
lidar_obstacle_node — LiDAR nokta bulutundan engel tespiti (SİM UYARLAMASI).

Kaynak: DEOS_MimariDev/perception/sensor_fusion/lidar_obstacle_node.py
Kümeleme (euclidean cluster), koridor filtresi, çap-tabanlı sınıflandırma ve
JSON çıktı SÖZLEŞMESİ mimariyle birebir aynıdır. Sim için değişenler:
  1. Topic'ler: deos_topic_layout yerine sim köprüsündeki '/lidar/scan/points'
     girişi ve '/perception/lidar_obstacles' çıkışı (parametreyle değiştirilebilir).
  2. _unpack_xyz vektörize edildi (numpy strided) — mimarideki nokta-başına
     struct.unpack döngüsü Gazebo'nun büyük bulutlarında/WSL'de çok yavaş kalır.
  3. Zemin filtresi (min_z_m): sim lidar'ı 16 kanallı 3B (±15° dikey) — alt
     kanalların zemine çarpan halkaları kümelenip sahte "barrier" üretiyordu
     (tam önde 4.5m'de kalıcı road_blocked). Sensör ~1.19m yükseklikte, zemin
     sensör çerçevesinde z≈-1.5; z>-1.35 (zeminden ~15cm üstü) engel sayılır.
  4. Öne-referans ofseti (bumper_offset_m=1.1873): lidar araçta ARKAYA montelidir
     (URDF x=-1.1873) — mesafeler lidar'dan ölçülünce ObstacleLogic'in 1.5 m acil
     bandı tampon temasına denk geliyordu (bariyer çarpması). Mimari bunu pcl_ros
     voxel filtresinin `output_frame: base_link` dönüşümüyle çözer (mesafeler araç
     çerçevesinde); simde aynı referansa x ofsetiyle çevrilir. base_link orijini
     ön aksın 0.36 m önünde ≈ aracın burnu — yani mesafeler fiilen tampondan.
  5. YOL MASKESİ kapısı (centerlines_file + road_halfwidth_m): koridor araca göre
     düz bir kutu olduğundan virajda yol DIŞINI tarar — durak/levha yapıları
     "önümde engel" olup aracı kilitliyordu.
     Kümenin dünya konumu (odom pozu + yaw + lidar ofseti) hesaplanır; çizili yol
     merkez çizgilerine road_halfwidth_m'den uzaksa YOL DIŞI sayılır ve hiç
     yayınlanmaz. Odom/harita yoksa filtre devre dışı (fail-open).
  6. KAMERA ŞERİT-MASKESİ kapısı (lane_mask_topic): lane_test_node'un ego-şerit
     segmentasyon maskesi + kamera iç parametreleriyle kümenin zemin noktası
     görüntüye projeksiyonlanır; nokta maskede değilse "şerit dışı" oyu. Gerçek
     araçta harita/GPS'ten bağımsız çalışan kapı budur. OY BİRLEŞTİRME: her kapı
     yol-içi/yol-dışı/çekimser döner; küme yalnızca EN AZ BİR kapı oy verip
     HİÇBİRİ "yol içi" demediyse elenir (tek kapının yanlış vetosu engeli
     gizleyemez; ikisi de çekimserse her şey engel sayılır — fail-open).

Çıktı: String topic, JSON liste:
  [{"kind": "cone|unknown|barrier", "confidence": 0.7,
    "distance_m": ..., "lateral_m": ..., "bbox_px": null}, ...]
perception_pipeline_node bunu LidarObstacle'a çevirip fuse(lidar=...) ile
engel mantığına besler; speed_controller acil freni zaten dinliyor.
"""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, PointCloud2
from std_msgs.msg import String

EARTH_R_M = 6371000.0


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_node")

        self.declare_parameter("cluster_epsilon_m", 0.5)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("max_distance_m", 20.0)
        self.declare_parameter("corridor_half_width_m", 3.0)
        self.declare_parameter("min_z_m", -1.35)
        # Üst yükseklik sınırı: trafik tabelaları direk üstünde yüksek panel taşır
        # (yer üstü ~1.5-2.5 m); araç yolda olmasalar bile bunları engel sayıp
        # kaçınmaya çalışıyordu. Koniler alçak (~0.5 m). Zemin z≈-1.5; max_z=-0.5 ≈
        # yerden ~1.0 m: koni/yaya-alt/bariyer-alt noktaları kalır, yüksek panel
        # noktaları elenir. Sınıflandırma yalnız yatay çaptan olduğundan yükseklik
        # bandını daraltmak sınıfı bozmaz. (Direk adayları ayrı topic'te.)
        self.declare_parameter("max_z_m", -0.5)
        self.declare_parameter("bumper_offset_m", 1.1873)  # lidar -> base_link (URDF x, mimari konvansiyonu)
        # Yol maskesi (docstring 5): boş bırakılırsa filtre pasif
        self.declare_parameter("centerlines_file", "")
        # Eşik dünya SDF nesneleriyle doğrulandı: gerçek engeller merkez çizgisine
        # ≤1.17 m, yol mobilyası (tabela/durak/tünel) ≥1.73 m — 1.45 iki sınıfın tam
        # ortası, iki yöne de ~0.3 m pay bırakır.
        self.declare_parameter("road_halfwidth_m", 1.45)  # merkez çizgisine bu mesafe = yol içi
        self.declare_parameter("datum_lat", 40.7899)      # world SDF / final_odom ile aynı datum
        self.declare_parameter("datum_lon", 29.5089)
        # DİKKAT: /odom SPAWN-göreli — dünya (datum-ENU) pozu final_odom'dan gelir
        self.declare_parameter("odom_topic", "/localization/odom/final")
        # Kamera şerit-maskesi kapısı (docstring 6)
        self.declare_parameter("lane_mask_topic", "/perception/lane_mask")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        # lane_test ROI dikey bandı (0.60h–0.88h): projeksiyon bunun dışına
        # düşerse maske orada zaten boştur -> kamera kapısı çekimser kalır
        self.declare_parameter("lane_roi_top_ratio", 0.62)
        self.declare_parameter("lane_roi_bot_ratio", 0.86)
        # Sim köprüsü topic'leri (mimaride deos_topic_layout'tan gelir)
        self.declare_parameter("cloud_topic", "/lidar/scan/points")
        self.declare_parameter("lidar_obstacles_topic", "/perception/lidar_obstacles")
        # Direk adayları (trafik ışığı mesafesi için): ışık direği yol kenarında
        # olduğundan koridor/yol kapıları onu eler;
        # kapılardan önce yüksek noktalardan çıkarılan ham aday listesi ayrı
        # topic'te yayınlanır, perception_pipeline kamera bbox'ıyla eşleştirir.
        self.declare_parameter("pole_candidates_topic", "/perception/pole_candidates")
        self.declare_parameter("pole_min_z_m", 1.2)        # lidar'a göre; ~1.75 m yer üstü (araç/yaya çatısı üstü)
        self.declare_parameter("pole_max_lateral_m", 8.0)  # aday arama bandı (koridordan geniş)
        self.declare_parameter("pole_min_points", 3)       # direk ince: az nokta yeter
        self.declare_parameter("pole_max_diameter_m", 1.0) # direk+lamba gövdesi ince kalmalı

        self._eps = float(self.get_parameter("cluster_epsilon_m").value)
        self._min_pts = int(self.get_parameter("cluster_min_points").value)
        self._max_dist = float(self.get_parameter("max_distance_m").value)
        self._corridor_hw = float(self.get_parameter("corridor_half_width_m").value)
        self._min_z = float(self.get_parameter("min_z_m").value)
        self._max_z = float(self.get_parameter("max_z_m").value)
        self._bumper_offset = float(self.get_parameter("bumper_offset_m").value)
        self._road_hw = float(self.get_parameter("road_halfwidth_m").value)
        self._pole_min_z = float(self.get_parameter("pole_min_z_m").value)
        self._pole_max_lat = float(self.get_parameter("pole_max_lateral_m").value)
        self._pole_min_pts = int(self.get_parameter("pole_min_points").value)
        self._pole_max_diam = float(self.get_parameter("pole_max_diameter_m").value)
        # Lidar'ın base_link'teki (x, y) konumu — küme dünya konumu hesabı için (URDF)
        self._lidar_off = np.array([-1.1873, -0.675])

        # Araç pozu (yol maskesi için): KONUM /gps/fix -> datum-ENU, YÖN /odom yaw.
        # (odom/final konum yayınlamıyor — bkz. _odom_cb notu)
        self._pose_xy: np.ndarray | None = None
        self._pose_yaw: float = 0.0
        self._pose_stamp: float = 0.0
        self._diag_stamp: float = 0.0   # KUME teşhis logu kısıtlaması
        self._datum_lat = float(self.get_parameter("datum_lat").value)
        self._datum_lon = float(self.get_parameter("datum_lon").value)

        # Kamera kapısı durumu (URDF: cam_sensor base_link'te, rpy 0)
        self._cam_off = np.array([-1.2724, -0.675, 0.94789])
        self._cam_fx = self._cam_fy = self._cam_cx = self._cam_cy = 0.0
        self._cam_w = self._cam_h = 0
        self._lane_mask: np.ndarray | None = None
        self._lane_mask_stamp: float = 0.0
        self._roi_top = float(self.get_parameter("lane_roi_top_ratio").value)
        self._roi_bot = float(self.get_parameter("lane_roi_bot_ratio").value)

        # ── Kolon testi (iki korumalı) ──
        # Kümenin üstünde yüksek nokta varsa dikey yapıdır (tabela/direk) -> statik engel
        # değil. Koni bandın üstüne çıkmaz. İki koruma, veto'nun yanlışlıkla koniyi
        # elemesini engeller (yoksa koni son yaklaşmada elenip çarpılıyor):
        #  1) Şerit-içi engeli asla eleme: |yanal| < in_lane_lat_m ise koru (koni şerit
        #     içinde ~0.8 m; tabela yol kenarında >1.2 m). Poz gerektirmez.
        #  2) Onaylanmış engeli eleme: son confirm_hold_s içinde engel olarak yayınlanan
        #     bir dünya konumuna yakın küme vetolanamaz.
        self.declare_parameter("column_enable", True)
        self.declare_parameter("column_radius_m", 0.3)      # direk kalınlığı civarı (dar)
        self.declare_parameter("column_min_points", 2)
        self.declare_parameter("column_in_lane_lat_m", 1.2)  # bu yanaldan içerisi = koru
        self.declare_parameter("column_confirm_hold_s", 3.0)
        self.declare_parameter("column_confirm_radius_m", 1.0)
        self._column_enable = bool(self.get_parameter("column_enable").value)
        self._column_radius_m = float(self.get_parameter("column_radius_m").value)
        self._column_min_pts = int(self.get_parameter("column_min_points").value)
        self._column_in_lane_lat = float(self.get_parameter("column_in_lane_lat_m").value)
        self._column_confirm_hold = float(self.get_parameter("column_confirm_hold_s").value)
        self._column_confirm_r = float(self.get_parameter("column_confirm_radius_m").value)
        self._tall_xy: np.ndarray | None = None
        self._recent_obstacles: list[tuple[np.ndarray, float]] = []  # (dünya xy, zaman)


        # Yol maskesi segmentleri (ENU, datum merkezli) — yoksa filtre pasif
        self._seg_a: np.ndarray | None = None
        self._seg_ab: np.ndarray | None = None
        self._seg_len2: np.ndarray | None = None
        cl_file = str(self.get_parameter("centerlines_file").value)
        if cl_file:
            try:
                self._load_road_mask(cl_file)
            except Exception as e:
                self.get_logger().error(f"yol maskesi yüklenemedi ({cl_file}): {e} — filtre pasif")

        self.create_subscription(
            PointCloud2, str(self.get_parameter("cloud_topic").value), self._cloud_cb, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10
        )
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(
            Image, str(self.get_parameter("lane_mask_topic").value), self._lane_mask_cb, 10
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._caminfo_cb, 10
        )
        self._pub = self.create_publisher(String, str(self.get_parameter("lidar_obstacles_topic").value), 10)
        self._pub_poles = self.create_publisher(
            String, str(self.get_parameter("pole_candidates_topic").value), 10
        )
        self.get_logger().info(
            "lidar_obstacle_node ready"
            + (f" — yol maskesi aktif ({len(self._seg_a)} segment, ±{self._road_hw}m)"
               if self._seg_a is not None else " — yol maskesi PASİF")
        )

    def _load_road_mask(self, path: str) -> None:
        """Centerlines GeoJSON'daki tüm LineString'leri datum-merkezli ENU
        segmentlerine çevirir (sim dünya koordinatları bu ENU ile çakışıktır)."""
        lat0 = float(self.get_parameter("datum_lat").value)
        lon0 = float(self.get_parameter("datum_lon").value)
        cos0 = math.cos(math.radians(lat0))

        def enu(lon: float, lat: float) -> tuple[float, float]:
            return (
                math.radians(lon - lon0) * cos0 * EARTH_R_M,
                math.radians(lat - lat0) * EARTH_R_M,
            )

        with open(path) as f:
            gj = json.load(f)
        a_list, b_list = [], []
        for feat in gj.get("features", []):
            geom = feat.get("geometry") or {}
            if geom.get("type") != "LineString":
                continue
            coords = geom.get("coordinates") or []
            pts = [enu(float(c[0]), float(c[1])) for c in coords]
            for p, q in zip(pts, pts[1:]):
                a_list.append(p)
                b_list.append(q)
        if not a_list:
            raise ValueError("LineString segmenti bulunamadı")
        self._seg_a = np.array(a_list)
        ab = np.array(b_list) - self._seg_a
        self._seg_ab = ab
        self._seg_len2 = np.maximum((ab * ab).sum(axis=1), 1e-9)

    def _odom_cb(self, msg: Odometry) -> None:
        """KONUM + YÖNELİM buradan (/localization/odom/final).

        TARİHÇE — bu iki kez değişti, ikisinin de sebebi kayıtlı olsun:
        * Koşu-24: konum buradan alınıyordu ama o zamanki lokalizasyon hattı (EKF)
          poz olarak (0,0) yayınlıyordu -> kümelerin dünya konumu çöp -> yol maskesi
          gerçek konileri "yol dışı" sayıp eliyordu. Çare olarak konum HAM GPS'e
          (/gps/fix) alınmıştı.
        * Koşu-40: EKF hattı terk edildi, final_odom (GPS+IMU+odom füzyonu) devrede
          ve KONUM YAYINLIYOR. Ham GPS ise simde 1 Hz: araç ~2 m/s giderken poz
          yarım-bir saniye bayat kalıyor ve kümeler dünyada ~1 m KAYIK çıkıyordu.
          Ölçüldü: gerçek koni (28.0, 24.8) -> hesaplanan (28.8, 25.5); yola uzaklık
          0.80 yerine 1.50 çıkıp 1.45 eşiğini KIL PAYI aşıyor ve koni ELENİYORDU.
          Füzyonlu poz hem taze (~3 Hz + ölü hesap) hem doğru -> konum tekrar buradan.
        """
        p = msg.pose.pose.position
        self._pose_xy = np.array([float(p.x), float(p.y)])
        self._pose_stamp = self._now()
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny, cosy)

    def _gps_cb(self, msg: NavSatFix) -> None:
        """YEDEK dünya konumu: yalnız füzyonlu poz (odom/final) hiç gelmediyse.

        Ham GPS simde 1 Hz — araç hareket hâlindeyken bayat kalır ve kümeleri
        dünyada ~1 m kaydırır (bkz. _odom_cb tarihçesi). Bu yüzden ARTIK BİRİNCİL
        DEĞİL; sadece füzyonlu poz yoksa devreye girer ki maske büsbütün körelmesin.
        """
        if (self._now() - self._pose_stamp) < 2.0:
            return                      # füzyonlu poz taze — ham GPS'e gerek yok
        cos0 = math.cos(math.radians(self._datum_lat))
        x = math.radians(float(msg.longitude) - self._datum_lon) * cos0 * EARTH_R_M
        y = math.radians(float(msg.latitude) - self._datum_lat) * EARTH_R_M
        self._pose_xy = np.array([x, y])
        self._pose_stamp = self._now()

    def _caminfo_cb(self, msg: CameraInfo) -> None:
        self._cam_fx = float(msg.k[0])
        self._cam_cx = float(msg.k[2])
        self._cam_fy = float(msg.k[4])
        self._cam_cy = float(msg.k[5])
        self._cam_w = int(msg.width)
        self._cam_h = int(msg.height)

    def _lane_mask_cb(self, msg: Image) -> None:
        try:
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            arr = arr.reshape(msg.height, msg.step)[:, : msg.width]
            self._lane_mask = arr
            self._lane_mask_stamp = self._now()
        except Exception:
            self._lane_mask = None

    def _dist_to_road_m(self, p_world: np.ndarray) -> float:
        """Nokta -> en yakın merkez çizgisi segmenti mesafesi (m)."""
        ap = p_world - self._seg_a
        t = np.clip((ap * self._seg_ab).sum(axis=1) / self._seg_len2, 0.0, 1.0)
        proj = self._seg_a + t[:, None] * self._seg_ab
        return float(np.linalg.norm(p_world - proj, axis=1).min())

    def _now(self) -> float:
        """SİM saati (s). Tazelik eşikleri sensörlerin SİM hızıyla aynı
        birimde olmalı — duvar saatinde RTF düşünce her şey bayat görünür."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_cluster(self, cl: np.ndarray, rv, cv, dropped: bool) -> None:
        """Her kümeyi araç çerçevesinde + dünya çerçevesinde + kapı oylarıyla logla.
        Yalnız 15 m içindekiler ve saniyede en fazla bir kez (log seli olmasın)."""
        d = float(np.min(cl[:, 0]))
        if d > 15.0:
            return
        now = self._now()
        if now - self._diag_stamp < 0.5:
            return
        self._diag_stamp = now
        lat = float(np.mean(cl[:, 1]))
        lo, hi = float(np.min(cl[:, 1])), float(np.max(cl[:, 1]))
        wtxt = "poz-yok"
        if self._pose_xy is not None:
            c, s = math.cos(self._pose_yaw), math.sin(self._pose_yaw)
            cen = cl[:, :2].mean(axis=0)
            w = self._pose_xy + np.array([c * cen[0] - s * cen[1],
                                          s * cen[0] + c * cen[1]])
            wtxt = (f"dunya=({w[0]:.1f},{w[1]:.1f}) yol_mesafe="
                    f"{self._dist_to_road_m(w):.2f}")
        self.get_logger().info(
            f"KUME d={d:.1f} lat={lat:+.2f} [{lo:+.2f}..{hi:+.2f}] {wtxt} "
            f"yol_oy={rv} kam_oy={cv} {'ELENDI' if dropped else 'tutuldu'}")

    def _road_vote(self, cl_xy: np.ndarray) -> bool | None:
        """Harita kapısı oyu: True=yol içi, False=yol dışı, None=çekimser (veri yok)."""
        if self._seg_a is None or self._pose_xy is None:
            return None
        # Sim saati (bkz. _now). Duvar saatiyle ölçersek: GPS sim'de 1 Hz ama RTF ~0.2
        # olduğundan duvar saatinde 5 saniyede bir geliyor -> poz her zaman "bayat"
        # sayılıp yol kapısı çekimser kalıyor. O durumda tek hakem kamera şerit maskesi
        # kalınca, araç kaçınmak için şeritten kaydığı anda koni maskenin dışına düşüp
        # eleniyor: engel listeden kaybolup geri geliyor, kaçınma sıfırlanıyor.
        if (self._now() - self._pose_stamp) > 3.0:
            return None
        centroid = cl_xy.mean(axis=0) + self._lidar_off  # lidar -> base_link
        c, s = math.cos(self._pose_yaw), math.sin(self._pose_yaw)
        world = self._pose_xy + np.array(
            [c * centroid[0] - s * centroid[1], s * centroid[0] + c * centroid[1]]
        )
        return bool(self._dist_to_road_m(world) <= self._road_hw)

    def _cluster_world_xy(self, cl_xy: np.ndarray) -> np.ndarray | None:
        """Kümenin dünya (datum-ENU) konumu — poz taze değilse None."""
        if self._pose_xy is None or (self._now() - self._pose_stamp) > 3.0:
            return None
        centroid = cl_xy.mean(axis=0) + self._lidar_off  # base_link
        c, s = math.cos(self._pose_yaw), math.sin(self._pose_yaw)
        return self._pose_xy + np.array(
            [c * centroid[0] - s * centroid[1], s * centroid[0] + c * centroid[1]])

    def _remember_obstacle(self, cl_xy: np.ndarray) -> None:
        """Engel olarak yayınlanan kümenin dünya konumunu + zamanını sakla (koruma 2)."""
        w = self._cluster_world_xy(cl_xy)
        if w is None:
            return
        now = self._now()
        self._recent_obstacles = [
            (p, t) for (p, t) in self._recent_obstacles
            if now - t <= self._column_confirm_hold]
        self._recent_obstacles.append((w, now))

    def _column_vote(self, cl_xy: np.ndarray) -> bool:
        """Küme dikey yapı (tabela/direk) mı -> True = statik engel DEĞİL. İKİ KORUMA
        koninin elenmesini önler; ancak ikisi de kümeyi korumazsa yüksek-nokta testine
        bakılır."""
        if not self._column_enable or self._tall_xy is None or len(self._tall_xy) == 0:
            return False
        cen = cl_xy.mean(axis=0)
        # KORUMA 1 — ŞERİT İÇİ: yanalı küçükse (kendi şeridinde) ASLA veto etme (koni).
        if abs(float(cen[1])) < self._column_in_lane_lat:
            return False
        # KORUMA 2 — ONAYLANMIŞ ENGEL: son karelerde engel yayınlanan bir konuma
        # yakınsa veto etme (koni 9 kare onaylanmıştı, tabela hiç onaylanmaz).
        w = self._cluster_world_xy(cl_xy)
        if w is not None and self._recent_obstacles:
            now = self._now()
            for (p, t) in self._recent_obstacles:
                if now - t <= self._column_confirm_hold and \
                        float(np.hypot(*(w - p))) < self._column_confirm_r:
                    return False
        # Yüksek-nokta testi: kümenin dar yarıçapında en az N yüksek nokta var mı?
        d2 = ((self._tall_xy - cen) ** 2).sum(axis=1)
        return bool(int((d2 < self._column_radius_m ** 2).sum()) >= self._column_min_pts)

    def _camera_vote(self, cl_xy: np.ndarray) -> bool | None:
        """Kamera şerit-maskesi oyu: kümenin zemin noktası görüntüye projeksiyonlanır.
        True=şerit maskesinde, False=maske dışında, None=çekimser (veri yok /
        görüş veya ROI dışı / kameraya çok yakın)."""
        if self._lane_mask is None or self._cam_fx <= 0.0:
            return None
        if (self._now() - self._lane_mask_stamp) > 1.0:
            return None
        cb = cl_xy.mean(axis=0) + self._lidar_off  # base_link, zemin noktası (z=0)
        px = float(cb[0] - self._cam_off[0])       # kamera çerçevesi: x ileri
        py = float(cb[1] - self._cam_off[1])       # y sol
        pz = float(-self._cam_off[2])              # zemin, kameranın altında
        if px < 1.5:
            return None  # kameraya çok yakın/arkada: projeksiyon güvenilmez
        u = self._cam_cx - self._cam_fx * (py / px)
        v = self._cam_cy - self._cam_fy * (pz / px)
        mh, mw = self._lane_mask.shape
        if self._cam_w > 0 and mw != self._cam_w:
            u *= mw / float(self._cam_w)   # maske küçültülmüşse ölçekle
            v *= mh / float(self._cam_h)
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < mw):
            return None  # yatay görüş dışı
        if not (self._roi_top * mh <= vi <= self._roi_bot * mh):
            return None  # lane ROI bandı dışı: maske orada zaten boş, oy verme
        du = max(3, int(mw * 0.03))
        dv = max(3, int(mh * 0.04))
        win = self._lane_mask[max(0, vi - dv): min(mh, vi + dv + 1),
                              max(0, ui - du): min(mw, ui + du + 1)]
        return bool((win > 0).any())

    def _cloud_cb(self, msg: PointCloud2) -> None:
        pts = self._unpack_xyz(msg)
        if pts is None or len(pts) == 0:
            self._pub.publish(String(data="[]"))
            self._pub_poles.publish(String(data="[]"))
            return

        # ── Yanal (y) dönüşümü yok — lidar zaten araç merkezinde ──
        # URDF'e göre sol tekerler y=+0.068/+0.001, sağ tekerler y=-1.418/-1.351 ->
        # aracın merkez ekseni y=-0.675. Lidar/kamera/GPS de tam y=-0.675'te. Yani
        # base_link aracın merkezinde değil (sol teker hizasında), ama lidar merkez
        # eksende. Dolayısıyla ham lidar y'si zaten araç merkezine göredir ve doğrudur;
        # bunu base_link'e çevirmeye kalkmak 0.68 m'lik bir kayma üretir. x ise base_link'e
        # çevrilir (aşağıda bumper_offset ile). Kapılar (yol/kamera) küme merkezini
        # base_link'e çevirir: cl + _lidar_off.

        # Direk adayları koridor/yol kapılarından ÖNCE (ham buluttan) çıkar
        self._publish_pole_candidates(pts)

        # KOLON TESTİ için engel bandının ÜSTÜNDEKİ noktalar (koridor içi)
        if self._column_enable:
            tall_mask = (
                (pts[:, 0] > 0.3)
                & (pts[:, 0] < self._max_dist)
                & (np.abs(pts[:, 1]) < self._corridor_hw)
                & (pts[:, 2] > self._max_z)
                & (pts[:, 2] < self._max_z + 1.2)
            )
            self._tall_xy = pts[tall_mask][:, :2] if bool(tall_mask.any()) else None

        mask = (
            (pts[:, 0] > 0.3)
            & (pts[:, 0] < self._max_dist)
            & (np.abs(pts[:, 1]) < self._corridor_hw)
            & (pts[:, 2] > self._min_z)
            & (pts[:, 2] < self._max_z)   # yüksek tabela panellerini ele (alçak koni kalır)
        )
        pts = pts[mask]
        if len(pts) == 0:
            self._pub.publish(String(data="[]"))
            return

        clusters = self._euclidean_cluster(pts[:, : 2])
        obstacles = []
        for cl in clusters:
            # Yol-içi kapıları (docstring 5-6): oy veren kapılardan hiçbiri
            # "yol içi" demiyorsa küme engel DEĞİLDİR — hiç yayınlanmaz.
            # (En az bir True -> tut; hepsi çekimser -> tut; aksi hâlde ele.)
            # KOLON VETOSU (kapılardan önce): dikey yapı (tabela/direk) mı? İki koruma
            # koninin elenmesini önler (bkz. __init__).
            if self._column_vote(cl):
                self._log_cluster(cl, None, None, True)
                continue
            rv = self._road_vote(cl)
            cv = self._camera_vote(cl)
            votes = [v for v in (rv, cv) if v is not None]
            dropped = bool(votes) and not any(votes)
            # Teşhis logu: kümenin araç-göreli konumu + hangi kapının elediği.
            self._log_cluster(cl, rv, cv, dropped)
            if dropped:
                continue
            # Mesafe ön tampondan: lidar arkaya monteli (URDF x=-1.1873, bkz. docstring 4)
            dist = max(0.0, float(np.min(cl[:, 0])) - self._bumper_offset)
            lat = float(np.mean(cl[:, 1]))
            diameter = float(max(np.max(cl[:, 0]) - np.min(cl[:, 0]), np.max(cl[:, 1]) - np.min(cl[:, 1])))
            obstacles.append(
                {
                    "kind": self._classify(diameter),
                    "confidence": 0.7,
                    "distance_m": dist,
                    "lateral_m": lat,
                    # Kümenin gerçek yanal sınırları: boşluk-takibi kaçınması engeli
                    # ortalama noktayla değil kapladığı bantla şişirir — geniş bariyerde
                    # ortalama tek başına yanıltıcı. Ek anahtar; eski tüketiciler
                    # (perception_fusion) yok sayar.
                    "lateral_min_m": float(np.min(cl[:, 1])),
                    "lateral_max_m": float(np.max(cl[:, 1])),
                    "bbox_px": None,
                }
            )
            # ONAY KAYDI (koruma 2): bu küme ENGEL olarak yayınlandı — dünya konumunu
            # kaydet ki sonraki karelerde kolon vetosu onu silemesin.
            self._remember_obstacle(cl)

        self._pub.publish(String(data=json.dumps(obstacles)))

    def _publish_pole_candidates(self, pts: np.ndarray) -> None:
        """Yüksek (z > pole_min_z) noktalardan ince dikey yapı kümeleri: trafik
        ışığı/levha direkleri. Araç ve yaya bu yüksekliğe ulaşmaz; mesafe bilgisi
        perception_pipeline'da ışık bbox'ının bakış yönüyle eşleştirilir."""
        m = (
            (pts[:, 0] > 0.3)
            & (pts[:, 0] < self._max_dist)
            & (np.abs(pts[:, 1]) < self._pole_max_lat)
            & (pts[:, 2] > self._pole_min_z)
        )
        high = pts[m]
        poles = []
        if len(high) >= self._pole_min_pts:
            for cl in self._euclidean_cluster(high, min_pts=self._pole_min_pts):
                diam = float(
                    max(np.max(cl[:, 0]) - np.min(cl[:, 0]), np.max(cl[:, 1]) - np.min(cl[:, 1]))
                )
                if diam > self._pole_max_diam:
                    continue
                poles.append(
                    {
                        "distance_m": max(0.0, float(np.min(cl[:, 0])) - self._bumper_offset),
                        "lateral_m": float(np.mean(cl[:, 1])),
                        "z_max_m": float(np.max(cl[:, 2])),
                    }
                )
        self._pub_poles.publish(String(data=json.dumps(poles)))

    @staticmethod
    def _classify(diameter: float) -> str:
        if diameter < 0.4:
            return "cone"
        if diameter < 1.5:
            return "unknown"
        return "barrier"

    def _euclidean_cluster(self, pts_xy: np.ndarray, min_pts: int | None = None) -> list[np.ndarray]:
        """Kümeleme her zaman (x, y) düzleminde yapılır; girdi (N,2) veya (N,3)
        olabilir (fazla sütunlar — ör. z — küme satırlarıyla birlikte döner)."""
        if min_pts is None:
            min_pts = self._min_pts
        xy = pts_xy[:, :2]
        visited = np.zeros(len(pts_xy), dtype=bool)
        clusters: list[np.ndarray] = []

        for i in range(len(pts_xy)):
            if visited[i]:
                continue

            dists = np.linalg.norm(xy - xy[i], axis=1)
            nbrs = np.where(dists < self._eps)[0]
            if len(nbrs) < min_pts:
                visited[i] = True
                continue

            cluster_idx: set[int] = set(nbrs.tolist())
            queue = list(nbrs)
            while queue:
                j = queue.pop()
                if visited[j]:
                    continue
                visited[j] = True
                d2 = np.linalg.norm(xy - xy[j], axis=1)
                new_nbrs = np.where(d2 < self._eps)[0]
                if len(new_nbrs) >= min_pts:
                    for nb in new_nbrs:
                        if nb not in cluster_idx:
                            cluster_idx.add(int(nb))
                            queue.append(int(nb))

            clusters.append(pts_xy[list(cluster_idx)])

        return clusters

    def _unpack_xyz(self, msg: PointCloud2) -> np.ndarray | None:
        """PointCloud2 → (N,3) float32. Mimarideki nokta-başına struct.unpack yerine
        numpy strided görünümüyle tek seferde çözer (büyük bulutta ~100x hızlı)."""
        try:
            fmap = {f.name: f.offset for f in msg.fields}
            if not all(k in fmap for k in ("x", "y", "z")):
                return None

            n = msg.width * msg.height
            if n == 0:
                return None
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
            pts = np.empty((n, 3), dtype=np.float32)
            for col, key in enumerate(("x", "y", "z")):
                off = fmap[key]
                pts[:, col] = raw[:, off:off + 4].copy().view(np.float32).ravel()
            return pts[np.isfinite(pts).all(axis=1)]
        except Exception as e:
            self.get_logger().error(f"PointCloud2 unpack: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
