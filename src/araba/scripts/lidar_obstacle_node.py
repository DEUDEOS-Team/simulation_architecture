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
     "önümde engel" olup aracı kilitliyordu (canlı 2026-07-11, iki koşu üst üste).
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
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
        self.declare_parameter("bumper_offset_m", 1.1873)  # lidar -> base_link (URDF x, mimari konvansiyonu)
        # Yol maskesi (docstring 5): boş bırakılırsa filtre pasif
        self.declare_parameter("centerlines_file", "")
        # Eşik doğrulaması (dünya SDF nesneleriyle, 2026-07-11): gerçek engeller
        # merkez çizgisine ≤1.17 m, yol mobilyası (tabela/durak/tünel) ≥1.73 m —
        # 1.45 iki sınıfın tam ortası, iki yöne de ~0.3 m pay bırakır.
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

        self._eps = float(self.get_parameter("cluster_epsilon_m").value)
        self._min_pts = int(self.get_parameter("cluster_min_points").value)
        self._max_dist = float(self.get_parameter("max_distance_m").value)
        self._corridor_hw = float(self.get_parameter("corridor_half_width_m").value)
        self._min_z = float(self.get_parameter("min_z_m").value)
        self._bumper_offset = float(self.get_parameter("bumper_offset_m").value)
        self._road_hw = float(self.get_parameter("road_halfwidth_m").value)
        # Lidar'ın base_link'teki (x, y) konumu — küme dünya konumu hesabı için (URDF)
        self._lidar_off = np.array([-1.1873, -0.675])

        # Araç pozu (yol maskesi için): /odom dünya (ENU) çerçevesinde
        self._pose_xy: np.ndarray | None = None
        self._pose_yaw: float = 0.0
        self._pose_stamp: float = 0.0

        # Kamera kapısı durumu (URDF: cam_sensor base_link'te, rpy 0)
        self._cam_off = np.array([-1.2724, -0.675, 0.94789])
        self._cam_fx = self._cam_fy = self._cam_cx = self._cam_cy = 0.0
        self._cam_w = self._cam_h = 0
        self._lane_mask: np.ndarray | None = None
        self._lane_mask_stamp: float = 0.0
        self._roi_top = float(self.get_parameter("lane_roi_top_ratio").value)
        self._roi_bot = float(self.get_parameter("lane_roi_bot_ratio").value)

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
        self.create_subscription(
            Image, str(self.get_parameter("lane_mask_topic").value), self._lane_mask_cb, 10
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._caminfo_cb, 10
        )
        self._pub = self.create_publisher(String, str(self.get_parameter("lidar_obstacles_topic").value), 10)
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
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_xy = np.array([p.x, p.y])
        self._pose_yaw = math.atan2(siny, cosy)
        self._pose_stamp = time.monotonic()

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
            self._lane_mask_stamp = time.monotonic()
        except Exception:
            self._lane_mask = None

    def _dist_to_road_m(self, p_world: np.ndarray) -> float:
        """Nokta -> en yakın merkez çizgisi segmenti mesafesi (m)."""
        ap = p_world - self._seg_a
        t = np.clip((ap * self._seg_ab).sum(axis=1) / self._seg_len2, 0.0, 1.0)
        proj = self._seg_a + t[:, None] * self._seg_ab
        return float(np.linalg.norm(p_world - proj, axis=1).min())

    def _road_vote(self, cl_xy: np.ndarray) -> bool | None:
        """Harita kapısı oyu: True=yol içi, False=yol dışı, None=çekimser (veri yok)."""
        if self._seg_a is None or self._pose_xy is None:
            return None
        if (time.monotonic() - self._pose_stamp) > 1.0:
            return None
        centroid = cl_xy.mean(axis=0) + self._lidar_off  # base_link çerçevesi
        c, s = math.cos(self._pose_yaw), math.sin(self._pose_yaw)
        world = self._pose_xy + np.array(
            [c * centroid[0] - s * centroid[1], s * centroid[0] + c * centroid[1]]
        )
        return bool(self._dist_to_road_m(world) <= self._road_hw)

    def _camera_vote(self, cl_xy: np.ndarray) -> bool | None:
        """Kamera şerit-maskesi oyu: kümenin zemin noktası görüntüye projeksiyonlanır.
        True=şerit maskesinde, False=maske dışında, None=çekimser (veri yok /
        görüş veya ROI dışı / kameraya çok yakın)."""
        if self._lane_mask is None or self._cam_fx <= 0.0:
            return None
        if (time.monotonic() - self._lane_mask_stamp) > 1.0:
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
            return

        mask = (
            (pts[:, 0] > 0.3)
            & (pts[:, 0] < self._max_dist)
            & (np.abs(pts[:, 1]) < self._corridor_hw)
            & (pts[:, 2] > self._min_z)
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
            votes = [v for v in (self._road_vote(cl), self._camera_vote(cl)) if v is not None]
            if votes and not any(votes):
                continue
            # Mesafe ÖN TAMPONDAN (lidar arkada monteli — bkz. docstring 4)
            dist = max(0.0, float(np.min(cl[:, 0])) - self._bumper_offset)
            lat = float(np.mean(cl[:, 1]))
            diameter = float(max(np.max(cl[:, 0]) - np.min(cl[:, 0]), np.max(cl[:, 1]) - np.min(cl[:, 1])))
            obstacles.append(
                {
                    "kind": self._classify(diameter),
                    "confidence": 0.7,
                    "distance_m": dist,
                    "lateral_m": lat,
                    "bbox_px": None,
                }
            )

        self._pub.publish(String(data=json.dumps(obstacles)))

    @staticmethod
    def _classify(diameter: float) -> str:
        if diameter < 0.4:
            return "cone"
        if diameter < 1.5:
            return "unknown"
        return "barrier"

    def _euclidean_cluster(self, pts_xy: np.ndarray) -> list[np.ndarray]:
        visited = np.zeros(len(pts_xy), dtype=bool)
        clusters: list[np.ndarray] = []

        for i in range(len(pts_xy)):
            if visited[i]:
                continue

            dists = np.linalg.norm(pts_xy - pts_xy[i], axis=1)
            nbrs = np.where(dists < self._eps)[0]
            if len(nbrs) < self._min_pts:
                visited[i] = True
                continue

            cluster_idx: set[int] = set(nbrs.tolist())
            queue = list(nbrs)
            while queue:
                j = queue.pop()
                if visited[j]:
                    continue
                visited[j] = True
                d2 = np.linalg.norm(pts_xy - pts_xy[j], axis=1)
                new_nbrs = np.where(d2 < self._eps)[0]
                if len(new_nbrs) >= self._min_pts:
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
