#!/usr/bin/env python3
"""
avoidance_node.py — boşluk-takibi kaçınması (gap_avoid_logic'in ROS sarmalayıcısı).

Girdi : /perception/lidar_obstacles (String/JSON)  — lidar_obstacle_node'dan;
        zaten yol maskesi + şerit maskesi + yükseklik bandından geçmiş,
        ARAÇ ÇERÇEVESİNDE kümeler (distance_m ön tampondan, lateral_m + = SOL).
Çıktı : /planning/lateral_offset (Float32) — şerit merkezine göre yanal kayma (+ = SOL).
        autonomous_control_node bunu kamera hedefine uygular.
        /planning/avoid_debug (String/JSON) — teşhis (ofset, gereken kaçış, sebep).

Bu node DİREKSİYON YAYINLAMAZ. Acil freni de yapmaz (o safety/obstacle_logic'te).
Yol kapalıysa blocked bayrağını debug'a yazar; durdurma kararı üst katmanındır.

SESSİZ FALLBACK YASAK (eski hatanın dersi): karar sebebi her değiştiğinde loglanır,
lidar bayatlarsa WARN basılır ve ofset güvenli tarafa (0 = kendi şeridi) sönümlenir.
"""

import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, Float32MultiArray, String

EARTH_R_M = 6378137.0

from gap_avoid_logic import GapAvoidConfig, GapAvoidState, GapObstacle, plan_offset

# Zaman birimi: bu node sim saatini kullanır (use_sim_time). Sim RTF ~0.2 ve lidar
# duvar saatinde ~1.7 Hz akıyor; ofsetin hız sınırı (m/s) ve sönümlemesi aracın yaşadığı
# zamanla aynı birimde olmalı, yoksa duvar saatinde 1.2 m/s sim'de ~6 m/s'ye karşılık
# gelip ofset fırlıyor. Tazelik eşikleri de sim saniyesi: lidar 10 Hz, kamera ~15 Hz.
LIDAR_TIMEOUT_S = 1.0
# Şerit noktaları bundan eskiyse yanal konum ölçülemez (kapalı döngü açılır).
LANE_TIMEOUT_S = 0.8
# Harita (GPS + merkez çizgileri) ölçümü — final_odom ~3 Hz yayınlıyor.
MAP_TIMEOUT_S = 2.0
# ── Çoğunluk filtresi (titreyen tabelayı ele, sürekli koniyi geçir) ──
# Bir engel son OBST_WINDOW karenin >= OBST_CONFIRM'ünde görülmeli. 8/5 ile koni ~%90
# oranında geçer (7.2/8, 3 ardışık kaybı atlatır); ~%50 titreyen tabela 4/8 < 5 elenir.
OBST_WINDOW = 8
OBST_CONFIRM = 5
OBST_LAT_GATE = 0.7      # kareler arası eşleme: yanal merkez farkı (m)
OBST_DIST_GATE = 2.0     # kareler arası eşleme: mesafe farkı (m)

# Direksiyon düzeltmesi (pure-pursuit) sabitleri — URDF/kontrolcüyle aynı olmalı.
WHEEL_BASE_M = 1.675
STEER_LIMIT_RAD = 0.5236        # 30°
MIN_LOOKAHEAD_M = 3.0           # çok yakın engelde δ patlamasın
AVOID_STEER_GAIN = 1.6          # pure-pursuit çıktısını kuvvetlendir
AVOID_STEER_MAX = 0.70          # şerit/plan direksiyonunu tamamen ezmesin
# Bu kadar küçük bir kaçış için direksiyona dokunma (bkz. _tick (a)).
B_DEADBAND_M = 0.35
# "Engele doğru kırma" yasağı (bkz. _tick (b)): bu mesafedeki ve bu yanal bandın
# içindeki en yakın engelin tarafına düzeltme uygulanamaz.
NO_TURN_TOWARD_DIST_M = 8.0
NO_TURN_TOWARD_LAT_M = 2.2
# base_link -> aracın MERKEZ EKSENİ (URDF: sol teker hizasında origin, merkez y=-0.675)
BASE_TO_CENTER_Y = -0.675


class AvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("avoidance_node")

        self.declare_parameter("obstacles_topic", "/perception/lidar_obstacles")
        self.declare_parameter("offset_topic", "/planning/lateral_offset")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("band_half_width_m", 3.0)
        self.declare_parameter("car_half_width_m", 0.80)   # gap_avoid_logic ile AYNI kalmalı
        self.declare_parameter("safety_margin_m", 0.55)   # gap_avoid_logic ile AYNI kalmalı
        self.declare_parameter("trigger_distance_m", 15.0)
        self.declare_parameter("max_rate_mps", 1.8)
        self.declare_parameter("return_rate_mps", 0.6)

        self._cfg = GapAvoidConfig(
            band_half_width_m=float(self.get_parameter("band_half_width_m").value),
            car_half_width_m=float(self.get_parameter("car_half_width_m").value),
            safety_margin_m=float(self.get_parameter("safety_margin_m").value),
            trigger_distance_m=float(self.get_parameter("trigger_distance_m").value),
            max_rate_mps=float(self.get_parameter("max_rate_mps").value),
            return_rate_mps=float(self.get_parameter("return_rate_mps").value),
        )
        self._state = GapAvoidState()
        self._obstacles: list[GapObstacle] = []
        self._obstacles_stamp = 0.0
        self._tracks: list[dict] = []   # çoğunluk filtresi izleyicisi (bkz. _confirm_obstacles)
        self._last_tick = self._now()
        self._last_reason = ""
        self._stale_warned = False

        # ── Kapalı döngü ölçümü: aracın şerit merkezine göre yanal konumu ──
        self.declare_parameter("cam_width", 1280.0)
        self.declare_parameter("cam_height_m", 0.948)     # kamera yerden yüksekliği (URDF)
        self._cam_w = float(self.get_parameter("cam_width").value)
        self._cam_h_m = float(self.get_parameter("cam_height_m").value)
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._measured_offset: float | None = None
        self._lane_stamp = 0.0
        self._lane_warned = False

        self.create_subscription(
            String, str(self.get_parameter("obstacles_topic").value),
            self._obstacles_cb, 10)
        self.create_subscription(
            Float32MultiArray, "/perception/center_pts", self._center_pts_cb, 10)
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._cam_info_cb, 10)

        # ── ŞERİT KONUMU: HARİTA ÖLÇÜMÜ (birincil) ──
        # Kamera ölçümü çoğu zaman gelmiyor (şerit segmentasyonu düşüyor) ve o zaman
        # "şeritten çıkma" kısıtı tamamen kalkıp araç asfaltın dışına kadar kayıyordu.
        # GPS + merkez çizgileri (final_odom) her zaman var. Bu yalnız bir konum ölçümüdür;
        # direksiyon buradan üretilmez, sadece band kısıtını besler.
        self.declare_parameter("centerlines_file", "")
        self.declare_parameter("datum_lat", 40.7899)
        self.declare_parameter("datum_lon", 29.5089)
        self._seg_a = self._seg_ab = self._seg_len2 = self._seg_dir = None
        self._map_offset: float | None = None
        self._map_stamp = 0.0
        cl = str(self.get_parameter("centerlines_file").value)
        if cl:
            try:
                self._load_centerlines(cl)
                self.get_logger().info(
                    f"şerit referansı: harita ({len(self._seg_a)} segment) + kamera yedek")
            except Exception as e:                   # noqa: BLE001
                self.get_logger().error(
                    f"centerlines yüklenemedi ({e}) — band kısıtı yalnız kamera ölçümüyle!")
        else:
            self.get_logger().warn(
                "centerlines_file verilmedi — şerit bandı kısıtı yalnız kamera ölçümüne bağlı.")
        self.create_subscription(
            Odometry, "/localization/odom/final", self._odom_cb, 10)
        self._pub_offset = self.create_publisher(
            Float32, str(self.get_parameter("offset_topic").value), 10)
        self._pub_debug = self.create_publisher(String, "/planning/avoid_debug", 10)
        # Moddan bağımsız kaçınma düzeltmesi: ofseti yalnız kamera şerit takipçisi
        # uygularsa, şerit kaybolunca vehicle_controller PLAN moduna düşüp /cmd_vel_lane'i
        # yok sayıyor ve kaçınma komutu çöpe gidiyor. Bu düzeltme son direksiyona eklenir
        # (LANE/PLAN/TURN fark etmez).
        self._pub_steer = self.create_publisher(Float32, "/planning/avoid_steer", 10)

        period = 1.0 / max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"avoidance_node hazır (boşluk-takibi). band=±{self._cfg.band_half_width_m} m, "
            f"tetik={self._cfg.trigger_distance_m} m, şişirme="
            f"{self._cfg.car_half_width_m + self._cfg.safety_margin_m:.2f} m "
            f"-> /planning/lateral_offset")

    def _obstacles_cb(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f"engel JSON parse: {e}")
            return

        obstacles: list[GapObstacle] = []
        for d in raw:
            lat = float(d["lateral_m"])
            # lateral_min/max yeni alanlar; yoksa noktasal engel varsayılır.
            lo = float(d.get("lateral_min_m", lat))
            hi = float(d.get("lateral_max_m", lat))
            obstacles.append(GapObstacle(
                distance_m=float(d["distance_m"]),
                lateral_min_m=min(lo, hi),
                lateral_max_m=max(lo, hi)))
        self._obstacles = obstacles
        self._obstacles_stamp = self._now()
        self._stale_warned = False

    def _now(self) -> float:
        """SİM saati (saniye). Ofsetin hız sınırı/sönümlemesi aracın
        yaşadığı zamanla aynı birimde olmalı — bkz. dosya başı notu."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _confirm_obstacles(self, raw: list) -> list:
        """Basit çoğunluk-filtresi izleyicisi: bir engel yalnız son OBST_WINDOW karenin
        en az OBST_CONFIRM'ünde görülürse GERÇEK sayılır. Kareler arası eşleme yanal
        merkez + mesafe yakınlığıyla yapılır. Titreyen (yol kenarı tabela) engel eşiği
        geçemez; sürekli görülen (koni) geçer ve nadir kaybını da atlatır."""
        used = [False] * len(raw)
        # Mevcut track'leri en yakın eşleşmeyle güncelle
        for tr in self._tracks:
            tc = 0.5 * (tr["obs"].lateral_min_m + tr["obs"].lateral_max_m)
            best, bestd = -1, 1e9
            for ri, o in enumerate(raw):
                if used[ri]:
                    continue
                oc = 0.5 * (o.lateral_min_m + o.lateral_max_m)
                dd = abs(oc - tc) + abs(o.distance_m - tr["obs"].distance_m)
                if abs(oc - tc) < OBST_LAT_GATE and \
                        abs(o.distance_m - tr["obs"].distance_m) < OBST_DIST_GATE and dd < bestd:
                    bestd, best = dd, ri
            if best >= 0:
                used[best] = True
                tr["obs"] = raw[best]              # en güncel geometri
                tr["hist"].append(True)
            else:
                tr["hist"].append(False)
            if len(tr["hist"]) > OBST_WINDOW:
                tr["hist"].pop(0)
        # Eşleşmeyen ham engeller için yeni track
        for ri, o in enumerate(raw):
            if not used[ri]:
                self._tracks.append({"obs": o, "hist": [True]})
        # Ölü track'leri at; onaylananları topla
        alive, confirmed = [], []
        for tr in self._tracks:
            if sum(tr["hist"]) == 0:
                continue
            alive.append(tr)
            if sum(tr["hist"]) >= OBST_CONFIRM:
                confirmed.append(tr["obs"])
        self._tracks = alive
        return confirmed

    def _load_centerlines(self, path: str) -> None:
        """GeoJSON LineString'leri datum-merkezli ENU segmentlerine çevir (sim dünya
        koordinatlarıyla çakışık). Her segmentin birim yönü de saklanır — aracın KENDİ
        şeridini seçmek için (yollar tek yön; ters yöndeki çizgi karşı şerittir)."""
        lat0 = float(self.get_parameter("datum_lat").value)
        lon0 = float(self.get_parameter("datum_lon").value)
        cos0 = math.cos(math.radians(lat0))

        def enu(lon, lat):
            return (math.radians(lon - lon0) * cos0 * EARTH_R_M,
                    math.radians(lat - lat0) * EARTH_R_M)

        with open(path) as f:
            gj = json.load(f)
        a, b = [], []
        for feat in gj.get("features", []):
            geom = feat.get("geometry") or {}
            if geom.get("type") != "LineString":
                continue
            pts = [enu(float(c[0]), float(c[1])) for c in geom.get("coordinates") or []]
            for p, q in zip(pts, pts[1:]):
                a.append(p)
                b.append(q)
        if not a:
            raise ValueError("LineString segmenti yok")
        self._seg_a = np.array(a)
        ab = np.array(b) - self._seg_a
        self._seg_ab = ab
        self._seg_len2 = np.maximum((ab * ab).sum(axis=1), 1e-9)
        self._seg_dir = ab / np.sqrt(self._seg_len2)[:, None]

    def _odom_cb(self, msg: Odometry) -> None:
        """Aracın KENDİ şerit merkezine göre yanal konumu (+ = SOL) — harita ölçümü."""
        if self._seg_a is None:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        head = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        # base_link ARACIN MERKEZİ DEĞİL (URDF: sol teker hizasında; merkez eksen
        # y=-0.675). Şerit ofsetini aracın GERÇEK merkezinden ölçmeliyiz, yoksa
        # 0.675 m sistematik hata olur (araç şeridin ortasında sanılırken değil).
        pos = np.array([float(p.x), float(p.y)]) + BASE_TO_CENTER_Y * left

        # Yalnız aracın gittiği yöne bakan segmentler = KENDİ şeridi (yollar tek yön)
        aligned = (self._seg_dir @ head) > 0.5
        if not aligned.any():
            return
        idx = np.flatnonzero(aligned)
        ap = pos - self._seg_a[idx]
        t = np.clip((ap * self._seg_ab[idx]).sum(axis=1) / self._seg_len2[idx], 0.0, 1.0)
        proj = self._seg_a[idx] + t[:, None] * self._seg_ab[idx]
        d = np.linalg.norm(pos - proj, axis=1)
        k = int(np.argmin(d))
        if float(d[k]) > 6.0:          # şeride oturmuyor (kavşak/park) -> ölçüm yok
            return
        # İşaret: aracın SOL birim vektörüne izdüşüm (+ = araç şerit merkezinin solunda)
        self._map_offset = float(np.dot(pos - proj[k], left))
        self._map_stamp = self._now()

    def _cam_info_cb(self, msg: CameraInfo) -> None:
        s = self._cam_w / float(msg.width) if msg.width else 1.0
        self._fx = float(msg.k[0]) * s
        self._cx = float(msg.k[2]) * s
        self._fy = float(msg.k[4]) * s
        self._cy = float(msg.k[5]) * s

    def _center_pts_cb(self, msg: Float32MultiArray) -> None:
        """Şerit merkez noktalarından aracın yanal konumunu ÖLÇ (+ = araç solda).

        Araca EN YAKIN nokta (en büyük satır v) alınır ve yer düzlemine geri
        yansıtılır:  X = fy·h/(v−cy),  y = (cx − u)·X/fx.
        y = şerit merkezinin araca göre yanal konumu; aracın şeride göre ofseti
        bunun TERSİDİR (şerit merkezi sağımda ise ben soldayım).
        """
        d = msg.data
        if len(d) < 2 or self._fx <= 0.0:
            self._measured_offset = None
            return
        pts = [(d[i], d[i + 1]) for i in range(0, len(d) - 1, 2)]
        u, v = max(pts, key=lambda p: p[1])       # en alt satır = araca en yakın
        dv = float(v) - self._cy
        if dv <= 1.0:
            self._measured_offset = None
            return
        x_fwd = self._fy * self._cam_h_m / dv
        if not (1.0 <= x_fwd <= 30.0):
            self._measured_offset = None
            return
        y_lane = (self._cx - float(u)) * x_fwd / self._fx
        self._measured_offset = -y_lane
        self._lane_stamp = self._now()
        self._lane_warned = False

    def _tick(self) -> None:
        now = self._now()
        dt = now - self._last_tick
        self._last_tick = now

        fresh = (now - self._obstacles_stamp) < LIDAR_TIMEOUT_S
        if not fresh:
            if self._obstacles_stamp > 0.0 and not self._stale_warned:
                self.get_logger().warn(
                    "KAÇINMA PASİF: lidar engel listesi bayat (>1 s) — ofset 0'a "
                    "sönümleniyor, araç kendi şeridine döner.")
                self._stale_warned = True
            obstacles: list[GapObstacle] = []
        else:
            obstacles = self._obstacles

        # ── Çoğunluk filtresi ──
        # Yol kenarındaki tabelalar poz titremesiyle arada bir yol kapısını geçip
        # kaçınmayı tetikleyip salınım yaratıyordu. Bir engel son WINDOW karenin en az
        # CONFIRM'ünde görülmedikçe kaçınmaya beslenmez. Koni çoğunlukta var (nadir
        # kayıpları köprülenir); tabela çoğunlukta yok (titremesi elenir).
        obstacles = self._confirm_obstacles(obstacles)

        # Yanal konum ölçümü: harita birincil (her zaman var), kamera yedek. Bu ölçüm
        # olmadan "şeritten çıkma" kısıtı çalışmaz ve araç asfaltın dışına kadar kayabilir.
        map_fresh = (now - self._map_stamp) < MAP_TIMEOUT_S
        lane_fresh = (now - self._lane_stamp) < LANE_TIMEOUT_S
        if map_fresh and self._map_offset is not None:
            measured = self._map_offset
            src = "harita"
        elif lane_fresh:
            measured = self._measured_offset
            src = "kamera"
        else:
            measured = None
            src = "yok"
        if measured is None and obstacles and not self._lane_warned:
            self._lane_warned = True
            self.get_logger().warn(
                "KAÇINMA KISITSIZ: ne harita ne kamera yanal konum veriyor — "
                "şerit bandı kısıtı UYGULANAMIYOR (yalnız engelden kaçış).")

        self._state = plan_offset(obstacles, self._state, dt, self._cfg, measured)

        # ── Moddan bağımsız direksiyon düzeltmesi ──
        # Girdi: b = gereken yanal kaçış (ARAÇ çerçevesinde, LİDARDAN ölçülü — harita,
        # GPS, başlık yok). Pure-pursuit: δ = atan2(2·L·b, d²). b ne kadar büyük ve
        # engel ne kadar yakınsa o kadar sert. Araç kaçtıkça b KENDİLİĞİNDEN 0'a iner
        # -> düzeltme de sıfırlanır. Yani kapalı döngü; "geri dön" için ayrı bir şey
        # gerekmez, şerit takibi/plan zaten kendi işini yapar.
        b = float(self._state.required_b_m)
        rel = [o for o in obstacles if o.distance_m >= self._cfg.min_distance_m]
        nearest_d = min((o.distance_m for o in rel), default=float("inf"))
        steer_bias = 0.0

        # (a) Ölü bant: gereken kaçış küçükse direksiyona dokunma. Sabit bir direksiyon
        # açısı aracı "10 cm kaydırmaz", sürekli döndürür; üstelik δ = atan2(2Lb, d²)
        # engel yaklaştıkça sertleşir, 10 cm'lik bir düzeltme aracı koninin üstüne çevirir.
        if abs(b) >= B_DEADBAND_M and math.isfinite(nearest_d):
            d = max(nearest_d, MIN_LOOKAHEAD_M)
            delta = math.atan2(2.0 * WHEEL_BASE_M * b, d * d)     # rad, + = sola
            # Oranla (+ = SAĞ olacak şekilde işaret çevir), sınırla.
            steer_bias = -delta / STEER_LIMIT_RAD * AVOID_STEER_GAIN
            steer_bias = max(-AVOID_STEER_MAX, min(AVOID_STEER_MAX, steer_bias))

        # (b) ENGELE DOĞRU KIRMA YASAĞI — mutlak güvenlik kuralı.
        # Yakındaki en yakın engel hangi taraftaysa, düzeltme O TARAFA olamaz.
        # (steer_bias + = SAĞ; engel laterali + = SOL.) Bu kural sayesinde kaçınma
        # katmanı, hesap ne kadar yanlış olursa olsun, aracı engele KIRAMAZ.
        close = [o for o in rel if o.distance_m <= NO_TURN_TOWARD_DIST_M]
        if close:
            near = min(close, key=lambda o: o.distance_m)
            centre = 0.5 * (near.lateral_min_m + near.lateral_max_m)
            if abs(centre) < NO_TURN_TOWARD_LAT_M:
                if centre < 0.0:                 # engel SAĞDA -> sağa kırma yok
                    steer_bias = min(steer_bias, 0.0)
                else:                            # engel SOLDA -> sola kırma yok
                    steer_bias = max(steer_bias, 0.0)

        self._pub_steer.publish(Float32(data=float(steer_bias)))

        self._pub_offset.publish(Float32(data=float(self._state.offset_m)))
        self._pub_debug.publish(String(data=json.dumps({
            "offset_m": round(self._state.offset_m, 3),
            "measured_m": None if measured is None else round(measured, 3),
            "measured_src": src,
            "required_b_m": round(self._state.required_b_m, 3),
            "steer_bias": round(steer_bias, 3),
            "blocked": self._state.blocked,
            "active": self._state.active,
            "reason": self._state.reason,
            "n_obstacles": len(obstacles),
        })))

        if self._state.reason != self._last_reason:
            self._last_reason = self._state.reason
            nearest = min((o.distance_m for o in obstacles), default=float("inf"))
            meas_s = "yok" if measured is None else f"{measured:+.2f}({src})"
            self.get_logger().info(
                f"KAÇINMA [{self._state.reason}] ofset={self._state.offset_m:+.2f} m "
                f"ölçülen={meas_s} gereken={self._state.required_b_m:+.2f} m "
                f"engel={len(obstacles)} en_yakın={nearest:.1f} m")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
