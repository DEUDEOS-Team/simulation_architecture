#!/usr/bin/env python3
"""
mission_planning_node.py — mimarideki planning/mission_planning node'unun sim uyarlaması.

Mimari orijinalinden farklar (geri taşıma diff'ini küçük tutmak için başka şey değişmedi):
  - `heading_is_enu_yaw` parametresi eklendi: final_odom_node STANDART ENU yaw yayınlar
    (0°=Doğu, saat yönünün tersi); waypoint_manager ise PUSULA heading bekler
    (0°=Kuzey, saat yönü). True ise pusula = (90 − enu_deg) % 360 çevrimi uygulanır.
    Bu bir yansıma olduğu için heading_offset_deg ile telafi EDİLEMEZ.
  - sys.path bootstrap'i: node install lib/araba altından symlink çalıştığı için
    scripts/ dizini path'e eklenir (deos_algorithms finder'ı da oradan gelir).

Sim launch'ta ayrıca: require_go_signal:=False, topic'ler sim adlarına parametreyle
yönlendirilir (mission_file/centerlines_file = share/araba/missions/*.geojson).
"""

import math
import os
import sys
import time
import json

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.mission_manager import MissionManager
from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, load_centerlines_geojson, nearest_node_id
from deos_algorithms.ros_topic_layout import build_deos_topics
from deos_algorithms.route_planner import (
    advance_mission_index_by_position,
    route_remaining_mission_via_graph,
    route_mission_plan_via_graph,
    route_mission_plan_without_graph,
)
from deos_algorithms.waypoint_manager import GpsPosition


GPS_TIMEOUT_SLOW_S = 2.0
GPS_TIMEOUT_STOP_S = 5.0
SPEED_DECAY_PER_SEC = 0.2
# Hıza-orantılı kontrol: araç bu kadar yol aldıysa kontrolü odom pozuyla yeniden
# koştur (ZAMAN değil KONUM çözünürlüğü sabit -> hız arttıkça güncelleme sıklaşır).
# 0.25 m: 2 m/s'de ~8 Hz, 4 m/s'de ~16 Hz. GPS tek başına ~0.9 Hz idi (geç dönüş).
CONTROL_STEP_M = 0.25
TURN_RULE_APPLY_DISTANCE_M = 8.0
# Kısıt uygulanan kavşaktan bu kadar uzaklaşınca "geçildi" sayarız ve perception'a
# temizleme sinyali yollarız; yoksa kısıt yapışkan kalıp sonraki kavşakları da bloklar.
INTERSECTION_PASSED_HYST_M = 4.0
TURN_PERM_CLEAR_GRACE_S = 1.0  # geçiş sonrası bayat turn_permissions'ı yok sayma süresi (duvar saati)

# Kavşak dönüş önceliği (/planning/turn_active): rotanın sıradaki bacağı belirgin bir
# yön değişimi istiyorsa vehicle_controller şerit görünse bile PLAN referansını sürer.
TURN_MIN_ANGLE_DEG = 35.0        # bacaklar arası açı eşiği: üstü "dönüş noktası"
TURN_ENGAGE_DISTANCE_M = 10.0    # dönüş noktasına bu mesafede öncelik başlar
# Köşe ancak önceki waypoint geçilince "current" oluyor, yani dönüşü ~6 m kala fark
# ediyoruz. Geç fark edilen (yakın) dönüşlerde direksiyonu yakınlıkla orantılı güçlendir.
# Yalnız kavşak dönüşü için; viraj (şerit takibi) ve halka bunun dışında.
TURN_ENGAGE_REF = 10.0           # bu mesafede kazanç 1.0
TURN_LATE_MAX_GAIN = 1.8         # kazanç tavanı
# Dönüş çıkış eşiği yöne göre ayrı: sağ dönüşü erken bitirmek gerekiyor (tünel girişinde
# uzun süren kırış duvara sürtüyor), ama aynı erken bitirmeyi sola uygularsak 90°'lik sol
# dönüş yarıda kalıyor. Sağda erken bırak, solda yayın sonuna kadar dön.
TURN_EXIT_ALIGN_DEG = 25.0       # SOL (ve varsayılan): tam dönsün
TURN_EXIT_ALIGN_DEG_RIGHT = 50.0 # SAĞ: erken bırak (tünel)

# Kavşak dönüş yayı: köşe waypoint'ine kilitlenmek yerine giriş/çıkış bacaklarına teğet
# bir daire yayı çizip onu izliyoruz. Köşeye nişan almak, hedef tam önde olduğundan köşe
# üstüne binene kadar sıfır direksiyon üretiyor, sonra ani tam kilitle şerit dışına taşıyordu.
TURN_ARC_RADIUS_M = 4.0          # yay yarıçapı (min dönüş yarıçapı ~2.9 m + takip marjı)
TURN_ARC_LOOKAHEAD_M = 2.5       # yay üzerinde nişan alınan ileri nokta
TURN_ARC_PRE_M = 12.0            # yay öncesi düz koşu (yaklaşırken projeksiyon için)
TURN_ARC_POST_M = 8.0            # yay sonrası düz koşu (çıkışta lookahead için)
# Sağ dönüşte yay, çıkış teğetine bu kadar kala bırakılır: son metrelerde direksiyon
# zaten açılıyor, kalan hizalanmayı şerit takibi toparlar. Solda 0 (yay sonuna kadar dön).
TURN_ARC_RELEASE_LEAD_M = 0.0        # SOL / varsayılan
TURN_ARC_RELEASE_LEAD_RIGHT_M = 1.5  # SAĞ (tünel dönüşü)

# Döner kavşak izinin yarıçapı. Haritanın merkez çizgisi düğümleri halkanın iç kenarına
# (r≈6.5-7.1 m) yapışık; araç yarı genişliği 0.8 m olunca iç tekerlek orta adaya ~0.4 m
# kalıyor ve dönüşte biraz kesince adaya çıkıyor. Sürülebilir bant 5.3-10.7 m; izi bandın
# ortasına (8 m) kaydırınca iki yana da ~2.7 m pay kalıyor.
RB_LANE_RADIUS_M = 8.0
# Köşe kesme sınırı: yay apeksi köşe düğümünün en fazla bu kadar içinden geçsin (şerit
# yarı genişliği 1.45 - araç yarısı ~0.65 - pay). Aşarsa yay açıortay boyunca dışa kaydırılır;
# bu kayma giriş/çıkış hattını yandan en fazla TURN_ARC_MAX_OUT_SWING_M öteler (dıştan al,
# içten çık).
TURN_ARC_MAX_INNER_CUT_M = 0.7
TURN_ARC_MAX_OUT_SWING_M = 0.65
EARTH_R_M = 6371000.0
WHEELBASE_M = 1.675              # araç dingil mesafesi (URDF)
MAX_STEER_RAD = 0.5236           # tam kilit tekerlek açısı (vehicle_controller ile aynı)

# Rotadan sapma bekçisi: aktif hedefe mesafe, o hedef için görülen minimumdan bu kadar
# artarsa dönüş kaçırılmış demektir -> mevcut konumdan rota yeniden planlanır.
OFF_ROUTE_MARGIN_M = 10.0
OFF_ROUTE_REPLAN_COOLDOWN_S = 5.0

# Rota yeniden hesaplanırken dönüş yok. "Sola dönülemez" gibi bir kısıt görülünce rota
# baştan planlanır; bu sürede (özellikle kısıt altında rota bulunamayıp eski plan
# korunurken) eski plan hâlâ yasak dönüşü gösterir. O yüzden replan başlar başlamaz dönüş
# önceliğini bırakıp kontrolü şerit takibine veriyoruz; rota oturunca dönüşler geri açılır.
ROUTE_UNSTABLE_HOLD_S = 2.0      # replan denemesinden sonra dönüşün bastırıldığı süre
NO_ROUTE_COOLDOWN_S = 2.0        # rota bulunamazsa bu süre boyunca tekrar deneme (fırtına önleme)


def _param_as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


def _angle_diff_deg(a: float, b: float) -> float:
    """Return signed smallest difference a-b in degrees in [-180, 180]."""
    d = (float(a) - float(b) + 180.0) % 360.0 - 180.0
    return d


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Forward azimuth (deg) from (lat1,lon1) to (lat2,lon2)."""
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dl = math.radians(float(lon2) - float(lon1))
    y = math.sin(dl) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

class MissionPlanningNode(Node):
    def __init__(self):
        super().__init__("mission_planning_node")

        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))

        self.declare_parameter("mission_file", "")
        self.declare_parameter("centerlines_file", "")
        self.declare_parameter("centerlines_round_decimals", 7)
        # Centerlines'ta tunnel: true varsa rota en az bir tünel kenarından geçer (görev GeoJSON gerekmez).
        self.declare_parameter("tunnel_mandatory", True)
        self.declare_parameter("mission_only_reorder_by_nearest", True)
        self.declare_parameter("mission_only_keep_park_last", True)
        # UMS-2 Go: araç göreve başlamadan önce onay bekle
        self.declare_parameter("require_go_signal", True)
        self.declare_parameter("go_topic", _T["hardware_motion_enable"])
        self.declare_parameter("heading_offset_deg", 0.0)
        # SİM UYARLAMASI: heading kaynağı ENU yaw yayınlıyorsa (final_odom_node) pusulaya çevir.
        self.declare_parameter("heading_is_enu_yaw", False)
        # Ek mimari uyumu: heading kaynağı opsiyonel olarak final_odom'dan alınabilir
        self.declare_parameter("heading_source", "final_odom")  # "final_odom" tercih (KISS-ICP+EKF); "imu" sadece odometri yoksa
        self.declare_parameter("final_odom_topic", _T["localization_odom_final"])
        # Datum: final_odom'un ENU pozunu lat/lon'a geri çevirmek için (dönüş kontrolü)
        self.declare_parameter("datum_lat", 40.7899)
        self.declare_parameter("datum_lon", 29.5089)
        self.declare_parameter("gps_fix_topic", _T["sensors_gps_fix"])
        self.declare_parameter("imu_topic", _T["sensors_imu"])
        self.declare_parameter("perception_turn_permissions_topic", _T["perception_fusion_turn_permissions"])
        self.declare_parameter("perception_decision_debug_topic", _T["perception_fusion_decision_debug"])
        self.declare_parameter("perception_park_complete_topic", _T["perception_fusion_park_complete"])
        self.declare_parameter("planning_steering_topic", _T["planning_steering_ref"])
        self.declare_parameter("planning_speed_topic", _T["planning_speed_limit"])
        self.declare_parameter("planning_current_task_topic", _T["planning_current_task"])
        self.declare_parameter("planning_arrived_topic", _T["planning_arrived"])
        self.declare_parameter("planning_park_mode_topic", _T["planning_park_mode"])
        self.declare_parameter("planning_park_remaining_topic", _T["planning_park_remaining_s"])
        # SİM EKİ (5. adım): kavşak dönüş önceliği sinyali (ros_topic_layout'ta yok)
        self.declare_parameter("planning_turn_active_topic", "/planning/turn_active")
        # SİM EKİ: kısıtın bağlandığı kavşak geçildi → perception tabela kısıtlarını temizlesin
        self.declare_parameter("planning_intersection_passed_topic", "/planning/intersection_passed")

        mission_file = str(self.get_parameter("mission_file").value)
        centerlines_file = str(self.get_parameter("centerlines_file").value)
        centerlines_round = int(self.get_parameter("centerlines_round_decimals").value)
        self._tunnel_mandatory = _param_as_bool(self.get_parameter("tunnel_mandatory").value)
        self._mission_only_reorder = _param_as_bool(self.get_parameter("mission_only_reorder_by_nearest").value)
        self._mission_only_keep_park_last = _param_as_bool(self.get_parameter("mission_only_keep_park_last").value)
        self._heading_offset = float(self.get_parameter("heading_offset_deg").value)
        self._heading_is_enu = _param_as_bool(self.get_parameter("heading_is_enu_yaw").value)
        self._require_go = _param_as_bool(self.get_parameter("require_go_signal").value)
        self._go_ok: bool = False
        self._go_stamp: float = 0.0

        self._manager: MissionManager | None = None
        self._base_plan = None
        if mission_file:
            reader = GeoJsonMissionReader()
            self._base_plan = reader.read_file(mission_file)
        else:
            self.get_logger().warn("mission_file parametresi boş — waypoint takibi pasif")

        # Yol graph'ı (lane centerlines) — node/edge'ler açılışta otomatik çıkarılır.
        self._route_graph = None
        if centerlines_file:
            try:
                gj = load_centerlines_geojson(centerlines_file)
                self._route_graph = build_graph_from_centerlines_geojson(gj, coord_round_decimals=centerlines_round)
                n_nodes = len(self._route_graph.nodes)
                n_edges = sum(len(v) for v in self._route_graph.adj.values())
                self.get_logger().info(
                    f"Centerlines yüklendi: nodes={n_nodes} edges={n_edges} — {centerlines_file}"
                )
            except Exception as e:
                self.get_logger().error(f"Centerlines yüklenemedi: {centerlines_file} — {e}")

        # Döner kavşak düğümleri (centerlines props tur='kavsak'): halka üzerindeki
        # plan noktaları kavşak-dönüşü mekanizmasıyla (plan önceliği + pure pursuit)
        # izlenir — bacak açısı eşiği halkanın yumuşak kıvrımını yakalayamaz, yoksa araç
        # halkada düz devam ediyor.
        self._rb_keys: set[tuple[float, float]] = set()
        self._rb_center_ll: tuple[float, float] | None = None
        self._rb_node_radius_m: float = 0.0
        if self._route_graph is not None:
            by_id = {n.id: n for n in self._route_graph.nodes}
            for _edges in self._route_graph.adj.values():
                for _e in _edges:
                    if str((_e.props or {}).get("tur", "")) == "kavsak":
                        for _nid in (int(_e.u), int(_e.v)):
                            _n = by_id.get(_nid)
                            if _n is not None:
                                self._rb_keys.add((round(float(_n.lat), 7), round(float(_n.lon), 7)))
            if self._rb_keys:
                # Halkanın MERKEZİ (tam daire olduğu için düğümlerin ağırlık merkezi)
                # ve ortalama yarıçapı — iz dışarı kaydırılırken kullanılır (bkz.
                # RB_LANE_RADIUS_M ve _begin_roundabout).
                _lats = [k[0] for k in self._rb_keys]
                _lons = [k[1] for k in self._rb_keys]
                self._rb_center_ll = (sum(_lats) / len(_lats), sum(_lons) / len(_lons))
                _rs = [math.hypot(*self._ll_to_enu_m(la, lo, *self._rb_center_ll))
                       for la, lo in self._rb_keys]
                self._rb_node_radius_m = sum(_rs) / len(_rs)
                self.get_logger().info(
                    f"Döner kavşak: {len(self._rb_keys)} düğüm, merkez çizgisi yarıçapı "
                    f"{self._rb_node_radius_m:.1f} m -> iz {RB_LANE_RADIUS_M:.1f} m'ye "
                    f"kaydırılacak (sürülebilir halka 5.3-10.7 m, ortası 8.0)")

        # Routing: Mission noktalarını graph'a snap edip Dijkstra ile bir route waypoint listesi üret.
        self._blocked_edges: set[tuple[int, int]] = set()
        self._road_blocked: bool = False
        self._road_blocked_streak: int = 0  # debounce: tek karelik sahte road_blocked replan tetiklemesin
        self._entry_blocked: bool = False
        self._mission_idx: int = 0  # base_plan hedef indeksi (replan için)
        self._last_turn_replan_sig: str = ""
        self._last_turn_replan_t: float = 0.0
        # Rota oturmamışken (replan sürerken/rota bulunamazken) dönüş bastırılır
        self._route_unstable_until: float = 0.0
        self._no_route_until: float = 0.0
        self._route_unstable_logged: bool = False
        # Kavşak dönüş durumu (vehicle_controller'a PLAN önceliği sinyali)
        self._turn_active: bool = False
        self._turn_is_roundabout: bool = False
        self._turn_rb_exit_idx: int | None = None
        self._turn_wp_index: int | None = None
        self._turn_path: list | None = None            # dönüş yayı ENU noktaları (köşe=origin)
        self._turn_path_origin: tuple | None = None    # yay ENU çerçevesinin (lat, lon) orijini
        self._turn_arc_s_exit: float = 0.0             # yay bitiş (T2) kümülatif mesafesi
        self._turn_arc_s_here: float = 0.0             # aracın yay üzerindeki son ilerlemesi
        self._turn_engage_dist: float | None = None    # dönüş tetiklendiğindeki wp mesafesi
        self._turn_is_right: bool = False              # dönüş yönü: sağ mı? (çıkış eşiği yöne göre)
        self._curve_skip_logged_idx: int = -1  # viraj-atlandı logu wp başına bir kez
        self._plan_points: list = []  # aktif planın noktaları (dönüş geometrisi için)
        # Rotadan sapma bekçisi durumu
        self._min_dist_to_wp: float = float("inf")
        self._min_dist_wp_idx: int = -1
        self._last_offroute_replan_t: float = 0.0

        if self._base_plan is not None:
            plan = self._base_plan
            if self._route_graph is not None:
                try:
                    plan = route_mission_plan_via_graph(
                        self._base_plan,
                        self._route_graph,
                        tunnel_mandatory=self._tunnel_mandatory,
                    )
                    self.get_logger().info(
                        f"Routing aktif: {len(self._base_plan)} hedef -> {len(plan)} route waypoint"
                    )
                except Exception as e:
                    self.get_logger().error(f"Routing başarısız, mission plan kullanılacak — {e}")
                    plan = self._base_plan
            else:
                plan = route_mission_plan_without_graph(
                    self._base_plan,
                    reorder_checkpoints_by_nearest=self._mission_only_reorder,
                    keep_park_last=self._mission_only_keep_park_last,
                )
                self.get_logger().info(
                    f"Mission-only routing aktif: {len(self._base_plan)} hedef -> {len(plan)} waypoint"
                )
            self._manager = MissionManager(plan)
            self._plan_points = list(plan.points)  # dönüş tespiti için rota geometrisi
            self.get_logger().info(f"Görev yüklendi: {len(plan)} waypoint — {mission_file}")

        self._heading_deg = 0.0
        self._gps_stamp: float = 0.0
        self._final_odom_stamp: float = 0.0
        # Dönüş kontrolü için konum ve heading aynı kaynaktan gelmeli. Konum GPS'ten
        # (~0.9 Hz), heading odom'dan (~50 Hz) alınırsa araç dönerken heading döner ama
        # konum donuk kalır; pure-pursuit bearing hatası salınır ve dönüş çok uzar.
        self._datum_lat_deg = float(self.get_parameter("datum_lat").value)
        self._datum_lon_deg = float(self.get_parameter("datum_lon").value)
        _dlat = math.radians(self._datum_lat_deg)
        _A = 6378137.0
        _E2 = 6.69437999014e-3
        _s2 = math.sin(_dlat) ** 2
        _n = _A / math.sqrt(1.0 - _E2 * _s2)
        _m = _A * (1.0 - _E2) / (1.0 - _E2 * _s2) ** 1.5
        self._m_per_deg_lat = math.radians(1.0) * _m       # final_odom ile AYNI (WGS84)
        self._m_per_deg_lon = math.radians(1.0) * _n * math.cos(_dlat)
        self._odom_lat: float | None = None
        self._odom_lon: float | None = None
        self._last_turn_dist_m: float = 999.0   # son wp mesafesi (odom taze direksiyon için)
        self._last_ctrl_xy: tuple[float, float] | None = None  # son kontrol pozu (mesafe kapısı)

        self._last_steering = 0.0
        self._last_speed = 0.0
        self._last_task = ""
        # _tick_timeout'un yeniden yayınlaması için son yayın değerleri (_publish doldurur)
        self._last_arrived = False
        self._last_park_mode = False
        self._last_park_remaining = 0.0
        self._turn_perm: dict | None = None
        # Dönüş kısıtı kavşak-kapsamı durumu (bkz. _track_restriction_junction)
        self._restr_junction: tuple[float, float] | None = None
        self._restr_junction_min_d: float = float("inf")
        self._turn_perm_ignore_until: float = 0.0

        self.create_subscription(NavSatFix, str(self.get_parameter("gps_fix_topic").value), self._gps_cb, 10)
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self._imu_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter("final_odom_topic").value), self._final_odom_cb, 10)
        self.create_subscription(
            String, str(self.get_parameter("perception_turn_permissions_topic").value), self._turn_perm_cb, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("perception_decision_debug_topic").value), self._decision_debug_cb, 10
        )
        self.create_subscription(Bool, str(self.get_parameter("go_topic").value), self._go_cb, 10)

        self._pub_steer = self.create_publisher(Float32, str(self.get_parameter("planning_steering_topic").value), 10)
        self._pub_speed = self.create_publisher(Float32, str(self.get_parameter("planning_speed_topic").value), 10)
        self._pub_task = self.create_publisher(String, str(self.get_parameter("planning_current_task_topic").value), 10)
        self._pub_arrived = self.create_publisher(Bool, str(self.get_parameter("planning_arrived_topic").value), 10)
        self._pub_park_mode = self.create_publisher(Bool, str(self.get_parameter("planning_park_mode_topic").value), 10)
        self._pub_park_remaining = self.create_publisher(
            Float32, str(self.get_parameter("planning_park_remaining_topic").value), 10
        )
        self._pub_turn_active = self.create_publisher(
            Bool, str(self.get_parameter("planning_turn_active_topic").value), 10
        )
        self._pub_intersection_passed = self.create_publisher(
            Bool, str(self.get_parameter("planning_intersection_passed_topic").value), 10
        )

        # Park tamamlandı sinyali (perception) — park girişinden sonra 3dk içinde park etmek için
        self.create_subscription(
            Bool, str(self.get_parameter("perception_park_complete_topic").value), self._park_complete_cb, 10
        )

        self.create_timer(0.2, self._tick_timeout)  # 5 Hz

    def _turn_blocked_edges(self, *, cur_node: int, heading_deg: float) -> set[tuple[int, int]]:
        """
        Turn permissions -> temporarily blocked directed edges out of `cur_node`.
        Lightweight approximation: classify outgoing edges as left/straight/right
        by comparing edge bearing to current heading.
        """
        if self._turn_perm is None or self._route_graph is None:
            return set()

        forced = self._turn_perm.get("forced_direction")
        left_ok = bool(self._turn_perm.get("left", True))
        straight_ok = bool(self._turn_perm.get("straight", True))
        right_ok = bool(self._turn_perm.get("right", True))

        # Treat pass_left/pass_right/roundabout as non-forced for global routing
        if forced in {"pass_left", "pass_right", "roundabout", None}:
            forced = None

        by_id = {n.id: n for n in self._route_graph.nodes}
        n0 = by_id.get(int(cur_node))
        if n0 is None:
            return set()

        edges = self._route_graph.adj.get(int(cur_node), [])
        if not edges:
            return set()

        blocked: set[tuple[int, int]] = set()
        straight_thr = 25.0
        side_thr = 25.0

        for e in edges:
            n1 = by_id.get(int(e.v))
            if n1 is None:
                continue
            b = _bearing_deg(n0.lat, n0.lon, n1.lat, n1.lon)
            rel = _angle_diff_deg(b, float(heading_deg))  # + => right, - => left

            is_left = rel <= -side_thr
            is_right = rel >= side_thr
            is_straight = abs(rel) < straight_thr

            if forced == "left":
                if not is_left:
                    blocked.add((int(e.u), int(e.v)))
                continue
            if forced == "right":
                if not is_right:
                    blocked.add((int(e.u), int(e.v)))
                continue
            if forced == "straight":
                if not is_straight:
                    blocked.add((int(e.u), int(e.v)))
                continue

            if (not left_ok) and is_left:
                blocked.add((int(e.u), int(e.v)))
            if (not right_ok) and is_right:
                blocked.add((int(e.u), int(e.v)))
            if (not straight_ok) and is_straight:
                blocked.add((int(e.u), int(e.v)))

        return blocked

    def _go_cb(self, msg: Bool) -> None:
        self._go_ok = bool(msg.data)
        self._go_stamp = time.monotonic()

    def _park_complete_cb(self, msg: Bool) -> None:
        if self._manager is None:
            return
        if bool(msg.data):
            self._manager.notify_park_completed()

    def _turn_perm_cb(self, msg: String) -> None:
        try:
            perm = json.loads(msg.data) if msg.data else None
        except Exception:
            perm = None
        # Kavşak yeni geçildi: perception'ın temizliği yetişene dek bayat kısıtları yok say
        if (
            perm is not None
            and self._perm_is_restrictive(perm)
            and time.monotonic() < self._turn_perm_ignore_until
        ):
            perm = None
        self._turn_perm = perm

    @staticmethod
    def _perm_is_restrictive(perm: dict) -> bool:
        """turn_permissions rota/dönüş kısıtı içeriyor mu? (pass_left/right ve
        roundabout geçici manevra ipuçlarıdır, rota kısıtı sayılmaz)."""
        if perm.get("forced_direction") in {"left", "right", "straight"}:
            return True
        return not (
            bool(perm.get("left", True))
            and bool(perm.get("straight", True))
            and bool(perm.get("right", True))
        )

    def _track_restriction_junction(self, pos, wp_state) -> None:
        """
        Aktif dönüş kısıtını, uygulandığı KAVŞAK waypoint'ine bağlar; araç o
        kavşaktan INTERSECTION_PASSED_HYST_M uzaklaşınca perception'a
        intersection_passed yayınlar (traffic_sign_logic bekleyen kısıtları
        temizler). Kısıt koordinata bağlandığından waypoint ilerlese de takip sürer.
        """
        if self._restr_junction is None:
            if (
                self._turn_perm is None
                or not self._perm_is_restrictive(self._turn_perm)
                or wp_state.current_wp is None
                or float(wp_state.distance_to_wp_m) > TURN_RULE_APPLY_DISTANCE_M
                or not self._is_junction(wp_state.current_wp)
            ):
                return
            self._restr_junction = (float(wp_state.current_wp.lat), float(wp_state.current_wp.lon))
            self._restr_junction_min_d = float(wp_state.distance_to_wp_m)
            self.get_logger().info(
                f"Dönüş kısıtı kavşağa bağlandı: ({self._restr_junction[0]:.7f}, {self._restr_junction[1]:.7f})"
            )
            return
        ex, ny = self._ll_to_enu_m(pos.lat, pos.lon, self._restr_junction[0], self._restr_junction[1])
        d = math.hypot(ex, ny)
        if d < self._restr_junction_min_d:
            self._restr_junction_min_d = d
            return
        if d >= self._restr_junction_min_d + INTERSECTION_PASSED_HYST_M:
            self._pub_intersection_passed.publish(Bool(data=True))
            self._turn_perm = None
            self._turn_perm_ignore_until = time.monotonic() + TURN_PERM_CLEAR_GRACE_S
            self._restr_junction = None
            self._restr_junction_min_d = float("inf")
            self.get_logger().warn("KAVŞAK GEÇİLDİ: tabela dönüş kısıtları temizlendi (intersection_passed)")

    def _decision_debug_cb(self, msg: String) -> None:
        # decision_debug JSON: {final: {...}, candidates: [...], entry_blocked: bool, ...}
        try:
            d = json.loads(msg.data) if msg.data else {}
            reasons = d.get("final", {}).get("reasons") or d.get("reasons") or []
            rb = any(str(r) == "road_blocked" for r in reasons)
            # Debounce: kalıcı kenar bloklaması pahalı bir karar — algıdan art arda
            # en az 3 mesaj (5 Hz'de ~0.6 s) road_blocked gelmeden tetikleme.
            self._road_blocked_streak = self._road_blocked_streak + 1 if rb else 0
            self._road_blocked = self._road_blocked_streak >= 3
            self._entry_blocked = bool(d.get("entry_blocked", False))
        except Exception:
            self._road_blocked = False
            self._road_blocked_streak = 0
            self._entry_blocked = False

    def _reset_route_tracking(self) -> None:
        """Replan sonrası: waypoint indeksleri değişti, dönüş/sapma takibini sıfırla."""
        self._turn_active = False
        self._turn_is_roundabout = False
        self._turn_rb_exit_idx = None
        self._turn_wp_index = None
        self._turn_path = None
        self._turn_path_origin = None
        self._turn_arc_s_exit = 0.0
        self._turn_arc_s_here = 0.0
        self._min_dist_to_wp = float("inf")
        self._min_dist_wp_idx = -1

    def _is_junction(self, wp) -> bool:
        """
        Dönüş noktası gerçek bir KAVŞAK mı? Graph'ta düğümden 2+ çıkış varsa
        kavşaktır (düz gitme/dönme seçeneği var — ONNX'in düz şeridi TURN
        önceliğini gerektirir). Tek çıkışlı düğüm VİRAJDIR: yol zaten kıvrılır,
        şerit takibi virajı kendisi döner, TURN önceliği gereksiz ve zararlıdır.
        """
        if self._route_graph is None:
            return True  # graph yoksa ayrım yapılamaz: eski (her dönüşte) davranış
        try:
            nid = int(nearest_node_id(self._route_graph, lat=float(wp.lat), lon=float(wp.lon)))
            return len(self._route_graph.adj.get(nid, [])) >= 2
        except Exception:
            return True

    @staticmethod
    def _ll_to_enu_m(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
        """(lat,lon) -> (E,N) metre, (lat0,lon0) orijinli eş-dikdörtgen yaklaşım."""
        dn = math.radians(float(lat) - float(lat0)) * EARTH_R_M
        de = math.radians(float(lon) - float(lon0)) * EARTH_R_M * math.cos(math.radians(float(lat0)))
        return de, dn

    def _build_turn_arc(self, cur, nxt, leg_in_deg: float, turn_deg: float) -> None:
        """
        Giriş/çıkış bacaklarına teğet dairesel yay üretir: dönüş, köşe noktası yerine
        giriş->çıkış arasında tanımlı bir rota üzerinden takip edilir. Yay köşeden önce
        başladığından araç direksiyonu erken kırar ve şeritte kalır. Nokta listesi
        köşe-orijinli ENU'dur; başına/sonuna düz koşular eklenir.
        """
        lat0, lon0 = float(cur.lat), float(cur.lon)
        ex, ny = self._ll_to_enu_m(float(nxt.lat), float(nxt.lon), lat0, lon0)
        leg_out_len = math.hypot(ex, ny)
        if leg_out_len < 1.0:
            self._turn_path = None
            self._turn_path_origin = None
            return
        b_in = math.radians(float(leg_in_deg))
        u_in = (math.sin(b_in), math.cos(b_in))          # pusula -> ENU birim vektör
        u_out = (ex / leg_out_len, ny / leg_out_len)
        theta = math.radians(min(170.0, abs(float(turn_deg))))
        r = TURN_ARC_RADIUS_M
        t = r * math.tan(theta / 2.0)
        t_max = max(2.0, 0.6 * leg_out_len)              # çıkış bacağından taşma
        if t > t_max:
            t = t_max
            r = t / math.tan(theta / 2.0)
        left = float(turn_deg) < 0.0                     # pusulada negatif fark = sola
        # Köşe kesme derinliği: apeksin köşeden içeri girme mesafesi.
        # Sınırı aşarsa köşeyi açıortay boyunca dışa kaydır (kesme tarafının tersi).
        depth = r * (1.0 / max(1e-6, math.cos(theta / 2.0)) - 1.0)
        d_out = max(0.0, depth - TURN_ARC_MAX_INNER_CUT_M)
        sin_half = math.sin(theta / 2.0)
        if sin_half > 1e-6:
            d_out = min(d_out, TURN_ARC_MAX_OUT_SWING_M / sin_half)
        bx, by = u_out[0] - u_in[0], u_out[1] - u_in[1]  # açıortay (kesme tarafına bakar)
        bl = math.hypot(bx, by)
        ox, oy = ((-bx / bl * d_out, -by / bl * d_out) if (bl > 1e-6 and d_out > 0.0) else (0.0, 0.0))
        t1 = (ox - u_in[0] * t, oy - u_in[1] * t)        # giriş teğet noktası
        t2 = (ox + u_out[0] * t, oy + u_out[1] * t)      # çıkış teğet noktası
        nrm = (-u_in[1], u_in[0]) if left else (u_in[1], -u_in[0])
        cx, cy = t1[0] + nrm[0] * r, t1[1] + nrm[1] * r  # yay merkezi
        a1 = math.atan2(t1[1] - cy, t1[0] - cx)
        a2 = math.atan2(t2[1] - cy, t2[0] - cx)
        sweep = a2 - a1
        if left and sweep < 0.0:
            sweep += 2.0 * math.pi
        if not left and sweep > 0.0:
            sweep -= 2.0 * math.pi
        pts: list[tuple[float, float]] = []
        n_pre = max(1, int(TURN_ARC_PRE_M / 2.0))
        for i in range(n_pre, 0, -1):
            # Dışa kayma yaklaşma boyunca 0 -> tam değere rampalanır: uzak uçta
            # şerit merkezinde kal, T1'e doğru yumuşakça dışa süzül (ani kırış olmasın).
            back = 2.0 * i
            scale = max(0.0, 1.0 - back / TURN_ARC_PRE_M)
            px_ = ox * scale - u_in[0] * (t + back)
            py_ = oy * scale - u_in[1] * (t + back)
            pts.append((px_, py_))
        n_arc = max(4, int(abs(sweep) * r))              # ~1 m aralıklı örnekleme
        for i in range(n_arc + 1):
            a = a1 + sweep * i / n_arc
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        s_exit = 0.0  # T2'ye kadar kümülatif yol (pre + yay) — dönüş bitiş eşiği
        for i in range(len(pts) - 1):
            s_exit += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        n_post = max(1, int(TURN_ARC_POST_M / 2.0))
        for i in range(1, n_post + 1):
            pts.append((t2[0] + u_out[0] * 2.0 * i, t2[1] + u_out[1] * 2.0 * i))
        self._turn_path = pts
        self._turn_path_origin = (lat0, lon0)
        self._turn_arc_s_exit = s_exit
        self._turn_arc_s_here = 0.0

    def _turn_late_gain(self) -> float:
        """GEÇ tetiklenen kavşak dönüşünde steer kazancı. Köşe ancak önceki wp geçilince
        current olduğundan dönüş ~6 m kala başlıyor (10 m yerine); yakınlıkla orantılı
        güçlendirme catch-up sağlar. Roundabout muaf.
        Kazanç = clamp(TURN_ENGAGE_REF / engage_dist, 1.0, TURN_LATE_MAX_GAIN).
          10 m -> 1.00x,  6 m -> 1.67x,  ≤5.6 m -> 1.80x (tavan)."""
        if self._turn_is_roundabout or self._turn_engage_dist is None:
            return 1.0
        d = max(1.0, float(self._turn_engage_dist))
        return max(1.0, min(TURN_LATE_MAX_GAIN, TURN_ENGAGE_REF / d))

    def _turn_arc_steering(self, pos: GpsPosition) -> float | None:
        """Dönüş yayı üzerinde lookahead noktasına nişan + yaydan yanal sapma düzeltmesi."""
        if not self._turn_path or self._turn_path_origin is None:
            return None
        lat0, lon0 = self._turn_path_origin
        px, py = self._ll_to_enu_m(pos.lat, pos.lon, lat0, lon0)
        pts = self._turn_path
        best = None  # (d2, s_projeksiyon, segment_birim, projeksiyon_noktası)
        s_acc = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            dx, dy = bx - ax, by - ay
            seg = math.hypot(dx, dy)
            if seg < 1e-9:
                continue
            tt = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (seg * seg)))
            qx, qy = ax + tt * dx, ay + tt * dy
            d2 = (px - qx) ** 2 + (py - qy) ** 2
            if best is None or d2 < best[0]:
                best = (d2, s_acc + tt * seg, (dx / seg, dy / seg), (qx, qy))
            s_acc += seg
        if best is None:
            return None
        total_s = s_acc
        _, s_here, seg_u, proj = best
        self._turn_arc_s_here = float(s_here)
        s_target = min(total_s, s_here + TURN_ARC_LOOKAHEAD_M)
        tx, ty = pts[-1]
        s_acc = 0.0
        for i in range(len(pts) - 1):
            seg = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            if s_acc + seg >= s_target and seg > 1e-9:
                f = (s_target - s_acc) / seg
                tx = pts[i][0] + f * (pts[i + 1][0] - pts[i][0])
                ty = pts[i][1] + f * (pts[i + 1][1] - pts[i][1])
                break
            s_acc += seg
        # Pure pursuit: hedefe bakış açısından gereken eğrilik -> tekerlek açısı.
        # Düz-bacak kazancı (b_err/90) 3.5 m'lik yayı süremez: yay ~0.85 oran ister,
        # küçük açı hatası o kazançla asla bu değeri üretemez.
        des_bearing = math.degrees(math.atan2(tx - px, ty - py))  # atan2(E,N) = pusula
        alpha = math.radians(_angle_diff_deg(des_bearing, float(pos.heading_deg)))  # + = sağda
        ld = max(1.0, math.hypot(tx - px, ty - py))
        kappa = 2.0 * math.sin(alpha) / ld
        delta = math.atan(kappa * WHEELBASE_M)           # + = sağ teker açısı
        # GEÇ tetikleme telafisi: yakın başlayan dönüşte steer'i güçlendir (yalnız
        # kavşak; roundabout muaf). Kazanç pure-pursuit TALEBİYLE ölçeklenir — hizalanınca
        # talep küçülür, kazanç etkisiz kalır (kendini sınırlar), çıkışta aşırı kırış yok.
        steer = delta / MAX_STEER_RAD * self._turn_late_gain()
        return max(-1.0, min(1.0, steer))

    def _is_roundabout_pt(self, p) -> bool:
        """Plan noktası döner kavşak halka düğümü mü? (graph düğümlerine snap'li
        rota noktaları koordinat eşleşmesiyle bulunur — 7 hane, graph yuvarlamasıyla aynı)"""
        if not self._rb_keys or p is None:
            return False
        return (round(float(p.lat), 7), round(float(p.lon), 7)) in self._rb_keys

    def _begin_roundabout(self, idx: int, wp_state) -> None:
        """Halka boyunca plan noktalarından pure-pursuit izi kur ve PLAN önceliğini aç.
        İz: yaklaşma ön-koşusu + halka düğümleri + çıkış yolu ilk düğümü + lookahead
        payı. Bitiş eşiği (s_exit) çıkış yolu düğümüne ulaşmaktır — sonrası şerit."""
        n = len(self._plan_points)
        if idx >= n:
            return
        k = idx
        while k + 1 < n and self._is_roundabout_pt(self._plan_points[k + 1]):
            k += 1
        exit_idx = min(n - 1, k + 1)  # halkadan sonraki ilk normal yol noktası
        i0 = max(0, idx - 1)
        origin = self._plan_points[idx]
        lat0, lon0 = float(origin.lat), float(origin.lon)
        seg = self._plan_points[i0:exit_idx + 1]
        pts = [self._ll_to_enu_m(float(p.lat), float(p.lon), lat0, lon0) for p in seg]
        if len(pts) < 2:
            return
        # ── HALKA İZİNİ DIŞARI KAYDIR (bkz. RB_LANE_RADIUS_M) ──
        # Yalnız halka düğümleri kaydırılır; giriş/çıkış bacakları yerinde kalır,
        # pure-pursuit aradaki geçişi yumuşatır. Yan etki (istenen): araç halkaya
        # daha erken/daha geniş girer — "kavşağa biraz mesafe koyarak dön".
        if self._rb_center_ll is not None:
            ccx, ccy = self._ll_to_enu_m(self._rb_center_ll[0], self._rb_center_ll[1],
                                         lat0, lon0)
            for i, p in enumerate(seg):
                if not self._is_roundabout_pt(p):
                    continue
                dx, dy = pts[i][0] - ccx, pts[i][1] - ccy
                r = math.hypot(dx, dy)
                if r < 1e-6:
                    continue
                pts[i] = (ccx + dx / r * RB_LANE_RADIUS_M,
                          ccy + dy / r * RB_LANE_RADIUS_M)
        # Yaklaşma ön-koşusu: ilk bacak yönünde geriye uzat (araç halkaya varmadan
        # önce de izin üzerine projeksiyon alabilsin)
        ux, uy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        ul = math.hypot(ux, uy)
        n_pre = 0
        if ul > 1e-6:
            ux, uy = ux / ul, uy / ul
            n_pre = max(1, int(TURN_ARC_PRE_M / 2.0))
            pre = [(pts[0][0] - ux * 2.0 * i, pts[0][1] - uy * 2.0 * i) for i in range(n_pre, 0, -1)]
            pts = pre + pts
        # Bitiş eşiği: halkanın SON düğümü + bağlanma payı — araç çıkış yoluna
        # oturunca kontrol şeride döner (çıkış yolunu plan ile sürmeye gerek yok)
        last_ring_i = n_pre + (k - i0)
        s_exit = 0.0
        for i in range(min(last_ring_i, len(pts) - 1)):
            s_exit += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        s_exit += 4.0
        # Çıkış sonrası düz koşu (lookahead payı)
        vx, vy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        vl = math.hypot(vx, vy)
        if vl > 1e-6:
            vx, vy = vx / vl, vy / vl
            bx, by = pts[-1]
            n_post = max(1, int(TURN_ARC_POST_M / 2.0))
            for i in range(1, n_post + 1):
                pts.append((bx + vx * 2.0 * i, by + vy * 2.0 * i))
        self._turn_active = True
        self._turn_is_roundabout = True
        self._turn_wp_index = idx
        self._turn_rb_exit_idx = int(exit_idx)
        self._turn_path = pts
        self._turn_path_origin = (lat0, lon0)
        self._turn_arc_s_exit = s_exit
        self._turn_arc_s_here = 0.0
        self.get_logger().info(
            f"DÖNER KAVŞAK başlıyor: wp[{idx}..{exit_idx}] halka={k - idx + 1} düğüm "
            f"iz={s_exit:.1f}m — plan önceliği (pure pursuit)"
        )

    def _route_unstable(self) -> bool:
        """Rota yeniden hesaplanıyor mu / kısıt altında rota bulunamadı mı?
        True iken kavşak dönüşü tetiklenmez, aktif dönüş bırakılır (şerit takibi sürer)."""
        return time.monotonic() < self._route_unstable_until

    def _update_turn_state(self, wp_state) -> None:
        """
        Kavşak dönüş tespiti. Giriş bacağı ROTA GEOMETRİSİNDEN alınır (önceki wp ->
        aktif wp) — araç->waypoint bearing'i DEĞİL: o, waypoint'e yaklaşırken küçük
        yanal sapmalarda bile şişip sahte dönüş tetikliyordu (araç durak yapısına
        dönüp kilitleniyordu). Çıkış
        bacağı aktif wp -> sonraki wp. Açı TURN_MIN_ANGLE_DEG'i aşıyorsa, waypoint
        yaklaşıldıysa VE nokta gerçek bir kavşaksa (_is_junction) turn_active=True.
        Waypoint geçilip (advance) araç yeni bacağa hizalanınca biter. Sinyali
        vehicle_controller PLAN önceliği olarak kullanır.
        """
        idx = int(wp_state.wp_index)

        # ── ROTA OTURMAMIŞ: dönüş yok, kontrol şerit takibinde (bkz. ROUTE_UNSTABLE_HOLD_S) ──
        # Halka MUAF: halka izinin ortasında bırakmak aracı adaya sokar; halka zaten
        # rota kısıtından etkilenmiyor.
        if self._route_unstable() and not self._turn_is_roundabout:
            if self._turn_active:
                self.get_logger().warn(
                    f"ROTA YENİDEN HESAPLANIYOR: dönüş bırakıldı wp[{idx}] — şerit takibine geçildi"
                )
                self._turn_active = False
                self._turn_wp_index = None
                self._turn_path = None
                self._turn_path_origin = None
            return

        if self._turn_active and self._turn_is_roundabout:
            # Döner kavşak: hizalanma koşulu KULLANILMAZ — halka üzerinde araç
            # sıradaki yoğun wp'lere sık sık hizalanır, erken bırakma olur.
            # Bitiş: izin sonuna (çıkış yolu noktası) ulaşmak ya da çıkış wp'sinin
            # manager'ca geçilmesi. Sonra kontrol şerit takibine döner.
            arc_done = (
                self._turn_path is not None
                and self._turn_arc_s_here >= self._turn_arc_s_exit
            )
            passed_exit = (
                self._turn_rb_exit_idx is not None and idx > int(self._turn_rb_exit_idx)
            )
            if arc_done or passed_exit or self._turn_path is None:
                self.get_logger().info(
                    f"DÖNER KAVŞAK bitti: wp[{idx}] "
                    f"{'iz tamamlandı' if arc_done else 'çıkış wp geçildi'} — şerit takibine dönüldü"
                )
                self._turn_active = False
                self._turn_is_roundabout = False
                self._turn_rb_exit_idx = None
                self._turn_wp_index = None
                self._turn_path = None
                self._turn_path_origin = None
            return

        if self._turn_active:
            if self._turn_wp_index is not None and idx != self._turn_wp_index:
                # Bitiş: yeni bacağa hizalanma YA DA yayın çıkış teğetini (T2) geçme.
                # Yalnız hizalanma beklemek dönüşü geciktiriyor: araç yola oturduğu hâlde
                # waypoint yanda kaldığından b_err geç küçülüyor.
                right = bool(self._turn_is_right)
                lead = TURN_ARC_RELEASE_LEAD_RIGHT_M if right else TURN_ARC_RELEASE_LEAD_M
                align = TURN_EXIT_ALIGN_DEG_RIGHT if right else TURN_EXIT_ALIGN_DEG
                arc_done = (
                    self._turn_path is not None
                    and self._turn_arc_s_here >= max(0.0, self._turn_arc_s_exit - lead)
                )
                if abs(float(wp_state.bearing_error_deg)) <= align or arc_done:
                    self.get_logger().info(
                        f"DÖNÜŞ bitti: wp[{idx}] {'SAĞ' if right else 'SOL'} "
                        f"{'yay tamamlandı' if arc_done else 'yeni bacağa hizalandı'} "
                        f"(b_err={wp_state.bearing_error_deg:+.0f}°, eşik={align:.0f}°)"
                    )
                    self._turn_active = False
                    self._turn_wp_index = None
                    self._turn_path = None
                    self._turn_path_origin = None
            return

        cur = wp_state.current_wp
        nxt = wp_state.next_wp
        if cur is None:
            return
        # Döner kavşak girişi: aktif hedef halka düğümüyse plan önceliğine geç. Halka,
        # kavşak dönüşüyle aynı mekanizmayla izlenir; çıkış yoluna bağlanınca kontrol
        # şeride bırakılır.
        if (
            self._is_roundabout_pt(cur)
            and float(wp_state.distance_to_wp_m) <= TURN_ENGAGE_DISTANCE_M
        ):
            self._begin_roundabout(idx, wp_state)
            return
        if nxt is None:
            return
        leg_out = _bearing_deg(float(cur.lat), float(cur.lon), float(nxt.lat), float(nxt.lon))
        prev = self._plan_points[idx - 1] if 1 <= idx < len(self._plan_points) else None
        if prev is not None:
            leg_in = _bearing_deg(float(prev.lat), float(prev.lon), float(cur.lat), float(cur.lon))
        else:
            leg_in = float(wp_state.bearing_to_wp_deg)  # ilk wp: geometri yok, pos->wp
        turn_deg = _angle_diff_deg(leg_out, leg_in)
        if abs(turn_deg) >= TURN_MIN_ANGLE_DEG and float(wp_state.distance_to_wp_m) <= TURN_ENGAGE_DISTANCE_M:
            if not self._is_junction(cur):
                if self._curve_skip_logged_idx != idx:
                    self._curve_skip_logged_idx = idx
                    self.get_logger().info(
                        f"VİRAJ (kavşak değil): wp[{idx}] '{cur.name}' açı={turn_deg:+.0f}° "
                        f"— şerit takibine bırakıldı"
                    )
                return
            self._turn_active = True
            self._turn_wp_index = idx
            # Pusula farkı: + = SAĞ, − = SOL. Çıkış koşulu yöne göre ayrışır.
            self._turn_is_right = bool(turn_deg > 0.0)
            # Tetikleme anındaki mesafe: dönüş geç fark edilirse (köşe ancak önceki wp
            # geçilince current olur -> ~6 m kala) steer bu mesafeye göre güçlendirilir
            # (bkz. _turn_arc_steering / _turn_late_gain). Yalnız kavşak dönüşü; viraj
            # şerit takibinde, halka arc'ı da muaf.
            self._turn_engage_dist = float(wp_state.distance_to_wp_m)
            try:
                self._build_turn_arc(cur, nxt, leg_in, turn_deg)
            except Exception as e:
                self._turn_path = None
                self._turn_path_origin = None
                self.get_logger().warn(f"Dönüş yayı üretilemedi ({e}) — wp steering'e düşüldü")
            self.get_logger().info(
                f"DÖNÜŞ başlıyor: wp[{idx}] '{cur.name}' açı={turn_deg:+.0f}° "
                f"mesafe={wp_state.distance_to_wp_m:.1f}m "
                f"kazanç={self._turn_late_gain():.2f}x "
                f"yay={'ok (%d nokta)' % len(self._turn_path) if self._turn_path else 'yok'}"
            )

    def _offroute_watchdog(self, pos: GpsPosition, wp_state, park_mode: bool):
        """
        Rotadan sapma bekçisi: aktif hedefe mesafe, o hedef için görülen minimumun
        OFF_ROUTE_MARGIN_M üstüne çıkarsa (kaçırılan dönüş vb.) mevcut konumdan
        rota yeniden planlanır. Başarılıysa güncel (wp_state, mission_dec) döner.
        """
        idx = int(wp_state.wp_index)
        dist = float(wp_state.distance_to_wp_m)
        if idx != self._min_dist_wp_idx:
            self._min_dist_wp_idx = idx
            self._min_dist_to_wp = dist
            return None
        self._min_dist_to_wp = min(self._min_dist_to_wp, dist)

        if park_mode or self._route_graph is None or self._base_plan is None:
            return None
        if dist <= self._min_dist_to_wp + OFF_ROUTE_MARGIN_M:
            return None
        now_m = time.monotonic()
        if (now_m - self._last_offroute_replan_t) < OFF_ROUTE_REPLAN_COOLDOWN_S:
            return None
        self._last_offroute_replan_t = now_m
        self.get_logger().warn(
            f"ROTADAN SAPMA: wp[{idx}] mesafe {dist:.1f}m (görülen min {self._min_dist_to_wp:.1f}m) "
            f"— konumdan yeniden planlanıyor"
        )
        try:
            new_plan = route_remaining_mission_via_graph(
                self._base_plan,
                self._route_graph,
                current_lat=pos.lat,
                current_lon=pos.lon,
                start_index=self._mission_idx,
                blocked_edges=set(self._blocked_edges),
                fallback_to_original_on_failure=False,
                tunnel_mandatory=self._tunnel_mandatory,
            )
        except Exception as e:
            self.get_logger().error(f"OFF-ROUTE REPLAN başarısız: {e}")
            return None
        if not new_plan.points:
            self.get_logger().error("OFF-ROUTE REPLAN: no_route (son plan korundu)")
            return None
        self._manager = MissionManager(new_plan)
        self._plan_points = list(new_plan.points)
        self._reset_route_tracking()
        self.get_logger().warn(f"OFF-ROUTE REPLAN: ok -> new_route_waypoints={len(new_plan.points)}")
        return self._manager.update(pos, now_s=time.monotonic())

    def _apply_turn_permissions(self, steer: float, dist_to_wp_m: float) -> tuple[float, float]:
        """
        Tabela bazlı dönüş kısıtlarını waypoint'e yaklaşırken steer/speed üzerinde uygula.
        Çıktı: (steer, speed_multiplier)
        """
        if self._turn_perm is None:
            return steer, 1.0
        if dist_to_wp_m > TURN_RULE_APPLY_DISTANCE_M:
            return steer, 1.0

        forced = self._turn_perm.get("forced_direction")
        left_ok = bool(self._turn_perm.get("left", True))
        straight_ok = bool(self._turn_perm.get("straight", True))
        right_ok = bool(self._turn_perm.get("right", True))

        # forced_direction: left/right/straight/pass_left/pass_right/roundabout
        if forced == "left":
            return min(steer, -0.6), 0.7
        if forced == "right":
            return max(steer, 0.6), 0.7
        if forced == "straight":
            return 0.0, 0.7
        if forced == "pass_left":
            return min(steer, -0.35), 0.8
        if forced == "pass_right":
            return max(steer, 0.35), 0.8
        if forced == "roundabout":
            # Basit yaklaşım: hız düşür, steer'i sınırlama (roundabout için özel planner gerekebilir)
            return steer, 0.6

        # Yasak dönüşleri “yumuşak” şekilde engelle (tam replanning yok; güvenli yavaşlama)
        if not left_ok and steer < -0.2:
            return 0.0, 0.6
        if not right_ok and steer > 0.2:
            return 0.0, 0.6
        if not straight_ok and abs(steer) < 0.2:
            return (0.35 if right_ok else (-0.35 if left_ok else 0.0)), 0.6

        return steer, 1.0

    def _yaw_to_heading_deg(self, raw_deg: float) -> float:
        """Quaternion yaw (derece) -> pusula heading. ENU yaw ise (90-yaw) yansıması uygulanır."""
        if self._heading_is_enu:
            raw_deg = 90.0 - raw_deg
        return (raw_deg + self._heading_offset) % 360.0

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        if str(self.get_parameter("heading_source").value).strip().lower() == "imu":
            self._heading_deg = self._yaw_to_heading_deg(raw_deg)

    def _final_odom_cb(self, msg: Odometry) -> None:
        self._final_odom_stamp = time.monotonic()
        # Konum: dünya ENU (araç merkezi) -> lat/lon (final_odom'un tersi, aynı datum).
        p = msg.pose.pose.position
        self._odom_lat = self._datum_lat_deg + float(p.y) / self._m_per_deg_lat
        self._odom_lon = self._datum_lon_deg + float(p.x) / self._m_per_deg_lon

        if str(self.get_parameter("heading_source").value).strip().lower() != "final_odom":
            return
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        self._heading_deg = self._yaw_to_heading_deg(raw_deg)

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._gps_stamp = time.monotonic()

        if self._manager is None:
            self._publish(0.0, 0.0, "no_mission", False, False, 0.0)
            return

        if self._require_go and not self._go_ok:
            # Go gelmeden görev başlamasın: steering ref yayınla ama speed=0 ver
            pos0 = GpsPosition(lat=float(msg.latitude), lon=float(msg.longitude), heading_deg=self._heading_deg)
            wp_state, mission_dec = self._manager.update(pos0, now_s=time.monotonic())
            self._last_steering = float(wp_state.steering_ref)
            self._last_speed = 0.0
            self._last_task = "waiting_go"
            self._publish(self._last_steering, 0.0, self._last_task, False, mission_dec.park_mode, mission_dec.park_remaining_s)
            return

        pos = GpsPosition(lat=float(msg.latitude), lon=float(msg.longitude), heading_deg=self._heading_deg)
        # Base mission ilerleme (routing resetlerini güvenli yapmak için)
        if self._base_plan is not None:
            self._mission_idx = advance_mission_index_by_position(
                self._base_plan, start_index=self._mission_idx, lat=pos.lat, lon=pos.lon
            )

        wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())

        # Dönüş kısıtı kavşak-kapsamı: kısıt uygulanan kavşak geçildiyse temizlet
        self._track_restriction_junction(pos, wp_state)

        # Replanning:
        # - road_blocked: edge blocking
        # - turn_permissions: temporary blocked outgoing edges at current node (approaching waypoint)
        do_replan = (
            self._route_graph is not None
            and self._base_plan is not None
            and not bool(mission_dec.park_mode)
            and (
                self._road_blocked
                or self._entry_blocked
                or (self._turn_perm is not None and float(wp_state.distance_to_wp_m) <= TURN_RULE_APPLY_DISTANCE_M)
            )
            # Rota bulunamadıysa her GPS karesinde yeniden denemeyelim (replan fırtınası);
            # soğuma bitene kadar eski planla devam, dönüş zaten bastırık.
            and time.monotonic() >= self._no_route_until
        )
        if do_replan:
            try:
                cur_node = int(nearest_node_id(self._route_graph, lat=pos.lat, lon=pos.lon))
                # "Önümüzdeki" waypoint'i edge olarak bloke et (directed) — SADECE yol
                # gerçekten kapalıysa. Turn/entry replan'lerinde kalıcı kenar bloklamak,
                # tek yönlü şeritlerde düğümün tek çıkışını kapatıp rotasız bırakır.
                if self._road_blocked and wp_state.current_wp is not None:
                    nxt_node = int(nearest_node_id(self._route_graph, lat=float(wp_state.current_wp.lat), lon=float(wp_state.current_wp.lon)))
                    if cur_node != nxt_node:
                        self._blocked_edges.add((cur_node, nxt_node))

                turn_blocks = self._turn_blocked_edges(cur_node=cur_node, heading_deg=float(self._heading_deg))
                # Kısıt yoksa replan boş iş: turn_perm artık sürekli yayınlandığından
                # (izin verici olsa bile) bu dal aksi hâlde her waypoint yaklaşmasında
                # 0.8 s'de bir aynı rotayı yeniden üretip manager'ı sıfırlar.
                if not turn_blocks and not self._road_blocked and not self._entry_blocked:
                    raise RuntimeError("replan debounce")
                sig = json.dumps(
                    {
                        "road_blocked": bool(self._road_blocked),
                        "turn_perm": self._turn_perm,
                        "turn_blocks_n": len(turn_blocks),
                        "mission_idx": int(self._mission_idx),
                        "cur_node": int(cur_node),
                    },
                    sort_keys=True,
                    ensure_ascii=False,
                )
                now_m = time.monotonic()
                if (not self._road_blocked) and sig == self._last_turn_replan_sig and (now_m - self._last_turn_replan_t) < 0.8:
                    raise RuntimeError("replan debounce")

                # Buradan sonra rota gerçekten yeniden hesaplanıyor: kısıt altındaki
                # eski plan hâlâ yasak dönüşü gösterdiğinden dönüş önceliğini bırak,
                # kontrolü şerit takibine ver.
                if not self._route_unstable():
                    self.get_logger().warn("ROTA YENİDEN HESAPLANIYOR — dönüş bastırıldı")
                self._route_unstable_until = now_m + ROUTE_UNSTABLE_HOLD_S

                # fallback=False: rota bulunamazsa ham görev noktalarına DÜŞME —
                # aşağıdaki else dalı son geçerli rotayı korur (ham 5 nokta ile
                # manager'ı değiştirmek ara waypoint'leri ve tünel şartını kaybettirir).
                new_plan = route_remaining_mission_via_graph(
                    self._base_plan,
                    self._route_graph,
                    current_lat=pos.lat,
                    current_lon=pos.lon,
                    start_index=self._mission_idx,
                    blocked_edges=set(self._blocked_edges) | set(turn_blocks),
                    fallback_to_original_on_failure=False,
                    tunnel_mandatory=self._tunnel_mandatory,
                )
                if (
                    not self._road_blocked
                    and new_plan.points
                    and len(new_plan.points) == len(self._plan_points)
                    and all(
                        abs(float(a.lat) - float(b.lat)) < 1e-9 and abs(float(a.lon) - float(b.lon)) < 1e-9
                        for a, b in zip(new_plan.points, self._plan_points)
                    )
                ):
                    # Aynı rota geldi: manager'ı resetleme. Kısıt aktifken her ~1 s'de
                    # aynı rotayla reset, arrived/park sinyallerini titretiyor ve wp
                    # ilerlemesini kaybettiriyor.
                    self._last_turn_replan_sig = sig
                    self._last_turn_replan_t = now_m
                    raise RuntimeError("replan debounce")
                if new_plan.points:
                    self._manager = MissionManager(new_plan)
                    self._plan_points = list(new_plan.points)
                    self._road_blocked = False
                    self._entry_blocked = False
                    self._last_turn_replan_sig = sig
                    self._last_turn_replan_t = now_m
                    self._reset_route_tracking()
                    wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())
                    # Rota oturdu: dönüşler tekrar açılabilir.
                    self._route_unstable_until = 0.0
                    self._no_route_until = 0.0
                    self.get_logger().warn(
                        f"REPLAN: ok -> new_route_waypoints={len(new_plan.points)} blocked_edges={len(self._blocked_edges)} turn_blocks={len(turn_blocks)}"
                    )
                else:
                    # No route under constraints: keep last good plan/manager, keep moving.
                    # Dönüş bastırık kalır (rota oturmadı) + soğuma: her karede tekrar deneme.
                    self._no_route_until = now_m + NO_ROUTE_COOLDOWN_S
                    self._route_unstable_until = max(
                        self._route_unstable_until, now_m + NO_ROUTE_COOLDOWN_S + ROUTE_UNSTABLE_HOLD_S
                    )
                    self.get_logger().error(
                        f"REPLAN: no_route (kept last plan) blocked_edges={len(self._blocked_edges)} turn_blocks={len(turn_blocks)}"
                    )
            except Exception as e:
                if str(e) != "replan debounce":
                    self.get_logger().error(f"REPLAN başarısız: {e}")

        # Rotadan sapma bekçisi (gerekirse konumdan replan) + kavşak dönüş tespiti.
        # manager.update tek kez, satır içi çağrılmalı: iki kez çağırmak rota/replan
        # durumunu bozup no_route spam'ine ve yasak yola dönüşe yol açıyor.
        _res = self._offroute_watchdog(pos, wp_state, bool(mission_dec.park_mode))
        if _res is not None:
            wp_state, mission_dec = _res
        self._update_turn_state(wp_state)

        steer = float(wp_state.steering_ref)
        _src = "wp"
        if self._turn_active:
            _arc_steer = self._turn_arc_steering(pos)
            if _arc_steer is not None:
                steer = float(_arc_steer)
                _src = "halka" if self._turn_is_roundabout else "yay"
        _steer_pre = steer
        base_speed = float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio)
        _dist = float(wp_state.distance_to_wp_m)
        self._last_turn_dist_m = _dist
        steer, turn_speed_mul = self._apply_turn_permissions(steer, _dist)

        # Durum logu: konum/yön ve direksiyonun katman katman izi
        # (steer + = SAĞ; kaynak wp/yay/halka -> tabela kısıtı -> yayınlanan değer).
        _x = (pos.lon - self._datum_lon_deg) * self._m_per_deg_lon
        _y = (pos.lat - self._datum_lat_deg) * self._m_per_deg_lat
        _perm = "-" if self._turn_perm is None else ",".join(
            k for k in ("left", "straight", "right") if not bool(self._turn_perm.get(k, True))
        ) or "serbest"
        self.get_logger().info(
            f"KONUM x={_x:+.1f} y={_y:+.1f} yön={self._heading_deg:+.0f}° | "
            f"wp[{int(wp_state.wp_index)}] d={float(wp_state.distance_to_wp_m):.1f}m "
            f"b_err={float(wp_state.bearing_error_deg):+.0f}° | "
            f"steer {_src}={_steer_pre:+.2f} -> kısıt({_perm})={steer:+.2f} | "
            f"dönüş={'SAĞ' if self._turn_is_right else 'SOL'}:{self._turn_active} "
            f"rota_oturmadi={self._route_unstable()}",
            throttle_duration_sec=0.5,
        )
        base_speed *= float(turn_speed_mul)
        speed = self._apply_gps_timeout(base_speed)

        self._last_steering = steer
        self._last_speed = speed
        self._last_task = str(wp_state.current_task or "")

        self._publish(steer, speed, self._last_task, bool(wp_state.arrived),
                      mission_dec.park_mode, mission_dec.park_remaining_s)

    def _gps_age_s(self) -> float:
        if self._gps_stamp == 0.0:
            return float("inf")
        return time.monotonic() - self._gps_stamp

    def _apply_gps_timeout(self, base_speed: float) -> float:
        # LOKALİZASYON tazeliği = GPS veya final_odom'un TAZE olanı. Kontrol artık
        # odom'dan da sürülüyor; GPS bir an kesilse de odom canlıysa yavaşlatma.
        gps_age = self._gps_age_s()
        odom_age = (time.monotonic() - self._final_odom_stamp
                    if self._final_odom_stamp > 0.0 else float("inf"))
        age = min(gps_age, odom_age)
        if age < GPS_TIMEOUT_SLOW_S:
            return base_speed
        if age < GPS_TIMEOUT_STOP_S:
            return base_speed * 0.5
        excess = age - GPS_TIMEOUT_STOP_S
        return max(0.0, base_speed - excess * SPEED_DECAY_PER_SEC)

    def _tick_timeout(self) -> None:
        # GPS kesilirse son değerleri yayınlamayı sürdür (controller tarafında stabil kalır).
        # GPS canlıyken susmalı: yoksa buradaki arrived/park_mode değerleri _gps_cb'ninkiyle
        # 10 Hz'de çakışıp park modunu titretiyor ve ParkingLogic her sahte moda-girişte
        # resetlenip park ilerleyemiyor.
        if self._manager is None:
            return
        if self._gps_age_s() < 1.0:
            return
        self._publish(self._last_steering, self._last_speed, self._last_task,
                      self._last_arrived, self._last_park_mode, self._last_park_remaining)

    def _publish(self, steer: float, speed: float, task: str, arrived: bool, park_mode: bool, park_remaining_s: float) -> None:
        self._last_arrived = bool(arrived)
        self._last_park_mode = bool(park_mode)
        self._last_park_remaining = float(park_remaining_s)
        self._pub_steer.publish(Float32(data=float(steer)))
        self._pub_speed.publish(Float32(data=float(speed)))
        self._pub_task.publish(String(data=str(task)))
        self._pub_arrived.publish(Bool(data=bool(arrived)))
        self._pub_park_mode.publish(Bool(data=bool(park_mode)))
        self._pub_park_remaining.publish(Float32(data=float(park_remaining_s)))
        self._pub_turn_active.publish(Bool(data=bool(self._turn_active)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

