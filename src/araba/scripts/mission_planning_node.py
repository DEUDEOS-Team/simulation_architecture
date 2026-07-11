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
TURN_RULE_APPLY_DISTANCE_M = 8.0

# Kavşak dönüş önceliği (/planning/turn_active): rotanın sıradaki bacağı belirgin
# yön değişimi istiyorsa vehicle_controller şerit görünse bile PLAN referansını sürer.
TURN_MIN_ANGLE_DEG = 35.0        # bacaklar arası açı eşiği: üstü "dönüş noktası"
TURN_ENGAGE_DISTANCE_M = 10.0    # dönüş noktasına bu mesafede öncelik başlar
TURN_EXIT_ALIGN_DEG = 25.0       # waypoint geçilip yeni bacağa bu kadar hizalanınca biter

# Rotadan sapma bekçisi: aktif hedefe mesafe, o hedef için görülen minimumdan bu
# kadar artarsa dönüş kaçırılmıştır -> mevcut konumdan rota yeniden planlanır.
OFF_ROUTE_MARGIN_M = 10.0
OFF_ROUTE_REPLAN_COOLDOWN_S = 5.0


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

        # Routing: Mission noktalarını graph'a snap edip Dijkstra ile bir route waypoint listesi üret.
        self._blocked_edges: set[tuple[int, int]] = set()
        self._road_blocked: bool = False
        self._road_blocked_streak: int = 0  # debounce: tek karelik sahte road_blocked replan tetiklemesin
        self._entry_blocked: bool = False
        self._mission_idx: int = 0  # base_plan hedef indeksi (replan için)
        self._last_turn_replan_sig: str = ""
        self._last_turn_replan_t: float = 0.0
        # Kavşak dönüş durumu (vehicle_controller'a PLAN önceliği sinyali)
        self._turn_active: bool = False
        self._turn_wp_index: int | None = None
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

        self._last_steering = 0.0
        self._last_speed = 0.0
        self._last_task = ""
        self._turn_perm: dict | None = None

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
            self._turn_perm = json.loads(msg.data) if msg.data else None
        except Exception:
            self._turn_perm = None

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
        self._turn_wp_index = None
        self._min_dist_to_wp = float("inf")
        self._min_dist_wp_idx = -1

    def _is_junction(self, wp) -> bool:
        """
        Dönüş noktası gerçek bir KAVŞAK mı? Graph'ta düğümden 2+ çıkış varsa
        kavşaktır (düz gitme/dönme seçeneği var — ONNX'in düz şeridi TURN
        önceliğini gerektirir). Tek çıkışlı düğüm VİRAJDIR: yol zaten kıvrılır,
        şerit takibi virajı kendisi döner, TURN önceliği gereksiz ve zararlı
        (kullanıcı gözlemi 2026-07-11).
        """
        if self._route_graph is None:
            return True  # graph yoksa ayrım yapılamaz: eski (her dönüşte) davranış
        try:
            nid = int(nearest_node_id(self._route_graph, lat=float(wp.lat), lon=float(wp.lon)))
            return len(self._route_graph.adj.get(nid, [])) >= 2
        except Exception:
            return True

    def _update_turn_state(self, wp_state) -> None:
        """
        Kavşak dönüş tespiti. Giriş bacağı ROTA GEOMETRİSİNDEN alınır (önceki wp ->
        aktif wp) — araç->waypoint bearing'i DEĞİL: o, waypoint'e yaklaşırken küçük
        yanal sapmalarda bile şişip sahte dönüş tetikliyordu (canlı koşu 2026-07-11:
        wp[11]'de −49° sahte tetik, araç durak yapısına döndü ve kilitlendi). Çıkış
        bacağı aktif wp -> sonraki wp. Açı TURN_MIN_ANGLE_DEG'i aşıyorsa, waypoint
        yaklaşıldıysa VE nokta gerçek bir kavşaksa (_is_junction) turn_active=True.
        Waypoint geçilip (advance) araç yeni bacağa hizalanınca biter. Sinyali
        vehicle_controller PLAN önceliği olarak kullanır.
        """
        idx = int(wp_state.wp_index)

        if self._turn_active:
            if self._turn_wp_index is not None and idx != self._turn_wp_index:
                if abs(float(wp_state.bearing_error_deg)) <= TURN_EXIT_ALIGN_DEG:
                    self.get_logger().info(
                        f"DÖNÜŞ bitti: wp[{idx}] yeni bacağa hizalandı "
                        f"(b_err={wp_state.bearing_error_deg:+.0f}°)"
                    )
                    self._turn_active = False
                    self._turn_wp_index = None
            return

        cur = wp_state.current_wp
        nxt = wp_state.next_wp
        if cur is None or nxt is None:
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
            self.get_logger().info(
                f"DÖNÜŞ başlıyor: wp[{idx}] '{cur.name}' açı={turn_deg:+.0f}° "
                f"mesafe={wp_state.distance_to_wp_m:.1f}m"
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
            self._publish(0.0, 0.0, "no_mission", False)
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
                if new_plan.points:
                    self._manager = MissionManager(new_plan)
                    self._plan_points = list(new_plan.points)
                    self._road_blocked = False
                    self._entry_blocked = False
                    self._last_turn_replan_sig = sig
                    self._last_turn_replan_t = now_m
                    self._reset_route_tracking()
                    wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())
                    self.get_logger().warn(
                        f"REPLAN: ok -> new_route_waypoints={len(new_plan.points)} blocked_edges={len(self._blocked_edges)} turn_blocks={len(turn_blocks)}"
                    )
                else:
                    # No route under constraints: keep last good plan/manager, keep moving.
                    self.get_logger().error(
                        f"REPLAN: no_route (kept last plan) blocked_edges={len(self._blocked_edges)} turn_blocks={len(turn_blocks)}"
                    )
            except Exception as e:
                if str(e) != "replan debounce":
                    self.get_logger().error(f"REPLAN başarısız: {e}")

        # Rotadan sapma bekçisi (gerekirse konumdan replan) + kavşak dönüş tespiti
        _res = self._offroute_watchdog(pos, wp_state, bool(mission_dec.park_mode))
        if _res is not None:
            wp_state, mission_dec = _res
        self._update_turn_state(wp_state)

        steer = float(wp_state.steering_ref)
        base_speed = float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio)
        steer, turn_speed_mul = self._apply_turn_permissions(steer, float(wp_state.distance_to_wp_m))
        base_speed *= float(turn_speed_mul)
        speed = self._apply_gps_timeout(base_speed)

        self._last_steering = steer
        self._last_speed = speed
        self._last_task = str(wp_state.current_task or "")

        self._publish(steer, speed, self._last_task, bool(wp_state.arrived), mission_dec.park_mode, mission_dec.park_remaining_s)

    def _gps_age_s(self) -> float:
        if self._gps_stamp == 0.0:
            return float("inf")
        return time.monotonic() - self._gps_stamp

    def _apply_gps_timeout(self, base_speed: float) -> float:
        age = self._gps_age_s()
        if age < GPS_TIMEOUT_SLOW_S:
            return base_speed
        if age < GPS_TIMEOUT_STOP_S:
            return base_speed * 0.5
        excess = age - GPS_TIMEOUT_STOP_S
        return max(0.0, base_speed - excess * SPEED_DECAY_PER_SEC)

    def _tick_timeout(self) -> None:
        # GPS yoksa da son değerleri yayınlayalım (controller tarafında stabil kalır)
        if self._manager is None:
            return
        self._publish(self._last_steering, self._last_speed, self._last_task, False, False, 0.0)

    def _publish(self, steer: float, speed: float, task: str, arrived: bool, park_mode: bool, park_remaining_s: float) -> None:
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

