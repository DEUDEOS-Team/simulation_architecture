import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String, Int32

from deos_algorithms.obstacle_logic import ObstacleLogic
from deos_algorithms.parking_logic import ParkingLogic
from deos_algorithms.perception_fusion import fuse, parking_detections_from_signs
from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode, lane_contains_lateral
from deos_algorithms.lane_violation import LaneViolationTracker, wheels_outside_lane
from deos_algorithms.ros_topic_layout import build_deos_topics
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.slalom_logic import SlalomLogic
from deos_algorithms.traffic_light_logic import LightColor, TrafficLightLogic
from deos_algorithms.traffic_sign_logic import TrafficSignLogic

from deos_logging.logger import DeosLogger


class PerceptionFusionNode(Node):
    STEREO_TIMEOUT_S = 0.5
    LIDAR_TIMEOUT_S = 0.5
    LANE_WALLS_TIMEOUT_S = 0.5

    def __init__(self):
        super().__init__("perception_fusion_node")
        self.logger = DeosLogger(self.get_logger(), "perception_fusion_node")

        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))

        self.declare_parameter("hardware_motion_enable_topic", _T["hardware_motion_enable"])
        self.declare_parameter("hardware_motion_enable_timeout_s", 0.5)
        self.declare_parameter("hardware_motion_enable_fail_safe_stop", True)
        # Manuel/otonom ayrımı: otonom kapalıysa algı kararları yayınlamayı durdur (nötr publish)
        self.declare_parameter("autonomy_enable_topic", _T["hardware_autonomy_enable"])
        self.declare_parameter("require_autonomy_enable", True)
        # Sensör fail-safe (lokal planlama): veri yoksa hız düşür / dur
        self.declare_parameter("fail_safe_stop_on_all_sensors_lost", True)
        self.declare_parameter("lidar_missing_speed_cap", 0.20)   # engel için kritik
        self.declare_parameter("stereo_missing_speed_cap", 0.30)  # ışık/tabela için kritik
        # Şerit dışına çıkmamak: engel kaçınma override'ı sadece şerit algısı tazeyken aktif olsun
        self.declare_parameter("lane_walls_topic", _T["perception_lane_walls"])
        self.declare_parameter("stereo_detections_topic", _T["perception_stereo_detections"])
        self.declare_parameter("lidar_obstacles_topic", _T["perception_lidar_obstacles"])
        self.declare_parameter("imu_topic", _T["sensors_imu"])
        self.declare_parameter("planning_park_mode_topic", _T["planning_park_mode"])
        self.declare_parameter("perception_fusion_emergency_stop_topic", _T["perception_fusion_emergency_stop"])
        self.declare_parameter("perception_fusion_speed_cap_topic", _T["perception_fusion_speed_cap"])
        self.declare_parameter("perception_fusion_steering_override_topic", _T["perception_fusion_steering_override"])
        self.declare_parameter(
            "perception_fusion_has_steering_override_topic", _T["perception_fusion_has_steering_override"]
        )
        self.declare_parameter("perception_fusion_park_complete_topic", _T["perception_fusion_park_complete"])
        self.declare_parameter("perception_fusion_turn_permissions_topic", _T["perception_fusion_turn_permissions"])
        self.declare_parameter("perception_fusion_decision_debug_topic", _T["perception_fusion_decision_debug"])
        self.declare_parameter("safety_lane_violation_topic", _T["safety_lane_violation"])
        self.declare_parameter("safety_lane_violation_count_topic", _T["safety_lane_violation_count"])
        self.declare_parameter("safety_lane_violation_seconds_topic", _T["safety_lane_violation_seconds"])
        self.declare_parameter("require_lane_walls_for_avoidance", True)
        # Lane bounds çıkarımı (şerit içinde kalma kısıtı için)
        self.declare_parameter("lane_bounds_min_points", 20)
        self.declare_parameter("lane_bounds_x_min_m", 1.0)
        self.declare_parameter("lane_bounds_x_max_m", 4.0)
        self.declare_parameter("lane_bounds_z_max_m", 0.3)
        self.declare_parameter("lane_bounds_margin_m", 0.25)
        self.declare_parameter("publish_decision_debug", True)
        self.declare_parameter("perception_fusion_green_elapsed_s_topic", _T["perception_fusion_green_elapsed_s"])
        # Şerit ihlali metriği (şartname): 2 tekerlek dışarı + 10s bucket sayacı
        self.declare_parameter("publish_lane_violation", True)
        self.declare_parameter("vehicle_half_width_m", 0.75)  # yaklaşık: araç genişliği/2
        self.declare_parameter("lane_violation_bucket_s", 10.0)
        # Kırmızı ışık mesafe bantları (safety_logic / şartname mesafe tablosu ile aynı varsayılanlar)
        self.declare_parameter("traffic_light_red_emergency_m", 3.0)
        self.declare_parameter("traffic_light_red_hard_m", 8.0)
        self.declare_parameter("traffic_light_red_soft_m", 15.0)
        self.declare_parameter("traffic_light_red_hard_speed_ratio", 0.5)
        self.declare_parameter("traffic_light_red_soft_speed_ratio", 0.8)
        self.declare_parameter("traffic_light_red_unknown_must_stop", True)

        self._sign = TrafficSignLogic()
        self._light = TrafficLightLogic(
            red_emergency_m=float(self.get_parameter("traffic_light_red_emergency_m").value),
            red_hard_m=float(self.get_parameter("traffic_light_red_hard_m").value),
            red_soft_m=float(self.get_parameter("traffic_light_red_soft_m").value),
            red_hard_speed_ratio=float(self.get_parameter("traffic_light_red_hard_speed_ratio").value),
            red_soft_speed_ratio=float(self.get_parameter("traffic_light_red_soft_speed_ratio").value),
            red_unknown_must_stop=bool(self.get_parameter("traffic_light_red_unknown_must_stop").value),
        )
        self._obstacle = ObstacleLogic()
        self._slalom = SlalomLogic()
        self._parking = ParkingLogic()
        self._arbiter = DecisionArbiter()
        self._lane_violation = LaneViolationTracker()

        self._stereo: list[StereoBbox] = []
        self._lidar: list[LidarObstacle] = []
        self._imu: ImuSample | None = None
        self._stereo_stamp: float = 0.0
        self._lidar_stamp: float = 0.0
        self._lane_walls_stamp: float = 0.0
        self._lane_bounds: LaneBounds | None = None

        # STM32 -> Pi: tek topic, olay bazlı (sadece değişimde publish edilir)
        # std_msgs/Bool: false = DUR (algoritmalar durur), true = DEVAM
        # STM32 ilk "DEVAM" mesajını gönderene kadar güvenli tarafta kal
        self._motion_enable: bool = False
        self._motion_enable_stamp: float = 0.0
        self._prev_motion_enable: bool = False
        self._reset_requested: bool = False
        self._autonomy_enable: bool = True
        self._autonomy_stamp: float = 0.0

        # Planning -> Perception: park arama/manevra modu
        self._park_mode: bool = False

        self.create_subscription(String, str(self.get_parameter("stereo_detections_topic").value), self._stereo_cb, 10)
        self.create_subscription(String, str(self.get_parameter("lidar_obstacles_topic").value), self._lidar_cb, 10)
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self._imu_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("planning_park_mode_topic").value), self._park_mode_cb, 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lane_walls_topic").value),
            self._lane_walls_cb,
            1,
        )

        motion_topic = str(self.get_parameter("hardware_motion_enable_topic").value)
        self.create_subscription(Bool, motion_topic, self._motion_enable_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("autonomy_enable_topic").value), self._autonomy_cb, 10)

        self._pub_estop = self.create_publisher(
            Bool, str(self.get_parameter("perception_fusion_emergency_stop_topic").value), 10
        )
        self._pub_speed = self.create_publisher(
            Float32, str(self.get_parameter("perception_fusion_speed_cap_topic").value), 10
        )
        self._pub_steer = self.create_publisher(
            Float32, str(self.get_parameter("perception_fusion_steering_override_topic").value), 10
        )
        self._pub_has_steer = self.create_publisher(
            Bool, str(self.get_parameter("perception_fusion_has_steering_override_topic").value), 10
        )
        self._pub_park_complete = self.create_publisher(
            Bool, str(self.get_parameter("perception_fusion_park_complete_topic").value), 10
        )
        self._pub_turn_permissions = self.create_publisher(
            String, str(self.get_parameter("perception_fusion_turn_permissions_topic").value), 10
        )
        self._pub_decision_debug = self.create_publisher(
            String, str(self.get_parameter("perception_fusion_decision_debug_topic").value), 10
        )
        self._pub_lane_violation = self.create_publisher(Bool, str(self.get_parameter("safety_lane_violation_topic").value), 10)
        self._pub_lane_violation_count = self.create_publisher(
            Int32, str(self.get_parameter("safety_lane_violation_count_topic").value), 10
        )
        self._pub_lane_violation_seconds = self.create_publisher(
            Float32, str(self.get_parameter("safety_lane_violation_seconds_topic").value), 10
        )
        self._pub_green_elapsed = self.create_publisher(
            Float32, str(self.get_parameter("perception_fusion_green_elapsed_s_topic").value), 10
        )

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.logger.info(
            "perception_fusion_node ready — "
            f"STM32 motion topic={motion_topic} (Bool: false=STOP algorithms, true=RUN)"
        )

    def _motion_enable_cb(self, msg: Bool) -> None:
        new_val = bool(msg.data)
        # Rising edge: manual -> autonomous start. Reset internal memories.
        if new_val and not self._motion_enable:
            self._reset_requested = True
        self._motion_enable = new_val
        self._motion_enable_stamp = time.monotonic()

    def _autonomy_cb(self, msg: Bool) -> None:
        self._autonomy_enable = bool(msg.data)
        self._autonomy_stamp = time.monotonic()

    def _park_mode_cb(self, msg: Bool) -> None:
        self._park_mode = bool(msg.data)

    def _stereo_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._stereo = [
                StereoBbox(
                    class_name=d["class_name"],
                    confidence=float(d["confidence"]),
                    bbox_px=tuple(float(v) for v in d["bbox_px"]),
                    distance_m=d.get("distance_m"),
                    lateral_m=d.get("lateral_m"),
                )
                for d in raw
            ]
            self._stereo_stamp = time.monotonic()
        except Exception as e:
            self.logger.error(f"stereo parse: {e}")

    def _lidar_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._lidar = [
                LidarObstacle(
                    kind=d["kind"],
                    confidence=float(d["confidence"]),
                    distance_m=float(d["distance_m"]),
                    lateral_m=float(d["lateral_m"]),
                    bbox_px=tuple(d["bbox_px"]) if d.get("bbox_px") else None,
                )
                for d in raw
            ]
            self._lidar_stamp = time.monotonic()
        except Exception as e:
            self.logger.error(f"lidar parse: {e}")

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        heading_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        yaw_rate_dps = math.degrees(msg.angular_velocity.z)
        self._imu = ImuSample(heading_deg=heading_deg, yaw_rate_dps=yaw_rate_dps)

    def _lane_walls_cb(self, _msg: PointCloud2) -> None:
        # Şerit algısı tazeliği için stamp tutuyoruz + basit sol/sağ sınır çıkarımı.
        self._lane_walls_stamp = time.monotonic()
        self._try_extract_lane_bounds(_msg)

    def _try_extract_lane_bounds(self, msg: PointCloud2) -> None:
        """
        /lane_walls pointcloud'undan basit sol/sağ sınır çıkarımı.
        Varsayım: pointcloud base_link frame'de, x ileri, y sol.
        """
        try:
            import struct

            x_min = float(self.get_parameter("lane_bounds_x_min_m").value)
            x_max = float(self.get_parameter("lane_bounds_x_max_m").value)
            z_max = float(self.get_parameter("lane_bounds_z_max_m").value)
            min_pts = int(self.get_parameter("lane_bounds_min_points").value)
            margin = float(self.get_parameter("lane_bounds_margin_m").value)

            if msg.point_step < 12:
                return

            ys: list[float] = []
            for i in range(0, len(msg.data), msg.point_step):
                x, y, z = struct.unpack_from("fff", msg.data, i)
                if x < x_min or x > x_max:
                    continue
                if z < -0.1 or z > z_max:
                    continue
                ys.append(float(y))

            if len(ys) < min_pts:
                return

            lb = LaneBounds(left_y_m=float(max(ys)), right_y_m=float(min(ys)), margin_m=float(margin))
            if lb.is_valid:
                self._lane_bounds = lb
        except Exception:
            return

    def _tick(self) -> None:
        now = time.monotonic()
        timeout_s = float(self.get_parameter("hardware_motion_enable_timeout_s").value)
        fail_safe = bool(self.get_parameter("hardware_motion_enable_fail_safe_stop").value)

        motion_ok = (now - self._motion_enable_stamp) <= timeout_s
        if not motion_ok and fail_safe:
            # STM32 komut akışı kesildi: güvenli tarafta kal (algoritmaları durdur)
            self._pub_estop.publish(Bool(data=True))
            self._pub_speed.publish(Float32(data=0.0))
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_has_steer.publish(Bool(data=False))
            return

        if bool(self.get_parameter("require_autonomy_enable").value) and not bool(self._autonomy_enable):
            # Manual mode: publish neutral constraints (do not command stop/steer).
            self._pub_estop.publish(Bool(data=False))
            self._pub_speed.publish(Float32(data=1.0))
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_has_steer.publish(Bool(data=False))
            return

        if not self._motion_enable:
            # STM32: DUR — algoritma/model çalıştırma, güvenli kısıtları yayınla
            self._pub_estop.publish(Bool(data=True))
            self._pub_speed.publish(Float32(data=0.0))
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_has_steer.publish(Bool(data=False))
            return

        if self._reset_requested:
            # Clear logic memories on autonomous start.
            try:
                self._sign.reset()
            except Exception:
                pass
            try:
                self._light.reset()
            except Exception:
                pass
            try:
                self._slalom.reset()
            except Exception:
                pass
            # Obstacle/Park logic have internal trackers; re-instantiate to fully reset.
            self._obstacle = ObstacleLogic()
            self._parking = ParkingLogic()
            self._reset_requested = False

        stereo_fresh = (now - self._stereo_stamp) < self.STEREO_TIMEOUT_S
        lidar_fresh = (now - self._lidar_stamp) < self.LIDAR_TIMEOUT_S
        lane_fresh = (now - self._lane_walls_stamp) < self.LANE_WALLS_TIMEOUT_S
        stereo = self._stereo if stereo_fresh else []
        lidar = self._lidar if lidar_fresh else []

        frame = fuse(stereo=stereo, lidar=lidar, imu=self._imu)

        sign_state = self._sign.update(frame.sign_dets)
        light_state = self._light.update(frame.light_dets)
        # Sensör önceliği:
        # - Engeller: LiDAR öncelikli; stereo sadece "pedestrian" ile destek (sınıflandırma).
        # - Slalom: LiDAR statik engeller (koni/bariyer/unknown) — geometri-tabanlı sınıflandırma.
        lidar_obs = frame.lidar_obstacle_dets
        stereo_obs = frame.stereo_obstacle_dets
        lane = self._lane_bounds if lane_fresh else None

        # Same-lane verification (statik engel): Şerit sınırı tazeyse, şerit dışındaki statik engeller
        # kaçınma / road_blocked tetiklemesin.
        filtered_lidar_obs = list(lidar_obs)
        if lane is not None and lane.is_valid:
            filtered_lidar_obs = []
            for d in lidar_obs:
                if d.kind in {"barrier", "cone", "vehicle", "unknown"}:
                    if lane_contains_lateral(lane, float(d.lateral_m)):
                        filtered_lidar_obs.append(d)
                else:
                    filtered_lidar_obs.append(d)

        obs_input = list(filtered_lidar_obs) + [d for d in stereo_obs if d.kind == "pedestrian"]
        slalom_input = [d for d in filtered_lidar_obs if d.kind in {"cone", "barrier", "unknown"}]

        obs_state = self._obstacle.update(obs_input)
        slalom_state = self._slalom.update(slalom_input)
        park_dets = parking_detections_from_signs(frame.sign_dets)
        park_state = self._parking.update(park_dets)
        self._pub_turn_permissions.publish(
            String(
                data=json.dumps(
                    {
                        "left": bool(sign_state.turn_permissions.left),
                        "straight": bool(sign_state.turn_permissions.straight),
                        "right": bool(sign_state.turn_permissions.right),
                        "forced_direction": sign_state.turn_permissions.forced_direction,
                    },
                    ensure_ascii=False,
                )
            )
        )

        # --- Decision Arbiter: adayları topla ve tek karar üret ---
        candidates: list[Candidate] = []

        # Işık / levha (kural)
        if light_state.must_stop:
            candidates.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
        elif float(light_state.speed_cap_ratio) < 1.0:
            lr = (
                ReasonCode.LIGHT_RED_SLOW
                if light_state.active_color == LightColor.RED
                else ReasonCode.LIGHT_YELLOW_SLOW
            )
            candidates.append(
                Candidate(name="light", emergency_stop=False, speed_cap=float(light_state.speed_cap_ratio), reasons=[lr])
            )

        if sign_state.must_stop_soon:
            candidates.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.SIGN_MUST_STOP]))
        elif float(sign_state.speed_cap_ratio) < 1.0:
            candidates.append(Candidate(name="sign", emergency_stop=False, speed_cap=float(sign_state.speed_cap_ratio), reasons=[ReasonCode.SIGN_SPEED_CAP]))

        # Engel
        if obs_state.emergency_stop:
            candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
        if obs_state.road_blocked:
            candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ROAD_BLOCKED]))
        if float(obs_state.speed_cap_ratio) < 1.0:
            reasons = [ReasonCode.STATIC_AVOID] if obs_state.suggest_lane_change else []
            candidates.append(Candidate(name="obstacle", emergency_stop=False, speed_cap=float(obs_state.speed_cap_ratio), reasons=reasons))

        # Park
        if self._park_mode and not park_state.complete:
            reasons = [ReasonCode.PARK_MODE]
            if bool(park_state.no_eligible_spot):
                reasons.append(ReasonCode.PARK_NO_ELIGIBLE)
            candidates.append(
                Candidate(
                    name="park",
                    emergency_stop=False,
                    speed_cap=float(park_state.speed_ratio),
                    steer_override=float(park_state.steering),
                    reasons=reasons,
                )
            )

        # Slalom (düşük öncelik)
        if slalom_state.aktif:
            candidates.append(
                Candidate(
                    name="slalom",
                    emergency_stop=False,
                    speed_cap=1.0,
                    steer_override=float(slalom_state.steering),
                    reasons=[ReasonCode.SLALOM],
                )
            )

        # Statik kaçınma override (lane ile clamp/disable)
        # steer_bias ±0.6: komşu şeride geçiş için yeterli steer gücü
        # lane clamp hedef konumu şerit/yol sınırlarına göre kırpar
        if obs_state.suggest_lane_change and not obs_state.road_blocked:
            steer_bias = -0.6 if obs_state.avoidance_direction == "left" else 0.6
            candidates.append(
                Candidate(
                    name="static_avoid",
                    emergency_stop=False,
                    speed_cap=0.35,
                    steer_override=float(steer_bias),
                    reasons=[ReasonCode.STATIC_AVOID],
                )
            )

        # Dinamik kaçınma (2 aşama): durduktan sonra lidar ile ters taraftan çok yavaş geç
        # Kural: kaçınma yönü (steer override) sadece lane tazeyken aktif olsun; lane yoksa sadece dur/bekle.
        if (
            (lane is not None and lane.is_valid)
            and bool(obs_state.dynamic_avoid_active)
            and (obs_state.dynamic_avoidance_direction in {"left", "right"})
        ):
            steer_bias = -0.25 if obs_state.dynamic_avoidance_direction == "left" else 0.25
            candidates.append(
                Candidate(
                    name="dynamic_avoid",
                    emergency_stop=False,
                    speed_cap=0.2,
                    steer_override=float(steer_bias),
                    reasons=[ReasonCode.DYNAMIC_AVOID],
                )
            )

        # Sensör fail-safe
        if not stereo_fresh and not lidar_fresh and bool(self.get_parameter("fail_safe_stop_on_all_sensors_lost").value):
            candidates.append(Candidate(name="failsafe", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ALL_SENSORS_LOST]))
        else:
            if not lidar_fresh:
                candidates.append(Candidate(name="failsafe", emergency_stop=False, speed_cap=float(self.get_parameter("lidar_missing_speed_cap").value)))
            if not stereo_fresh:
                candidates.append(Candidate(name="failsafe", emergency_stop=False, speed_cap=float(self.get_parameter("stereo_missing_speed_cap").value)))

        lane = lane
        decision = self._arbiter.arbitrate(
            candidates=candidates,
            lane=lane,
            lane_required_for_avoidance=bool(self.get_parameter("require_lane_walls_for_avoidance").value),
        )

        # --- Şerit ihlali metriği ---
        if bool(self.get_parameter("publish_lane_violation").value):
            half_w = float(self.get_parameter("vehicle_half_width_m").value)
            bucket_s = float(self.get_parameter("lane_violation_bucket_s").value)
            self._lane_violation.bucket_s = max(1.0, bucket_s)
            outside = False
            if lane_fresh and lane is not None and lane.is_valid:
                outside = wheels_outside_lane(lane=lane, half_width_m=half_w)
            else:
                # Lane yoksa ihlal ölçemiyoruz; sayaç birikmesin
                outside = False
            self._lane_violation.update(now_s=now, outside=bool(outside), speed_mps=None)
            self._pub_lane_violation.publish(Bool(data=bool(outside)))
            self._pub_lane_violation_count.publish(Int32(data=int(self._lane_violation.violation_count)))
            self._pub_lane_violation_seconds.publish(Float32(data=float(self._lane_violation.outside_accum_s)))

        # Yeşil ışık geçen süre (şartname tepki süresi takibi): -1.0 = yeşil yok
        green_elapsed = float(light_state.green_elapsed_s) if light_state.green_elapsed_s is not None else -1.0
        self._pub_green_elapsed.publish(Float32(data=green_elapsed))

        if bool(self.get_parameter("publish_decision_debug").value):
            try:
                self._pub_decision_debug.publish(
                    String(
                        data=json.dumps(
                            {
                                "ts_monotonic": now,
                                "fresh": {
                                    "stereo": stereo_fresh,
                                    "lidar": lidar_fresh,
                                    "lane_walls": lane_fresh,
                                },
                                "lane_bounds": (
                                    None
                                    if lane is None
                                    else {
                                        "left_y_m": lane.left_y_m,
                                        "right_y_m": lane.right_y_m,
                                        "margin_m": lane.margin_m,
                                    }
                                ),
                                "entry_blocked": bool(sign_state.entry_blocked),
                                "candidates": [
                                    {
                                        "name": c.name,
                                        "emergency_stop": c.emergency_stop,
                                        "speed_cap": c.speed_cap,
                                        "steer_override": c.steer_override,
                                        "reasons": [r.value for r in c.reasons],
                                    }
                                    for c in candidates
                                ],
                                "final": {
                                    "emergency_stop": decision.emergency_stop,
                                    "speed_cap": decision.speed_cap,
                                    "has_steer_override": decision.has_steer_override,
                                    "steer_override": decision.steer_override,
                                    "reasons": [r.value for r in decision.reasons],
                                },
                            },
                            ensure_ascii=False,
                        )
                    )
                )
            except Exception:
                # Debug yayınları kritik değil; ana akışı bozmasın.
                pass

        self._pub_park_complete.publish(Bool(data=bool(park_state.complete)))

        self._pub_estop.publish(Bool(data=bool(decision.emergency_stop)))
        self._pub_speed.publish(Float32(data=float(decision.speed_cap)))
        self._pub_steer.publish(Float32(data=float(decision.steer_override)))
        self._pub_has_steer.publish(Bool(data=bool(decision.has_steer_override)))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

