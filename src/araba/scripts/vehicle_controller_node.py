#!/usr/bin/env python3
"""
vehicle_controller_node — şerit takibi ile görev planlamayı ARBİTRE eden kontrol katmanı
(SİM UYARLAMASI, 5. adım).

Kaynak: DEOS_MimariDev/control/vehicle_controller/vehicle_controller_node.py.
Mimarideki öncelik mantığı korunur: taze şerit referansı varsa şerit sürer, yoksa
planning referansına düşülür; hız her zaman kısıtların min'idir; algı steer
override'ı (park manevrası) her ikisini de ezer. Sim için değişenler:

  1. Girdi biçimi: mimari lane_control'den Float32 oran referansları alır; simde
     autonomous_control_node hazır bir Twist yayınlar → /cmd_vel_lane olarak alınır.
     Şerit geçerliliği Twist tazeliğinden DEĞİL /perception/center_pts doluluğundan
     izlenir (autonomous_control şerit kaybında da sıfır Twist yayınlamayı sürdürür).
  2. Çıkış: /cmd_vel_raw — mevcut speed_controller_node güvenlik katmanı (ışık/
     levha/engel, acil fren) değişmeden bunun üzerinden /cmd_vel üretir. Mimarideki
     hardware_motion_enable / autonomy / failsafe kilitleri simde yok.
  3. İŞARET ÇEVİRİSİ: planning steering_ref ve fusion steer_override POZİTİF=SAĞA
     oranıdır [-1,1]; ROS Twist angular.z POZİTİF=SOLA. Oran, Ackermann ön teker
     açısına (steer_limit) çevrilir: angular.z = v·tan(-oran·steer_limit)/wheel_base
     (autonomous_control_node'daki eşlemeyle aynı, işaret ters).
  4. Fusion kısıtları ayrı topic'ler yerine /perception/decision_debug JSON'undan
     okunur (final.has_steer_override / steer_override / speed_cap).

Şerit kaybı senaryosu (bu node'un varlık sebebi): kavşakta lane ONNX center_pts'i
boş verince araç eskiden 1 s sonra donup kalıyordu; şimdi planning steering_ref
ile yavaş "kavşak sürünmesi" yapıp rotadaki dönüşü tamamlar, karşıda şerit
yakalanınca şerit takibine geri döner.

Kavşak dönüş önceliği (TURN modu): kavşakta düz giden şerit ONNX'te görünür
kaldığından LANE modu dönüşleri bastırıyordu. mission_planning, rotanın sıradaki
bacağı belirgin yön değişimi istiyorsa /planning/turn_active=True yayınlar; bu
sinyal tazeyken PLAN referansı şeridi ezer (OVERRIDE yine en üstte). Öncelik:
OVERRIDE > TURN > LANE > PLAN(şerit yok) > STOP.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


class VehicleControllerNode(Node):
    def __init__(self):
        super().__init__("vehicle_controller_node")

        # autonomous_control ile aynı Ackermann sabitleri (URDF)
        self.declare_parameter("wheel_base", 1.675)
        self.declare_parameter("steer_limit", 0.5236)      # rad, ±30°
        # Hız ölçekleri: plan speed_limit ORANDIR (0..1) → m/s'ye bu çarpanla döner
        self.declare_parameter("max_speed_mps", 2.5)       # şerit seyir hızıyla uyumlu (0.5·5.0 = 9 km/h)
        self.declare_parameter("fallback_speed_mps", 1.4)  # kavşak dönüşü/sürünme hızı (~5 km/h)
        # Tazelik pencereleri
        self.declare_parameter("lane_lost_patience_s", 0.6)  # son DOLU center_pts'ten itibaren
        self.declare_parameter("lane_cmd_timeout_s", 0.5)
        self.declare_parameter("plan_timeout_s", 2.0)
        self.declare_parameter("fusion_timeout_s", 0.5)
        self.declare_parameter("lane_cmd_topic", "/cmd_vel_lane")
        self.declare_parameter("cmd_out_topic", "/cmd_vel_raw")
        # Kavşak dönüş önceliği: mission_planning turn_active yayınlarken şerit
        # görünse bile PLAN referansı sürer (aksi hâlde LANE kavşaktan düz geçirir)
        self.declare_parameter("turn_active_topic", "/planning/turn_active")
        self.declare_parameter("turn_active_timeout_s", 1.0)

        self._wheel_base = float(self.get_parameter("wheel_base").value)
        self._steer_limit = float(self.get_parameter("steer_limit").value)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._fallback_speed = float(self.get_parameter("fallback_speed_mps").value)

        self._lane_twist = Twist()
        self._lane_cmd_stamp = 0.0
        self._lane_seen_stamp = 0.0          # son DOLU center_pts anı
        self._plan_steer = 0.0               # oran [-1,1], pozitif=SAĞA
        self._plan_speed = 0.0               # oran [0,1]
        self._plan_stamp = 0.0
        self._ovr_active = False
        self._ovr_steer = 0.0                # oran, pozitif=SAĞA
        self._ovr_speed_cap = 1.0
        self._fusion_stamp = 0.0
        self._turn_active = False
        self._turn_stamp = 0.0
        self._mode = ""                      # log için: LANE / PLAN / TURN / OVERRIDE / STOP

        self.create_subscription(
            Twist, str(self.get_parameter("lane_cmd_topic").value), self._lane_cmd_cb, 10)
        self.create_subscription(
            Float32MultiArray, "/perception/center_pts", self._center_pts_cb, 10)
        self.create_subscription(Float32, "/planning/steering_ref", self._plan_steer_cb, 10)
        self.create_subscription(Float32, "/planning/speed_limit", self._plan_speed_cb, 10)
        self.create_subscription(String, "/perception/decision_debug", self._decision_cb, 10)
        self.create_subscription(
            Bool, str(self.get_parameter("turn_active_topic").value), self._turn_active_cb, 10)

        self._pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter("cmd_out_topic").value), 10)

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            "vehicle_controller_node hazır — şerit varsa şerit, yoksa planning "
            f"steering_ref (fallback {self._fallback_speed} m/s) → "
            f"{str(self.get_parameter('cmd_out_topic').value)}")

    # ── callbacks ──
    def _lane_cmd_cb(self, msg: Twist) -> None:
        self._lane_twist = msg
        self._lane_cmd_stamp = time.monotonic()

    def _center_pts_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) > 0:
            self._lane_seen_stamp = time.monotonic()

    def _plan_steer_cb(self, msg: Float32) -> None:
        self._plan_steer = max(-1.0, min(1.0, float(msg.data)))
        self._plan_stamp = time.monotonic()

    def _plan_speed_cb(self, msg: Float32) -> None:
        self._plan_speed = max(0.0, min(1.0, float(msg.data)))
        self._plan_stamp = time.monotonic()

    def _turn_active_cb(self, msg: Bool) -> None:
        self._turn_active = bool(msg.data)
        self._turn_stamp = time.monotonic()

    def _decision_cb(self, msg: String) -> None:
        try:
            final = (json.loads(msg.data) if msg.data else {}).get("final", {})
            self._ovr_active = bool(final.get("has_steer_override", False))
            self._ovr_steer = max(-1.0, min(1.0, float(final.get("steer_override", 0.0))))
            self._ovr_speed_cap = max(0.0, min(1.0, float(final.get("speed_cap", 1.0))))
            self._fusion_stamp = time.monotonic()
        except Exception:
            self._ovr_active = False

    # ── yardımcılar ──
    def _ratio_to_twist(self, steer_ratio: float, v: float) -> Twist:
        """Pozitif=SAĞA direksiyon oranı + hız → ROS Twist (angular.z pozitif=SOLA)."""
        t = Twist()
        t.linear.x = float(v)
        steer_rad = -steer_ratio * self._steer_limit  # işaret çevirisi burada
        if abs(v) > 1e-3:
            t.angular.z = float(v * math.tan(steer_rad) / self._wheel_base)
        return t

    def _tick(self) -> None:
        now = time.monotonic()
        lane_ok = (
            (now - self._lane_seen_stamp) < float(self.get_parameter("lane_lost_patience_s").value)
            and (now - self._lane_cmd_stamp) < float(self.get_parameter("lane_cmd_timeout_s").value)
        )
        plan_ok = (now - self._plan_stamp) < float(self.get_parameter("plan_timeout_s").value)
        ovr_ok = (
            self._ovr_active
            and (now - self._fusion_stamp) < float(self.get_parameter("fusion_timeout_s").value)
        )
        turn_ok = (
            self._turn_active
            and (now - self._turn_stamp) < float(self.get_parameter("turn_active_timeout_s").value)
        )
        plan_speed_mps = self._plan_speed * self._max_speed if plan_ok else self._max_speed

        if ovr_ok:
            # Park/kaçınma manevrası: fusion direksiyonu ezer (mimari öncelik sırası)
            v = min(self._ovr_speed_cap * self._max_speed, plan_speed_mps)
            cmd = self._ratio_to_twist(self._ovr_steer, v)
            mode = "OVERRIDE"
        elif turn_ok and plan_ok and plan_speed_mps > 1e-3:
            # Kavşak dönüşü: rota belirgin yön değişimi istiyor — şerit görünse
            # bile planning steering_ref sürer, dönüş bitince LANE'e dönülür
            v = min(self._fallback_speed, plan_speed_mps)
            cmd = self._ratio_to_twist(self._plan_steer, v)
            mode = "TURN"
        elif lane_ok:
            cmd = Twist()
            cmd.linear.x = self._lane_twist.linear.x
            cmd.angular.z = self._lane_twist.angular.z
            if cmd.linear.x > plan_speed_mps:
                # Plan hız sınırı: yaw-rate'i de oranla ki ön teker açısı değişmesin
                scale = plan_speed_mps / cmd.linear.x if cmd.linear.x > 1e-3 else 0.0
                cmd.linear.x = plan_speed_mps
                cmd.angular.z *= scale
            mode = "LANE"
        elif plan_ok and plan_speed_mps > 1e-3:
            # Şerit yok → rota referansıyla yavaş sürünme (kavşak dönüşü)
            v = min(self._fallback_speed, plan_speed_mps)
            cmd = self._ratio_to_twist(self._plan_steer, v)
            mode = "PLAN"
        else:
            cmd = Twist()
            mode = "STOP"

        if mode != self._mode:
            self.get_logger().info(
                f"mod: {self._mode or '-'} -> {mode} "
                f"(lane_ok={lane_ok} plan_ok={plan_ok} turn_ok={turn_ok} "
                f"steer_ref={self._plan_steer:+.2f})")
            self._mode = mode

        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
