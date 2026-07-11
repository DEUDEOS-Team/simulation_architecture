from __future__ import annotations

"""
lane_control_node — şerit bazlı direksiyon/hız referansı (deos mimarisi).

GÜNCELLEME: Kontrol matematiği, şu an kullandığımız kanıtlanmış sürümle
(autonomous_control_node) birebir eşitlendi:
  • kp kazancı (varsayılan 0.8) — eski sürümde kazanç yoktu (örtük 1.0)
  • Direksiyon EMA yumuşatma (steer_alpha=0.3) — eski sürümde YOKTU (ham steer titrer)
  • Gaz EMA yumuşatma (throttle_alpha=0.2) — eski sürümde YOKTU
  • Şerit-kayıp toleransı: 1.0 s boyunca son geçerli steer + %80 gaz, sonra dur
  • Hedef nokta: pts[len//2 + 2] (orta-ileri) — intent varsa yakın alan (dönüş hazırlığı)

Mimari uyumu DEĞİŞMEDİ:
  - Bu node DOĞRUDAN /cmd_vel üretmez.
  - /lane/steering_ref (-1..1) ve /lane/speed_limit (0..1 oran) yayınlar.
  - vehicle_controller_node parametre ile bu lane referanslarını tercih edebilir
    (tazelik kontrolüyle).

NOT: steer_cmd_mul varsayılanı 0.65 -> 1.0 yapıldı; kazanç artık kp üzerinden
(0.8) uygulanıyor. İkisi birden uygulanırsa direksiyon aşırı sönük kalır.
Araç sahasında ayar gerekirse kp ile oynayın.
"""

import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8

from deos_algorithms.ros_topic_layout import build_deos_topics


class LaneControlNode(Node):
    """
    /perception/center_pts -> şerit bazlı steer/speed referansı üretir.
    """

    def __init__(self):
        super().__init__("lane_control_node")

        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))
        self.declare_parameter("center_pts_topic", _T["perception_lane_center_pts"])
        self.declare_parameter("lane_steer_topic", _T["lane_steering_ref"])
        self.declare_parameter("lane_speed_topic", _T["lane_speed_limit"])
        self.declare_parameter("camera_w", 640)   # D415 node 640x480 yayınlar
        self.declare_parameter("camera_h", 480)

        # ── Kontrol parametreleri (autonomous_control ile AYNI) ──
        # Kontrolcü kazancı (büyük = aynı hata için daha sert direksiyon)
        self.declare_parameter("kp", 0.8)
        # steer_alpha: büyük = daha çevik/hızlı direksiyon tepkisi (az gecikme)
        self.declare_parameter("steer_alpha", 0.3)
        self.declare_parameter("throttle_alpha", 0.2)
        # Son çarpan (eski steer_cmd_mul) — kazanç artık kp'de; 1.0 bırakın
        self.declare_parameter("steer_cmd_mul", 1.0)
        self.declare_parameter("lane_lost_patience_s", 1.0)

        # intent (opsiyonel): 1=sol, 2=sağ, 0=reset
        self.declare_parameter("intent_topic", _T["control_intent"])
        self.declare_parameter("use_intent", False)

        self._cam_w = float(self.get_parameter("camera_w").value)
        self._cam_h = float(self.get_parameter("camera_h").value)
        self._kp = float(self.get_parameter("kp").value)
        self._steer_alpha = float(self.get_parameter("steer_alpha").value)
        self._throttle_alpha = float(self.get_parameter("throttle_alpha").value)

        self._pub_steer = self.create_publisher(Float32, str(self.get_parameter("lane_steer_topic").value), 10)
        self._pub_speed = self.create_publisher(Float32, str(self.get_parameter("lane_speed_topic").value), 10)

        self._center_pts_list: list[list[tuple[float, float]]] = []
        self._last_valid_steer = 0.0
        self._last_valid_throttle = 0.0
        self._last_lane_t = time.perf_counter()

        self._intent_queue = deque()
        if bool(self.get_parameter("use_intent").value):
            self.create_subscription(Int8, str(self.get_parameter("intent_topic").value), self._intent_cb, 10)

        self.create_subscription(Float32MultiArray, str(self.get_parameter("center_pts_topic").value), self._pts_cb, 10)

    def _intent_cb(self, msg: Int8) -> None:
        v = int(msg.data)
        if v in (1, 2):
            self._intent_queue.append(v)
        elif v == 0:
            self._intent_queue.clear()

    def _calculate_commands(self) -> tuple[float, float]:
        now = time.perf_counter()

        if self._center_pts_list and self._center_pts_list[0]:
            pts = self._center_pts_list[0]
            # Hedef nokta: orta-ileri; intent varsa yakın alan (dönüş hazırlığı)
            if self._intent_queue:
                target_idx = min(2, len(pts) - 1)
            else:
                target_idx = min(len(pts) // 2 + 2, len(pts) - 1)

            target_x, _ = pts[target_idx]
            center_camera_x = self._cam_w / 2.0
            error = (float(target_x) - center_camera_x) / center_camera_x

            # P kontrolcü + hız politikası (autonomous_control ile AYNI)
            raw_steer = float(np.clip(self._kp * error, -1.0, 1.0))
            raw_throttle = 0.5 if abs(raw_steer) < 0.2 else 0.35

            # EMA yumuşatma — titremeyi keser, viraj geçişini akıcı yapar
            steer = (self._steer_alpha * raw_steer) + ((1.0 - self._steer_alpha) * self._last_valid_steer)
            throttle = (self._throttle_alpha * raw_throttle) + ((1.0 - self._throttle_alpha) * self._last_valid_throttle)

            steer = float(np.clip(steer * float(self.get_parameter("steer_cmd_mul").value), -1.0, 1.0))

            self._last_valid_steer = steer
            self._last_valid_throttle = throttle
            self._last_lane_t = now
            return steer, throttle

        # Şerit-kayıp toleransı: kısa süre son geçerli komutla devam, sonra dur
        patience = float(self.get_parameter("lane_lost_patience_s").value)
        if (now - self._last_lane_t) < patience:
            return self._last_valid_steer, self._last_valid_throttle * 0.8
        return 0.0, 0.0

    def _pts_cb(self, msg: Float32MultiArray) -> None:
        pts_data = list(msg.data)
        if pts_data:
            self._center_pts_list = [[(pts_data[i], pts_data[i + 1]) for i in range(0, len(pts_data), 2)]]
        else:
            self._center_pts_list = []

        steer, throttle = self._calculate_commands()

        self._pub_steer.publish(Float32(data=float(steer)))       # -1..1
        self._pub_speed.publish(Float32(data=float(throttle)))    # 0..1 (ratio)


def main(args=None):
    rclpy.init(args=args)
    node = LaneControlNode()
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
