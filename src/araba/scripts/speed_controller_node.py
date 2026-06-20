#!/usr/bin/env python3
"""
speed_controller_node.py
-------------------------
Algılama pipeline'ından gelen trafik ışığı, tabela ve engel durumlarını
okuyup DecisionArbiter ile birleştirerek aracın HIZINI kontrol eder.

Kullanıcı direksiyonu (angular.z) kontrol eder — bu node SADECE hıza karışır.
Acil durumda hem hız hem direksiyon sıfırlanır.

Mimari:
  keyboard_teleop → /cmd_vel ──┐
  perception_pipeline → /perception/* ──┤→ SpeedController → /cmd_vel_filtered → Gazebo
                                        │
  (steering korunur, hız sınırlanır)    │

Bu dosya SADECE altyapıdır — ROS iletişimi ve karar birleştirme.
Algoritma mantığı logic modüllerinde ve DecisionArbiter'dadır.
"""

import os
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import math

from decision_arbiter import DecisionArbiter, Candidate, LaneBounds, ReasonCode

# ─── AYARLAR ─────────────────────────────────────────────────
MAX_SPEED        = 5.56   # m/s = 20 km/h (maksimum hız)
TARGET_CRUISE    = 3.50   # m/s = ~12.6 km/h (algoritma seyir hızı)
REV_SPEED        = 2.00   # m/s = ~7.2 km/h (manuel geri vites)
CONTROL_RATE     = 0.05   # 20 Hz kontrol döngüsü
DATA_TIMEOUT     = 1.0    # algılama verisi bu saniyeden eskiyse güvenli moda geç
SAFE_SPEED_RATIO = 0.3    # veri kesilince hızı bununla çarp
MIN_SPEED        = 0.5    # m/s — en yavaş seyir (tam durmak yerine çok yavaş geçiş)
START_DELAY      = 10.0   # saniye — simülasyon başladıktan sonra bekleme süresi


class SpeedControllerNode(Node):
    """Algılama → Hız sınırlama düğümü."""

    def __init__(self):
        super().__init__('speed_controller_node')

        # ── DecisionArbiter ──
        self._arbiter = DecisionArbiter()

        # ── Son bilinen cmd_vel (kullanıcı girişi) ──
        self._raw_twist = Twist()
        self._raw_cmd_time: float | None = None

        # ── Son bilinen algılama durumları ──
        self._light_speed_cap: float = 1.0
        self._light_must_stop: bool = False
        self._light_time: float = 0.0

        self._sign_speed_cap: float = 1.0
        self._sign_must_stop: bool = False
        self._sign_time: float = 0.0

        self._obs_speed_cap: float = 1.0
        self._obs_emergency: bool = False
        self._obs_behavior: str = "clear"
        self._obs_avoid_dir: str | None = None
        self._obs_time: float = 0.0

        # ── ROS Abonelikler ──
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_raw', self._cmd_vel_callback, 10)

        self._sub_light = self.create_subscription(
            String, '/perception/traffic_light_state', self._light_callback, 10)

        self._sub_sign = self.create_subscription(
            String, '/perception/traffic_sign_state', self._sign_callback, 10)

        self._sub_obs = self.create_subscription(
            String, '/perception/obstacle_state', self._obstacle_callback, 10)

        # ── ROS Yayıncı ──
        self._pub_cmd = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # ── State publish (debug) ──
        self._pub_state = self.create_publisher(
            String, '/speed_controller/state', 10)

        # ── Kontrol döngüsü ──
        self._start_time = time.time()
        self._timer = self.create_timer(CONTROL_RATE, self._control_loop)

        self.get_logger().info(
            f'SpeedControllerNode baslatildi. '
            f'TARGET_CRUISE={TARGET_CRUISE}m/s, START_DELAY={START_DELAY}s, '
            f'{1.0/CONTROL_RATE:.0f}Hz kontrol'
        )

    # ═══════════════════════════════════════════════════════════════
    # Callback'ler
    # ═══════════════════════════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._raw_twist = msg
        self._raw_cmd_time = time.time()

    def _light_callback(self, msg: String) -> None:
        """Parse: color:red|must_stop:True|speed_cap:0.00|..."""
        self._light_time = time.time()
        data = self._parse_kv(msg.data)
        self._light_must_stop = data.get('must_stop', 'False') == 'True'
        self._light_speed_cap = float(data.get('speed_cap', '1.0'))

    def _sign_callback(self, msg: String) -> None:
        """Parse: must_stop:True|speed_cap:0.50|..."""
        self._sign_time = time.time()
        data = self._parse_kv(msg.data)
        self._sign_must_stop = data.get('must_stop', 'False') == 'True'
        self._sign_speed_cap = float(data.get('speed_cap', '1.0'))

    def _obstacle_callback(self, msg: String) -> None:
        """Parse: behavior:emergency_stop|emergency:True|speed_cap:0.00|..."""
        self._obs_time = time.time()
        data = self._parse_kv(msg.data)
        self._obs_emergency = data.get('emergency', 'False') == 'True'
        self._obs_speed_cap = float(data.get('speed_cap', '1.0'))
        self._obs_behavior = data.get('behavior', 'clear')
        self._obs_avoid_dir = data.get('avoid_dir', 'none')
        if self._obs_avoid_dir == 'none':
            self._obs_avoid_dir = None

    @staticmethod
    def _parse_kv(data: str) -> dict[str, str]:
        """key:value|key:value → dict."""
        result: dict[str, str] = {}
        if not data:
            return result
        for pair in data.split('|'):
            if ':' not in pair:
                continue
            k, v = pair.split(':', 1)
            result[k.strip()] = v.strip()
        return result

    # ═══════════════════════════════════════════════════════════════
    # Kontrol döngüsü
    # ═══════════════════════════════════════════════════════════════

    def _control_loop(self) -> None:
        now = time.time()

        # ── Başlangıç gecikmesi (simülasyonun oturması için) ──
        elapsed = now - self._start_time
        if elapsed < START_DELAY:
            remaining = int(START_DELAY - elapsed) + 1
            out = Twist()
            out.linear.x = 0.0
            out.angular.z = 0.0
            self._pub_cmd.publish(out)
            self.get_logger().info(
                f'Baslangic gecikmesi: {remaining}s...',
                throttle_duration_sec=2.0)
            return

        # ── Veri tazeliği kontrolü ──
        light_ok = (now - self._light_time) < DATA_TIMEOUT
        sign_ok = (now - self._sign_time) < DATA_TIMEOUT
        obs_ok = (now - self._obs_time) < DATA_TIMEOUT

        # ── Hız sınırlarını birleştir ──
        speed_caps = []
        emergency = False
        reasons: list[str] = []

        # Trafik ışığı
        if light_ok:
            speed_caps.append(self._light_speed_cap)
            if self._light_must_stop:
                emergency = True
                reasons.append('light: must_stop')
        else:
            speed_caps.append(SAFE_SPEED_RATIO)
            reasons.append('light: data timeout')

        # Trafik tabelası
        if sign_ok:
            speed_caps.append(self._sign_speed_cap)
            if self._sign_must_stop:
                emergency = True
                reasons.append('sign: must_stop')
        else:
            speed_caps.append(SAFE_SPEED_RATIO)
            reasons.append('sign: data timeout')

        # Engel
        if obs_ok:
            speed_caps.append(self._obs_speed_cap)
            if self._obs_emergency:
                emergency = True
                reasons.append(f'obstacle: {self._obs_behavior}')
        else:
            speed_caps.append(SAFE_SPEED_RATIO)
            reasons.append('obstacle: data timeout')

        # ── En kısıtlayıcı hız sınırını uygula ──
        effective_cap = min(speed_caps) if speed_caps else 1.0

        # ── Kullanıcı komutu (SADECE direksiyon) ──
        user_linear = self._raw_twist.linear.x
        user_angular = self._raw_twist.angular.z

        # Kullanıcı komutu timeout → direksiyon sıfırla
        cmd_stale = self._raw_cmd_time is None or (now - self._raw_cmd_time) > 2.0
        if cmd_stale:
            user_angular = 0.0

        # ── Algoritma hedef hızını belirle ──
        # Geri vites: kullanıcı S tuşuna basarsa manuel geri git
        # İleri: algoritma otomatik seyir hızı × sınır katsayısı
        if user_linear < -0.01:
            # Manuel geri vites — hız sınırı uygulanır
            target_linear = -REV_SPEED * effective_cap
        else:
            # Algoritma kontrollü ileri seyir
            target_linear = TARGET_CRUISE * effective_cap
            # Sıfıra çok yakınsa minimum seyir hızını koru (tam durma değilse)
            if 0.0 < target_linear < MIN_SPEED and not emergency:
                target_linear = MIN_SPEED

        # ── Çıkış komutu oluştur ──
        out = Twist()

        if emergency:
            # Acil durum: tam duruş, direksiyon sıfırla
            out.linear.x = 0.0
            out.angular.z = 0.0
        else:
            # Kullanıcı direksiyonunu aynen koru
            out.angular.z = user_angular
            out.linear.x = target_linear

        self._pub_cmd.publish(out)

        # ── Debug durum mesajı (throttled) ──
        state_msg = String()
        state_msg.data = (
            f"emergency:{emergency}|"
            f"speed_cap:{effective_cap:.2f}|"
            f"target:{TARGET_CRUISE:.1f}|"
            f"cmd_linear:{out.linear.x:.2f}|"
            f"cmd_angular:{out.angular.z:.2f}|"
            f"user_steer:{user_angular:.2f}|"
            f"light_ok:{light_ok}|sign_ok:{sign_ok}|obs_ok:{obs_ok}|"
            f"behavior:{self._obs_behavior}|"
            f"reasons:{' '.join(reasons)}"
        )
        self._pub_state.publish(state_msg)

        # Log
        if emergency:
            status = "ACIL DURUS"
        elif out.linear.x < 0:
            status = f"GERI {abs(out.linear.x):.1f}m/s"
        else:
            status = f"seyir {out.linear.x:.1f}m/s"
        self.get_logger().info(
            f'{status} | cap={effective_cap:.0%} | '
            f'light={self._light_speed_cap:.0%} sign={self._sign_speed_cap:.0%} obs={self._obs_speed_cap:.0%} | '
            f'direksiyon={out.angular.z:.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
