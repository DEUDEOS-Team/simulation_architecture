#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading
import math

WHEEL_BASE  = 1.675
MAX_SPEED   = 5.56   # 20 km/h
REV_SPEED   = 2.78   # 10 km/h
STEER_ANGLE = math.radians(30)

MSG = """
=== Araç Klavye Kontrolü ===
  W        : İleri  (20 km/h)
  S        : Geri   (10 km/h)
  A        : Sol    (30°)
  D        : Sağ    (30°)
  W + A/D  : İleri + Direksiyon (aynı anda)
  SPACE    : Fren + Düzelt
  ESC / Q  : Çıkış
============================
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20 Hz

        self._pressed = set()
        self._lock = threading.Lock()
        self._running = True

        listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        listener.daemon = True
        listener.start()

        print(MSG)

    def _on_press(self, key):
        with self._lock:
            self._pressed.add(self._name(key))

    def _on_release(self, key):
        name = self._name(key)
        with self._lock:
            self._pressed.discard(name)

        if name in ('esc', 'q'):
            self._running = False
            rclpy.shutdown()

    @staticmethod
    def _name(key):
        try:
            return key.char.lower()
        except AttributeError:
            return key.name  # 'space', 'esc' vb.

    def publish_cmd(self):
        if not self._running:
            return

        with self._lock:
            pressed = set(self._pressed)

        # Fren
        if 'space' in pressed:
            self._send(0.0, 0.0)
            return

        # Hız
        speed = 0.0
        if 'w' in pressed:
            speed = MAX_SPEED
        elif 's' in pressed:
            speed = -REV_SPEED

        # Direksiyon
        steer = 0.0
        if 'a' in pressed:
            steer = STEER_ANGLE       # sol → pozitif angular.z → sol dönüş
        elif 'd' in pressed:
            steer = -STEER_ANGLE      # sağ → negatif angular.z → sağ dönüş

        self._send(speed, steer)

    def _send(self, speed, steer):
        msg = Twist()
        msg.linear.x = speed
        ref = speed if abs(speed) > 0.1 else 0.5
        msg.angular.z = ref * math.tan(steer) / WHEEL_BASE
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
