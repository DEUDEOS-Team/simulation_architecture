#!/usr/bin/env python3
"""
Otonom şerit-takip kontrol node'u (autonomous_control_node).

Basit bir P kontrolcü + yumuşatma + şerit-kayıp toleransı. Notlar:
  1) CAM_W parametrik (varsayılan 1280 = kameranın gerçek genişliği; lane_test_node
     noktaları orijinal çözünürlükte yayınlıyor). Yanlış değer aracı sürekli yana saptırır.
  2) /cmd_vel (geometry_msgs/Twist) yayını — araç Gazebo'da Ackermann plugin ile
     /cmd_vel dinliyor. target_angle(derece)+throttle -> Ackermann yaw-rate'e çevrilir.
     (Orijinal /control/target_angle ve /control/throttle yayınları da korunur.)
  3) Yanal ofset: /planning/lateral_offset (metre, + = SOL) dinlenir ve şerit hedef
     noktası o kadar yana kaydırılır ("şerit merkezi 1.4 m solda gibi izle"). Engelden
     kaçınma direksiyonu buradan geçer; tek direksiyon üreteci bu node. Böylece kaçınma
     ile şerit takibi aynı çıktıyı iki yerden çekmez.
     Metre -> piksel: yer düzlemi projeksiyonu (lidar_obstacle_node ile aynı model)
         X = fy·h / (v − cy)   [hedef satırın yerdeki ileri mesafesi]
         du = −fx · ofset / X  [u = cx − fx·y/X olduğundan +sol ofset u'yu KÜÇÜLTÜR]
     Ofset bayatlarsa (>1 s) 0 kabul edilir -> araç kendi şeridine döner.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import numpy as np
import time

# Ofset bu süre boyunca gelmezse 0 sayılır (kaçınma node'u ölmüşse şeride dön).
OFFSET_TIMEOUT_S = 1.0


class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # ─── Parametreler ───
        # Kamera genişliği = lane_test_node'un nokta yayınladığı çözünürlük (proc_width=0 -> 1280)
        self.cam_w        = float(self.declare_parameter('cam_width', 1280.0).value)
        # throttle (0..0.5) -> linear hız (m/s) ölçeği
        self.speed_scale  = float(self.declare_parameter('speed_scale', 4.0).value)
        # Ackermann geometrisi (URDF ile aynı)
        self.wheel_base   = float(self.declare_parameter('wheel_base', 1.675).value)
        # Maks direksiyon (rad). URDF/Ackermann eklentisindeki steering_limit ile AYNI olmalı
        # — yoksa fazlası kırpılır. 0.5236 ≈ 30°.
        self.steer_limit  = float(self.declare_parameter('steer_limit', 0.5236).value)
        # Kontrolcü kazancı (büyük = aynı hata için daha sert direksiyon = daha keskin dönüş).
        self.kp           = float(self.declare_parameter('kp', 0.8).value)
        # Hedef açı tepesi (derece). steer_limit ile tutarlı tutulur (~30°).
        self.max_angle    = float(self.declare_parameter('max_angle', 30.0).value)

        # Durum Değişkenleri
        self.last_valid_steer = 0.0
        self.last_valid_throttle = 0.0
        self.last_lane_time = time.perf_counter()
        self.LANE_LOST_PATIENCE = 1.0

        # steer_alpha: büyük = daha çevik/hızlı direksiyon tepkisi (az gecikme).
        self.steer_alpha = float(self.declare_parameter('steer_alpha', 0.3).value)
        self.throttle_alpha = 0.2

        # ── Yanal ofset (engelden kaçınma) ──
        # Kamera yüksekliği (URDF: kamera z = 0.948 m, pitch 0)
        self.cam_height_m = float(self.declare_parameter('cam_height_m', 0.948).value)
        self.lateral_offset_m = 0.0
        self.offset_stamp = 0.0
        self._cam_fx = self._cam_fy = self._cam_cx = self._cam_cy = 0.0
        self._cam_info_warned = False

        # Subscriber: Perception node'dan gelen noktaları dinler
        self.sub_pts = self.create_subscription(
            Float32MultiArray,
            '/perception/center_pts',
            self.pts_callback,
            10
        )
        self.sub_offset = self.create_subscription(
            Float32, '/planning/lateral_offset', self.offset_callback, 10)
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.cam_info_callback, 10)

        # Hedef açıyı ve hız değişimini ayrı ayrı Float32 olarak yayınlıyoruz (orijinal arayüz)
        self.pub_target_angle = self.create_publisher(Float32, '/control/target_angle', 10)
        self.pub_throttle = self.create_publisher(Float32, '/control/throttle', 10)
        # Gazebo Ackermann sürüşü için /cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'✅ autonomous_control_node hazır. cam_w={self.cam_w}, speed_scale={self.speed_scale}. '
            f'Dinleniyor: /perception/center_pts -> yayın: /cmd_vel'
        )

    def pts_callback(self, msg):
        flat_data = msg.data
        pts = [(flat_data[i], flat_data[i+1]) for i in range(0, len(flat_data), 2)]
        self.run_control_algorithm(pts)

    def offset_callback(self, msg):
        self.lateral_offset_m = float(msg.data)
        self.offset_stamp = time.perf_counter()

    def cam_info_callback(self, msg):
        # Şerit noktaları cam_w çözünürlüğünde geliyor; camera_info başka
        # çözünürlükteyse iç parametreleri ölçekle.
        s = self.cam_w / float(msg.width) if msg.width else 1.0
        self._cam_fx = float(msg.k[0]) * s
        self._cam_cx = float(msg.k[2]) * s
        self._cam_fy = float(msg.k[4]) * s
        self._cam_cy = float(msg.k[5]) * s

    def _offset_pixels(self, target_v):
        """Aktif yanal ofseti (m) hedef satırdaki piksel kaymasına çevir.

        Yer düzlemi: X = fy·h/(v − cy). Ofset + = SOL ve u = cx − fx·y/X olduğundan
        sola kayma hedef pikselini SOLA (küçük u) taşır -> du = −fx·ofset/X.
        Ofset yoksa/bayatsa veya satır ufkun üstündeyse 0 döner (kaydırma yok).
        """
        if abs(self.lateral_offset_m) < 1e-3:
            return 0.0
        if (time.perf_counter() - self.offset_stamp) > OFFSET_TIMEOUT_S:
            return 0.0
        if self._cam_fx <= 0.0:
            if not self._cam_info_warned:
                self._cam_info_warned = True
                self.get_logger().warn(
                    'KAÇINMA UYGULANAMIYOR: /camera/camera_info gelmedi — '
                    'yanal ofset piksele çevrilemiyor, şerit takibi ofsetsiz sürüyor.')
            return 0.0

        dv = float(target_v) - self._cam_cy
        if dv <= 1.0:                      # ufkun üstü/çok uzak -> güvenilmez
            return 0.0
        x_fwd = self._cam_fy * self.cam_height_m / dv
        x_fwd = float(np.clip(x_fwd, 2.0, 30.0))
        return -self._cam_fx * self.lateral_offset_m / x_fwd

    def run_control_algorithm(self, pts):
        now = time.perf_counter()

        msg_angle = Float32()
        msg_throttle = Float32()

        if len(pts) > 0:
            target_idx = min(len(pts) // 2 + 2, len(pts) - 1)
            target_x, target_y = pts[target_idx]

            # Kaçınma ofseti burada uygulanmıyor: bu yol yalnız LANE modunda çalışıyor,
            # engelin olduğu yerde şerit segmentasyonu düşünce vehicle_controller PLAN'a
            # geçip /cmd_vel_lane'i yok sayıyor ve kaçınma komutu çöpe gidiyordu. Kaçınma
            # artık vehicle_controller'da, son direksiyona ekleniyor (/planning/avoid_steer)
            # — LANE/PLAN/TURN fark etmeksizin. _offset_pixels() ileride kullanım için duruyor.

            center_camera_x = self.cam_w / 2.0
            error = (target_x - center_camera_x) / center_camera_x

            raw_steer = np.clip(self.kp * error, -1.0, 1.0)
            raw_throttle = 0.5 if abs(raw_steer) < 0.2 else 0.35

            steer = (self.steer_alpha * raw_steer) + ((1.0 - self.steer_alpha) * self.last_valid_steer)
            throttle = (self.throttle_alpha * raw_throttle) + ((1.0 - self.throttle_alpha) * self.last_valid_throttle)

            self.last_valid_steer = steer
            self.last_valid_throttle = throttle
            self.last_lane_time = now

            # Teşhis logu: "şerit ortayı gösteriyor ama araç yana kayıyor" sorununu
            # ayırmak için. hata sabit + araç kayıyorsa -> kontrol/kalibrasyon; hata
            # zıplıyorsa -> algı.
            self.get_logger().info(
                f"ŞERİT hata={error:+.3f} nokta={len(pts)} target_x={target_x:.0f}/{self.cam_w:.0f} "
                f"raw={raw_steer:+.2f} steer={steer:+.2f}", throttle_duration_sec=0.5)

            # Değerleri mesajlara atıyoruz
            msg_angle.data = float(-steer * self.max_angle)
            msg_throttle.data = float(throttle)

        else:
            if (now - self.last_lane_time) < self.LANE_LOST_PATIENCE:
                msg_throttle.data = float(self.last_valid_throttle * 0.8)
                msg_angle.data = float(-self.last_valid_steer)
            else:
                msg_throttle.data = 0.0
                msg_angle.data = 0.0

        self.pub_target_angle.publish(msg_angle)
        self.pub_throttle.publish(msg_throttle)

        # ── /cmd_vel'e çevir (Gazebo Ackermann plugin için) ──
        # linear.x = ileri hız, angular.z = yaw-rate = v * tan(steer) / wheel_base
        twist = Twist()
        linear_x = msg_throttle.data * self.speed_scale
        steer_rad = float(np.clip(np.radians(msg_angle.data), -self.steer_limit, self.steer_limit))
        twist.linear.x = float(linear_x)
        if abs(linear_x) > 1e-3:
            twist.angular.z = float(linear_x * np.tan(steer_rad) / self.wheel_base)
        else:
            twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
