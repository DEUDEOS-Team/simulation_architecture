#!/usr/bin/env python3
"""
Ham kamera karesi toplayıcı (dataset_recorder).

Amaç: modeli ÇALIŞTIRMADAN, harita boyunca araç sürülürken /camera/image'den ham
(overlay'siz) kareleri PNG olarak kaydetmek — model fine-tune / yeniden eğitim için.
/odom'dan konum okuyup araç ancak MIN_DIST metre ilerlediğinde kare kaydeder (harita
boyunca dengeli kapsama, binlerce kopya kare olmaz). Her kareye konum etiketi dosya
adına yazılır (gerekirse sonradan eşleştirmek için).

Çalıştırma (Gazebo + teleop AÇIKKEN, AYRI bir terminalde):
  python3 src/araba/scripts/dataset_recorder.py

Ayarlar (ortam değişkeni ile, isteğe bağlı):
  REC_OUT=<dizin>        kayıt klasörü (varsayılan: dataset/capture_<zaman>)
  REC_MIN_DIST=<metre>   iki kare arası min mesafe (varsayılan 0.5)
  REC_TOPIC=<topic>      kamera topic'i (varsayılan /camera/image)

Durdurma: Ctrl+C. Kaç kare kaydedildiğini özetler.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        self.bridge = CvBridge()

        self.topic    = os.environ.get('REC_TOPIC', '/camera/image')
        self.min_dist = float(os.environ.get('REC_MIN_DIST', '0.5'))
        default_out   = os.path.join('dataset', 'capture_' + time.strftime('%Y%m%d_%H%M%S'))
        self.out      = os.path.abspath(os.environ.get('REC_OUT', default_out))
        os.makedirs(self.out, exist_ok=True)

        self.pos = None              # güncel (x, y) — /odom'dan
        self.last_saved_pos = None   # son kaydedilen karenin konumu
        self.have_odom = False
        self.saved_any = False
        self.frame_idx = 0
        self.frames_since_save = 0
        self.SAVE_EVERY_N = 15       # odom YOKSA: her N karede bir kaydet (fallback)

        # use_sim_time: sim saatiyle uyumlu olsun (sadece kayıt için kritik değil ama tutarlı)
        self.create_subscription(Image, self.topic, self.on_image, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.get_logger().info(
            f'📸 dataset_recorder hazır.\n'
            f'   topic={self.topic}   min_dist={self.min_dist} m\n'
            f'   kayıt klasörü: {self.out}\n'
            f'   (aracı WASD ile sür — her {self.min_dist} m\'de bir ham kare kaydedilecek)'
        )

    def on_odom(self, msg):
        self.have_odom = True
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)

    def on_image(self, msg):
        save = False
        if self.have_odom and self.pos is not None:
            if not self.saved_any or self.last_saved_pos is None:
                save = True
            else:
                d = math.hypot(self.pos[0] - self.last_saved_pos[0],
                               self.pos[1] - self.last_saved_pos[1])
                save = d >= self.min_dist
        else:
            # /odom akmıyorsa kare sayısına göre kaydet (fallback)
            self.frames_since_save += 1
            save = (not self.saved_any) or (self.frames_since_save >= self.SAVE_EVERY_N)

        if not save:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'görüntü dönüşüm hatası: {e}')
            return

        self.frame_idx += 1
        self.frames_since_save = 0
        self.saved_any = True

        if self.pos is not None:
            self.last_saved_pos = self.pos
            name = f'frame_{self.frame_idx:05d}_x{self.pos[0]:.2f}_y{self.pos[1]:.2f}.png'
        else:
            name = f'frame_{self.frame_idx:05d}.png'

        cv2.imwrite(os.path.join(self.out, name), frame)
        if self.frame_idx % 10 == 0:
            self.get_logger().info(f'  kaydedilen kare: {self.frame_idx}   (son: {name})')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'⏹  Toplam {node.frame_idx} kare kaydedildi → {node.out}'
        )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
