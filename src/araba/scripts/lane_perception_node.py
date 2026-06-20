#!/usr/bin/env python3
"""
lane_perception_node.py
------------------------
ROS 2 wrapper: /camera/image → model.onnx (YOLO-seg) → /perception/center_pts + görselleştirme.

Şerit segmentasyonu yapar, şerit merkez eğrisi noktalarını çıkarır ve
OpenCV penceresinde canlı takip gösterimi sunar.

Bu dosya SADECE altyapıdır — ONNX inference ve ROS iletişimi.
Şerit eğrisi uydurma ve EMA yumuşatma parametreleri ayar olarak dışarıdan
verilebilir, algoritma mantığı değiştirilmez.
"""

import os
import sys

# deos_algorithms import'ları için sys.path düzenlemesi
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
from collections import deque


# ─── AYARLAR ─────────────────────────────────────────────────
IMG_SIZE        = 640
CONF_THRESH     = 0.40
IOU_THRESH      = 0.45
PROCESS_EVERY_N = 2           # her N kareden birini işle (segmentation daha ağır)
SHOW_WINDOW     = True
WINDOW_NAME     = "Serit Takibi (model.onnx)"

# Şerit eğrisi uydurma
POLY_DEGREE    = 2            # 2. derece polinom
NUM_CURVE_PTS  = 20           # eğri üzerinde örneklenen nokta sayısı
EMA_ALPHA      = 0.35         # üstel yumuşatma katsayısı (0=tam geçmiş, 1=anlık)

# ROI (ilgi bölgesi) — görüntünün alt kısmı
ROI_TOP_RATIO    = 0.45       # üstten itibaren ROI başlangıcı
ROI_BOTTOM_RATIO = 0.92       # alttan itibaren ROI bitişi
ROI_SIDE_MARGIN  = 0.05       # yanlardan marj

# Şerit maskesi işleme
MIN_CONTOUR_AREA  = 200       # minimum kontur alanı (px²)
MORPH_KERNEL_SIZE = 5         # morfolojik işlem kernel boyutu


class LanePerceptionNode(Node):
    """Kamera görüntüsünü YOLO-seg ONNX modeliyle işleyen şerit takip düğümü."""

    def __init__(self):
        super().__init__('lane_perception_node')

        # ── Parametreler ──
        self.declare_parameter('model_path', '')
        self.declare_parameter('show_window', SHOW_WINDOW)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._show_window = self.get_parameter('show_window').get_parameter_value().bool_value

        # ── ONNX oturumu ──
        providers = ['CPUExecutionProvider']
        self._session = ort.InferenceSession(model_path, providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        self.get_logger().info(f'ONNX model yüklendi: {model_path}')
        self.get_logger().info(f'Kullanılan provider: {self._session.get_providers()[0]}')

        # ── cv_bridge ──
        self._bridge = CvBridge()

        # ── Durum ──
        self._frame_count = 0
        self._last_center_pts: list[float] = []    # [x1,y1, x2,y2, ...]
        self._last_mask_frame = None                # son maske overlay
        self._last_curve = None                     # son polinom katsayıları (EMA uygulanmış)
        self._poly_history = deque(maxlen=5)        # polinom geçmişi (EMA için)

        # ── ROI maskesi (bir kez oluştur, her frame'de resize) ──
        self._roi_mask = None

        # ── ROS iletişimi ──
        self._sub = self.create_subscription(
            Image, '/camera/image', self._image_callback, 10)

        self._pub_pts = self.create_publisher(
            Float32MultiArray, '/perception/center_pts', 10)

        self._pub_mask = self.create_publisher(
            Image, '/perception/mask_overlay', 10)

        self.get_logger().info(
            f'LanePerceptionNode başlatıldı. '
            f'Her {PROCESS_EVERY_N} karede bir inference. '
            f'Pencere: {"açık" if self._show_window else "kapalı"}'
        )

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1

        # ── Görüntüyü çöz ──
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Görüntü çözme hatası: {e}')
            return

        # ── Periyodik inference ──
        if self._frame_count % PROCESS_EVERY_N == 0:
            self._run_inference(frame)

        # ── Son bilinen şerit noktalarını yayınla (her frame'de) ──
        pts_msg = Float32MultiArray()
        pts_msg.data = self._last_center_pts if self._last_center_pts else []
        self._pub_pts.publish(pts_msg)

        # ── Maske overlay yayını ──
        if self._last_mask_frame is not None:
            try:
                mask_msg = self._bridge.cv2_to_imgmsg(self._last_mask_frame, 'bgr8')
                mask_msg.header.stamp = msg.header.stamp
                self._pub_mask.publish(mask_msg)
            except Exception as e:
                self.get_logger().error(f'Maske yayınlama hatası: {e}')

        # ── Debug penceresi ──
        if self._show_window and self._last_mask_frame is not None:
            cv2.imshow(WINDOW_NAME, self._last_mask_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.get_logger().info('ESC — pencere kapatılıyor')
                cv2.destroyWindow(WINDOW_NAME)
                self._show_window = False

    def _run_inference(self, frame: np.ndarray) -> None:
        """ONNX modeli ile segmentasyon çalıştır ve şerit eğrisi çıkar."""
        h, w = frame.shape[:2]

        # ROI maskesini güncelle (frame boyutu değişirse)
        if self._roi_mask is None or self._roi_mask.shape[:2] != (h, w):
            self._roi_mask = self._create_roi_mask(h, w)

        # ── Ön işleme ──
        img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.expand_dims(np.transpose(img, (2, 0, 1)), 0)

        # ── Inference ──
        try:
            output = self._session.run(None, {self._input_name: inp})
        except Exception as e:
            self.get_logger().error(f'Inference hatası: {e}')
            return

        # ── Segmentasyon maskesini çıkar ──
        mask_frame, center_pts = self._process_segmentation(output, frame)

        if center_pts:
            self._last_center_pts = [v for pt in center_pts for v in pt]

        self._last_mask_frame = mask_frame

        # Log
        self.get_logger().info(
            f'Şerit: {len(center_pts)} nokta',
            throttle_duration_sec=1.0)

    def _process_segmentation(self, output: list, frame: np.ndarray):
        """
        YOLO-seg çıktısını işleyip şerit merkez noktalarını ve görselleştirme karesini üretir.

        output[0]: detection boxes (1, N, ...) veya (N, ...)
        output[1] (varsa): prototype masks (1, 32, 160, 160)
        output[...]: mask coefficients
        """
        h, w = frame.shape[:2]
        vis = frame.copy()

        center_pts: list[tuple[float, float]] = []

        # ── YOLO-seg formatı: output[0] detection, output[1] mask protos ──
        if len(output) >= 2:
            dets = output[0]   # (1, N, 4+1+32) → boxes, scores, mask_coeffs
            proto = output[1]  # (1, 32, 160, 160) — maske prototipleri

            if isinstance(dets, np.ndarray) and len(dets.shape) >= 2:
                # dets sıkıştır: ilk boyutu kaldır
                if dets.shape[0] == 1:
                    dets = dets[0]

                # YOLO-seg formatı: [cx, cy, w, h, score, mask_coeff_0..mask_coeff_31]
                if dets.shape[1] >= 37:  # 4 + 1 + 32 = 37
                    boxes_cx = dets[:, :4]
                    scores = dets[:, 4]
                    mask_coeffs = dets[:, 5:37]  # 32 maske katsayısı

                    # Sınıf bilgisi: scores'tan threshold geçenler (class-agnostic)
                    valid = scores > CONF_THRESH
                    boxes_cx = boxes_cx[valid]
                    scores = scores[valid]
                    mask_coeffs = mask_coeffs[valid]

                    if len(scores) > 0:
                        # Bbox'ları orijinal koordinatlara dönüştür
                        sx, sy = w / IMG_SIZE, h / IMG_SIZE
                        x1 = ((boxes_cx[:, 0] - boxes_cx[:, 2] / 2) * sx).astype(int)
                        y1 = ((boxes_cx[:, 1] - boxes_cx[:, 3] / 2) * sy).astype(int)
                        x2 = ((boxes_cx[:, 0] + boxes_cx[:, 2] / 2) * sx).astype(int)
                        y2 = ((boxes_cx[:, 1] + boxes_cx[:, 3] / 2) * sy).astype(int)

                        # ── Maske birleştirme ──
                        if proto.shape[0] == 1:
                            proto = proto[0]  # (32, 160, 160)

                        combined_mask = np.zeros((160, 160), dtype=np.float32)
                        for i, coeffs in enumerate(mask_coeffs):
                            m = np.dot(coeffs, proto.reshape(32, -1)).reshape(160, 160)
                            # Numerik kararlı sigmoid
                            m = np.clip(m, -20.0, 20.0)
                            m = 1.0 / (1.0 + np.exp(-m))
                            combined_mask = np.maximum(combined_mask, m)

                        # Maske threshold ve resize
                        combined_mask = (combined_mask > 0.5).astype(np.uint8) * 255
                        combined_mask = cv2.resize(combined_mask, (w, h),
                                                   interpolation=cv2.INTER_LINEAR)

                        # ── ROI uygula ──
                        if self._roi_mask is not None:
                            combined_mask = cv2.bitwise_and(combined_mask, self._roi_mask)

                        # ── Morfolojik temizlik ──
                        kernel = cv2.getStructuringElement(
                            cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
                        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
                        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

                        # ── Şerit merkez noktalarını çıkar ──
                        center_pts = self._extract_center_points(combined_mask, h, w)

                        # ── EMA yumuşatma ──
                        center_pts = self._ema_smooth(center_pts)

                        # ── Görselleştirme ──
                        # Maske overlay (mavi)
                        mask_overlay = np.zeros_like(vis)
                        mask_overlay[combined_mask > 128] = (255, 100, 0)  # mavi ton
                        vis = cv2.addWeighted(vis, 0.7, mask_overlay, 0.3, 0)

                        # Şerit merkez noktaları ve eğri
                        for pt in center_pts:
                            px, py = int(pt[0]), int(pt[1])
                            cv2.circle(vis, (px, py), 4, (0, 255, 255), -1)

                        # Eğri çizgisi
                        if len(center_pts) >= 3:
                            pts_arr = np.array([[int(pt[0]), int(pt[1])] for pt in center_pts])
                            cv2.polylines(vis, [pts_arr], False, (0, 255, 0), 2)

                        # Bbox'lar (debug)
                        for i in range(len(scores)):
                            cv2.rectangle(vis, (x1[i], y1[i]), (x2[i], y2[i]),
                                          (0, 255, 255), 1)
                            cv2.putText(vis, f'{scores[i]:.2f}',
                                        (x1[i], y1[i] - 4),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                else:
                    self.get_logger().warn(
                        f'Beklenmeyen detection formatı: shape={dets.shape}',
                        throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(
                    f'Detection verisi numpy array değil: {type(dets)}',
                    throttle_duration_sec=5.0)
        else:
            # Segmentation çıktısı yok — sadece detection olabilir
            self.get_logger().debug(
                f'Segmentasyon çıktısı bulunamadı, output len={len(output)}',
                throttle_duration_sec=5.0)

        # ── FPS ve bilgi ──
        cv2.putText(vis, f"Frame: {self._frame_count} | Serit nokta: {len(center_pts)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        # ROI sınırı
        if self._roi_mask is not None:
            roi_border = cv2.Canny(self._roi_mask, 50, 150)
            vis[roi_border > 0] = (100, 100, 100)

        return vis, center_pts

    def _extract_center_points(self, mask: np.ndarray, h: int, w: int) -> list[tuple[float, float]]:
        """Maske üzerindeki şerit bölgesinin yatay merkezlerini yukarıdan aşağıya örnekler."""
        points = []
        step = h // NUM_CURVE_PTS

        for row in range(0, h, step):
            # Bu satırdaki şerit piksellerini bul
            row_pixels = np.where(mask[row, :] > 128)[0]
            if len(row_pixels) > MIN_CONTOUR_AREA // step:  # yeterli piksel var mı?
                cx = float(np.median(row_pixels))
                points.append((cx, float(row)))

        return points

    def _ema_smooth(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Üstel hareketli ortalama ile şerit eğrisini yumuşat."""
        if not points:
            self._poly_history.clear()
            return []

        pts = np.array(points)
        if len(pts) < POLY_DEGREE + 1:
            return points

        # Polinom uydur (x = f(y))
        try:
            coeffs = np.polyfit(pts[:, 1], pts[:, 0], POLY_DEGREE)
        except np.linalg.LinAlgError:
            return points

        # EMA
        if self._poly_history:
            # Tüm geçmiş katsayıların EMA'sı
            prev_avg = np.mean(list(self._poly_history), axis=0)
            coeffs = EMA_ALPHA * coeffs + (1 - EMA_ALPHA) * prev_avg

        self._poly_history.append(coeffs)

        # Yumuşatılmış eğriyi örnekle
        y_vals = np.linspace(pts[:, 1].min(), pts[:, 1].max(), NUM_CURVE_PTS)
        x_vals = np.polyval(coeffs, y_vals)

        return [(float(x), float(y)) for x, y in zip(x_vals, y_vals)]

    def _create_roi_mask(self, h: int, w: int) -> np.ndarray:
        """İlgi bölgesi (ROI) maskesi oluştur."""
        mask = np.zeros((h, w), dtype=np.uint8)
        pts = np.array([[
            (int(w * ROI_SIDE_MARGIN), int(h * ROI_BOTTOM_RATIO)),
            (int(w * ROI_SIDE_MARGIN), int(h * ROI_TOP_RATIO)),
            (int(w * (1 - ROI_SIDE_MARGIN)), int(h * ROI_TOP_RATIO)),
            (int(w * (1 - ROI_SIDE_MARGIN)), int(h * ROI_BOTTOM_RATIO)),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, pts, 255)
        return mask

    def close(self):
        if self._show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = LanePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
