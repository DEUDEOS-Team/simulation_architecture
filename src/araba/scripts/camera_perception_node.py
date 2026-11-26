#!/usr/bin/env python3
"""
camera_perception_node.py
--------------------------
ROS 2 wrapper: /camera/image → detection.onnx → /perception/detections + görselleştirme.

29 sınıf tespit yapar (trafik levhaları, ışıklar, koni, bariyer, yaya) ve
/perception/detections (Float32MultiArray) konusuna yayınlar.
Aynı zamanda OpenCV penceresinde canlı gösterim sunar.

Bu dosya SADECE altyapıdır — ONNX inference ve ROS iletişimi.
Algoritma mantığı algorithm modüllerindedir.
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

# ─── Algoritma modülleri (sadece veri dönüşümü için, mantık değiştirilmez) ───
from perception_fusion import PerceptionFrame, fuse
from sensors.types import StereoBbox

# ─── AYARLAR ─────────────────────────────────────────────────
IMG_SIZE       = 640
CONF_THRESH    = 0.40
IOU_THRESH     = 0.45
PROCESS_EVERY_N = 3       # her N kareden birini işle
SHOW_WINDOW    = True     # OpenCV debug penceresi
WINDOW_NAME    = "Kamera Algılama (detection.onnx)"

# 29 sınıf — Türkçe isimler (algoritma modülleriyle uyumlu)
CLASS_NAMES = {
    0:  'ada etrafinda donunuz',  1:  'dur tabelasi',
    2:  'durak',                   3:  'girilmez',
    4:  'iki yonlu yol',           5:  'ileri mecburi',
    6:  'ileri ve saga mecburi yon', 7:  'ileri ve sola mecburi yon',
    8:  'ileriden saga mecburi yon', 9:  'ileriden sola mecburi yon',
    10: 'isikli isaret cihazi',    11: 'kirmizi isik',
    12: 'park',                    13: 'park yapilmaz',
    14: 'sag seridin sonu',        15: 'saga donulmez',
    16: 'saga mecburi',            17: 'sagdan gidin',
    18: 'sari isik',               19: 'sol seridin sonu',
    20: 'sola donulmez',           21: 'sola mecburi',
    22: 'soldan gidin',            23: 'tunel',
    24: 'yaya gecidi',             25: 'yesil isik',
    26: 'trafik konisi',           27: 'bariyer',
    28: 'yaya',
}

# Hata ayıklama: her sınıfa sabit renk
np.random.seed(42)
COLORS = {i: tuple(int(c) for c in np.random.randint(80, 255, 3)) for i in CLASS_NAMES}


class CameraPerceptionNode(Node):
    """Kamera görüntüsünü ONNX modeliyle işleyen ROS 2 düğümü."""

    def __init__(self):
        super().__init__('camera_perception_node')

        # ── Parametreler ──
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', CONF_THRESH)
        self.declare_parameter('process_every_n', PROCESS_EVERY_N)
        self.declare_parameter('show_window', SHOW_WINDOW)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._conf_thresh = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self._process_every_n = self.get_parameter('process_every_n').get_parameter_value().integer_value
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
        self._last_dets: list[float] = []    # [x1,y1,x2,y2,score,cls_id]*N
        self._last_vis = None                 # son görselleştirme karesi

        # ── ROS iletişimi ──
        self._sub = self.create_subscription(
            Image, '/camera/image', self._image_callback, 10)

        self._pub_dets = self.create_publisher(
            Float32MultiArray, '/perception/detections', 10)

        self._pub_vis = self.create_publisher(
            Image, '/perception/sign_image', 10)

        self.get_logger().info(
            f'CameraPerceptionNode başlatıldı. '
            f'Her {self._process_every_n} karede bir inference. '
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
        if self._frame_count % self._process_every_n == 0:
            self._run_inference(frame)

        # ── Son bilinen tespitleri yayınla (her frame'de) ──
        self._publish_detections()

        # ── Görselleştirme ──
        if self._last_vis is not None:
            try:
                vis_msg = self._bridge.cv2_to_imgmsg(self._last_vis, 'bgr8')
                vis_msg.header.stamp = msg.header.stamp
                self._pub_vis.publish(vis_msg)
            except Exception as e:
                self.get_logger().error(f'Görüntü yayınlama hatası: {e}')

        # ── Debug penceresi ──
        if self._show_window and self._last_vis is not None:
            cv2.imshow(WINDOW_NAME, self._last_vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.get_logger().info('ESC — pencere kapatılıyor')
                cv2.destroyWindow(WINDOW_NAME)
                self._show_window = False

    def _run_inference(self, frame: np.ndarray) -> None:
        """ONNX modeli ile bir kare işle."""
        h, w = frame.shape[:2]

        # Ön işleme: resize + normalize + BGR→RGB + CHW
        img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.expand_dims(np.transpose(img, (2, 0, 1)), 0)

        # Inference
        try:
            output = self._session.run(None, {self._input_name: inp})
        except Exception as e:
            self.get_logger().error(f'Inference hatası: {e}')
            return

        # Son işleme: (8400, 4+29) → bbox listesi
        bboxes, scores, class_ids = self._postprocess(output[0], w, h)

        # ── Görselleştirme karesi oluştur ──
        vis = frame.copy()
        dets: list[float] = []

        for bbox, score, cls_id in zip(bboxes, scores, class_ids):
            x1, y1, x2, y2 = bbox
            cls_id_int = int(cls_id)
            color = COLORS.get(cls_id_int, (0, 255, 0))

            # Bbox çiz
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

            # Etiket
            label = f"{CLASS_NAMES.get(cls_id_int, str(cls_id_int))} {score:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(vis, (int(x1), int(y1) - th - 6),
                          (int(x1) + tw + 4, int(y1)), color, -1)
            cv2.putText(vis, label, (int(x1) + 2, int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

            # Tespit verisini topla
            dets.extend([float(x1), float(y1), float(x2), float(y2),
                         float(score), float(cls_id)])

        # FPS ve durum bilgisi
        fps_text = f"Frame: {self._frame_count} | Tespit: {len(bboxes)}"
        cv2.putText(vis, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2, cv2.LINE_AA)

        self._last_dets = dets
        self._last_vis = vis

        # Log
        if len(bboxes) > 0:
            names = [CLASS_NAMES.get(int(c), str(int(c))) for c in class_ids[:5]]
            scores_subset = scores[:5] if hasattr(scores, '__len__') else [scores]
            self.get_logger().info(
                f'{len(bboxes)} tespit: {", ".join(f"{n}({s:.2f})" for n, s in zip(names, scores_subset))}',
                throttle_duration_sec=1.0)

    def _publish_detections(self) -> None:
        """Son bilinen tespit verisini /perception/detections'ta yayınla."""
        msg = Float32MultiArray()
        msg.data = self._last_dets if self._last_dets else []
        self._pub_dets.publish(msg)

    @staticmethod
    def _postprocess(output: np.ndarray, orig_w: int, orig_h: int):
        """
        ONNX çıktısını (cx,cy,w,h + class scores) → bbox listesi olarak dönüştür.
        Altyapıdır — NMS ve koordinat dönüşümü.

        İki olası ONNX formatını destekler:
        - (1, 4+29, N): transp. gerekir → pred = output.squeeze(0).T  (N, 33)
        - (1, N, 4+29): transp. gerekmez → pred = output.squeeze(0)   (N, 33)
        """
        # Batch boyutunu kaldır
        if output.ndim == 3 and output.shape[0] == 1:
            output = output[0]  # (D0, D1) → şekline bağlı

        # Hangi eksenin 33 (4 bbox + 29 sınıf) olduğunu bul
        # Tipik YOLO: (8400, 33) → axis=1 = 33, ya da (33, 8400) → axis=0 = 33
        num_classes = 29
        if output.shape[-1] == 4 + num_classes:
            pred = output  # zaten (N, 33)
        elif output.shape[0] == 4 + num_classes:
            pred = output.T  # (33, N) → (N, 33)
        else:
            # Beklenmeyen format, en iyi tahmin
            pred = output.T if output.shape[0] < output.shape[1] else output

        boxes_cx = pred[:, :4]
        scores = np.max(pred[:, 4:], axis=1)
        class_ids = np.argmax(pred[:, 4:], axis=1)

        mask = scores > CONF_THRESH
        boxes_cx, scores, class_ids = boxes_cx[mask], scores[mask], class_ids[mask]
        if len(scores) == 0:
            return [], [], []

        sx, sy = orig_w / IMG_SIZE, orig_h / IMG_SIZE
        x1 = ((boxes_cx[:, 0] - boxes_cx[:, 2] / 2) * sx).astype(int)
        y1 = ((boxes_cx[:, 1] - boxes_cx[:, 3] / 2) * sy).astype(int)
        x2 = ((boxes_cx[:, 0] + boxes_cx[:, 2] / 2) * sx).astype(int)
        y2 = ((boxes_cx[:, 1] + boxes_cx[:, 3] / 2) * sy).astype(int)
        bboxes = np.stack([x1, y1, x2, y2], axis=1)

        # NMS
        idx = cv2.dnn.NMSBoxes(bboxes.tolist(), scores.tolist(), CONF_THRESH, IOU_THRESH)
        if len(idx) == 0:
            return [], [], []
        idx = idx.flatten()

        return bboxes[idx], scores[idx], class_ids[idx]

    def close(self):
        if self._show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
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
