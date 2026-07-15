#!/usr/bin/env python3
"""
Şerit takip DENEME node'u #2 — ULTRALYTICS YOLO maskeleme tekniği (lane_yolo_node).

Maskeleme mantığı ultralytics YOLO ile:
  - ultralytics.YOLO(model, task='segment') ile segmentasyon (letterbox/NMS/maske
    decode'unu ultralytics kendi içinde yapar — elle onnxruntime postprocess YOK)
  - results.masks.xy POLİGON noktaları kullanılır — bunlar ZATEN orijinal görüntü
    koordinatlarında gelir, yani elle cv2.resize YOK → letterbox/padding kayması YOK
  - renkli dolu maske (fillPoly) + kontur (polylines)

Bu node mevcut lane_test_node.py'ye DOKUNMAZ; tamamen ayrı bir alternatiftir.
Aynı /perception/center_pts topic'ini yayınlar, böylece autonomous_control_node
bunu da sürebilir. Orta-nokta çıkarımı lane_test_node ile aynı basit mantığı
kullanır (ROI + satır-medyanı + EMA) — yani SADECE maskeleme farklıdır.

Çalıştırma:
  ros2 launch araba lane_yolo.launch.py
"""

import os
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from ultralytics import YOLO

# ─── AYARLAR ───────────────────────────────────────────
CONF_THRESH = 0.30          # test (1).py ile aynı (conf=0.3)

# ROI — lane_test_node.py ile birebir aynı (karşılaştırma adil olsun diye)
ROI_TOP   = 0.6
ROI_BOT   = 0.88
ROI_TOP_L = 0.05
ROI_TOP_R = 0.95
ROI_BOT_L = 0.05
ROI_BOT_R = 0.95

SMOOTH_ALPHA      = 0.35    # kareler arası EMA (aynı y satırı)
BOTTOM_BAND_RATIO = 0.15    # uzak kopuk blob filtresi bandı

# Renkler sınıf ADINA göre (BGR) — sınıf indeksi modele göre değişebildiği için
# (lane_seg: 0=road, 2206_2: 0=alternative) indeks değil İSİM kullanıyoruz.
# lane_test ile aynı görsel: yol = yeşil.
COLOR_BY_NAME = {
    'road':        (0, 255, 0),     # gidilebilir yol = yeşil (lane_test ile aynı)
    'alternative': (255, 100, 0),   # alternatif = turuncu
}
DEFAULT_COLOR = (255, 255, 255)


# ─── ROI / ORTA NOKTA YARDIMCILARI (lane_test_node ile paralel) ──────────

def get_manual_roi_pts(h, w):
    return np.array([[(int(w * ROI_BOT_L), int(h * ROI_BOT)),
                      (int(w * ROI_BOT_R), int(h * ROI_BOT)),
                      (int(w * ROI_TOP_R), int(h * ROI_TOP)),
                      (int(w * ROI_TOP_L), int(h * ROI_TOP))]], dtype=np.int32)


def keep_bottom_component(m, band_ratio=BOTTOM_BAND_RATIO):
    """Maskede aracın üzerinde olduğu (en alttaki) bölgeye bağlı bileşeni tutar;
    ona bağlı OLMAYAN uzaktaki kopuk blob'ları atar. Erozyon yok → ince yolu kırpmaz."""
    ys = np.where(m > 0)[0]
    if len(ys) == 0:
        return m
    num, labels = cv2.connectedComponents(m, connectivity=8)
    if num <= 2:
        return m
    h = m.shape[0]
    y_bottom = int(ys.max())
    band_top = max(0, int(y_bottom - h * band_ratio))
    band_labels = labels[band_top:y_bottom + 1, :]
    band_labels = band_labels[band_labels > 0]
    if len(band_labels) == 0:
        return m
    keep = np.bincount(band_labels).argmax()
    return (labels == keep).astype(np.uint8)


def extract_center_points(frame, road_mask, prev_state=None):
    """road_mask: TÜM yol poligonlarının BİRLEŞİMİ (union), uint8 (0/1), frame boyutunda.
    Boru hattı lane_test_node ile aynı: CLOSE+OPEN morfoloji → ROI → araç altına bağlı
    bileşen (keep_bottom_component) → satır-medyanı + EMA. Tek tek maske seçmek yerine
    union kullanılır; böylece YOLO yolu birden çok parçaya bölse de tamamı değerlendirilir."""
    h, w = frame.shape[:2]
    roi_pts  = get_manual_roi_pts(h, w)
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    kernel = np.ones((7, 7), np.uint8)
    prev_cx = prev_state if isinstance(prev_state, dict) else {}

    # Morfoloji: CLOSE (poligon kenarındaki boşlukları/delikleri doldur) + OPEN (kopuk
    # gürültüyü temizle). Erozyon ince yolu kırpabildiği için 7x7 ile sınırlı (lane_test ile aynı).
    m = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    m = cv2.bitwise_and(m, roi_mask)

    new_cx, pts = {}, []
    if int(np.count_nonzero(m)) > 0:
        m = keep_bottom_component(m)   # aracın altına bağlı yolu tut, uzak kopuk blobları at
        for y in range(h - 1, int(h * ROI_TOP), -15):
            xs = np.where(m[y, :] > 0)[0]
            if len(xs) == 0:
                continue
            cx = float(np.median(xs))
            if y in prev_cx:
                cx = SMOOTH_ALPHA * cx + (1.0 - SMOOTH_ALPHA) * prev_cx[y]
            new_cx[y] = cx
            pts.append((int(round(cx)), y))

    all_pts = [pts] if len(pts) > 0 else []
    return all_pts, new_cx


# ─── ROS 2 DÜĞÜMÜ ──────────────────────────────────────

class LaneYoloNode(Node):
    def __init__(self):
        super().__init__('lane_yolo_node')
        self.bridge = CvBridge()

        default_model = os.path.join(
            get_package_share_directory('araba'), 'models', 'onnx', '2206_2model.onnx'
        )

        self.camera_topic = self.declare_parameter('camera_topic', '/camera/image').value
        model_path        = self.declare_parameter('model_path', default_model).value
        self.show_window  = self.declare_parameter('show_window', True).value
        self.conf         = float(self.declare_parameter('conf', CONF_THRESH).value)
        # device: 'cpu' = CPU (varsayılan). CPU YOLO ~18 FPS yapıyor — sim kamerasının
        # (~8.5 FPS) üstünde, yani darboğaz değil. GPU için cu12-uyumlu torch gerekir
        # (cu13 torch onnxruntime-gpu'nun cu12 cuDNN'iyle çakışıyor).
        self.device       = self.declare_parameter('device', 'cpu').value

        self.sub_image = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.pub_pts = self.create_publisher(
            Float32MultiArray, '/perception/center_pts', 10
        )
        self.pub_debug_img = self.create_publisher(
            Image, '/perception/debug_image', 10
        )

        self._smooth_state = {}

        # ── ultralytics YOLO (ONNX segment) yükle ──
        try:
            self.model = YOLO(model_path, task='segment')
            self.model.overrides['device'] = self.device
            # Yol (gidilebilir) sınıf indeksini modelin isimlerinden DİNAMİK bul.
            # lane_seg: {0:'road',...}  2206_2: {0:'alternative',1:'road'} → indeks ters!
            self.names = {int(k): str(v) for k, v in self.model.names.items()}
            self.road_cls = next(
                (i for i, n in self.names.items() if n.lower() in ('road', 'drivable', 'yol')),
                0,
            )
            self.get_logger().info(
                f'✅ YOLO modeli yüklendi ({model_path}). task={self.model.task} '
                f'device={self.device} | sınıflar={self.names} | yol sınıfı={self.road_cls}'
            )
        except Exception as e:
            self.get_logger().error(f'YOLO modeli yüklenemedi: {e}')
            raise

        self._fps = 0.0
        self._last_cb = None
        self._frame_count = 0

        if self.show_window:
            cv2.namedWindow('Serit Tespit (lane_yolo)', cv2.WINDOW_NORMAL)

    def _draw_hud(self, img, infer_ms, post_ms):
        lines = [
            f'FPS: {self._fps:.1f}  [YOLO/{self.device}]',
            f'infer: {infer_ms:4.0f} ms',
            f'post : {post_ms:4.0f} ms',
        ]
        y = 26
        for i, ln in enumerate(lines):
            scale = 0.8 if i == 0 else 0.55
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 1, cv2.LINE_AA)
            y += 26 if i == 0 else 20

    def image_callback(self, msg):
        t_start = time.time()
        if self._last_cb is not None:
            dt = t_start - self._last_cb
            if dt > 0:
                inst = 1.0 / dt
                self._fps = inst if self._fps == 0.0 else 0.9 * self._fps + 0.1 * inst
        self._last_cb = t_start

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Görüntü dönüşüm hatası: {e}')
            return

        h, w = frame.shape[:2]

        # ── 1) YOLO çıkarımı (test (1).py ile aynı çağrı) ──
        t0 = time.time()
        try:
            results = self.model(frame, conf=self.conf, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f'YOLO çıkarım hatası: {e}')
            return
        infer_ms = (time.time() - t0) * 1000.0

        # ── 2) Maske çözme (test_onnx.py: masks.xy poligonları, orijinal koord) + orta nokta ──
        t1 = time.time()
        overlay = frame.copy()
        road_union = np.zeros((h, w), dtype=np.uint8)   # tüm yol poligonlarının birleşimi
        draw_items = []          # (poly_pts, color, cls, conf, box) çizim için

        if results.masks is not None:
            # masks.xy: poligon noktaları ZATEN orijinal görüntü koordinatlarında gelir.
            polygons = results.masks.xy
            classes  = results.boxes.cls.cpu().numpy().astype(int)
            confs    = results.boxes.conf.cpu().numpy()
            boxes    = results.boxes.xyxy.cpu().numpy().astype(int)

            for poly, cls, conf_val, box in zip(polygons, classes, confs, boxes):
                cls = int(cls)
                if poly.size == 0:
                    continue
                cls_name = self.names.get(cls, str(cls))
                color = COLOR_BY_NAME.get(cls_name.lower(), DEFAULT_COLOR)
                pts = poly.astype(np.int32).reshape(-1, 1, 2)
                draw_items.append((pts, color, cls_name, conf_val, box))
                if cls == self.road_cls:
                    # yol poligonunu ortak union maskesine rasterize et (orta nokta için).
                    # Ayrı maske listesi yerine union: YOLO yolu parçalara bölse de tamamı birleşir.
                    cv2.fillPoly(road_union, [pts], 1)

        all_smooth_pts, self._smooth_state = extract_center_points(
            frame, road_union, self._smooth_state
        )
        post_ms = (time.time() - t1) * 1000.0

        # ── Orta noktaları publish et (otonom sürüş için) ──
        msg_pts = Float32MultiArray()
        if len(all_smooth_pts) > 0 and len(all_smooth_pts[0]) > 0:
            msg_pts.data = [float(c) for pt in all_smooth_pts[0] for c in pt]
        else:
            msg_pts.data = []
        self.pub_pts.publish(msg_pts)

        # ── 3) Debug görseli ──
        need_debug = self.show_window or self.pub_debug_img.get_subscription_count() > 0
        if need_debug:
            for pts, color, cls_name, conf_val, box in draw_items:
                cv2.fillPoly(overlay, [pts], color)               # dolu maske
                cv2.polylines(frame, [pts], True, color, 2)       # kontur

            result = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0)

            for pts, color, cls_name, conf_val, box in draw_items:
                label = f"{cls_name} {conf_val:.2f}"
                cv2.putText(result, label, (box[0], max(box[1] - 5, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # orta-nokta çizgisi + ROI
            for pts in all_smooth_pts:
                for i in range(len(pts) - 1):
                    cv2.line(result, pts[i], pts[i + 1], (0, 0, 255), 3)
                for pt in pts:
                    cv2.circle(result, pt, 4, (0, 255, 255), -1)
            cv2.polylines(result, get_manual_roi_pts(h, w), True, (0, 255, 255), 2)

            self._draw_hud(result, infer_ms, post_ms)

            if self.show_window:
                cv2.imshow('Serit Tespit (lane_yolo)', result)
                cv2.waitKey(1)
            if self.pub_debug_img.get_subscription_count() > 0:
                try:
                    self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))
                except Exception as e:
                    self.get_logger().error(f'Debug görüntü yayınlama hatası: {e}')

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            # ── TANI: yol neden tam yakalanmıyor? Sınıf/conf/alan dökümü + yol ROI kaplaması ──
            # Her sınıf için: kaç tespit, toplam poligon alanı (px), conf aralığı.
            agg = defaultdict(lambda: [0, 0.0, 0.0, 1.0])  # ad -> [adet, alan, conf_max, conf_min]
            for pts, _color, cls_name, conf_val, _box in draw_items:
                a = float(cv2.contourArea(pts))
                e = agg[cls_name]
                e[0] += 1
                e[1] += a
                e[2] = max(e[2], float(conf_val))
                e[3] = min(e[3], float(conf_val))
            if agg:
                det_str = ' | '.join(
                    f'{n}×{e[0]} (alan~{int(e[1])}px, conf {e[3]:.2f}-{e[2]:.2f})'
                    for n, e in agg.items()
                )
            else:
                det_str = 'HİÇ TESPİT YOK'

            # Yolun ROI içini ne kadar doldurduğu (orta nokta çıkarımının gördüğü alan)
            roi_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(roi_mask, get_manual_roi_pts(h, w), 1)
            roi_area = int(np.count_nonzero(roi_mask))
            road_in_roi = int(np.count_nonzero(cv2.bitwise_and(road_union, roi_mask)))
            cover = 100.0 * road_in_roi / max(roi_area, 1)

            self.get_logger().info(
                f'[YOLO/{self.device}] FPS={self._fps:.1f} | infer={infer_ms:.1f}ms post={post_ms:.1f}ms\n'
                f'   ├─ tespitler: {det_str}\n'
                f'   ├─ yol sınıfı=\'{self.names.get(self.road_cls, "?")}\' (idx {self.road_cls}) '
                f'| conf_eşik={self.conf:.2f}\n'
                f'   └─ yol ROI kaplaması: %{cover:.1f}  ({road_in_roi}/{roi_area} px)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
