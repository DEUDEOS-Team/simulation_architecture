#!/usr/bin/env python3
"""
Şerit takip DENEME node'u (lane_test_node).

Kaynak: kullanıcının kök dizine attığı ros2deneme.py. Görüntü işleme mantığı
(preprocess / postprocess / curve fitting / çizim) BİREBİR korunmuştur; yalnızca
ROS 2 entegrasyonu için 3 değişiklik yapıldı:
  1) Kamera topic'i parametrik, varsayılan = /camera/image  (bridge'deki gerçek topic)
  2) Model yolu parametrik; varsayılan paket içinden çözülür (göreceli yol patlamasın)
  3) İşlenmiş çıktı otomatik bir OpenCV penceresinde gösterilir (show_window=True)

Hızlı iterasyon: model/kod düzenle -> launch'ı kapat-aç (symlink-install sayesinde
yeniden derleme gerekmez).
"""

import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import onnxruntime as ort
import warnings

# ─── AYARLAR ───────────────────────────────────────────
IMG_W, IMG_H = 640, 640
CONF_THRESH  = 0.40

# İstersen ROI'yi ekranın tam altına dayamak için ROI_BOT = 1.0 yapabilirsin
ROI_TOP   = 0.6
ROI_BOT   = 0.88
ROI_TOP_L = 0.05
ROI_TOP_R = 0.95
ROI_BOT_L = 0.05
ROI_BOT_R = 0.95

USE_NCHW = True

# ─── CURVE FITTING VE GÜVENLİK AYARLARI ────────────────
POLY_DEGREE    = 2    # Sağa/sola uçmayı engelleyen doğrusal fit
FIT_STEP       = 15
MIN_POINTS_FIT = 5
TEMPORAL_ALPHA = 0.35
MAX_JUMP       = 15   # Satırlar arası maksimum piksel kayma izni

# ─── MASKE SEÇİMİ / FİLTRELEME (uzaktaki yanlış yolları eler) ───
# ROI alanına oran: bir maskenin ROI içindeki piksel sayısı bunun altındaysa
# "uzak/küçük yama" sayılıp tamamen yok sayılır.
MIN_MASK_AREA_RATIO = 0.02
# Maske seçimi: alt-orta noktaya (araç altı) yakınlık önceliklidir; alan ikincil ağırlık.
AREA_SELECT_WEIGHT  = 0.5
# Kayan-pencere yarı genişliği (ROI/görüntü genişliğine oran): her satırda önceki
# merkez etrafında bu kadar piksel aranır. Büyük = dönüşü daha rahat izler ama yan
# parçalara açılır; küçük = daha sıkı takip ama keskin dönüşte yolu kaybedebilir.
SLIDE_WIN_RATIO     = 0.18

# ─── HAT STABİLİTESİ ───────────────────────────────────
# Kareler arası zamansal yumuşatma (EMA): cx = α·yeni + (1-α)·önceki (aynı y satırı için).
# Küçük α = daha stabil/yavaş tepki, büyük α = daha çevik/titrek. (0..1)
SMOOTH_ALPHA = 0.35

# Uzak blob filtresi: maskenin en alt bu oranlık bandı "araç altı" sayılır; o banttaki
# bileşene bağlı olmayan uzaktaki kopuk parçalar atılır.
BOTTOM_BAND_RATIO = 0.15

# ─── RENK PALETİ ───────────────────────────────────────
CLASS_COLORS = [
    (0, 255, 0),    # Class 0: Yeşil
    (255, 0, 0),    # Class 1: Mavi
    (0, 0, 255),    # Class 2: Kırmızı
    (0, 255, 255),  # Class 3: Sarı
]

# ─── GÖRÜNTÜ İŞLEME FONKSİYONLARI ──────────────────────

def preprocess(frame):
    img = cv2.resize(frame, (IMG_W, IMG_H), interpolation=cv2.INTER_LINEAR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    if USE_NCHW:
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
    else:
        img = np.expand_dims(img, axis=0)
    return img

def postprocess(outputs, orig_w, orig_h):
    pred   = outputs[0][0].T
    protos = outputs[1][0]

    num_masks        = protos.shape[0]
    proto_h, proto_w = protos.shape[1], protos.shape[2]
    num_classes      = pred.shape[1] - 4 - num_masks

    boxes       = pred[:, :4]
    class_probs = pred[:, 4:4 + num_classes]
    mask_coeffs = pred[:, -num_masks:]

    scores    = np.max(class_probs, axis=1)
    class_ids = np.argmax(class_probs, axis=1)
    valid     = scores > CONF_THRESH

    boxes, scores, class_ids, mask_coeffs = (
        boxes[valid], scores[valid], class_ids[valid], mask_coeffs[valid]
    )
    if len(boxes) == 0:
        return [], [], [], []

    x1 = (boxes[:, 0] - boxes[:, 2] / 2) * orig_w / IMG_W
    y1 = (boxes[:, 1] - boxes[:, 3] / 2) * orig_h / IMG_H
    x2 = (boxes[:, 0] + boxes[:, 2] / 2) * orig_w / IMG_W
    y2 = (boxes[:, 1] + boxes[:, 3] / 2) * orig_h / IMG_H
    bboxes = np.stack([x1, y1, x2, y2], axis=1).astype(int)

    idx = cv2.dnn.NMSBoxes(bboxes.tolist(), scores.tolist(), CONF_THRESH, 0.45)
    if len(idx) == 0:
        return [], [], [], []
    idx = idx.flatten()
    bboxes, scores, mask_coeffs, class_ids = (
        bboxes[idx], scores[idx], mask_coeffs[idx], class_ids[idx]
    )

    proto_flat = protos.reshape(num_masks, -1)
    masks = []
    for coeff in mask_coeffs:
        m = (coeff @ proto_flat).reshape(proto_h, proto_w)
        m = 1.0 / (1.0 + np.exp(-m))
        m = cv2.resize(m, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        masks.append(m > 0.5)

    return bboxes, scores, masks, class_ids

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
        return m  # tek bileşen var, ayıracak kopuk parça yok

    h = m.shape[0]
    y_bottom = int(ys.max())
    band_top = max(0, int(y_bottom - h * band_ratio))
    band_labels = labels[band_top:y_bottom + 1, :]
    band_labels = band_labels[band_labels > 0]
    if len(band_labels) == 0:
        return m
    keep = np.bincount(band_labels).argmax()  # alt bantta en baskın bileşen = aracın yolu
    return (labels == keep).astype(np.uint8)

def fit_lane_curve(raw_pts, h, w, prev_coeffs=None):
    """Şerit orta çizgisini x = f(y) olarak fit eder (her mesafe/satır için yanal merkez).
    Noktalar araca en yakın (en büyük y) baştan, uzağa doğru sıralıdır."""
    if len(raw_pts) < MIN_POINTS_FIT:
        return raw_pts, prev_coeffs

    ys = np.array([p[1] for p in raw_pts], dtype=np.float32)
    xs = np.array([p[0] for p in raw_pts], dtype=np.float32)

    try:
        with warnings.catch_warnings(record=True) as w_list:
            warnings.simplefilter("always")
            coeffs = np.polyfit(ys, xs, POLY_DEGREE)
            if len(w_list) > 0:
                return raw_pts, prev_coeffs
    except Exception:
        return raw_pts, prev_coeffs

    if prev_coeffs is not None and len(prev_coeffs) == len(coeffs):
        coeffs = TEMPORAL_ALPHA * coeffs + (1.0 - TEMPORAL_ALPHA) * prev_coeffs

    y_start = int(np.max(ys))
    y_end   = int(np.min(ys))

    if y_start <= y_end:
        return raw_pts, coeffs

    y_vals  = np.arange(y_start, y_end, -FIT_STEP)
    poly_fn = np.poly1d(coeffs)
    smooth_pts = [
        (int(np.clip(poly_fn(y), 0, w - 1)), int(y))
        for y in y_vals
    ]
    return smooth_pts, coeffs

def select_best_masks(masks, class_ids, bboxes, scores, h, w):
    """Her sınıftan ROI içinde en uygun TEK maskeyi tutar.

    Eleme: ROI alanının altında kalan küçük/uzak yamalar (MIN_MASK_AREA_RATIO).
    Seçim: alt-orta (araç altı) yakınlığı önce, büyük alan ödül (FIX 1 skoru).
    Sonuç: sınıf başına en fazla bir maske → ekranda tek 'gidilebilir' (yeşil) +
    tek 'alternatif'; yol kenarlarındaki tekrar maskeler elenir.
    """
    if len(masks) == 0:
        return [], np.array([], dtype=int), np.empty((0, 4), dtype=int), np.array([])

    roi_pts  = get_manual_roi_pts(h, w)
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    roi_area = max(int(np.count_nonzero(roi_mask)), 1)
    min_area = roi_area * MIN_MASK_AREA_RATIO
    center_x = w * 0.5
    kernel = np.ones((7, 7), np.uint8)

    best    = {}  # cls_id -> (score, idx)
    cleaned = {}  # idx -> temizlenmiş tek-bileşen maske (uint8 0/1, ROI içi)
    for i, (mask, cls) in enumerate(zip(masks, class_ids)):
        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        # Yalnız CLOSE (boşluk doldurur, aşındırmaz). OPEN/erozyon dönüşteki ince yolu
        # parçalayıp eleyebildiği için KULLANILMIYOR.
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)

        # Tek tespit kopuk parçalar içerebilir → SADECE en büyük bağlı bileşeni tut.
        # (Önceden bu yalnızca orta-nokta için yapılıyordu; artık çizilen maskeye de uygulanıyor.)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            m_clean = np.zeros_like(m)
            cv2.drawContours(m_clean, [largest], -1, 1, thickness=cv2.FILLED)
            m = m_clean

        m = cv2.bitwise_and(m, roi_mask)
        area = int(np.count_nonzero(m))
        if area < min_area:
            continue  # FIX 2: çok küçük = uzak/yanlış, ele

        ys_i, xs_i = np.where(m > 0)
        y_max = int(ys_i.max())
        band = max(1, int((y_max - int(ys_i.min())) * 0.15))
        sel = ys_i >= (y_max - band)
        bottom_cx = float(np.median(xs_i[sel])) if np.any(sel) else float(np.median(xs_i))
        score = abs(bottom_cx - center_x) / w - AREA_SELECT_WEIGHT * (area / roi_area)

        cleaned[i] = m
        cls = int(cls)
        if cls not in best or score < best[cls][0]:
            best[cls] = (score, i)

    keep = sorted(v[1] for v in best.values())
    if not keep:
        return [], np.array([], dtype=int), np.empty((0, 4), dtype=int), np.array([])

    # Temizlenmiş (tek-bileşen) maskeleri döndür → hem çizim hem orta-nokta aynı temiz maskeyi görür
    return (
        [cleaned[i] for i in keep],
        np.array([class_ids[i] for i in keep], dtype=int),
        np.array([bboxes[i] for i in keep], dtype=int),
        np.array([scores[i] for i in keep]),
    )

def extract_center_points(frame, masks, class_ids, prev_state=None):
    """Webots 'düz mod' mantığı + 3 stabilite eklentisi:
      (3) Birden çok cls-0 maske varsa ROI içinde EN BÜYÜK alanlı olanı seç (sıçramayı keser).
      (2) Her satırda x'lerin MEDIAN'ı (mean değil; aykırı piksellerden etkilenmez).
      (1) Kareler arası EMA: aynı y için cx = α·yeni + (1-α)·önceki (SMOOTH_ALPHA).
    Fit / MAX_JUMP / kayan-pencere YOK. İkinci dönüş, bir sonraki kareye taşınan
    {y: cx} yumuşatma durumudur."""
    h, w = frame.shape[:2]

    roi_pts  = get_manual_roi_pts(h, w)
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    kernel = np.ones((7, 7), np.uint8)

    prev_cx = prev_state if isinstance(prev_state, dict) else {}

    # (3) cls-0 maskeler içinden ROI'de en büyük alanlıyı seç
    best_m, best_area = None, 0
    for mask, cls_id in zip(masks, class_ids):
        if cls_id != 0:
            continue
        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
        m = cv2.bitwise_and(m, roi_mask)
        area = int(np.count_nonzero(m))
        if area > best_area:
            best_area, best_m = area, m

    new_cx = {}
    pts = []
    if best_m is not None:
        best_m = keep_bottom_component(best_m)  # uzaktaki kopuk blob'ları at
        for y in range(h - 1, int(h * ROI_TOP), -15):
            xs = np.where(best_m[y, :] > 0)[0]
            if len(xs) == 0:
                continue
            cx = float(np.median(xs))                       # (2) median
            if y in prev_cx:                                # (1) zamansal EMA
                cx = SMOOTH_ALPHA * cx + (1.0 - SMOOTH_ALPHA) * prev_cx[y]
            new_cx[y] = cx
            pts.append((int(round(cx)), y))

    all_pts = [pts] if len(pts) > 0 else []
    # best_m: seçilmiş ego-şerit maskesi (uint8 0/1, tam kare boyutunda) — engel
    # yol-içi doğrulaması için /perception/lane_mask olarak yayınlanır
    return all_pts, new_cx, best_m

def draw_results(image, bboxes, scores, masks, class_ids, all_smooth_pts):
    h, w = image.shape[:2]
    overlay = image.copy()

    roi_pts     = get_manual_roi_pts(h, w)
    roi_mask_2d = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask_2d, roi_pts, 255)

    kernel = np.ones((7, 7), np.uint8)
    for mask, cls_id, bbox, score in zip(masks, class_ids, bboxes, scores):
        color = CLASS_COLORS[cls_id] if cls_id < len(CLASS_COLORS) else (255, 255, 255)

        # Webots ile aynı: CLOSE+OPEN morfoloji + ROI (gösterilen maske = kullanılan maske)
        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
        m = cv2.bitwise_and(m, roi_mask_2d)
        if cls_id == 0:
            m = keep_bottom_component(m)  # yol maskesindeki uzak kopuk blob'ları ekranda da gösterme

        mask_bool = m.astype(bool)
        overlay[mask_bool] = (
            overlay[mask_bool] * 0.4 + np.array(color) * 0.6
        ).astype(np.uint8)

        x1, y1, x2, y2 = bbox
        cv2.putText(overlay, f'cls:{cls_id} {score:.2f}', (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    result = cv2.addWeighted(image, 0.55, overlay, 0.45, 0)

    for pts in all_smooth_pts:
        for i in range(len(pts) - 1):
            cv2.line(result, pts[i], pts[i + 1], (0, 0, 255), 3)
        for pt in pts:
            cv2.circle(result, pt, 4, (0, 255, 255), -1)

    cv2.polylines(result, roi_pts, True, (0, 255, 255), 2)

    return result

# ─── ROS 2 DÜĞÜMÜ ──────────────────────────────────────

class LaneTestNode(Node):
    def __init__(self):
        super().__init__('lane_test_node')
        self.bridge = CvBridge()

        # Varsayılan model yolu: paket içinden çöz (göreceli yol ROS 2'de patlar)
        default_model = os.path.join(
            get_package_share_directory('araba'), 'models', 'onnx', 'lane_seg.onnx'
        )

        # Parametreler (launch'tan override edilebilir)
        self.camera_topic = self.declare_parameter('camera_topic', '/camera/image').value
        model_path        = self.declare_parameter('model_path', default_model).value
        self.show_window  = self.declare_parameter('show_window', True).value
        # İşleme genişliği: >0 ise maske işlemeyi bu genişlikte yap (post/draw hızlanır).
        # NOT: darboğaz node değil Gazebo kamera hızı olduğu için varsayılan 0 (kapalı) —
        # görüntü kalitesini bozmamak için. İstersen 640 yapıp deneyebilirsin.
        self.proc_width   = int(self.declare_parameter('proc_width', 0).value)

        # Abonelik
        self.sub_image = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )

        # Yayıncılar
        self.pub_pts = self.create_publisher(
            Float32MultiArray, '/perception/center_pts', 10
        )
        self.pub_debug_img = self.create_publisher(
            Image, '/perception/debug_image', 10
        )
        # Ego-şerit maskesi (mono8, 0/255) — lidar_obstacle_node'un kamera kapısı
        # bunu kullanır: engelin zemin noktası maskede değilse "şerit dışı" oyu verir
        self.pub_lane_mask = self.create_publisher(
            Image, '/perception/lane_mask', 10
        )

        self._smooth_state = {}   # {y: cx} kareler arası hat yumuşatma durumu

        # ONNX Başlatma — GPU (CUDA) öncelikli; yoksa otomatik CPU'ya düşer
        try:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.session      = ort.InferenceSession(model_path, providers=providers)
            self.input_name   = self.session.get_inputs()[0].name
            self.output_names = [out.name for out in self.session.get_outputs()]
            self.active_provider = self.session.get_providers()[0]
            self.on_gpu = self.active_provider == 'CUDAExecutionProvider'
            self.get_logger().info(
                f'✅ ONNX Model yüklendi ({model_path}). '
                f'Aktif provider: {self.active_provider} '
                f'{"(GPU 🚀)" if self.on_gpu else "(CPU — GPU bulunamadi)"}'
            )
            if not self.on_gpu:
                self.get_logger().warn(
                    'CUDA provider aktif degil; cikarim CPU da yavas olabilir. '
                    'onnxruntime-gpu ve CUDA/cuDNN kurulu mu?'
                )
        except Exception as e:
            self.get_logger().error(f'ONNX model yüklenemedi: {e}')
            raise

        # ── FPS / zamanlama ölçümü ──
        self._fps = 0.0
        self._last_cb = None
        self._frame_count = 0

        if self.show_window:
            cv2.namedWindow('Serit Tespit (lane_test)', cv2.WINDOW_NORMAL)

    def _draw_hud(self, img, infer_ms, post_ms, draw_ms):
        """Sol üst köşeye FPS + aşama süreleri + GPU/CPU rozeti yazar."""
        dev = 'GPU' if self.on_gpu else 'CPU'
        lines = [
            f'FPS: {self._fps:.1f}  [{dev}]',
            f'infer: {infer_ms:4.0f} ms',
            f'post : {post_ms:4.0f} ms',
            f'draw : {draw_ms:4.0f} ms',
        ]
        y = 26
        for i, ln in enumerate(lines):
            scale = 0.8 if i == 0 else 0.55
            # siyah kontur (okunaklılık) + üstüne renkli metin
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 1, cv2.LINE_AA)
            y += 26 if i == 0 else 20

    def image_callback(self, msg):
        # ── Gerçek FPS: ardışık kareler arası süreden EMA ──
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

        # ── İşleme çözünürlüğünü düşür (FPS için) ──
        # Kamera 1280x720 yayınlıyor ama tüm maske işleme (post/draw) bu boyutta
        # yapılıyordu; model zaten 640 gördüğü için 720p işlemek israf.
        if self.proc_width > 0 and frame.shape[1] > self.proc_width:
            scale = self.proc_width / frame.shape[1]
            frame = cv2.resize(
                frame, (self.proc_width, int(round(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )

        inp = preprocess(frame)

        # ── 1) Model çıkarımı ──
        t0 = time.time()
        try:
            ort_outs = self.session.run(self.output_names, {self.input_name: inp})
        except Exception as e:
            self.get_logger().error(f'ONNX çıkarım hatası: {e}')
            return
        infer_ms = (time.time() - t0) * 1000.0

        # ── 2) Postprocess + merkez nokta çıkarımı ──
        orig_h, orig_w = frame.shape[:2]
        t1 = time.time()
        bboxes, scores, masks, class_ids = postprocess(ort_outs, orig_w, orig_h)
        all_smooth_pts, self._smooth_state, lane_mask = extract_center_points(
            frame, masks, class_ids, self._smooth_state
        )
        post_ms = (time.time() - t1) * 1000.0

        # ── Ego-şerit maskesini yayınla (kamera yol-içi kapısı için) ──
        if lane_mask is not None:
            try:
                mask_msg = self.bridge.cv2_to_imgmsg(
                    (lane_mask * 255).astype(np.uint8), 'mono8')
                mask_msg.header = msg.header
                self.pub_lane_mask.publish(mask_msg)
            except Exception:
                pass

        # ── Noktaları publish et (Otonom sürüş için) ────────────────────
        msg_pts = Float32MultiArray()
        if len(all_smooth_pts) > 0 and len(all_smooth_pts[0]) > 0:
            msg_pts.data = [float(coord) for pt in all_smooth_pts[0] for coord in pt]
        else:
            msg_pts.data = []
        self.pub_pts.publish(msg_pts)

        # ── 3) Debug görseli: çiz + HUD + göster ──
        draw_ms = 0.0
        need_debug = self.show_window or self.pub_debug_img.get_subscription_count() > 0
        if need_debug:
            t2 = time.time()
            debug_frame = draw_results(frame, bboxes, scores, masks, class_ids, all_smooth_pts)
            draw_ms = (time.time() - t2) * 1000.0

            self._draw_hud(debug_frame, infer_ms, post_ms, draw_ms)

            if self.show_window:
                cv2.imshow('Serit Tespit (lane_test)', debug_frame)
                cv2.waitKey(1)

            if self.pub_debug_img.get_subscription_count() > 0:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
                    self.pub_debug_img.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Debug görüntü yayınlama hatası: {e}')

        # ── Periyodik konsol tanısı (her ~30 karede bir) ──
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'[{"GPU" if self.on_gpu else "CPU"}] FPS={self._fps:.1f} | '
                f'infer={infer_ms:.1f}ms  post={post_ms:.1f}ms  draw={draw_ms:.1f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneTestNode()
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
