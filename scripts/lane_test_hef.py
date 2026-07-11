#!/usr/bin/env python3
"""
HEF (Hailo-8) şerit takip — lane_test maskeleme + orta-nokta mantığının DONANIM sürümü.

Bu dosya, maskeleme.py'deki Hailo HEF çıkarım iskeletini (DFL decode + proto çarpımı)
KORUR; ancak maskeleme.py'nin ilkel/yanlış maskeleme + çizim kısmını bizim
`lane_test_node.py` içindeki KANITLANMIŞ pipeline ile DEĞİŞTİRİR:

  • ROI (manuel yamuk) maskesi
  • CLOSE + OPEN morfoloji (boşluk doldur, küçük gürültüyü ele — dönüşteki ince yolu
    parçalamamak için extract'te CLOSE+OPEN, select'te yalnız CLOSE)
  • keep_bottom_component  → uzaktaki kopuk blob'ları atar (araç-altı bileşene bağlı kal)
  • extract_center_points  → cls-Road en büyük maske, her satırda x'lerin MEDIAN'ı,
                             kareler arası EMA (SMOOTH_ALPHA) ile orta çizgi
  • draw_results           → maske overlay + orta çizgi + ROI poligonu

ÇALIŞTIRMA (Hailo-8 + RealSense'li araçta):
    python3 lane_test_hef.py

NOT: Bu dosya Gazebo/ROS değildir; fiziksel araçta (Raspberry Pi + Hailo-8 + RealSense)
çalışır. Bu yüzden `hailo_platform` ve `pyrealsense2` paketleri yalnızca o makinede
kuruludur — geliştirme makinesinde import hatası vermesi normaldir.

Sınıf eşlemesi (HEF modeli 2206_2model.hef): 0 = Road (yol), 1 = Alternative.
Yol sınıfı = ROAD_CLASS_ID (aşağıda). Orta-nokta yalnız bu sınıftan çıkarılır.
"""

import time
import traceback

import cv2
import numpy as np

# ── Donanıma özel paketler (yalnız araçta kurulu) ──
import pyrealsense2 as rs
from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams,
                            InputVStreamParams, OutputVStreamParams, FormatType,
                            InferVStreams)

# ============================================================
# HEF / ÇIKARIM AYARLARI  (maskeleme.py'den korunan iskelet)
# ============================================================
HEF_FILE_PATH = "model.hef"

INPUT_SIZE   = 640      # HEF giriş boyutu (640x640)
IOU_THRESH   = 0.45     # NMS IoU
NUM_CLASSES  = 2        # 0=Road, 1=Alternative

# ── TEŞHİS ────────────────────────────────────────────────────────────────────
# DEBUG=True iken decode her ~30 karede bir: çıkarım süresi, eşiği geçen hücre
# sayısı ve cls sigmoid istatistiği yazar. "Her yer yol" sorununun MODEL mi yoksa
# KOD/dequant mi olduğunu kesinleştirir.
DEBUG        = True
PRE_NMS_TOPK = 300      # NMS'e girmeden önce skora göre tutulacak en fazla kutu.
                        # Model her yeri tespit etse bile NMS'i (O(n²)) bunaltmaz.

# ============================================================
# MASKELEME / ORTA-NOKTA AYARLARI  (lane_test_node.py ile AYNI)
# ============================================================
CONF_THRESH  = 0.501    # ÖNEMLİ: Bu modelin cls logit'leri ~0 (sigmoid tabanı tam 0.500).
                        # Yol sinyali sadece minik pozitif çıkıntı (sigmoid ~0.52-0.64).
                        # Eşik 0.50 olsaydı TÜM hücreler geçerdi (taban=eşik). 0.501 düz
                        # arka planı (0.500) eler, yolu tutar. Arka plan sızarsa 0.505'e
                        # çıkar; yol KARARSIZ/kayboluyorsa 0.500'e düşür (top-K yine seçer).
ROAD_CLASS_ID = 0       # Yol sınıfı indeksi (HEF modelinde Road=0)

# ── FPS / kalabalık ekran kontrolü ───────────────────────────────────────────
# Model "her yerde maske" üretince maske üretimi + çizim N tespitle ölçeklenip FPS'i
# düşürür. Aşağıdakiler işlenecek/çizilecek tespit sayısını sınırlar:
ONLY_ROAD      = True   # Yalnız yol sınıfını işle/çiz (alternatifi at) — hız + netlik
MAX_DETECTIONS = 3      # Maske üretmeden ÖNCE skora göre tutulacak en iyi tespit sayısı.
                        # Kontrol için tek "en iyi yol" maskesi yeterli; 1-3 idealdir.
SHOW_LABELS    = False  # bbox güven etiketlerini çiz (ekranı kaplamasın diye kapalı)

# ROI (manuel yamuk) — ekran oranına göre
ROI_TOP   = 0.6
ROI_BOT   = 0.88
ROI_TOP_L = 0.05
ROI_TOP_R = 0.95
ROI_BOT_L = 0.05
ROI_BOT_R = 0.95

# Kareler arası zamansal yumuşatma (EMA): cx = α·yeni + (1-α)·önceki (aynı y satırı).
# Küçük α = daha stabil/yavaş; büyük α = daha çevik/titrek.
SMOOTH_ALPHA = 0.35

# Uzak blob filtresi: maskenin en alt bu oranlık bandı "araç altı" sayılır; o banda
# bağlı OLMAYAN uzaktaki kopuk parçalar atılır.
BOTTOM_BAND_RATIO = 0.15

# Renk paleti (cls indexi -> BGR)
CLASS_COLORS = [
    (0, 255, 0),    # Class 0: Yeşil (yol)
    (255, 0, 0),    # Class 1: Mavi  (alternatif)
    (0, 0, 255),    # Class 2: Kırmızı
    (0, 255, 255),  # Class 3: Sarı
]


# ============================================================
# HEF TENSÖR DECODE  (maskeleme.py'den — Hailo split DFL çözümü)
# ============================================================

def compute_sigmoid(x: np.ndarray) -> np.ndarray:
    """Sayısal kararlı sigmoid."""
    return 1.0 / (1.0 + np.exp(-np.clip(x, -88, 88)))


def apply_non_max_suppression(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
    """OpenCV (C++) NMS."""
    if len(boxes_xyxy) == 0:
        return np.array([])
    boxes_xywh = np.empty_like(boxes_xyxy)
    boxes_xywh[:, 0] = boxes_xyxy[:, 0]
    boxes_xywh[:, 1] = boxes_xyxy[:, 1]
    boxes_xywh[:, 2] = boxes_xyxy[:, 2] - boxes_xyxy[:, 0]
    boxes_xywh[:, 3] = boxes_xyxy[:, 3] - boxes_xyxy[:, 1]
    indices = cv2.dnn.NMSBoxes(boxes_xywh.tolist(), scores.tolist(),
                               score_threshold=0.0, nms_threshold=iou_threshold)
    return indices.flatten() if len(indices) > 0 else np.array([])


def decode_yolov8_seg_tensors(raw_outputs: dict, confidence_threshold: float,
                              iou_threshold: float, num_classes: int = 2) -> tuple:
    """Hailo'nun ayrı conv katmanlarını (3 stride: 8/16/32) DFL ile decode edip
    mutlak bbox + sınıf skoru + maske katsayısı + proto tensörü üretir."""
    if 'yolov8n_seg/conv48' not in raw_outputs:
        raise ValueError("Prototype tensor (yolov8n_seg/conv48) model çıktısında yok.")

    proto_tensor = raw_outputs['yolov8n_seg/conv48'][0].transpose(2, 0, 1)

    # (Stride, BBox_DFL, Class_Scores, Mask_Coefficients)
    layer_scales = [
        (8,  raw_outputs['yolov8n_seg/conv44'], raw_outputs['yolov8n_seg/conv45'], raw_outputs['yolov8n_seg/conv46']),
        (16, raw_outputs['yolov8n_seg/conv60'], raw_outputs['yolov8n_seg/conv61'], raw_outputs['yolov8n_seg/conv62']),
        (32, raw_outputs['yolov8n_seg/conv73'], raw_outputs['yolov8n_seg/conv74'], raw_outputs['yolov8n_seg/conv75'])
    ]

    all_boxes, all_scores, all_coefs, all_classes = [], [], [], []
    dfl_weights = np.arange(16, dtype=np.float32)

    dbg_total, dbg_valid, dbg_smin, dbg_smax, dbg_smean = 0, 0, 1.0, 0.0, []

    for stride, bbox_layer, cls_layer, mask_layer in layer_scales:
        height, width = cls_layer.shape[1], cls_layer.shape[2]

        cls_pred = cls_layer[0].reshape(-1, num_classes)
        cls_scores = compute_sigmoid(cls_pred)

        max_scores = np.max(cls_scores, axis=1)
        valid_indices = max_scores >= confidence_threshold

        # ── teşhis: bu stride'da sigmoid istatistiği ──
        dbg_total += max_scores.size
        dbg_valid += int(valid_indices.sum())
        dbg_smin = min(dbg_smin, float(max_scores.min()))
        dbg_smax = max(dbg_smax, float(max_scores.max()))
        dbg_smean.append(float(max_scores.mean()))

        if not valid_indices.any():
            continue

        valid_scores = max_scores[valid_indices]
        valid_classes = np.argmax(cls_scores[valid_indices], axis=1)

        bbox_dfl = bbox_layer[0].reshape(-1, 64)[valid_indices].reshape(-1, 4, 16)
        exp_dfl = np.exp(bbox_dfl - np.max(bbox_dfl, axis=2, keepdims=True))
        softmax_dfl = exp_dfl / np.sum(exp_dfl, axis=2, keepdims=True)
        distances = np.sum(softmax_dfl * dfl_weights, axis=2)

        x_grid, y_grid = np.meshgrid(np.arange(width), np.arange(height))
        x_grid = x_grid.reshape(-1)[valid_indices]
        y_grid = y_grid.reshape(-1)[valid_indices]

        x_min = (x_grid + 0.5 - distances[:, 0]) * stride
        y_min = (y_grid + 0.5 - distances[:, 1]) * stride
        x_max = (x_grid + 0.5 + distances[:, 2]) * stride
        y_max = (y_grid + 0.5 + distances[:, 3]) * stride

        valid_coefs = mask_layer[0].reshape(-1, 32)[valid_indices]

        all_boxes.append(np.column_stack([x_min, y_min, x_max, y_max]))
        all_scores.append(valid_scores)
        all_coefs.append(valid_coefs)
        all_classes.append(valid_classes)

    if not all_boxes:
        return [], [], [], [], None

    boxes_xyxy = np.concatenate(all_boxes, axis=0)
    scores = np.concatenate(all_scores, axis=0)
    coefs = np.concatenate(all_coefs, axis=0)
    classes = np.concatenate(all_classes, axis=0)

    boxes_xyxy = np.clip(boxes_xyxy, 0, INPUT_SIZE)
    valid_area_mask = (boxes_xyxy[:, 2] - boxes_xyxy[:, 0]) > 1.0
    boxes_xyxy = boxes_xyxy[valid_area_mask]
    scores = scores[valid_area_mask]
    coefs = coefs[valid_area_mask]
    classes = classes[valid_area_mask]

    # ── TEŞHİS: model mi kod mu? ──
    if DEBUG:
        decode_yolov8_seg_tensors._n = getattr(decode_yolov8_seg_tensors, '_n', 0) + 1
        if decode_yolov8_seg_tensors._n % 30 == 0:
            mean = float(np.mean(dbg_smean)) if dbg_smean else 0.0
            pct = 100.0 * dbg_valid / max(dbg_total, 1)
            print(f"[DECODE] hücre={dbg_total} eşik_geçen={dbg_valid} ({pct:.1f}%) | "
                  f"cls_sigmoid min={dbg_smin:.3f} max={dbg_smax:.3f} ortalama={mean:.3f} | "
                  f"alan_sonrası_kutu={len(boxes_xyxy)}")

    if len(boxes_xyxy) == 0:
        return [], [], [], [], None

    # NMS'i bunaltmamak için ÖNCE skora göre en iyi PRE_NMS_TOPK kutuyu tut
    if len(scores) > PRE_NMS_TOPK:
        top = np.argsort(scores)[::-1][:PRE_NMS_TOPK]
        boxes_xyxy, scores, coefs, classes = (
            boxes_xyxy[top], scores[top], coefs[top], classes[top]
        )

    keep = apply_non_max_suppression(boxes_xyxy, scores, iou_threshold)
    if len(keep) == 0:
        return [], [], [], [], None

    return boxes_xyxy[keep], classes[keep], scores[keep], coefs[keep], proto_tensor


def process_mask_prototypes(boxes_xyxy, mask_coefs, proto_tensor, orig_h, orig_w) -> list:
    """Maske katsayıları × proto haritası → ikili (binary) segmentasyon maskeleri.
    Maske, bbox bölgesine kırpılır (YOLOv8-seg standardı; maske taşmasını önler)."""
    _, proto_h, proto_w = proto_tensor.shape
    proto_flat = proto_tensor.reshape(32, -1)
    binary_masks = []
    for i in range(len(boxes_xyxy)):
        mask_map = compute_sigmoid(mask_coefs[i] @ proto_flat).reshape(proto_h, proto_w)
        x1, y1, x2, y2 = boxes_xyxy[i]
        px1 = int(max(0, x1 / INPUT_SIZE * proto_w))
        py1 = int(max(0, y1 / INPUT_SIZE * proto_h))
        px2 = int(min(proto_w, x2 / INPUT_SIZE * proto_w))
        py2 = int(min(proto_h, y2 / INPUT_SIZE * proto_h))
        crop = np.zeros_like(mask_map)
        if py2 > py1 and px2 > px1:
            crop[py1:py2, px1:px2] = mask_map[py1:py2, px1:px2]
        full_mask = cv2.resize(crop, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        binary_masks.append(full_mask > 0.5)
    return binary_masks


def hef_postprocess(raw_outputs, orig_w, orig_h):
    """HEF ham çıktısını lane_test sözleşmesine çevirir:
        -> (bboxes[int, orig px], scores, masks[bool (h,w)], class_ids[int])
    Böylece extract_center_points / draw_results AYNEN kullanılabilir."""
    empty = (np.empty((0, 4), int), np.array([]), [], np.array([], int))

    boxes, class_ids, scores, coefs, proto = decode_yolov8_seg_tensors(
        raw_outputs, CONF_THRESH, IOU_THRESH, NUM_CLASSES
    )
    if len(boxes) == 0 or proto is None:
        return empty

    boxes = np.asarray(boxes, dtype=np.float32)
    class_ids = np.asarray(class_ids, dtype=int)
    scores = np.asarray(scores, dtype=np.float32)
    coefs = np.asarray(coefs, dtype=np.float32)

    # 1) (Opsiyonel) yalnız yol sınıfını tut — alternatif maskeleri at
    if ONLY_ROAD:
        sel = class_ids == ROAD_CLASS_ID
        boxes, class_ids, scores, coefs = boxes[sel], class_ids[sel], scores[sel], coefs[sel]
        if len(boxes) == 0:
            return empty

    # 2) Skora göre sırala + en iyi MAX_DETECTIONS tanesini tut.
    #    Maske ÜRETMEDEN önce budanır → pahalı resize/morfoloji sadece bunlar için.
    order = np.argsort(scores)[::-1][:MAX_DETECTIONS]
    boxes, class_ids, scores, coefs = boxes[order], class_ids[order], scores[order], coefs[order]

    # 3) Yalnız bu az sayıda kutu için maske üret
    masks = process_mask_prototypes(boxes, coefs, proto, orig_h, orig_w)

    # bbox'ları 640 -> orijinal çözünürlüğe ölçekle (yalnız etiket konumu/çizim için)
    sb = boxes.astype(np.float32).copy()
    sb[:, [0, 2]] *= orig_w / INPUT_SIZE
    sb[:, [1, 3]] *= orig_h / INPUT_SIZE
    bboxes = sb.astype(int)

    return bboxes, np.asarray(scores), masks, np.asarray(class_ids, dtype=int)


# ============================================================
# MASKELEME + ORTA-NOKTA  (lane_test_node.py'den BİREBİR)
# ============================================================

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


def extract_center_points(frame, masks, class_ids, prev_state=None):
    """Yol orta çizgisi: ROI'de en büyük cls-Road maskesi, satır başına median,
    kareler arası EMA. (lane_test ile aynı 'düz mod' mantığı.)"""
    h, w = frame.shape[:2]

    roi_pts = get_manual_roi_pts(h, w)
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    kernel = np.ones((7, 7), np.uint8)

    prev_cx = prev_state if isinstance(prev_state, dict) else {}

    # ROI içinde en büyük alanlı yol maskesini seç
    best_m, best_area = None, 0
    for mask, cls_id in zip(masks, class_ids):
        if cls_id != ROAD_CLASS_ID:
            continue
        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        m = cv2.bitwise_and(m, roi_mask)
        area = int(np.count_nonzero(m))
        if area > best_area:
            best_area, best_m = area, m

    new_cx = {}
    pts = []
    if best_m is not None:
        best_m = keep_bottom_component(best_m)
        for y in range(h - 1, int(h * ROI_TOP), -15):
            xs = np.where(best_m[y, :] > 0)[0]
            if len(xs) == 0:
                continue
            cx = float(np.median(xs))                       # satır median
            if y in prev_cx:                                # zamansal EMA
                cx = SMOOTH_ALPHA * cx + (1.0 - SMOOTH_ALPHA) * prev_cx[y]
            new_cx[y] = cx
            pts.append((int(round(cx)), y))

    all_pts = [pts] if len(pts) > 0 else []
    return all_pts, new_cx


def draw_results(image, bboxes, scores, masks, class_ids, all_smooth_pts):
    h, w = image.shape[:2]
    overlay = image.copy()

    roi_pts = get_manual_roi_pts(h, w)
    roi_mask_2d = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(roi_mask_2d, roi_pts, 255)

    kernel = np.ones((7, 7), np.uint8)
    for mask, cls_id, bbox, score in zip(masks, class_ids, bboxes, scores):
        color = CLASS_COLORS[cls_id] if cls_id < len(CLASS_COLORS) else (255, 255, 255)

        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        m = cv2.bitwise_and(m, roi_mask_2d)
        if cls_id == ROAD_CLASS_ID:
            m = keep_bottom_component(m)

        mask_bool = m.astype(bool)
        overlay[mask_bool] = (overlay[mask_bool] * 0.4 + np.array(color) * 0.6).astype(np.uint8)

        if SHOW_LABELS:
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


# ============================================================
# ANA ÇIKARIM DÖNGÜSÜ  (Hailo-8 + RealSense)
# ============================================================

def main():
    print("[INFO] Hailo-8 cihazı ve HEF modeli yükleniyor...")
    hef = HEF(HEF_FILE_PATH)
    target = VDevice()

    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_groups = target.configure(hef, configure_params)
    network_group = network_groups[0]
    network_group_params = network_group.create_params()

    in_params = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
    out_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

    input_name = hef.get_input_vstream_infos()[0].name

    print("[INFO] RealSense başlatılıyor...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    smooth_state = {}          # {y: cx} kareler arası hat yumuşatma durumu
    fps, last_t = 0.0, None

    try:
        with target, network_group.activate(network_group_params):
            with InferVStreams(network_group, in_params, out_params) as infer_pipeline:
                print("[INFO] Otonom algı aktif. (Çıkış için 'q')")

                while True:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue

                    frame = np.asanyarray(color_frame.get_data())
                    orig_h, orig_w = frame.shape[:2]

                    # ── FPS (kareler arası EMA) ──
                    now = time.time()
                    if last_t is not None:
                        dt = now - last_t
                        if dt > 0:
                            inst = 1.0 / dt
                            fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
                    last_t = now

                    # ── Ön işleme (HEF: UINT8, NHWC, RGB) ──
                    resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
                    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                    tensor_input = np.expand_dims(np.ascontiguousarray(rgb), axis=0)

                    # ── Donanım çıkarımı (süre ölç) ──
                    t_inf = time.time()
                    raw_outputs = infer_pipeline.infer({input_name: tensor_input})
                    infer_ms = (time.time() - t_inf) * 1000.0

                    # ── HEF çıktısı -> lane_test sözleşmesi (süre ölç) ──
                    t_post = time.time()
                    bboxes, scores, masks, class_ids = hef_postprocess(raw_outputs, orig_w, orig_h)
                    post_ms = (time.time() - t_post) * 1000.0
                    if DEBUG and last_t is not None:
                        # ~30 karede bir decode zaten yazıyor; burada süreyi ekle
                        pass

                    # ── Orta nokta (bizim maskeleme + median + EMA) ──
                    all_pts, smooth_state = extract_center_points(frame, masks, class_ids, smooth_state)

                    # ── center_pts: kontrol için (orijinal çözünürlükte) ──
                    # Gerçek araçta burada noktaları kontrol katmanına gönder
                    # (örn. seri/UART, ROS publish veya doğrudan direksiyon hesabı).
                    center_pts = all_pts[0] if all_pts else []

                    # ── Görselleştirme ──
                    result = draw_results(frame.copy(), bboxes, scores, masks, class_ids, all_pts)
                    hud = f"FPS:{fps:4.1f} pts:{len(center_pts)} inf:{infer_ms:4.0f}ms post:{post_ms:4.0f}ms"
                    cv2.putText(result, hud, (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(result, hud, (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)

                    cv2.imshow("Hailo-8 Serit Tespit (lane_test_hef)", result)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    except Exception:
        print(f"\n[CRITICAL ERROR] Pipeline çöktü:\n{traceback.format_exc()}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] Donanım kaynakları güvenle serbest bırakıldı.")


if __name__ == "__main__":
    main()
