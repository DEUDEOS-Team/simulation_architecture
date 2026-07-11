from __future__ import annotations

"""
lane_detection_node — Hailo-8 (HEF) şerit algılama (deos mimarisi).

GÜNCELLEME: Bu dosyanın ilk sürümündeki HEF çözümlemesi YANLIŞTI — modelin tek
birleşik çıktı verdiğini varsayıyordu. Bu HEF (yolov8n_seg) çıktıları AYRI conv
katmanlarına bölünmüştür ve 3 stride (8/16/32) üzerinden DFL çözümü gerektirir.
Maskeleme de ilkeldi (ROI yok, uzak blob filtresi yok, satır-ortalaması, EMA yok).

Bu sürüm, şu an kullandığımız KANITLANMIŞ pipeline'ın (lane_test_hef.py) birebir
taşınmasıdır:
  • UINT8 giriş / FLOAT32 çıkış vstream'leri (eski sürüm FLOAT32 giriş kullanıyordu)
  • decode_yolov8_seg_tensors — DFL decode + PRE_NMS_TOPK budaması (NMS'i bunaltmaz)
  • CONF_THRESH=0.501 — bu modelin cls sigmoid tabanı tam 0.500; eşik 0.50 olsaydı
    TÜM hücreler geçerdi ("her yer yol" + FPS çöküşü). 0.501 ölü tabanı eler.
  • ONLY_ROAD + MAX_DETECTIONS=3 — maske üretmeden ÖNCE en iyi tespitlere budama
  • ROI (manuel yamuk) + CLOSE/OPEN morfoloji + keep_bottom_component
  • extract_center_points — satır başına MEDIAN + kareler arası EMA (SMOOTH_ALPHA)
  • InferVStreams/activate bir KEZ açılır (kare başına değil — Pi'de FPS için kritik)

Çıktı sözleşmesi DEĞİŞMEDİ:
  - center_pts (Float32MultiArray): [x0,y0,x1,y1,...] orijinal piksel koordinatları
  - lane_debug (String, opsiyonel JSON)

Sınıf eşlemesi (2206_2model.hef): 0 = Road (yol), 1 = Alternative.
"""

import json
from contextlib import ExitStack
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from deos_algorithms.ros_topic_layout import build_deos_topics

# ============================================================
# HEF / ÇIKARIM AYARLARI
# ============================================================
INPUT_SIZE  = 640      # HEF giriş boyutu (640x640) — stride grid'i buna göre
IOU_THRESH  = 0.45     # NMS IoU
NUM_CLASSES = 2        # 0=Road, 1=Alternative

DEBUG        = False   # True: her ~30 karede decode istatistiği logla (model teşhisi)
PRE_NMS_TOPK = 300     # NMS'e girmeden önce skora göre tutulacak en fazla kutu

# ============================================================
# MASKELEME / ORTA-NOKTA AYARLARI  (lane_test ile AYNI)
# ============================================================
CONF_THRESH = 0.501    # ÖNEMLİ: cls sigmoid tabanı tam 0.500 — 0.50 yapma! (yukarı bkz.)
ROAD_CLASS_ID = 0

ONLY_ROAD      = True  # yalnız yol sınıfını işle
MAX_DETECTIONS = 3     # maske üretmeden önce tutulacak en iyi tespit sayısı

# ROI (manuel yamuk) — ekran oranına göre
ROI_TOP   = 0.6
ROI_BOT   = 0.88
ROI_TOP_L = 0.05
ROI_TOP_R = 0.95
ROI_BOT_L = 0.05
ROI_BOT_R = 0.95

SMOOTH_ALPHA      = 0.35  # kareler arası EMA: küçük=stabil, büyük=çevik
BOTTOM_BAND_RATIO = 0.15  # en alt bu oranlık banda bağlı olmayan uzak blob'lar atılır


# ============================================================
# HEF TENSÖR DECODE  (lane_test_hef.py'den birebir)
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
        -> (bboxes[int, orig px], scores, masks[bool (h,w)], class_ids[int])"""
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

    # 1) Yalnız yol sınıfını tut — alternatif maskeleri at
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

    # bbox'ları 640 -> orijinal çözünürlüğe ölçekle
    sb = boxes.astype(np.float32).copy()
    sb[:, [0, 2]] *= orig_w / INPUT_SIZE
    sb[:, [1, 3]] *= orig_h / INPUT_SIZE
    bboxes = sb.astype(int)

    return bboxes, np.asarray(scores), masks, np.asarray(class_ids, dtype=int)


# ============================================================
# MASKELEME + ORTA-NOKTA  (lane_test'ten BİREBİR)
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
    kareler arası EMA."""
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


# ============================================================
# ROS NODE
# ============================================================

class LaneDetectionNode(Node):
    """
    Raspberry hedefi: Hailo HEF ile segmentation inference yapıp centerline noktaları üretir.
    Çıktı:
    - /perception/center_pts (Float32MultiArray): [x0,y0,x1,y1,...] pixel koordinatları
    - /perception/lane_debug (String): basit JSON debug (opsiyonel)
    """

    def __init__(self):
        super().__init__("lane_detection_node")

        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))
        self.declare_parameter("image_topic", _T["sensors_camera_color"])
        self.declare_parameter("out_center_pts_topic", _T["perception_lane_center_pts"])
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("debug_topic", _T["perception_lane_debug"])

        # Parametreler modül sabitlerini günceller (pipeline fonksiyonları bunları okur)
        global CONF_THRESH, ROI_TOP, SMOOTH_ALPHA, MAX_DETECTIONS, DEBUG

        self.declare_parameter("hef_path", "model.hef")
        # Ayar parametreleri — modül sabitleriyle aynı varsayılanlar (yukarı bkz.)
        self.declare_parameter("conf_thresh", CONF_THRESH)
        self.declare_parameter("roi_top_ratio", ROI_TOP)
        self.declare_parameter("smooth_alpha", SMOOTH_ALPHA)
        self.declare_parameter("max_detections", MAX_DETECTIONS)
        self.declare_parameter("debug_decode", DEBUG)

        CONF_THRESH = float(self.get_parameter("conf_thresh").value)
        ROI_TOP = float(self.get_parameter("roi_top_ratio").value)
        SMOOTH_ALPHA = float(self.get_parameter("smooth_alpha").value)
        MAX_DETECTIONS = int(self.get_parameter("max_detections").value)
        DEBUG = bool(self.get_parameter("debug_decode").value)

        self._bridge = CvBridge()
        self._pub_pts = self.create_publisher(Float32MultiArray, str(self.get_parameter("out_center_pts_topic").value), 10)
        self._pub_dbg = self.create_publisher(String, str(self.get_parameter("debug_topic").value), 10)

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self._hailo_ok = False
        self._infer_pipeline = None
        self._input_name: Optional[str] = None
        self._smooth_state: dict = {}       # {y: cx} kareler arası EMA durumu
        self._hw_stack = ExitStack()        # Hailo kaynakları (kapanışta serbest bırakılır)

        self._init_hailo()

    def _init_hailo(self) -> None:
        hef_path = Path(str(self.get_parameter("hef_path").value))
        if not hef_path.exists():
            self.get_logger().error(f"HEF bulunamadı: {hef_path}")
            return

        try:
            from hailo_platform import (  # type: ignore
                HEF,
                VDevice,
                HailoStreamInterface,
                InferVStreams,
                ConfigureParams,
                InputVStreamParams,
                OutputVStreamParams,
                FormatType,
            )
        except Exception as e:
            self.get_logger().error(f"hailo_platform import edilemedi (Raspberry/Hailo gerekli): {e}")
            return

        try:
            hef = HEF(str(hef_path))
            target = self._hw_stack.enter_context(VDevice())
            configure_params = ConfigureParams.create_from_hef(hef=hef, interface=HailoStreamInterface.PCIe)
            network_group = target.configure(hef, configure_params)[0]
            network_group_params = network_group.create_params()

            # Giriş UINT8 (kamera karesi doğrudan; /255 float normalizasyonu YOK),
            # çıkış FLOAT32 (dequantize edilmiş) — lane_test_hef ile aynı.
            in_params = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
            out_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

            self._input_name = hef.get_input_vstream_infos()[0].name

            # Aktivasyon + InferVStreams BİR KEZ açılır ve callback'lerde yeniden
            # kullanılır. (Kare başına activate etmek Pi'de FPS'i öldürür.)
            self._hw_stack.enter_context(network_group.activate(network_group_params))
            self._infer_pipeline = self._hw_stack.enter_context(
                InferVStreams(network_group, in_params, out_params))

            self._hailo_ok = True
            self.get_logger().info("lane_detection_node: Hailo hazır (UINT8 giriş, kalıcı vstream)")
        except Exception as e:
            self.get_logger().error(f"Hailo init hatası: {e}")

    def _image_cb(self, msg: Image) -> None:
        if not self._hailo_ok:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        orig_h, orig_w = frame.shape[:2]

        # ── Ön işleme (HEF: UINT8, NHWC, RGB) ──
        resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor_input = np.expand_dims(np.ascontiguousarray(rgb), axis=0)

        try:
            raw_outputs = self._infer_pipeline.infer({str(self._input_name): tensor_input})
        except Exception as e:
            self.get_logger().error(f"Hailo inference: {e}")
            return

        # ── HEF çıktısı -> lane_test sözleşmesi ──
        bboxes, scores, masks, class_ids = hef_postprocess(raw_outputs, orig_w, orig_h)
        _ = (bboxes,)  # centerline çıkarımında kullanılmıyor

        # ── Orta nokta (ROI + morfoloji + median + EMA) ──
        all_pts, self._smooth_state = extract_center_points(
            frame, masks, class_ids, self._smooth_state)
        pts = all_pts[0] if all_pts else []

        msg_pts = Float32MultiArray()
        msg_pts.data = [float(v) for (x, y) in pts for v in (x, y)]
        self._pub_pts.publish(msg_pts)

        if bool(self.get_parameter("publish_debug").value):
            self._pub_dbg.publish(
                String(
                    data=json.dumps(
                        {
                            "n_pts": len(pts),
                            "n_masks": len(masks),
                            "best_score": float(scores[0]) if len(scores) else 0.0,
                            "img": {"w": orig_w, "h": orig_h},
                        },
                        ensure_ascii=False,
                    )
                )
            )

    def destroy_node(self):
        self._hw_stack.close()   # Hailo kaynaklarını serbest bırak
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
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
