import cv2
import numpy as np
import pyrealsense2 as rs
import traceback
from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams,
                            InputVStreamParams, OutputVStreamParams, FormatType,
                            InferVStreams)

# ============================================================
# CONFIGURATION
# ============================================================
HEF_FILE_PATH = "2206_2model.hef"

# Segmentation Classes and Visualization Colors (BGR)
CLASS_METADATA = {
    0: {"name": "Road",        "color": (0, 255, 100)},
    1: {"name": "Alternative", "color": (0, 100, 255)},
}

# Model and Processing Parameters
CONFIDENCE_THRESHOLD = 0.25
IOU_THRESHOLD        = 0.45
MASK_PIXEL_THRESHOLD = 0.5
OVERLAY_ALPHA        = 0.45
INPUT_SIZE           = 640

# ============================================================
# TENSOR PROCESSING & MATH UTILITIES
# ============================================================

def compute_sigmoid(x: np.ndarray) -> np.ndarray:
    """Applies stable sigmoid function to a tensor."""
    return 1.0 / (1.0 + np.exp(-np.clip(x, -88, 88)))

def apply_non_max_suppression(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
    """Executes C++ optimized NMS via OpenCV."""
    if len(boxes_xyxy) == 0:
        return np.array([])
    
    # Convert [x_min, y_min, x_max, y_max] to [x_min, y_min, width, height] for OpenCV
    boxes_xywh = np.empty_like(boxes_xyxy)
    boxes_xywh[:, 0] = boxes_xyxy[:, 0]
    boxes_xywh[:, 1] = boxes_xyxy[:, 1]
    boxes_xywh[:, 2] = boxes_xyxy[:, 2] - boxes_xyxy[:, 0]
    boxes_xywh[:, 3] = boxes_xyxy[:, 3] - boxes_xyxy[:, 1]
    
    indices = cv2.dnn.NMSBoxes(boxes_xywh.tolist(), scores.tolist(), score_threshold=0.0, nms_threshold=iou_threshold)
    return indices.flatten() if len(indices) > 0 else np.array([])

def decode_yolov8_seg_tensors(raw_outputs: dict, confidence_threshold: float, iou_threshold: float, num_classes: int = 2) -> tuple:
    """
    Decodes Hailo split DFL layers into absolute bounding boxes, class scores, and mask coefficients.
    """
    if 'yolov8n_seg/conv48' not in raw_outputs:
        raise ValueError("Prototype tensor (yolov8n_seg/conv48) not found in model outputs.")
        
    proto_tensor = raw_outputs['yolov8n_seg/conv48'][0].transpose(2, 0, 1)

    # Structure: (Stride, BBox_DFL, Class_Scores, Mask_Coefficients)
    layer_scales = [
        (8,  raw_outputs['yolov8n_seg/conv44'], raw_outputs['yolov8n_seg/conv45'], raw_outputs['yolov8n_seg/conv46']),
        (16, raw_outputs['yolov8n_seg/conv60'], raw_outputs['yolov8n_seg/conv61'], raw_outputs['yolov8n_seg/conv62']),
        (32, raw_outputs['yolov8n_seg/conv73'], raw_outputs['yolov8n_seg/conv74'], raw_outputs['yolov8n_seg/conv75'])
    ]

    all_boxes = []
    all_scores = []
    all_coefs = []
    all_classes = []

    dfl_weights = np.arange(16, dtype=np.float32)

    for stride, bbox_layer, cls_layer, mask_layer in layer_scales:
        height, width = cls_layer.shape[1], cls_layer.shape[2]

        # Class Prediction and Confidence Filtering
        cls_pred = cls_layer[0].reshape(-1, num_classes)
        cls_scores = compute_sigmoid(cls_pred)
        
        max_scores = np.max(cls_scores, axis=1)
        valid_indices = max_scores >= confidence_threshold
        
        if not valid_indices.any():
            continue
            
        valid_scores = max_scores[valid_indices]
        valid_classes = np.argmax(cls_scores[valid_indices], axis=1)

        # BBox Distribution Focal Loss (DFL) Decoding
        bbox_dfl = bbox_layer[0].reshape(-1, 64)[valid_indices].reshape(-1, 4, 16)
        
        exp_dfl = np.exp(bbox_dfl - np.max(bbox_dfl, axis=2, keepdims=True))
        softmax_dfl = exp_dfl / np.sum(exp_dfl, axis=2, keepdims=True)
        distances = np.sum(softmax_dfl * dfl_weights, axis=2)

        # Grid Coordinate Calculation
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

    # Spatial clipping and invalid area removal
    boxes_xyxy = np.clip(boxes_xyxy, 0, INPUT_SIZE)
    valid_area_mask = (boxes_xyxy[:, 2] - boxes_xyxy[:, 0]) > 1.0
    
    boxes_xyxy = boxes_xyxy[valid_area_mask]
    scores = scores[valid_area_mask]
    coefs = coefs[valid_area_mask]
    classes = classes[valid_area_mask]

    if len(boxes_xyxy) == 0:
        return [], [], [], [], None

    # Apply NMS
    keep_indices = apply_non_max_suppression(boxes_xyxy, scores, iou_threshold)
    
    if len(keep_indices) == 0:
        return [], [], [], [], None

    return boxes_xyxy[keep_indices], classes[keep_indices], scores[keep_indices], coefs[keep_indices], proto_tensor

def process_mask_prototypes(boxes_xyxy: np.ndarray, mask_coefs: np.ndarray, proto_tensor: np.ndarray, orig_h: int, orig_w: int) -> list:
    """Multiplies mask coefficients with prototype maps to generate binary segmentation masks."""
    _, proto_h, proto_w = proto_tensor.shape
    binary_masks = []
    
    for i in range(len(boxes_xyxy)):
        # Matrix multiplication and sigmoid activation
        mask_map = compute_sigmoid(mask_coefs[i] @ proto_tensor.reshape(32, -1))
        mask_map = mask_map.reshape(proto_h, proto_w)

        # Map bounding box coordinates to prototype resolution
        x1, y1, x2, y2 = boxes_xyxy[i]
        px1 = int(max(0, x1 / INPUT_SIZE * proto_w))
        py1 = int(max(0, y1 / INPUT_SIZE * proto_h))
        px2 = int(min(proto_w, x2 / INPUT_SIZE * proto_w))
        py2 = int(min(proto_h, y2 / INPUT_SIZE * proto_h))

        crop = np.zeros_like(mask_map)
        if py2 > py1 and px2 > px1:
            crop[py1:py2, px1:px2] = mask_map[py1:py2, px1:px2]

        full_mask = cv2.resize(crop, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        binary_masks.append(full_mask > MASK_PIXEL_THRESHOLD)

    return binary_masks

def render_segmentation_overlays(frame: np.ndarray, boxes: np.ndarray, class_ids: list, confidences: list, masks: list) -> np.ndarray:
    """Blends segmentation masks onto the original frame and draws confidence labels."""
    overlay = frame.copy()
    
    for i, mask in enumerate(masks):
        cls_id = class_ids[i]
        meta = CLASS_METADATA.get(cls_id, {"name": f"Class {cls_id}", "color": (200, 200, 200)})
        
        # Apply color to mask pixels
        overlay[mask] = meta["color"]

        # Find the top-most pixel of the mask to place the label
        y_indices, x_indices = np.where(mask)
        if len(y_indices) > 0:
            top_idx = np.argmin(y_indices)
            label_pos = (x_indices[top_idx], y_indices[top_idx])
            label_text = f"{meta['name']} {confidences[i]:.2f}"
            
            cv2.putText(overlay, label_text, (label_pos[0], max(label_pos[1] - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    return cv2.addWeighted(overlay, OVERLAY_ALPHA, frame, 1 - OVERLAY_ALPHA, 0)

# ============================================================
# MAIN INFERENCE PIPELINE
# ============================================================

def main():
    print("[INFO] Initializing Hailo-8 Device and HEF Model...")
    
    hef = HEF(HEF_FILE_PATH)
    target = VDevice()

    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_groups = target.configure(hef, configure_params)
    network_group = network_groups[0]
    network_group_params = network_group.create_params()

    in_params  = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
    out_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

    input_vstream_info = hef.get_input_vstream_infos()[0]
    input_name = input_vstream_info.name

    print("[INFO] Initializing RealSense Pipeline...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        with target, network_group.activate(network_group_params):
            with InferVStreams(network_group, in_params, out_params) as infer_pipeline:
                
                print("[INFO] Autonomous Perception System is Active. (Press 'q' to exit)")

                while True:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame: 
                        continue

                    frame = np.asanyarray(color_frame.get_data())
                    orig_h, orig_w = frame.shape[:2]

                    # Pre-processing
                    resized_frame = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
                    rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
                    tensor_input = np.expand_dims(np.ascontiguousarray(rgb_frame), axis=0)

                    # Hardware Inference
                    raw_outputs = infer_pipeline.infer({input_name: tensor_input})

                    # Post-processing
                    boxes, class_ids, confs, coefs, proto = decode_yolov8_seg_tensors(
                        raw_outputs, 
                        confidence_threshold=CONFIDENCE_THRESHOLD, 
                        iou_threshold=IOU_THRESHOLD,
                        num_classes=len(CLASS_METADATA)
                    )

                    result_frame = frame.copy()

                    if len(boxes) > 0 and proto is not None:
                        masks = process_mask_prototypes(boxes, coefs, proto, orig_h, orig_w)
                        result_frame = render_segmentation_overlays(result_frame, boxes, class_ids, confs, masks)

                    # HUD Overlay
                    cv2.putText(result_frame, f"Detections: {len(boxes)}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

                    cv2.imshow("Hailo-8 Perception Node", result_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'): 
                        break

    except Exception as e:
        print(f"\n[CRITICAL ERROR] Pipeline crashed:\n{traceback.format_exc()}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] Hardware resources safely released.")

if __name__ == "__main__":
    main()