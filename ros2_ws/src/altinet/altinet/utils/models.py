"""Model loading utilities for Altinet."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

try:  # pragma: no cover - optional dependency for tests
    import cv2
except ImportError:  # pragma: no cover - handled when OpenCV is missing
    cv2 = None

import numpy as np

try:  # pragma: no cover - optional dependency for tests
    import onnxruntime as ort
except ImportError:  # pragma: no cover - handled at runtime
    ort = None

from .types import BoundingBox, Detection


@dataclass
class YoloConfig:
    """Configuration for the YOLOv8 ONNX detector."""

    model_path: Path
    conf_thresh: float = 0.35
    iou_thresh: float = 0.5
    class_ids: Sequence[int] = (0,)
    providers: Optional[Sequence[str]] = None


class YoloV8Detector:
    """Run YOLOv8 ONNX inference for person detection."""

    def __init__(self, config: YoloConfig) -> None:
        self.config = config
        if cv2 is None:
            raise RuntimeError(
                "OpenCV is required to run the detector. Install opencv-python to use YoloV8Detector."
            )
        if ort is None:
            raise RuntimeError(
                "onnxruntime is required to run the detector. Install it to use YoloV8Detector."
            )
        self.session = ort.InferenceSession(
            str(config.model_path),
            providers=list(config.providers or ort.get_available_providers()),
        )
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        # Dynamic dimensions are represented by None.
        self.input_height = int(input_shape[2] or 640)
        self.input_width = int(input_shape[3] or 640)

    def detect(
        self, image: np.ndarray, room_id: str, frame_id: str, timestamp
    ) -> List[Detection]:
        """Run inference on a frame and return person detections."""

        img_resized, ratio, padding = _letterbox(
            image, new_shape=(self.input_height, self.input_width)
        )
        blob = img_resized[:, :, ::-1].transpose(2, 0, 1)
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        blob = blob[np.newaxis, ...]

        outputs = self.session.run(None, {self.input_name: blob})
        predictions = outputs[0]
        predictions = np.squeeze(predictions)
        if predictions.ndim == 1:
            predictions = np.expand_dims(predictions, 0)
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        boxes = predictions[:, :4]
        class_prob_matrix = predictions[:, 4:]

        detections: List[Detection] = []
        for idx in range(boxes.shape[0]):
            class_probs = class_prob_matrix[idx]
            if class_probs.size == 0:
                continue
            best_class = int(np.argmax(class_probs))
            if best_class not in self.config.class_ids:
                continue
            score = float(class_probs[best_class])
            if score < self.config.conf_thresh:
                continue
            box = boxes[idx]
            bbox = _scale_box(box, ratio, padding, image.shape[:2])
            detections.append(
                Detection(
                    bbox=BoundingBox(*bbox),
                    confidence=score,
                    room_id=room_id,
                    frame_id=frame_id,
                    timestamp=timestamp,
                    image_size=image.shape[:2],
                )
            )

        keep = _nms(
            [d.bbox for d in detections],
            [d.confidence for d in detections],
            self.config.iou_thresh,
        )
        return [detections[i] for i in keep]


def _letterbox(
    image: np.ndarray,
    new_shape: Tuple[int, int] = (640, 640),
    color: Tuple[int, int, int] = (114, 114, 114),
) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """Resize image to ``new_shape`` using letterbox padding."""

    height, width = image.shape[:2]
    scale = min(new_shape[0] / height, new_shape[1] / width)
    new_unpad = (int(round(width * scale)), int(round(height * scale)))
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    dw /= 2.0
    dh /= 2.0

    resized = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return padded, scale, (dw, dh)


def _scale_box(
    box: np.ndarray,
    ratio: float,
    padding: Tuple[float, float],
    original_shape: Tuple[int, int],
) -> Tuple[float, float, float, float]:
    """Scale a YOLO ``box`` back to the original image size."""

    cx, cy, w, h = box
    cx -= padding[0]
    cy -= padding[1]
    cx /= ratio
    cy /= ratio
    w /= ratio
    h /= ratio

    x1 = max(cx - w / 2.0, 0.0)
    y1 = max(cy - h / 2.0, 0.0)
    x2 = min(cx + w / 2.0, float(original_shape[1]))
    y2 = min(cy + h / 2.0, float(original_shape[0]))
    return x1, y1, x2 - x1, y2 - y1


def _nms(boxes: List[BoundingBox], scores: List[float], iou_thresh: float) -> List[int]:
    """Perform a simple non-maximum suppression."""

    if not boxes:
        return []

    boxes_xyxy = np.array([box.as_xyxy() for box in boxes], dtype=np.float32)
    scores_np = np.array(scores)
    order = scores_np.argsort()[::-1]
    keep: List[int] = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        others = boxes_xyxy[order[1:]]
        ious = _iou(boxes_xyxy[i], others)
        remaining = np.where(ious <= iou_thresh)[0]
        order = order[remaining + 1]

    return keep


def _iou(box: Sequence[float], others: np.ndarray) -> np.ndarray:
    """Compute IoU between ``box`` and ``others``."""

    x1 = np.maximum(box[0], others[:, 0])
    y1 = np.maximum(box[1], others[:, 1])
    x2 = np.minimum(box[2], others[:, 2])
    y2 = np.minimum(box[3], others[:, 3])

    inter_area = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    box_area = (box[2] - box[0]) * (box[3] - box[1])
    others_area = (others[:, 2] - others[:, 0]) * (others[:, 3] - others[:, 1])
    union = box_area + others_area - inter_area + 1e-9
    return inter_area / union


__all__ = ["YoloConfig", "YoloV8Detector"]
