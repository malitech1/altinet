"""Detector node performing YOLOv8 inference."""

from __future__ import annotations

import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, List, Optional

import numpy as np

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS is unavailable
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    from altinet.msg import PersonDetections as PersonDetectionsMsg
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Image = Header = CvBridge = PersonDetectionsMsg = None

from ..utils.config import default_yolo_config_path, load_file
from ..utils.models import YoloConfig, YoloV8Detector
from ..utils.ros_conversions import detections_to_msg
from ..utils.types import Detection


class DetectorPipeline:
    """High level interface around :class:`YoloV8Detector`."""

    def __init__(self, detector: YoloV8Detector) -> None:
        self.detector = detector
        self._fps_window: Deque[float] = deque(maxlen=60)

    def process(
        self, frame: np.ndarray, room_id: str, frame_id: str, timestamp: datetime
    ) -> List[Detection]:
        """Run person detection on ``frame``."""

        start = time.perf_counter()
        detections = self.detector.detect(frame, room_id, frame_id, timestamp)
        elapsed = time.perf_counter() - start
        if elapsed > 0:
            self._fps_window.append(1.0 / elapsed)
        return detections

    @property
    def fps(self) -> float:
        """Current rolling average FPS."""

        if not self._fps_window:
            return 0.0
        return float(sum(self._fps_window) / len(self._fps_window))


def load_config(path: Path) -> YoloConfig:
    """Load YOLO configuration from YAML."""

    data = load_file(path)
    model_path = Path(data.get("model_path", "assets/models/yolov8n.onnx"))
    conf_thresh = float(data.get("conf_thresh", 0.35))
    iou_thresh = float(data.get("iou_thresh", 0.5))
    return YoloConfig(
        model_path=model_path, conf_thresh=conf_thresh, iou_thresh=iou_thresh
    )


class DetectorNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node wrapping :class:`DetectorPipeline`."""

    def __init__(self) -> None:
        super().__init__("detector_node")
        default_config = default_yolo_config_path()
        self.declare_parameter("config", str(default_config))
        self.declare_parameter("room_id", "room_1")
        config_path = Path(self.get_parameter("config").value)
        room_id = str(self.get_parameter("room_id").value)
        config = load_config(config_path)
        self.pipeline = DetectorPipeline(YoloV8Detector(config))
        self.room_id = room_id
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            f"/altinet/camera/{room_id}",
            self._on_image,
            10,
        )
        self.publisher = self.create_publisher(
            PersonDetectionsMsg, "/altinet/person_detections", 10
        )

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp = datetime.utcnow()
        detections = self.pipeline.process(
            frame, self.room_id, msg.header.frame_id, timestamp
        )
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        self.publisher.publish(detections_to_msg(detections, header))


__all__ = ["DetectorPipeline", "DetectorNode", "load_config"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
