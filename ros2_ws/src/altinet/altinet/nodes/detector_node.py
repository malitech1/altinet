"""Detector node performing YOLOv8 inference."""

from __future__ import annotations

import time
from collections import deque
from datetime import datetime
from functools import partial
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
    from altinet.srv import CheckPersonIdentity
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Image = Header = CvBridge = PersonDetectionsMsg = CheckPersonIdentity = None

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
        self.declare_parameter("identity_service_enabled", True)
        self.declare_parameter("identity_service_timeout", 0.25)
        self.declare_parameter("min_detection_interval", 1.5)
        config_path = Path(self.get_parameter("config").value)
        room_id = str(self.get_parameter("room_id").value)
        config = load_config(config_path)
        self.pipeline = DetectorPipeline(YoloV8Detector(config))
        self.room_id = room_id
        self.bridge = CvBridge()
        self._min_detection_interval = float(
            self.get_parameter("min_detection_interval").value
        )
        self._last_detection_time = time.monotonic() - self._min_detection_interval
        self._skipped_frames = 0
        self._connection_logged = False
        self.subscription = self.create_subscription(
            Image,
            f"/altinet/camera/{room_id}",
            self._on_image,
            10,
        )
        self.publisher = self.create_publisher(
            PersonDetectionsMsg, "/altinet/person_detections", 10
        )

        self.identity_client = None
        self._identity_warned = False
        if (
            CheckPersonIdentity is not None
            and bool(self.get_parameter("identity_service_enabled").value)
        ):
            self.identity_client = self.create_client(
                CheckPersonIdentity, "/altinet/check_person_identity"
            )
            timeout = float(
                self.get_parameter("identity_service_timeout").value
            )
            if timeout > 0.0 and not self.identity_client.wait_for_service(
                timeout_sec=timeout
            ):
                self.get_logger().warn(
                    "Identity service '/altinet/check_person_identity' "
                    "not available yet; will retry in the background"
                )

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_detection_time < self._min_detection_interval:
            self._skipped_frames += 1
            if self._skipped_frames == 1 or self._skipped_frames % 10 == 0:
                self.get_logger().debug(
                    "Skipping camera frame; %d frame(s) deferred to honor "
                    "min_detection_interval=%.2fs",
                    self._skipped_frames,
                    self._min_detection_interval,
                )
            return
        if not self._connection_logged:
            self.get_logger().info(
                f"Established connection to camera feed for room '{self.room_id}'. "
                "Monitoring frames for people."
            )
            self._connection_logged = True
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp = datetime.utcnow()
        detections = self.pipeline.process(
            frame, self.room_id, msg.header.frame_id, timestamp
        )
        self._last_detection_time = time.monotonic()
        self._skipped_frames = 0
        if detections:
            for detection in detections:
                bbox = detection.bbox
                self.get_logger().info(
                    f"Detected person in room '{detection.room_id}' "
                    f"(frame '{detection.frame_id}') at "
                    f"x={bbox.x:.1f}, y={bbox.y:.1f}, w={bbox.w:.1f}, h={bbox.h:.1f} "
                    f"with confidence {detection.confidence:.2f}"
                )
                self._request_identity_check(detection)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        self.publisher.publish(detections_to_msg(detections, header))

    def _request_identity_check(self, detection: Detection) -> None:
        if self.identity_client is None:
            return
        if not self.identity_client.service_is_ready():
            if not self._identity_warned:
                self.get_logger().warn(
                    "Identity service unavailable; skipping person classification"
                )
                self._identity_warned = True
            return
        if self._identity_warned:
            self.get_logger().info(
                "Identity service available; resuming classification requests"
            )
            self._identity_warned = False

        request = CheckPersonIdentity.Request()
        request.track_id = -1
        request.x = float(detection.bbox.x)
        request.y = float(detection.bbox.y)
        request.w = float(detection.bbox.w)
        request.h = float(detection.bbox.h)
        request.detection_confidence = float(detection.confidence)
        request.room_id = detection.room_id
        request.frame_id = detection.frame_id
        image_height, image_width = detection.image_size
        request.image_width = int(image_width)
        request.image_height = int(image_height)

        future = self.identity_client.call_async(request)
        future.add_done_callback(partial(self._log_identity_result, detection=detection))

    def _log_identity_result(self, future, detection: Detection) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - logging best effort
            self.get_logger().warn(
                f"Identity classification failed for detection in room "
                f"'{detection.room_id}': {exc}"
            )
            return

        label = response.label or ("user" if response.is_user else "guest")
        bbox = detection.bbox
        message = (
            f"Identity check for detection in room '{detection.room_id}' "
            f"(frame '{detection.frame_id}') at x={bbox.x:.1f}, y={bbox.y:.1f} -> "
            f"{label} (confidence {response.confidence:.2f}). {response.reason}"
        )
        self.get_logger().info(message)


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
