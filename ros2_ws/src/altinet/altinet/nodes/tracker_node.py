"""Tracker node maintaining persistent person identities."""

from __future__ import annotations

from datetime import datetime
from typing import Iterable, List, Optional

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS is unavailable
    import rclpy
    from rclpy.node import Node
    from altinet.msg import PersonDetections as PersonDetectionsMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    PersonDetectionsMsg = PersonTracksMsg = None

from ..utils.tracking import ByteTrack, ByteTrackConfig
from ..utils.ros_conversions import tracks_to_msg
from ..utils.types import BoundingBox, Detection, Track


class TrackerPipeline:
    """Wraps :class:`ByteTrack` to provide a simplified interface."""

    def __init__(self, tracker: ByteTrack | None = None) -> None:
        self.tracker = tracker or ByteTrack()

    def update(self, detections: Iterable[Detection]) -> List[Track]:
        """Update tracker state based on ``detections``."""

        return self.tracker.update(detections)


class TrackerNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node bridging detection and tracking stages."""

    def __init__(self) -> None:
        super().__init__("tracker_node")
        self.declare_parameter("track_thresh", 0.4)
        self.declare_parameter("match_thresh", 0.7)
        self.declare_parameter("max_age", 30)

        config = ByteTrackConfig(
            track_thresh=float(self.get_parameter("track_thresh").value),
            match_thresh=float(self.get_parameter("match_thresh").value),
            max_age=int(self.get_parameter("max_age").value),
        )
        self.pipeline = TrackerPipeline(ByteTrack(config))
        self.subscription = self.create_subscription(
            PersonDetectionsMsg,
            "/altinet/person_detections",
            self._on_detections,
            10,
        )
        self.publisher = self.create_publisher(
            PersonTracksMsg, "/altinet/person_tracks", 10
        )

    def _on_detections(self, msg: PersonDetectionsMsg) -> None:
        detections = []
        for det in msg.detections:
            bbox = (det.x, det.y, det.w, det.h)
            image_height = int(getattr(det, "image_height", 0)) or 1
            image_width = int(getattr(det, "image_width", 0)) or 1
            detection = Detection(
                bbox=BoundingBox(*bbox),
                confidence=det.conf,
                room_id=msg.room_id,
                frame_id=det.frame_id,
                timestamp=datetime.utcnow(),
                image_size=(image_height, image_width),
            )
            detections.append(detection)
        tracks = self.pipeline.update(detections)
        header = msg.header
        self.publisher.publish(tracks_to_msg(tracks, header))


__all__ = ["TrackerPipeline", "TrackerNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
