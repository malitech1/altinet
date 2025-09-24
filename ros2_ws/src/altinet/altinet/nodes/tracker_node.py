"""Tracker node maintaining persistent person identities."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime
from typing import Iterable, List, Optional

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS is unavailable
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Header
    from altinet.msg import PersonDetections as PersonDetectionsMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Header = PersonDetectionsMsg = PersonTracksMsg = None

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

    def predict(
        self,
        tracks: Iterable[Track],
        now: datetime,
        max_seconds: Optional[float] = None,
    ) -> List[Track]:
        """Project tracks forward using their velocity estimates."""

        horizon = max_seconds if max_seconds is not None and max_seconds > 0.0 else None
        predicted: List[Track] = []
        for track in tracks:
            dt = (now - track.timestamp).total_seconds()
            if horizon is not None:
                dt = min(dt, horizon)
            if dt <= 0.0 or track.velocity == (0.0, 0.0):
                predicted.append(track)
                continue
            bbox = BoundingBox(
                track.bbox.x + track.velocity[0] * dt,
                track.bbox.y + track.velocity[1] * dt,
                track.bbox.w,
                track.bbox.h,
            )
            predicted.append(replace(track, bbox=bbox, timestamp=now))
        return predicted

    def assign_identity(self, track_id: int, identity_id: str, confidence: float) -> bool:
        """Associate a known identity with a track."""

        return self.tracker.assign_identity(track_id, identity_id, confidence)


class TrackerNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node bridging detection and tracking stages."""

    def __init__(self) -> None:
        super().__init__("tracker_node")
        self.declare_parameter("track_thresh", 0.4)
        self.declare_parameter("match_thresh", 0.7)
        self.declare_parameter("max_age", 30)
        self.declare_parameter("prediction_publish_rate", 10.0)
        self.declare_parameter("prediction_horizon_seconds", 1.5)

        config = ByteTrackConfig(
            track_thresh=float(self.get_parameter("track_thresh").value),
            match_thresh=float(self.get_parameter("match_thresh").value),
            max_age=int(self.get_parameter("max_age").value),
        )
        self.pipeline = TrackerPipeline(ByteTrack(config))
        self._last_tracks: List[Track] = []
        self._last_frame_id: str = ""
        rate_param = self.get_parameter("prediction_publish_rate").value
        try:
            prediction_rate = float(rate_param)
        except (TypeError, ValueError):
            prediction_rate = 0.0
        horizon_param = self.get_parameter("prediction_horizon_seconds").value
        try:
            horizon = float(horizon_param)
        except (TypeError, ValueError):
            horizon = 0.0
        self._prediction_horizon = horizon if horizon > 0.0 else None
        self.subscription = self.create_subscription(
            PersonDetectionsMsg,
            "/altinet/person_detections",
            self._on_detections,
            10,
        )
        self.publisher = self.create_publisher(
            PersonTracksMsg, "/altinet/person_tracks", 10
        )
        self._prediction_timer = None
        if prediction_rate > 0.0:
            period = 1.0 / prediction_rate
            self._prediction_timer = self.create_timer(
                period, self._publish_predictions
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
        self._last_tracks = list(tracks)
        self._last_frame_id = msg.header.frame_id
        header = msg.header
        self.publisher.publish(tracks_to_msg(tracks, header))

    def _publish_predictions(self) -> None:
        if not self._last_tracks or Header is None:
            return
        frame_id = self._last_frame_id
        if not frame_id:
            return
        now = datetime.utcnow()
        predicted = self.pipeline.predict(
            self._last_tracks, now, self._prediction_horizon
        )
        if not predicted:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        self.publisher.publish(tracks_to_msg(predicted, header))


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
