"""ROS 2 node that captures high-quality face snapshots for tracked people."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Optional

from ..utils.face_capture import FaceCaptureConfig, FaceCapturePipeline, InsightFaceAnalyzer
from ..utils.ros_conversions import face_snapshot_to_msg, face_snapshots_to_msg
from ..utils.types import BoundingBox, Track

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS is unavailable
    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    from altinet.msg import FaceSnapshots as FaceSnapshotsMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
    from altinet.srv import RequestFaceSnapshot
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Time = Image = Header = CvBridge = FaceSnapshotsMsg = PersonTracksMsg = RequestFaceSnapshot = None


def _ros_time_to_datetime(stamp: Time) -> datetime:
    """Convert a ROS ``builtin_interfaces/Time`` to ``datetime``."""

    if stamp is None:
        return datetime.utcnow()
    seconds = float(getattr(stamp, "sec", 0))
    nanoseconds = float(getattr(stamp, "nanosec", 0))
    total_seconds = seconds + nanoseconds / 1_000_000_000.0
    return datetime.fromtimestamp(total_seconds, tz=timezone.utc).replace(tzinfo=None)


class _FallbackAnalyzer:
    """Analyzer used when InsightFace cannot be initialised."""

    def detect(self, image: Any):  # pragma: no cover - trivial
        return []


class FaceCaptureNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node that produces ``FaceSnapshot`` messages."""

    def __init__(self) -> None:
        super().__init__("face_capture_node")
        if FaceSnapshotsMsg is None or PersonTracksMsg is None or CvBridge is None:
            raise RuntimeError("ROS message dependencies for face capture are unavailable")

        self.declare_parameter("room_id", "living_room")
        self.declare_parameter("model_name", "buffalo_l")
        self.declare_parameter("model_root", "")
        self.declare_parameter("capture_cadence_s", 2.0)
        self.declare_parameter("minimum_quality", 0.6)
        self.declare_parameter("improvement_margin", 0.05)
        self.declare_parameter("frame_history", 45)
        self.declare_parameter("frame_tolerance_s", 0.1)
        self.declare_parameter("face_padding", 0.15)
        self.declare_parameter("det_size", [640, 640])
        self.declare_parameter("ctx_id", 0)

        self.room_id = str(self.get_parameter("room_id").value)
        model_name = str(self.get_parameter("model_name").value)
        model_root_param = self.get_parameter("model_root").value
        model_root = str(model_root_param) if model_root_param else None
        det_size_param = self.get_parameter("det_size").value
        if isinstance(det_size_param, (list, tuple)) and len(det_size_param) == 2:
            det_size = int(det_size_param[0]), int(det_size_param[1])
        else:
            det_size = (640, 640)
        ctx_id = int(self.get_parameter("ctx_id").value)

        config = FaceCaptureConfig(
            minimum_quality=float(self.get_parameter("minimum_quality").value),
            improvement_margin=float(self.get_parameter("improvement_margin").value),
            min_time_between_snapshots=float(self.get_parameter("capture_cadence_s").value),
            frame_history=int(self.get_parameter("frame_history").value),
            face_padding=float(self.get_parameter("face_padding").value),
            frame_tolerance_s=float(self.get_parameter("frame_tolerance_s").value),
        )

        try:
            analyzer = InsightFaceAnalyzer(
                model_name=model_name,
                model_root=model_root,
                det_size=det_size,
                ctx_id=ctx_id,
            )
            self.get_logger().info(
                f"Initialized InsightFace analyzer for face capture (model={model_name})"
            )
        except Exception as exc:  # pragma: no cover - depends on optional deps
            self.get_logger().error(
                f"Falling back to disabled face capture: {exc}"
            )
            analyzer = _FallbackAnalyzer()

        self.pipeline = FaceCapturePipeline(analyzer, config=config)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            FaceSnapshotsMsg, "/altinet/face_snapshots", 10
        )
        self.track_subscription = self.create_subscription(
            PersonTracksMsg,
            "/altinet/person_tracks",
            self._on_tracks,
            10,
        )
        self.frame_subscription = self.create_subscription(
            Image,
            f"/altinet/camera/{self.room_id}",
            self._on_image,
            10,
        )
        self.service = self.create_service(
            RequestFaceSnapshot,
            "/altinet/request_face_snapshot",
            self._handle_snapshot_request,
        )
        self.get_logger().info(
            f"Face capture node ready for room '{self.room_id}'"
        )

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp = _ros_time_to_datetime(msg.header.stamp)
        self.pipeline.add_frame(self.room_id, timestamp, frame)

    def _on_tracks(self, msg: PersonTracksMsg) -> None:
        timestamp = _ros_time_to_datetime(msg.header.stamp)
        room_id = msg.room_id or self.room_id
        tracks = []
        for track_msg in msg.tracks:
            track_stamp = getattr(track_msg, "header", None)
            track_timestamp = _ros_time_to_datetime(track_stamp.stamp if track_stamp else None)
            if track_stamp and getattr(track_stamp.stamp, "sec", 0) == 0 and getattr(
                track_stamp.stamp, "nanosec", 0
            ) == 0:
                track_timestamp = timestamp
            bbox = BoundingBox(track_msg.x, track_msg.y, track_msg.w, track_msg.h)
            image_height = int(getattr(track_msg, "image_height", 0) or 0)
            image_width = int(getattr(track_msg, "image_width", 0) or 0)
            image_size = (image_height, image_width) if image_height and image_width else (0, 0)
            tracks.append(
                Track(
                    track_id=int(track_msg.track_id),
                    bbox=bbox,
                    confidence=1.0,
                    room_id=room_id,
                    timestamp=track_timestamp,
                    image_size=image_size,
                )
            )
        if not tracks:
            return
        snapshots = self.pipeline.process_tracks(
            room_id,
            msg.header.frame_id,
            timestamp,
            tracks,
        )
        if not snapshots:
            return
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        batch_msg = face_snapshots_to_msg(snapshots, header)
        self.publisher.publish(batch_msg)
        for snapshot in snapshots:
            self.get_logger().info(
                (
                    "Updated face snapshot for track "
                    f"{snapshot.track_id} in room '{snapshot.room_id}' "
                    f"(quality={snapshot.quality.score:.3f})"
                )
            )

    def _handle_snapshot_request(self, request, response):
        room_id = request.room_id or self.room_id
        snapshot = self.pipeline.get_best_snapshot(room_id, int(request.track_id))
        if snapshot is None:
            response.success = False
            return response
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = snapshot.frame_id
        response.success = True
        response.snapshot = face_snapshot_to_msg(snapshot, header)
        return response


__all__ = ["FaceCaptureNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = FaceCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
