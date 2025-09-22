"""Visualization node that overlays detections and tracks on camera images."""

from __future__ import annotations

import threading
from typing import List, Optional, Sequence, Tuple, TypeVar

try:  # pragma: no cover - optional dependency in tests
    import cv2
except ImportError:  # pragma: no cover - executed when OpenCV missing
    cv2 = None

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - ROS optional
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from altinet.msg import PersonDetection as PersonDetectionMsg
    from altinet.msg import PersonDetections as PersonDetectionsMsg
    from altinet.msg import PersonTrack as PersonTrackMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Time = None
    Image = (
        PersonDetectionMsg
    ) = PersonDetectionsMsg = PersonTrackMsg = PersonTracksMsg = CvBridge = None


BBox = Tuple[int, int, int, int]
_T = TypeVar("_T")

_SHUTDOWN_KEYS: Sequence[int] = (ord("q"), 27)  # letter q and escape key


def _color_from_id(identifier: int) -> Tuple[int, int, int]:
    """Return a BGR colour tuple that is deterministic for ``identifier``."""

    base = abs(int(identifier))
    hashed = (base * 2654435761) & 0xFFFFFFFF
    blue = 50 + hashed % 206
    green = 50 + (hashed >> 8) % 206
    red = 50 + (hashed >> 16) % 206
    return int(blue), int(green), int(red)


def _bbox_from_dimensions(
    x: float, y: float, w: float, h: float, width: int, height: int
) -> Optional[BBox]:
    """Clamp the bounding box to the image size and return integer corners."""

    if w <= 0 or h <= 0:
        return None
    x1 = int(round(x))
    y1 = int(round(y))
    x2 = int(round(x + w))
    y2 = int(round(y + h))
    if x2 <= 0 or y2 <= 0 or x1 >= width or y1 >= height:
        return None
    x1 = max(x1, 0)
    y1 = max(y1, 0)
    x2 = min(x2, width - 1)
    y2 = min(y2, height - 1)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


class VisualizerNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node that publishes annotated camera images for RViz."""

    def __init__(self) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is required to use VisualizerNode")
        super().__init__("visualizer_node")
        self.declare_parameter("room_id", "room_1")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("detections_topic", "/altinet/person_detections")
        self.declare_parameter("tracks_topic", "/altinet/person_tracks")
        self.declare_parameter("draw_detections", True)
        self.declare_parameter("draw_tracks", True)
        self.declare_parameter("timestamp_tolerance", 0.2)
        self.declare_parameter("display_window", True)
        self.declare_parameter("window_name", "")
        self.declare_parameter("wait_key_delay_ms", 1)

        room_id_param = self.get_parameter("room_id").value
        self.room_id = str(room_id_param) if room_id_param is not None else "room_1"

        camera_topic_param = self.get_parameter("camera_topic").value
        camera_topic = str(camera_topic_param) if camera_topic_param else ""
        if not camera_topic:
            camera_topic = f"/altinet/camera/{self.room_id}"

        detections_topic_param = self.get_parameter("detections_topic").value
        self._detections_topic = (
            str(detections_topic_param)
            if detections_topic_param
            else "/altinet/person_detections"
        )

        tracks_topic_param = self.get_parameter("tracks_topic").value
        self._tracks_topic = (
            str(tracks_topic_param) if tracks_topic_param else "/altinet/person_tracks"
        )

        self._draw_detections = bool(self.get_parameter("draw_detections").value)
        self._draw_tracks = bool(self.get_parameter("draw_tracks").value)
        self._display_window = bool(self.get_parameter("display_window").value)

        tolerance_param = self.get_parameter("timestamp_tolerance").value
        try:
            tolerance = max(float(tolerance_param), 0.0)
        except (TypeError, ValueError):
            tolerance = 0.2
        self._timestamp_tolerance_ns = int(tolerance * 1e9)

        self._bridge = CvBridge()
        self._annotated_publisher = self.create_publisher(
            Image, f"/altinet/visualizer/{self.room_id}/annotated", 10
        )

        window_param = self.get_parameter("window_name").value
        self._window_name = (
            str(window_param)
            if window_param
            else f"Altinet Visualizer ({self.room_id})"
        )
        delay_param = self.get_parameter("wait_key_delay_ms").value
        try:
            self._wait_key_delay = max(int(delay_param), 1)
        except (TypeError, ValueError):
            self._wait_key_delay = 1

        self._window_created = False
        self._window_available = True
        if self._display_window:
            self.get_logger().info(
                f"Displaying annotated frames in window '{self._window_name}'"
            )

        self._lock = threading.Lock()
        self._last_detections: List[PersonDetectionMsg] = []
        self._last_tracks: List[PersonTrackMsg] = []
        self._detections_stamp: Optional[Time] = None
        self._tracks_stamp: Optional[Time] = None

        self._image_subscription = self.create_subscription(
            Image, camera_topic, self._on_image, 10
        )
        self.get_logger().info(
            f"Subscribed to camera topic '{camera_topic}'"
        )

        if self._draw_detections:
            self._detections_subscription = self.create_subscription(
                PersonDetectionsMsg, self._detections_topic, self._on_detections, 10
            )
            self.get_logger().info(
                f"Listening for detections on '{self._detections_topic}'"
            )
        else:
            self._detections_subscription = None

        if self._draw_tracks:
            self._tracks_subscription = self.create_subscription(
                PersonTracksMsg, self._tracks_topic, self._on_tracks, 10
            )
            self.get_logger().info(
                f"Listening for tracks on '{self._tracks_topic}'"
            )
        else:
            self._tracks_subscription = None

        self._detection_colour: Tuple[int, int, int] = (0, 255, 255)  # BGR yellow
        self._track_text_colour: Tuple[int, int, int] = (255, 255, 255)

    def _on_detections(self, msg: PersonDetectionsMsg) -> None:
        if msg.room_id and msg.room_id != self.room_id:
            return
        if Time is None:
            return
        with self._lock:
            self._last_detections = list(msg.detections)
            self._detections_stamp = Time.from_msg(msg.header.stamp)

    def _on_tracks(self, msg: PersonTracksMsg) -> None:
        if msg.room_id and msg.room_id != self.room_id:
            return
        if Time is None:
            return
        with self._lock:
            self._last_tracks = list(msg.tracks)
            self._tracks_stamp = Time.from_msg(msg.header.stamp)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame is None:
            return
        height, width = frame.shape[:2]

        with self._lock:
            detections = list(self._last_detections)
            detection_stamp = self._detections_stamp
            tracks = list(self._last_tracks)
            tracks_stamp = self._tracks_stamp

        image_stamp = Time.from_msg(msg.header.stamp) if Time is not None else None
        if image_stamp is not None:
            detections = self._filter_by_timestamp(detections, detection_stamp, image_stamp)
            tracks = self._filter_by_timestamp(tracks, tracks_stamp, image_stamp)

        annotated = frame.copy()
        if self._draw_detections and detections:
            self._draw_detection_boxes(annotated, detections, width, height)
        if self._draw_tracks and tracks:
            self._draw_track_boxes(annotated, tracks, width, height)

        annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self._annotated_publisher.publish(annotated_msg)
        if self._display_window:
            self._show_annotated_image(annotated)

    def _ensure_window(self) -> None:
        if self._window_created or not self._window_available:
            return
        try:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            self._window_created = True
        except cv2.error as exc:  # pragma: no cover - graphical back-end issues
            self._window_available = False
            self.get_logger().error(
                "Unable to create OpenCV window '%s': %s", self._window_name, exc
            )

    def _show_annotated_image(self, image) -> None:
        self._ensure_window()
        if not self._window_available:
            return
        try:
            cv2.imshow(self._window_name, image)
            key = cv2.waitKey(self._wait_key_delay) & 0xFF
        except cv2.error as exc:  # pragma: no cover - graphical back-end issues
            self._window_available = False
            self.get_logger().error(
                "OpenCV error while displaying annotated frame: %s", exc
            )
            return
        if key in _SHUTDOWN_KEYS and rclpy is not None:
            self.get_logger().info("Shutdown requested from keyboard input")
            rclpy.shutdown()

    def destroy_node(self):  # pragma: no cover - requires ROS runtime
        if self._display_window and cv2 is not None and self._window_created:
            try:
                cv2.destroyWindow(self._window_name)
            except cv2.error:
                pass
        return super().destroy_node()

    def _filter_by_timestamp(
        self,
        entities: Sequence[_T],
        stamp: Optional[Time],
        image_stamp: Time,
    ) -> List[_T]:
        if stamp is None:
            return []
        delta_ns = int((image_stamp - stamp).nanoseconds)
        if delta_ns < 0:
            delta_ns = -delta_ns
        if delta_ns > self._timestamp_tolerance_ns:
            return []
        return list(entities)

    def _draw_detection_boxes(
        self,
        image,
        detections: Sequence[PersonDetectionMsg],
        width: int,
        height: int,
    ) -> None:
        for detection in detections:
            bbox = _bbox_from_dimensions(
                detection.x, detection.y, detection.w, detection.h, width, height
            )
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            cv2.rectangle(image, (x1, y1), (x2, y2), self._detection_colour, 2)
            label = f"{detection.conf:.2f}"
            cv2.putText(
                image,
                label,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                self._detection_colour,
                1,
                cv2.LINE_AA,
            )

    def _draw_track_boxes(
        self, image, tracks: Sequence[PersonTrackMsg], width: int, height: int
    ) -> None:
        for track in tracks:
            bbox = _bbox_from_dimensions(
                track.x, track.y, track.w, track.h, width, height
            )
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            colour = _color_from_id(track.track_id)
            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
            label = f"ID {track.track_id}"
            cv2.putText(
                image,
                label,
                (x1, min(y2 + 15, height - 1)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                self._track_text_colour,
                1,
                cv2.LINE_AA,
            )


__all__ = ["VisualizerNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
