"""Camera node capturing frames from USB/RTSP sources."""

from __future__ import annotations

from typing import Optional

try:  # pragma: no cover - optional dependency in tests
    import cv2
except ImportError:  # pragma: no cover - executed when OpenCV missing
    cv2 = None

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - ROS optional
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover - executed in tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    Image = Header = CvBridge = None


class CameraStream:
    """Abstraction over ``cv2.VideoCapture`` with FPS control."""

    def __init__(self, source: str | int, fps: float) -> None:
        self.source = source
        self.fps = fps
        if cv2 is None:
            raise RuntimeError("OpenCV is required to use CameraStream")
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera source {source}")

    def read(self):
        success, frame = self.capture.read()
        if not success:
            raise RuntimeError("Failed to read frame from camera")
        return frame

    def release(self) -> None:
        self.capture.release()


class CameraNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node that publishes frames to ``/altinet/camera/<room_id>``."""

    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("source", 0)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("room_id", "room_1")
        source = self.get_parameter("source").value
        fps = float(self.get_parameter("fps").value)
        self.room_id = str(self.get_parameter("room_id").value)
        self.stream = CameraStream(source, fps)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, f"/altinet/camera/{self.room_id}", 10
        )
        self.timer = self.create_timer(1.0 / fps, self._publish_frame)

    def _publish_frame(self) -> None:
        frame = self.stream.read()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.room_id
        msg.header = header
        self.publisher.publish(msg)

    def destroy_node(self):  # pragma: no cover - ROS runtime only
        self.stream.release()
        return super().destroy_node()


__all__ = ["CameraStream", "CameraNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
