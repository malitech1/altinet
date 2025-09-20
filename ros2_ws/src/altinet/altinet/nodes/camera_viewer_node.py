"""Simple ROS 2 node that displays frames from a camera topic."""

from __future__ import annotations

from typing import Optional, Sequence

try:  # pragma: no cover - optional dependency in tests
    import cv2
except ImportError:  # pragma: no cover - executed when OpenCV missing
    cv2 = None  # type: ignore

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - ROS optional
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge, CvBridgeError
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    Image = CvBridge = CvBridgeError = None  # type: ignore

_SHUTDOWN_KEYS: Sequence[int] = (ord("q"), 27)  # letter q and escape key


class CameraViewerNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node that renders incoming camera frames using OpenCV."""

    def __init__(self) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is required to use CameraViewerNode")
        super().__init__("camera_viewer_node")
        self.declare_parameter("room_id", "room_1")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("window_name", "")
        self.declare_parameter("wait_key_delay_ms", 1)
        self.declare_parameter("encoding", "bgr8")

        room_id_param = self.get_parameter("room_id").value
        self._room_id = str(room_id_param) if room_id_param else "room_1"

        camera_topic_param = self.get_parameter("camera_topic").value
        self._camera_topic = (
            str(camera_topic_param)
            if camera_topic_param
            else f"/altinet/camera/{self._room_id}"
        )

        window_param = self.get_parameter("window_name").value
        self._window_name = (
            str(window_param) if window_param else f"Altinet Camera ({self._room_id})"
        )

        encoding_param = self.get_parameter("encoding").value
        self._encoding = str(encoding_param) if encoding_param else "bgr8"

        delay_param = self.get_parameter("wait_key_delay_ms").value
        try:
            self._wait_key_delay = max(int(delay_param), 1)
        except (TypeError, ValueError):
            self._wait_key_delay = 1

        self._bridge = CvBridge()
        self._window_created = False
        self._window_available = True

        self._subscription = self.create_subscription(
            Image, self._camera_topic, self._on_image, 10
        )
        self.get_logger().info(
            f"Camera viewer subscribed to '{self._camera_topic}' with window "
            f"'{self._window_name}'"
        )

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

    def _on_image(self, msg: Image) -> None:
        if CvBridgeError is None:  # pragma: no cover - guards static analysis
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding=self._encoding)
        except CvBridgeError as exc:  # pragma: no cover - runtime only
            self.get_logger().warning("Failed to convert ROS image: %s", exc)
            return
        if frame is None:
            return
        self._ensure_window()
        if not self._window_available:
            return
        try:
            cv2.imshow(self._window_name, frame)
            key = cv2.waitKey(self._wait_key_delay) & 0xFF
        except cv2.error as exc:  # pragma: no cover - graphical back-end issues
            self._window_available = False
            self.get_logger().error("OpenCV error while displaying frame: %s", exc)
            return
        if key in _SHUTDOWN_KEYS and rclpy is not None:
            self.get_logger().info("Shutdown requested from keyboard input")
            rclpy.shutdown()

    def destroy_node(self):  # pragma: no cover - requires ROS runtime
        if cv2 is not None and self._window_created:
            try:
                cv2.destroyWindow(self._window_name)
            except cv2.error:
                pass
        return super().destroy_node()


__all__ = ["CameraViewerNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - runtime behaviour
        pass
    finally:
        node.destroy_node()
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        rclpy.shutdown()
