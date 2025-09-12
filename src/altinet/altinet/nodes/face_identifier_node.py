import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String

from altinet.services.face_recognition import FaceRecognitionService
from altinet.services.face_tracker import FaceTracker

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - dependency might be missing
    CvBridge = None  # type: ignore

try:
    import face_recognition
except ImportError:  # pragma: no cover - dependency might be missing
    face_recognition = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore


class FaceIdentifierNode(Node):
    """Identify faces only when their identity is not already known."""

    def __init__(self) -> None:
        super().__init__("face_identifier_node")
        self.image_subscription = self.create_subscription(
            Image, "camera/image", self._image_callback, 10
        )
        self.faces_subscription = self.create_subscription(
            Int32MultiArray, "faces", self._faces_callback, 10
        )
        self.publisher = self.create_publisher(String, "identified_faces", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        self.recognition = FaceRecognitionService()
        self.tracker = FaceTracker(self.recognition)
        self._last_frame = None
        if not face_recognition:
            self.get_logger().warning(
                "face_recognition module not installed; identification disabled",
            )

    def _image_callback(self, msg: Image) -> None:
        if not self.bridge:
            return
        self._last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _faces_callback(self, msg: Int32MultiArray) -> None:
        if self._last_frame is None:
            return
        data = msg.data
        boxes = [tuple(data[i : i + 4]) for i in range(0, len(data), 4)]
        results = self.tracker.update(self._last_frame, boxes)
        if results:
            payload = [f"{name}:{conf:.2f}" for name, conf in results]
            self.publisher.publish(String(data=", ".join(payload)))
            self.get_logger().info(f"Identified faces: {payload}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceIdentifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
