import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Topic used to publish camera images so that other nodes can subscribe
IMAGE_TOPIC = "camera/image"

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - dependency might be missing
    CvBridge = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore


class CameraNode(Node):
    """Publish images from the default camera device."""

    def __init__(self) -> None:
        super().__init__("camera_node")
        self.publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        self.cap = cv2.VideoCapture(0) if cv2 else None
        if not cv2 or not self.cap or not self.cap.isOpened():
            self.get_logger().warning("Camera not available; no images will be published")
        self.timer = self.create_timer(0.1, self.timer_callback)
        self._streaming_logged = False

    def timer_callback(self) -> None:
        if not self.cap or not self.cap.isOpened() or not self.bridge:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)
        if not self._streaming_logged:
            self.get_logger().info("Publishing video stream from default camera")
            self._streaming_logged = True

    def destroy_node(self) -> None:  # pragma: no cover - resource cleanup
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
