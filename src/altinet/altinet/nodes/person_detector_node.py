import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from .camera_node import IMAGE_TOPIC

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - dependency might be missing
    CvBridge = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore


class PersonDetectorNode(Node):
    """Detect people in incoming images and publish bounding boxes."""

    def __init__(self) -> None:
        super().__init__("person_detector_node")
        self.subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Int32MultiArray, "people", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        if cv2:
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self._hog = hog
        else:  # pragma: no cover - dependency might be missing
            self.get_logger().warning("OpenCV not installed; person detection disabled")
            self._hog = None
        self._warned_disabled = False

    def listener_callback(self, msg: Image) -> None:
        if not self.bridge or self._hog is None:
            if not self._warned_disabled:
                self.get_logger().warning(
                    "Person detector inactive; received frame but cannot process"
                )
                self._warned_disabled = True
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rects, _ = self._hog.detectMultiScale(frame, winStride=(8, 8))
        if len(rects):
            array = Int32MultiArray()
            array.data = rects.flatten().tolist()
            self.publisher.publish(array)
            self.get_logger().info(f"Detected {len(rects)} person(s) in current frame")


def main(args=None) -> None:  # pragma: no cover - CLI entry point
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
