import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - dependency might be missing
    CvBridge = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore


class FaceDetectorNode(Node):
    """Detect faces in incoming images and publish bounding boxes."""

    def __init__(self) -> None:
        super().__init__("face_detector_node")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Int32MultiArray, "faces", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        self.face_cascade = (
            cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
            if cv2
            else None
        )
        if not cv2:
            self.get_logger().warning("OpenCV not installed; face detection disabled")

    def listener_callback(self, msg: Image) -> None:
        if not self.bridge or self.face_cascade is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        array = Int32MultiArray()
        array.data = faces.flatten().tolist()
        self.publisher.publish(array)
        if len(faces):
            self.get_logger().info(f"Detected {len(faces)} face(s)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
