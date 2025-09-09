import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

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
    """Identify known faces from incoming images."""

    def __init__(self) -> None:
        super().__init__("face_identifier_node")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(String, "identified_faces", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        self.known_encodings = []
        self.known_names = []
        if not face_recognition:
            self.get_logger().warning(
                "face_recognition module not installed; identification disabled"
            )

    def listener_callback(self, msg: Image) -> None:
        if not (self.bridge and face_recognition):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        encodings = face_recognition.face_encodings(frame)
        names = []
        for encoding in encodings:
            matches = face_recognition.compare_faces(self.known_encodings, encoding)
            name = "Unknown"
            if True in matches:
                index = matches.index(True)
                name = self.known_names[index]
            names.append(name)
        if names:
            self.publisher.publish(String(data=", ".join(names)))
            self.get_logger().info(f"Identified faces: {names}")


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
