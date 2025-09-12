import time

import rclpy
from rclpy.node import Node
from pathlib import Path

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


class FaceDetectorNode(Node):
    """Detect faces in incoming images and publish bounding boxes."""

    def __init__(self) -> None:
        super().__init__("face_detector_node")
        self.subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Int32MultiArray, "faces", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        if cv2:
            # Declare parameter before checking cv2.data so a user-provided path
            # can be used even when OpenCV lacks bundled cascade files.
            cascade_param = self.declare_parameter("cascade_path", "").value
            xml_path = Path(cascade_param) if cascade_param else None
            data = getattr(cv2, "data", None)
            if data is None:
                if xml_path and xml_path.exists():
                    self.face_cascade = cv2.CascadeClassifier(str(xml_path))
                else:
                    self.get_logger().warning(
                        "cv2.data not available; provide cascade_path parameter"
                    )
                    self.face_cascade = None
            else:
                cascade_dir = Path(getattr(data, "haarcascades", data))
                default_xml = cascade_dir / "haarcascade_frontalface_default.xml"
                if xml_path is None:
                    xml_path = default_xml
                if xml_path.exists():
                    self.face_cascade = cv2.CascadeClassifier(str(xml_path))
                else:
                    self.get_logger().warning(
                        f"Face cascade XML not found at {xml_path}; face detection disabled"
                    )
                    self.face_cascade = None
        else:
            self.get_logger().warning("OpenCV not installed; face detection disabled")
            self.face_cascade = None
        self.face_present_since = None
        self.required_presence = 2.0

    def listener_callback(self, msg: Image) -> None:
        if not self.bridge:
            self.get_logger().warning("CvBridge not available; skipping frame")
            return
        if self.face_cascade is None:
            self.get_logger().info("Received image frame but face detection is disabled")
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        now = time.time()
        if len(faces):
            if self.face_present_since is None:
                self.face_present_since = now
            duration = now - self.face_present_since
            if duration >= self.required_presence:
                array = Int32MultiArray()
                array.data = faces.flatten().tolist()
                self.publisher.publish(array)
                self.get_logger().info(
                    f"Detected {len(faces)} face(s) with confidence {duration:.2f}s"
                )
        else:
            self.face_present_since = None


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
