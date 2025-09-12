import time
from pathlib import Path

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


# Default cascade shipped with the repository
REPO_CASCADE = (
    Path(__file__).resolve().parents[4]
    / "assets"
    / "haarcascades"
    / "haarcascade_frontalface_default.xml"
)


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
            cascade_param = self.declare_parameter("cascade_path", "").value
            xml_path = Path(cascade_param) if cascade_param else None
            candidates = []
            if xml_path:
                candidates.append(xml_path)
            data = getattr(cv2, "data", None)
            if data is not None:
                cascade_dir = Path(getattr(data, "haarcascades", data))
                candidates.append(cascade_dir / "haarcascade_frontalface_default.xml")
            candidates.append(REPO_CASCADE)
            chosen = next((p for p in candidates if p and p.exists()), None)
            if chosen:
                self.face_cascade = cv2.CascadeClassifier(str(chosen))
            else:
                self.get_logger().warning(
                    "Face cascade XML not found. Install OpenCV data or provide "
                    "--ros-args -p cascade_path:=/path/to/haarcascade_frontalface_default.xml"
                )
                self.face_cascade = None
        else:
            self.get_logger().warning("OpenCV not installed; face detection disabled")
            self.face_cascade = None
        self.face_present_since = None
        self.required_presence = 2.0
        self._warned_face_disabled = False

    def listener_callback(self, msg: Image) -> None:
        if not self.bridge:
            self.get_logger().warning("CvBridge not available; skipping frame")
            return
        if self.face_cascade is None:
            if not self._warned_face_disabled:
                self.get_logger().info(
                    "Received image frame but face detection is disabled"
                )
                self._warned_face_disabled = True
            return
        self._warned_face_disabled = False
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
