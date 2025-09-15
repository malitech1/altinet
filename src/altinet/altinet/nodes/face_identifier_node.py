import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String

from altinet.services.face_recognition import FaceRecognitionService
from altinet.services.person_tracker import PersonTracker
from altinet.settings import settings

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

REPO_USERS_DIR = Path(__file__).resolve().parents[3] / "assets" / "users"


class FaceIdentifierNode(Node):
    """Track and identify people using face recognition when available."""

    def __init__(self) -> None:
        super().__init__("face_identifier_node")
        self.image_subscription = self.create_subscription(
            Image, "camera/image", self._image_callback, 10
        )
        self.faces_subscription = self.create_subscription(
            Int32MultiArray, "faces", self._faces_callback, 10
        )
        self.people_subscription = self.create_subscription(
            Int32MultiArray, "people", self._people_callback, 10
        )
        self.publisher = self.create_publisher(String, "identified_people", 10)
        self.bridge = CvBridge() if CvBridge and cv2 else None
        self.recognition = FaceRecognitionService()
        self.tracker = PersonTracker()
        self._track_identities = {}
        self._last_frame = None
        self._last_people_boxes = []
        self._last_track_ids = []
        if settings.debug:
            try:  # pragma: no cover - depends on rclpy logging implementation
                from rclpy.logging import LoggingSeverity

                self.get_logger().set_level(LoggingSeverity.DEBUG)
            except Exception:
                pass
            self.get_logger().debug("Debug logging enabled")
        self._load_known_users()
        self.get_logger().info("FaceIdentifierNode initialised")
        if not face_recognition:
            self.get_logger().warning(
                "face_recognition module not installed; identification disabled",
            )

    def _image_callback(self, msg: Image) -> None:
        if not self.bridge:
            self.get_logger().debug("No CvBridge available; skipping frame")
            return
        self._last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.get_logger().debug(
            f"Received image frame {msg.width}x{msg.height} from camera",
        )

    def _faces_callback(self, msg: Int32MultiArray) -> None:
        if self._last_frame is None:
            self.get_logger().debug("No image frame available; skipping face data")
            return
        if face_recognition is None:
            self.get_logger().debug("face_recognition module missing; cannot process faces")
            return
        data = msg.data
        boxes = [tuple(data[i : i + 4]) for i in range(0, len(data), 4)]
        self.get_logger().debug(f"Processing {len(boxes)} face boxes")
        # First perform identification on the current frame
        results = []
        for box in boxes:
            x, y, w, h = box
            face_img = np.ascontiguousarray(self._last_frame[y : y + h, x : x + w])
            identity, confidence = self.recognition.recognize(face_img)
            results.append((box, identity, confidence))
        # Then update the body tracker and associate identities
        track_ids: list[int] = []
        if self._last_people_boxes:
            track_ids = self.tracker.update(self._last_frame, self._last_people_boxes)
            self.get_logger().debug(f"Updated tracker with {len(track_ids)} ids")
        else:
            self.get_logger().debug("No people boxes available for tracking")
        self._last_track_ids = track_ids
        payload = []
        for box, identity, confidence in results:
            best_iou = 0.0
            best_id = None
            for tid, pbox in zip(track_ids, self._last_people_boxes):
                iou = PersonTracker._iou(box, pbox)
                if iou > best_iou:
                    best_iou, best_id = iou, tid
            if best_id is None or best_iou == 0.0:
                continue
            self._track_identities[best_id] = (identity, confidence)
            payload.append(f"{best_id}:{identity}:{confidence:.2f}")
        if payload:
            self.publisher.publish(String(data=", ".join(payload)))
            self.get_logger().info(f"Identified people: {payload}")
        else:
            self.get_logger().debug("No faces identified in current frame")

    def _people_callback(self, msg: Int32MultiArray) -> None:
        data = msg.data
        self._last_people_boxes = [
            tuple(data[i : i + 4]) for i in range(0, len(data), 4)
        ]
        self.get_logger().debug(
            f"Received {len(self._last_people_boxes)} people boxes"
        )

    def _load_known_users(self) -> None:
        """Train recognition on any cached user photos."""
        if face_recognition is None:
            self.get_logger().debug(
                "face_recognition module missing; skipping user training"
            )
            return
        users_dir = REPO_USERS_DIR
        if not users_dir.exists():
            self.get_logger().warning(f"Users directory '{users_dir}' does not exist")
            return
        self.get_logger().debug(f"Loading known users from {users_dir}")
        trained_any = False
        for user_dir in users_dir.iterdir():
            if not user_dir.is_dir():
                continue
            metadata = user_dir / "metadata.json"
            name = user_dir.name
            if metadata.exists():
                try:
                    with metadata.open("r", encoding="utf-8") as fh:
                        name = json.load(fh).get("name", name)
                except Exception:
                    pass
            photos_dir = user_dir / "photos"
            if not photos_dir.exists():
                continue
            photo_files = list(photos_dir.glob("*.jpg"))
            if not photo_files:
                continue
            self.get_logger().debug(
                f"Training user '{name}' with {len(photo_files)} photos"
            )
            count = 0
            for photo in photo_files:
                image = face_recognition.load_image_file(photo)
                self.recognition.train(image, name)
                count += 1
            if count:
                trained_any = True
                self.get_logger().info(
                    f"Trained user '{name}' with {count} photos",
                )
        if not trained_any:
            self.get_logger().warning(
                f"No valid user directories found in '{users_dir}'",
            )
        else:
            self.get_logger().debug("Finished loading known users")


def main(args=None) -> None:
    from altinet.settings import configure_logging

    configure_logging()
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
