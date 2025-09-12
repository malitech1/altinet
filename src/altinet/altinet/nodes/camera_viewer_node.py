import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - dependency might be missing
    CvBridge = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore


class CameraViewerNode(Node):
    """Display images from the camera node in a window."""

    def __init__(self) -> None:
        super().__init__("camera_viewer_node")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.listener_callback, 10
        )
        self.bridge = CvBridge() if CvBridge and cv2 else None
        if not cv2 or not self.bridge:
            self.get_logger().warning("OpenCV not available; live view disabled")
        else:  # pragma: no cover - GUI interaction
            cv2.namedWindow("Camera")

    def listener_callback(self, msg: Image) -> None:  # pragma: no cover - requires GUI
        if not self.bridge or not cv2:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

    def destroy_node(self) -> None:  # pragma: no cover - resource cleanup
        if cv2:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
