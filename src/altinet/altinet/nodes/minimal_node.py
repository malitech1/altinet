import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    """A simple ROS 2 node used as a placeholder."""

    def __init__(self) -> None:
        super().__init__("minimal_node")
        self.get_logger().info("Minimal Altinet node has started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
