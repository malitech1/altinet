"""Tests for the face detector node."""

from types import ModuleType
import importlib
import sys


def test_initialization_without_cv2_data(monkeypatch, tmp_path) -> None:
    """Node should initialize even when ``cv2.data`` is missing."""

    # Stub required ROS message modules
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs.msg = ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.Image = object
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs.msg)

    std_msgs = ModuleType("std_msgs")
    std_msgs.msg = ModuleType("std_msgs.msg")
    std_msgs.msg.Int32MultiArray = object
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs.msg)

    # Minimal rclpy Node stub
    class DummyNode:
        def __init__(self, name):
            self.name = name

        def create_subscription(self, *args, **kwargs):
            pass

        def create_publisher(self, *args, **kwargs):
            pass

        class Logger:
            def warning(self, *args, **kwargs):
                pass

            def info(self, *args, **kwargs):
                pass

        def get_logger(self):  # pragma: no cover - trivial
            return self.Logger()

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    # Dummy cv2 without ``data`` attribute
    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):  # pragma: no cover - unused
            return []

    cv2.CascadeClassifier = DummyCascade
    cv2.__file__ = str(tmp_path / "cv2" / "__init__.py")
    (tmp_path / "cv2" / "data").mkdir(parents=True)
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert node.face_cascade is None

