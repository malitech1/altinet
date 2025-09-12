"""Tests for the camera viewer node."""

from types import ModuleType
import importlib
import sys
import builtins


def test_initialization_without_cv2(monkeypatch) -> None:
    """Node should initialize even when ``cv2`` is missing."""

    # Stub required ROS message modules
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs.msg = ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.Image = object
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs.msg)

    # Minimal rclpy Node stub
    class DummyNode:
        def __init__(self, name):
            self.name = name

        def create_subscription(self, *args, **kwargs):
            pass

        class Logger:
            def warning(self, *args, **kwargs):
                pass

        def get_logger(self):  # pragma: no cover - trivial
            return self.Logger()

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    # Dummy cv_bridge module
    cv_bridge = ModuleType("cv_bridge")
    class DummyBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
            return msg
    cv_bridge.CvBridge = DummyBridge
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    # Force ImportError for cv2
    orig_import = builtins.__import__
    def fake_import(name, *args, **kwargs):
        if name == "cv2":
            raise ImportError
        return orig_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", fake_import)

    camera_viewer_node = importlib.import_module("altinet.nodes.camera_viewer_node")
    node = camera_viewer_node.CameraViewerNode()
    assert node.bridge is None
