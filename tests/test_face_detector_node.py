"""Tests for the face detector node."""

from types import ModuleType
import importlib
import sys


def _stub_ros(monkeypatch, parameters=None):
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

    class DummyNode:
        params = parameters or {}

        def __init__(self, name):
            self.name = name
            self._logger = self.Logger()

        def create_subscription(self, *args, **kwargs):
            pass

        def create_publisher(self, *args, **kwargs):
            pass

        class Logger:
            def __init__(self):
                self.infos = []
                self.warnings = []

            def warning(self, msg, *args, **kwargs):
                self.warnings.append(msg)

            def info(self, msg, *args, **kwargs):
                self.infos.append(msg)

        def get_logger(self):
            return self._logger

        def declare_parameter(self, name, default):
            class Param:
                def __init__(self, value):
                    self.value = value

            return Param(self.params.get(name, default))

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)


def test_listener_callback_logs_once(monkeypatch):
    """Disabled face detection should log only once."""

    _stub_ros(monkeypatch)

    # stub cv2 and cv_bridge
    cv2 = ModuleType("cv2")
    cv2.data = None
    cv2.CascadeClassifier = lambda path: None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    cv_bridge = ModuleType("cv_bridge")
    class Bridge:
        pass
    cv_bridge.CvBridge = Bridge
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    # import module
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_module = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_module.FaceDetectorNode()
    node.face_cascade = None  # ensure disabled
    msg = object()
    logger = node.get_logger()

    node.listener_callback(msg)
    node.listener_callback(msg)

    assert logger.infos.count("Received image frame but face detection is disabled") == 1


def test_initialization_uses_packaged_cascade(monkeypatch):
    """Falls back to packaged cascade when cv2.data missing."""

    _stub_ros(monkeypatch)

    cv2 = ModuleType("cv2")
    cv2.data = None
    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path
    cv2.CascadeClassifier = DummyCascade
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_module = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_module.FaceDetectorNode()
    assert isinstance(node.face_cascade, DummyCascade)
    assert node.face_cascade.xml_path == str(face_module.PACKAGE_CASCADE)


def test_initialization_without_any_cascade(monkeypatch, tmp_path):
    """Disables detection when no cascade files are available."""

    _stub_ros(monkeypatch)

    cv2 = ModuleType("cv2")
    cv2.data = None
    cv2.CascadeClassifier = lambda path: None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_module = importlib.import_module("altinet.nodes.face_detector_node")
    monkeypatch.setattr(face_module, "PACKAGE_CASCADE", tmp_path / "missing.xml")

    node = face_module.FaceDetectorNode()
    assert node.face_cascade is None


def test_face_detector_subscribes_to_camera_topic(monkeypatch):
    """Face detector should listen on the camera image topic."""

    recorded = {}

    def custom_stub(monkeypatch):
        _stub_ros(monkeypatch)
        class DummyNode:  # override to capture topic
            params = {}
            def __init__(self, name):
                pass
            def create_subscription(self, msg, topic, callback, qos):
                recorded["topic"] = topic
            def create_publisher(self, *a, **k):
                pass
            class Logger:
                def warning(self, *a, **k):
                    pass
                def info(self, *a, **k):
                    pass
            def get_logger(self):
                return self.Logger()
            def declare_parameter(self, name, default):
                class Param:
                    def __init__(self, value):
                        self.value = value
                return Param(default)
        rclpy = ModuleType("rclpy")
        rclpy.node = ModuleType("rclpy.node")
        rclpy.node.Node = DummyNode
        monkeypatch.setitem(sys.modules, "rclpy", rclpy)
        monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    custom_stub(monkeypatch)

    cv2 = ModuleType("cv2")
    cv2.data = None
    cv2.CascadeClassifier = lambda path: None
    monkeypatch.setitem(sys.modules, "cv2", cv2)
    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    monkeypatch.delitem(sys.modules, "altinet.nodes.camera_node", raising=False)
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_module = importlib.import_module("altinet.nodes.face_detector_node")
    face_module.FaceDetectorNode()
    assert recorded["topic"] == face_module.IMAGE_TOPIC
