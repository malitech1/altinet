"""Tests for the face detector node."""
"""Tests for the face detector node."""

from types import ModuleType
import importlib
import sys


def _stub_ros_modules(monkeypatch):
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



def test_face_detector_subscribes_to_camera_topic(monkeypatch) -> None:
    """Face detector should listen on the camera image topic."""

    _stub_ros_modules(monkeypatch)

    captured = {}

    class DummyNode:
        def __init__(self, name):
            pass

        def create_subscription(self, msg, topic, callback, qos):
            captured["topic"] = topic

        def create_publisher(self, *args, **kwargs):
            pass

        class Logger:
            def warning(self, *args, **kwargs):
                pass

            def info(self, *args, **kwargs):
                pass

        def get_logger(self):  # pragma: no cover - trivial
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

    # cv2 not available to skip cascade loading
    monkeypatch.setitem(sys.modules, "cv2", None)

    monkeypatch.delitem(sys.modules, "altinet.nodes.camera_node", raising=False)
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    face_detector_node.FaceDetectorNode()
    assert captured["topic"] == face_detector_node.IMAGE_TOPIC


def test_initialization_without_cascade_file(monkeypatch, tmp_path) -> None:
    """Node should initialize even when the cascade XML is missing."""

    _stub_ros_modules(monkeypatch)

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

    # Dummy cv2 with ``data`` attribute pointing to non-existent cascades
    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):  # pragma: no cover - unused
            return []

    cv2.CascadeClassifier = DummyCascade

    class Data:
        pass

    cv2.data = Data()
    cv2.data.haarcascades = str(tmp_path / "does_not_exist")
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    monkeypatch.delitem(sys.modules, "altinet.nodes.camera_node", raising=False)
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert node.face_cascade is None


def test_initialization_with_cv2_data_missing_and_valid_path(monkeypatch, tmp_path) -> None:
    """Loads classifier from provided path when cv2 lacks bundled data."""

    _stub_ros_modules(monkeypatch)

    class DummyNode:
        parameters = {}

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

        def get_logger(self):
            return self.Logger()

        def declare_parameter(self, name, default):
            class Param:
                def __init__(self, value):
                    self.value = value

            return Param(self.parameters.get(name, default))

    # Provide path to existing cascade file
    xml_file = tmp_path / "cascade.xml"
    xml_file.write_text("test")
    DummyNode.parameters = {"cascade_path": str(xml_file)}

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):
            return []

    cv2.CascadeClassifier = DummyCascade
    cv2.data = None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    monkeypatch.delitem(sys.modules, "altinet.nodes.camera_node", raising=False)
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert isinstance(node.face_cascade, DummyCascade)
    assert node.face_cascade.xml_path == str(xml_file)


def test_initialization_with_cv2_data_missing_and_no_path(monkeypatch, tmp_path) -> None:
    """When cv2.data is missing and no path provided, detection disabled."""

    _stub_ros_modules(monkeypatch)

    class DummyNode:
        parameters = {}

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

        def get_logger(self):
            return self.Logger()

        def declare_parameter(self, name, default):
            class Param:
                def __init__(self, value):
                    self.value = value

            return Param(self.parameters.get(name, default))

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):
            return []

    cv2.CascadeClassifier = DummyCascade
    cv2.data = None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    monkeypatch.delitem(sys.modules, "altinet.nodes.camera_node", raising=False)
    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert node.face_cascade is None


def test_initialization_with_cv2_data_missing_and_valid_path(monkeypatch, tmp_path) -> None:
    """Loads classifier from provided path when cv2 lacks bundled data."""

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
        parameters = {}

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

        def get_logger(self):
            return self.Logger()

        def declare_parameter(self, name, default):
            class Param:
                def __init__(self, value):
                    self.value = value

            return Param(self.parameters.get(name, default))

    # Provide path to existing cascade file
    xml_file = tmp_path / "cascade.xml"
    xml_file.write_text("test")
    DummyNode.parameters = {"cascade_path": str(xml_file)}

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):
            return []

    cv2.CascadeClassifier = DummyCascade
    cv2.data = None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert isinstance(node.face_cascade, DummyCascade)
    assert node.face_cascade.xml_path == str(xml_file)


def test_initialization_with_cv2_data_missing_and_no_path(monkeypatch, tmp_path) -> None:
    """When cv2.data is missing and no path provided, detection disabled."""

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
        parameters = {}

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

        def get_logger(self):
            return self.Logger()

        def declare_parameter(self, name, default):
            class Param:
                def __init__(self, value):
                    self.value = value

            return Param(self.parameters.get(name, default))

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)

    cv2 = ModuleType("cv2")

    class DummyCascade:
        def __init__(self, xml_path):
            self.xml_path = xml_path

        def detectMultiScale(self, *args, **kwargs):
            return []

    cv2.CascadeClassifier = DummyCascade
    cv2.data = None
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    monkeypatch.delitem(sys.modules, "altinet.nodes.face_detector_node", raising=False)
    face_detector_node = importlib.import_module("altinet.nodes.face_detector_node")

    node = face_detector_node.FaceDetectorNode()
    assert node.face_cascade is None

