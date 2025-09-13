"""Tests for the face identifier node."""

from types import ModuleType
import importlib
import json
import sys


def _stub_ros(monkeypatch):
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs.msg = ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.Image = object
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs.msg)

    std_msgs = ModuleType("std_msgs")
    std_msgs.msg = ModuleType("std_msgs.msg")
    std_msgs.msg.Int32MultiArray = object
    std_msgs.msg.String = object
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs.msg)

    class DummyNode:
        def __init__(self, name):
            self.name = name
            self._logger = self.Logger()

        def create_subscription(self, *a, **k):
            pass

        def create_publisher(self, *a, **k):
            pass

        class Logger:
            def warning(self, *a, **k):
                pass

            def info(self, *a, **k):
                pass

        def get_logger(self):
            return self._logger

    rclpy = ModuleType("rclpy")
    rclpy.node = ModuleType("rclpy.node")
    rclpy.node.Node = DummyNode
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)


def test_load_known_users_on_startup(monkeypatch, tmp_path):
    _stub_ros(monkeypatch)

    # prepare dummy user data
    user_dir = tmp_path / "alice"
    photos_dir = user_dir / "photos"
    photos_dir.mkdir(parents=True)
    photo = photos_dir / "img.jpg"
    photo.write_text("dummy")
    (user_dir / "metadata.json").write_text(json.dumps({"name": "Alice"}))

    # stub external deps
    face_rec = ModuleType("face_recognition")
    face_rec.load_image_file = lambda p: f"img:{p}"
    monkeypatch.setitem(sys.modules, "face_recognition", face_rec)

    cv2 = ModuleType("cv2")
    monkeypatch.setitem(sys.modules, "cv2", cv2)

    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge)

    class DummyService:
        def __init__(self):
            self.trained = []

        def train(self, image, name, confidence=1.0):
            self.trained.append((image, name))

    import altinet.services.face_recognition as fr
    monkeypatch.setattr(fr, "FaceRecognitionService", DummyService)

    monkeypatch.delitem(sys.modules, "altinet.nodes.face_identifier_node", raising=False)
    face_module = importlib.import_module("altinet.nodes.face_identifier_node")
    monkeypatch.setattr(face_module, "REPO_USERS_DIR", tmp_path, raising=False)

    node = face_module.FaceIdentifierNode()
    assert node.recognition.trained == [(f"img:{photo}", "Alice")]
