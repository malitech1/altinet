"""Tests for the face recognition service."""

import logging

from altinet.services.face_recognition import FaceRecognitionService


class FakeEncoder:
    """Minimal encoder that echoes the image as its encoding."""

    @staticmethod
    def face_encodings(image):
        return [image]

    @staticmethod
    def compare_faces(known, encoding):
        return [k == encoding for k in known]


class FakeIdentifier:
    """Stub identifier that records calls and returns a fixed identity."""

    def __init__(self):
        self.calls = 0

    def __call__(self, encoding):
        self.calls += 1
        return "Alice", 0.9


def test_identifier_called_only_for_unknown_faces() -> None:
    backend = FakeIdentifier()
    service = FaceRecognitionService(encoder=FakeEncoder(), identifier=backend)
    first = service.recognize("face1")
    assert first == ("Alice", 0.9)
    assert backend.calls == 1

    second = service.recognize("face1")
    assert second == ("Alice", 0.9)
    assert backend.calls == 1


def test_training_adds_known_identity() -> None:
    service = FaceRecognitionService(encoder=FakeEncoder())
    service.train("face2", "Bob", 0.8)
    result = service.recognize("face2")
    assert result == ("Bob", 0.8)


def test_recognize_logs_identity(caplog) -> None:
    service = FaceRecognitionService(encoder=FakeEncoder())
    service.train("face3", "Charlie", 0.7)
    with caplog.at_level(logging.INFO):
        service.recognize("face3")
    assert "Checked face identity: Charlie" in caplog.text


def test_delete_user_removes_identity() -> None:
    service = FaceRecognitionService(encoder=FakeEncoder())
    service.train("face4", "Dave", 0.6)
    service.delete_user("Dave")
    # After deletion, the face should be unknown again
    assert service.recognize("face4") == ("Unknown", 0.0)
