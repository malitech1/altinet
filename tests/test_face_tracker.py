"""Tests for the face tracker."""

import numpy as np

from altinet.services.face_tracker import FaceTracker


class FakeRecognitionService:
    """Stub recognition service recording calls."""

    def __init__(self):
        self.calls = 0

    def recognize(self, image):
        self.calls += 1
        return "Alice", 0.9


def test_tracker_invokes_recognition_only_for_new_faces() -> None:
    recognizer = FakeRecognitionService()
    tracker = FaceTracker(recognizer)
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    boxes = [(10, 10, 20, 20)]
    result1 = tracker.update(frame, boxes)
    assert result1 == [("Alice", 0.9)]
    assert recognizer.calls == 1

    moved = [(12, 12, 20, 20)]  # overlapping, same face
    result2 = tracker.update(frame, moved)
    assert result2 == [("Alice", 0.9)]
    assert recognizer.calls == 1

    tracker.update(frame, [])  # face leaves frame

    result3 = tracker.update(frame, boxes)
    assert result3 == [("Alice", 0.9)]
    assert recognizer.calls == 2
