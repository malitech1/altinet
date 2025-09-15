"""Tests for the person tracker."""

import numpy as np

from altinet.services.person_tracker import PersonTracker


def test_tracker_reidentifies_after_disappearance() -> None:
    tracker = PersonTracker()
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[10:30, 10:30] = [10, 20, 30]

    first = tracker.update(frame, [(10, 10, 20, 20)])
    assert first == [1]

    tracker.update(frame, [])

    frame2 = np.zeros_like(frame)
    frame2[30:50, 30:50] = [10, 20, 30]
    second = tracker.update(frame2, [(30, 30, 20, 20)])

    assert second == first
