"""Tests for the tracker pipeline."""

from datetime import datetime, timedelta

import pytest

from altinet.nodes.tracker_node import TrackerPipeline
from altinet.utils.types import BoundingBox, Detection


def make_detection(x: float, y: float, timestamp: datetime) -> Detection:
    return Detection(
        bbox=BoundingBox(x, y, 100.0, 200.0),
        confidence=0.9,
        room_id="living_room",
        frame_id="cam",
        timestamp=timestamp,
        image_size=(720, 1280),
    )


def test_tracker_preserves_identity():
    tracker = TrackerPipeline()
    t0 = datetime.utcnow()
    detections_frame1 = [make_detection(10.0, 20.0, t0)]
    tracks_frame1 = tracker.update(detections_frame1)
    assert len(tracks_frame1) == 1
    track_id = tracks_frame1[0].track_id
    assert tracks_frame1[0].image_size == (720, 1280)

    detections_frame2 = [make_detection(12.0, 22.0, t0 + timedelta(milliseconds=33))]
    tracks_frame2 = tracker.update(detections_frame2)
    assert len(tracks_frame2) == 1
    assert tracks_frame2[0].track_id == track_id


def test_tracker_estimates_velocity_per_second():
    tracker = TrackerPipeline()
    t0 = datetime.utcnow()
    tracker.update([make_detection(10.0, 20.0, t0)])
    t1 = t0 + timedelta(seconds=2)
    tracks = tracker.update([make_detection(14.0, 24.0, t1)])
    assert len(tracks) == 1
    assert tracks[0].velocity == pytest.approx((2.0, 2.0))


def test_tracker_predict_projects_tracks_forward():
    tracker = TrackerPipeline()
    t0 = datetime.utcnow()
    tracker.update([make_detection(10.0, 20.0, t0)])
    t1 = t0 + timedelta(seconds=1)
    tracks = tracker.update([make_detection(12.0, 24.0, t1)])
    now = t1 + timedelta(seconds=1)
    predicted = tracker.predict(tracks, now, max_seconds=1.5)
    assert len(predicted) == 1
    original = tracks[0]
    projected = predicted[0]
    assert projected is not original
    assert projected.timestamp == now
    assert original.bbox.x == pytest.approx(12.0)
    assert projected.bbox.x == pytest.approx(14.0)
