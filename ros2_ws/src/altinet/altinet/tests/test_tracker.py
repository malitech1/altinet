"""Tests for the tracker pipeline."""

from datetime import datetime, timedelta

from altinet.nodes.tracker_node import TrackerPipeline
from altinet.utils.types import BoundingBox, Detection


def make_detection(x: float, y: float, timestamp: datetime) -> Detection:
    return Detection(
        bbox=BoundingBox(x, y, 100.0, 200.0),
        confidence=0.9,
        room_id="living_room",
        frame_id="cam",
        timestamp=timestamp,
        image_size=(1080, 1920),
    )


def test_tracker_preserves_identity():
    tracker = TrackerPipeline()
    t0 = datetime.utcnow()
    detections_frame1 = [make_detection(10.0, 20.0, t0)]
    tracks_frame1 = tracker.update(detections_frame1)
    assert len(tracks_frame1) == 1
    track_id = tracks_frame1[0].track_id

    detections_frame2 = [make_detection(12.0, 22.0, t0 + timedelta(milliseconds=33))]
    tracks_frame2 = tracker.update(detections_frame2)
    assert len(tracks_frame2) == 1
    assert tracks_frame2[0].track_id == track_id
