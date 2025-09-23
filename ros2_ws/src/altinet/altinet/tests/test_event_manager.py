"""Tests for the event manager logic."""

from datetime import datetime, timedelta

import pytest

from altinet.nodes.event_manager_node import EventManager, EventManagerConfig
from altinet.utils.geometry import RoomGeometry
from altinet.utils.types import BoundingBox, Track


class DummyGeometry(RoomGeometry):
    def __init__(self, room_id: str):
        super().__init__(room_id=room_id, homography=None, roi=None)


def make_track(
    track_id: int,
    x: float,
    y: float,
    timestamp: datetime,
    image_size=(720, 1280),
) -> Track:
    return Track(
        track_id=track_id,
        bbox=BoundingBox(x, y, 50.0, 80.0),
        confidence=0.9,
        room_id="living_room",
        timestamp=timestamp,
        image_size=image_size,
    )


def test_event_manager_generates_entry_exit_and_position_change():
    manager = EventManager(
        EventManagerConfig(presence_timeout_s=0.1, position_threshold=0.01),
        lambda room: DummyGeometry(room),
    )
    t0 = datetime.utcnow()
    events, presence = manager.update([make_track(1, 10.0, 20.0, t0)])
    entry_event = next(event for event in events if event.type == "ENTRY")
    centroid = entry_event.payload["centroid"]
    assert centroid[0] == pytest.approx((10.0 + 25.0) / 1280.0)
    assert centroid[1] == pytest.approx((20.0 + 40.0) / 720.0)
    assert presence[0].count == 1

    events, _ = manager.update(
        [make_track(1, 40.0, 20.0, t0 + timedelta(milliseconds=33))]
    )
    move_event = next(event for event in events if event.type == "POSITION_CHANGE")
    moved_centroid = move_event.payload["centroid"]
    previous_centroid = move_event.payload["previous"]
    assert previous_centroid[0] == pytest.approx(centroid[0])
    assert previous_centroid[1] == pytest.approx(centroid[1])
    assert moved_centroid[0] == pytest.approx((40.0 + 25.0) / 1280.0)
    assert moved_centroid[1] == pytest.approx((20.0 + 40.0) / 720.0)

    # Advance time to trigger timeout
    manager.last_seen[1] = datetime.utcnow() - timedelta(seconds=0.2)
    events, presence = manager.update([])
    assert any(event.type == "EXIT" for event in events)
    assert any(p.room_id == "living_room" and p.count == 0 for p in presence)
