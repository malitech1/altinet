"""Tests for the room context manager."""

from datetime import datetime, timedelta

from altinet.utils.context import RoomContextManager
from altinet.utils.types import BoundingBox, Track


def make_track(track_id: int, timestamp: datetime) -> Track:
    return Track(
        track_id=track_id,
        bbox=BoundingBox(10.0, 10.0, 50.0, 80.0),
        confidence=0.9,
        room_id="living_room",
        timestamp=timestamp,
        image_size=(720, 1280),
    )


def test_context_manager_tracks_people_and_environment():
    manager = RoomContextManager()
    t0 = datetime.utcnow()
    track = make_track(1, t0)

    manager.update_presence("living_room", [track], t0)
    context = manager.get_room_context("living_room")
    assert set(context.people.keys()) == {1}
    assert context.people[1].last_seen == t0

    manager.assign_identity(1, "alice", 0.87, t0 + timedelta(seconds=1))
    context = manager.get_room_context("living_room")
    person = context.people[1]
    assert person.identity_id == "alice"
    assert person.identity_confidence == 0.87

    manager.update_environment(
        "living_room",
        brightness=0.5,
        light_states={"ceiling": True},
        temperature=21.5,
        timestamp=t0 + timedelta(seconds=2),
    )
    manager.note_motion("living_room", t0 + timedelta(seconds=2))
    context = manager.get_room_context("living_room")
    assert context.brightness == 0.5
    assert context.light_states["ceiling"] is True
    assert context.temperature == 21.5
    assert context.last_motion == t0 + timedelta(seconds=2)

    manager.update_presence("living_room", [], t0 + timedelta(seconds=3))
    context = manager.get_room_context("living_room")
    assert not context.people
