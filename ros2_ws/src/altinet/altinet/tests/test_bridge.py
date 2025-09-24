"""Tests for the ROS→Django bridge."""

from datetime import datetime

from datetime import datetime

from altinet.nodes.ros2_django_bridge_node import BridgeConfig, DjangoBridge
from altinet.utils.types import (
    BoundingBox,
    Event,
    FaceEnrolmentConfirmation,
    FaceSnapshot,
    RoomPresence,
    Track,
)


class DummyTransport:
    def __init__(self, should_fail=False):
        self.should_fail = should_fail
        self.sent = []

    def send(self, payload):
        if self.should_fail:
            raise RuntimeError("network error")
        self.sent.append(payload)


def make_event(room_id="living_room") -> Event:
    return Event(
        type="ENTRY",
        subject_id="1",
        room_id=room_id,
        timestamp=datetime.utcnow(),
        payload={},
    )


def make_presence(room_id="living_room") -> RoomPresence:
    return RoomPresence(room_id=room_id, track_ids=[1], timestamp=datetime.utcnow())


def make_track(track_id=1) -> Track:
    return Track(
        track_id=track_id,
        bbox=BoundingBox(0.0, 0.0, 10.0, 10.0),
        confidence=0.9,
        room_id="living_room",
        timestamp=datetime.utcnow(),
        image_size=(1, 1),
    )


def test_bridge_respects_privacy_flag():
    config = BridgeConfig(
        api_base="http://localhost", ws_url=None, auth_token=None, forward_allowed=False
    )
    bridge = DjangoBridge(config, transport=DummyTransport())
    bridge.publish_event(make_event())
    assert len(bridge.queue) == 1
    assert bridge.queue[0]["endpoint"] == "/events/"


def test_bridge_flushes_queue_when_allowed():
    config = BridgeConfig(
        api_base="http://localhost", ws_url=None, auth_token=None, forward_allowed=True
    )
    transport = DummyTransport()
    bridge = DjangoBridge(config, transport=transport)
    bridge.publish_event(make_event())
    bridge.publish_presence(make_presence())
    bridge.publish_tracks("living_room", [make_track()], datetime.utcnow())
    assert not bridge.queue
    assert len(transport.sent) == 3
    assert all(item["endpoint"] == "/events/" for item in transport.sent)


def test_bridge_backoff_on_failure(monkeypatch):
    config = BridgeConfig(
        api_base="http://localhost", ws_url=None, auth_token=None, forward_allowed=True
    )
    transport = DummyTransport(should_fail=True)
    bridge = DjangoBridge(config, transport=transport)
    bridge.publish_event(make_event())
    assert len(bridge.queue) == 1
    assert bridge._backoff > 1.0


def test_bridge_handles_face_messages():
    config = BridgeConfig(
        api_base="http://localhost", ws_url=None, auth_token=None, forward_allowed=True
    )
    transport = DummyTransport()
    bridge = DjangoBridge(config, transport=transport)
    snapshot = FaceSnapshot(
        identity_id="alice",
        embedding_id="emb1",
        track_id=7,
        camera_id="cam-1",
        captured_at=datetime.utcnow(),
        quality=0.9,
        metadata={"note": "hat"},
    )
    confirmation = FaceEnrolmentConfirmation(
        identity_id="alice",
        embedding_ids=["emb1"],
        status="stored",
        created_at=datetime.utcnow(),
        metadata={"source": "test"},
    )
    bridge.publish_face_snapshot(snapshot)
    bridge.publish_enrolment_confirmation(confirmation)
    assert not bridge.queue
    assert len(transport.sent) == 2
    assert transport.sent[0]["endpoint"] == "/face-snapshots/"
    assert transport.sent[1]["endpoint"] == "/face-enrolments/"
