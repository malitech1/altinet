"""Tests for the lighting controller."""

from datetime import datetime

from altinet.drivers.lights import LightDriver
from altinet.nodes.lighting_control_node import LightingController
from altinet.utils.types import Event, RoomPresence


def test_lighting_rules_toggle_on_and_off():
    driver = LightDriver()
    controller = LightingController(driver, cooldown_s=0.0, override_duration_s=60.0)
    presence = RoomPresence(
        room_id="living_room", track_ids=[1], timestamp=datetime.utcnow()
    )
    controller.update_presence(presence)
    controller.handle_event(
        Event(
            type="ENTRY",
            subject_id="1",
            room_id="living_room",
            timestamp=datetime.utcnow(),
            payload={},
        )
    )
    assert driver.get_state("living_room").is_on

    controller.update_presence(
        RoomPresence(room_id="living_room", track_ids=[], timestamp=datetime.utcnow())
    )
    controller.handle_event(
        Event(
            type="EXIT",
            subject_id="1",
            room_id="living_room",
            timestamp=datetime.utcnow(),
            payload={},
        )
    )
    controller.tick()
    assert not driver.get_state("living_room").is_on


def test_manual_override_supersedes_rules():
    driver = LightDriver()
    controller = LightingController(driver, cooldown_s=0.0, override_duration_s=10.0)
    controller.manual_override("living_room", True)
    controller.update_presence(
        RoomPresence(room_id="living_room", track_ids=[], timestamp=datetime.utcnow())
    )
    controller.handle_event(
        Event(
            type="EXIT",
            subject_id="1",
            room_id="living_room",
            timestamp=datetime.utcnow(),
            payload={},
        )
    )
    controller.tick()
    assert driver.get_state("living_room").is_on
