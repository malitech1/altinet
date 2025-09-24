"""Tests for the scene change detector."""

import numpy as np

from altinet.utils.scene_monitor import SceneActivityConfig, SceneChangeDetector


class _Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        current = self.value
        self.value += 0.1
        return current


def test_scene_detector_triggers_on_motion():
    config = SceneActivityConfig(
        motion_threshold=0.01,
        checks_after_motion=2,
        checks_while_tracking=0,
        min_trigger_interval_s=0.0,
    )
    detector = SceneChangeDetector(config, clock=_Clock())
    frame_a = np.zeros((8, 8, 3), dtype=np.uint8)
    frame_b = np.full((8, 8, 3), 255, dtype=np.uint8)

    decision1 = detector.observe("living_room", frame_a)
    assert not decision1.should_detect

    decision2 = detector.observe("living_room", frame_b)
    assert decision2.should_detect
    assert decision2.motion_detected

    decision3 = detector.observe("living_room", frame_b)
    assert decision3.should_detect  # second check due to motion

    decision4 = detector.observe("living_room", frame_a)
    assert decision4.should_detect
    assert decision4.motion_detected  # returning to baseline counts as change

    decision5 = detector.observe("living_room", frame_a)
    assert decision5.should_detect
    assert not decision5.motion_detected

    decision6 = detector.observe("living_room", frame_a)
    assert not decision6.should_detect


def test_scene_detector_extends_after_detection():
    config = SceneActivityConfig(
        motion_threshold=0.01,
        checks_after_motion=1,
        checks_while_tracking=3,
        min_trigger_interval_s=0.0,
    )
    detector = SceneChangeDetector(config, clock=_Clock())
    frame_static = np.zeros((4, 4), dtype=np.uint8)
    frame_motion = np.full((4, 4), 255, dtype=np.uint8)

    detector.observe("hall", frame_static)
    detector.observe("hall", frame_motion)
    detector.notify_detection_result("hall", True)

    decisions = [detector.observe("hall", frame_static) for _ in range(3)]
    assert all(dec.should_detect for dec in decisions)

    final_decision = detector.observe("hall", frame_static)
    assert final_decision.should_detect
    assert not final_decision.motion_detected

    tail_decision = detector.observe("hall", frame_static)
    assert not tail_decision.should_detect

    detector.notify_detection_result("hall", False)
    assert not detector.observe("hall", frame_static).should_detect
