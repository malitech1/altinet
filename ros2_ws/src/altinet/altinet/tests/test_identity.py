"""Tests for the identity classification helpers."""

import pytest

from altinet.utils.identity import (
    IdentityClassificationConfig,
    IdentityClassifier,
    IdentityObservation,
    compute_area_ratio,
)


def make_observation(area_ratio: float, confidence: float) -> IdentityObservation:
    return IdentityObservation(
        room_id="room_1",
        frame_id="camera",
        track_id=7,
        area_ratio=area_ratio,
        detection_confidence=confidence,
    )


def test_classifier_labels_user_when_area_large():
    config = IdentityClassificationConfig(user_min_area_ratio=0.01)
    classifier = IdentityClassifier(config)
    observation = make_observation(0.02, 0.8)

    result = classifier.classify(observation)

    assert result.label == "user"
    assert result.is_user is True
    assert result.confidence == pytest.approx(1.0)
    assert "area ratio" in result.reason


def test_classifier_labels_guest_when_area_small():
    config = IdentityClassificationConfig(user_min_area_ratio=0.05)
    classifier = IdentityClassifier(config)
    observation = make_observation(0.02, 0.8)

    result = classifier.classify(observation)

    assert result.label == "guest"
    assert result.is_user is False
    expected_conf = 0.8 * config.guest_confidence_scale
    assert result.confidence == pytest.approx(expected_conf)
    assert "below threshold" in result.reason


def test_classifier_returns_unknown_for_low_confidence():
    config = IdentityClassificationConfig(min_detection_confidence=0.5)
    classifier = IdentityClassifier(config)
    observation = make_observation(0.1, 0.2)

    result = classifier.classify(observation)

    assert result.label == "unknown"
    assert result.is_user is False
    expected_conf = 0.2 * config.unknown_confidence_scale
    assert result.confidence == pytest.approx(expected_conf)
    assert "below threshold" in result.reason


def test_compute_area_ratio_handles_invalid_dimensions():
    assert compute_area_ratio(10.0, 20.0, 0, 100) == 0.0
    assert compute_area_ratio(10.0, 20.0, 100, 0) == 0.0
    ratio = compute_area_ratio(10.0, 20.0, 100, 200)
    assert ratio == pytest.approx((10.0 * 20.0) / (100 * 200))
