"""Tests for the identity classification helpers."""

import numpy as np
import pytest

from altinet.utils.identity import (
    IdentityClassificationConfig,
    IdentityClassifier,
    IdentityObservation,
    compute_area_ratio,
)
from altinet.utils.face_identity import (
    FaceEmbeddingIndex,
    FaceIdentityConfig,
    FaceIdentityResolver,
    FaceSnapshot,
    FaceSnapshotCache,
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


class FakeClock:
    def __init__(self) -> None:
        self._now = 0.0

    def now(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += float(seconds)


def make_face_resolver(
    embeddings: np.ndarray,
    labels,
    *,
    is_user_map=None,
    cache_ttl: float = 5.0,
    fetcher=None,
    config: FaceIdentityConfig | None = None,
):
    clock = FakeClock()
    cache = FaceSnapshotCache(cache_ttl, clock=clock.now)
    index = FaceEmbeddingIndex(embeddings, labels, label_is_user=is_user_map)
    classifier = IdentityClassifier(IdentityClassificationConfig(user_min_area_ratio=0.02))
    resolver = FaceIdentityResolver(
        classifier=classifier,
        index=index,
        cache=cache,
        config=config
        or FaceIdentityConfig(
            user_similarity_threshold=0.7,
            guest_similarity_threshold=0.6,
            unknown_label="unknown",
            fallback_to_heuristic_for_unknown=False,
        ),
        snapshot_fetcher=fetcher,
    )
    return resolver, cache, clock


def test_face_identity_respects_similarity_thresholds():
    embeddings = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=np.float32)
    labels = ["resident", "guest_1"]
    resolver, cache, _ = make_face_resolver(
        embeddings,
        labels,
        is_user_map={"resident": True, "guest_1": False},
        config=FaceIdentityConfig(
            user_similarity_threshold=0.8,
            guest_similarity_threshold=0.6,
            unknown_label="unknown",
            fallback_to_heuristic_for_unknown=False,
        ),
    )
    cache.update(FaceSnapshot(track_id=7, embedding=np.array([1.0, 0.0], dtype=np.float32)))
    observation = make_observation(0.02, 0.9)

    decision = resolver.resolve(observation, track_id=7)

    assert decision.result.label == "resident"
    assert decision.result.is_user is True
    assert decision.result.confidence == pytest.approx(1.0)
    assert decision.embedding_id == "resident"
    assert decision.used_fallback is False


def test_face_identity_falls_back_to_heuristic_when_snapshot_missing():
    embeddings = np.array([[1.0, 0.0]], dtype=np.float32)
    labels = ["resident"]
    resolver, _cache, _ = make_face_resolver(
        embeddings,
        labels,
        is_user_map={"resident": True},
    )
    observation = make_observation(0.005, 0.9)

    decision = resolver.resolve(observation, track_id=42)

    assert decision.used_fallback is True
    assert decision.result.label == "guest"
    assert "No face embedding" in decision.result.reason


def test_face_snapshot_cache_expires_and_requests_refresh():
    embeddings = np.array([[0.0, 1.0]], dtype=np.float32)
    labels = ["guest"]
    fetch_calls: list[int] = []

    def fetcher(track_id: int):
        fetch_calls.append(track_id)
        return FaceSnapshot(track_id=track_id, embedding=np.array([0.0, 1.0], dtype=np.float32))

    resolver, cache, clock = make_face_resolver(
        embeddings,
        labels,
        is_user_map={"guest": False},
        cache_ttl=1.0,
        fetcher=fetcher,
        config=FaceIdentityConfig(
            user_similarity_threshold=0.7,
            guest_similarity_threshold=0.5,
            unknown_label="unknown",
            fallback_to_heuristic_for_unknown=False,
        ),
    )
    cache.update(FaceSnapshot(track_id=3, embedding=np.array([0.0, 1.0], dtype=np.float32)))
    observation = make_observation(0.02, 0.8)

    decision_first = resolver.resolve(observation, track_id=3)
    assert decision_first.result.label == "guest"
    assert not fetch_calls

    clock.advance(2.0)
    decision_second = resolver.resolve(observation, track_id=3)

    assert fetch_calls == [3]
    assert decision_second.result.label == "guest"
