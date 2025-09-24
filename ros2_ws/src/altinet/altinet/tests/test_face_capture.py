"""Tests for the face capture pipeline."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Iterable, List

import numpy as np
import pytest

from altinet.utils.face_capture import FaceCaptureConfig, FaceCapturePipeline
from altinet.utils.types import (
    BoundingBox,
    FaceCandidate,
    FaceLandmarks,
    FaceQuality,
    Track,
)


class StubAnalyzer:
    """Deterministic face analyzer used in tests."""

    def __init__(self) -> None:
        self._responses: List[List[FaceCandidate]] = []

    def enqueue(self, candidates: Iterable[FaceCandidate]) -> None:
        self._responses.append(list(candidates))

    def detect(self, image: np.ndarray) -> List[FaceCandidate]:
        if not self._responses:
            return []
        return self._responses.pop(0)


def _quality_from_confidence(confidence: float) -> FaceQuality:
    return FaceQuality(score=confidence, sharpness=confidence, brightness=0.5, pose=0.5)


def _landmarks() -> FaceLandmarks:
    return FaceLandmarks(
        (
            (10.0, 10.0),
            (20.0, 10.0),
            (15.0, 18.0),
            (12.0, 24.0),
            (18.0, 24.0),
        )
    )


def _build_track(track_id: int, timestamp: datetime, bbox: BoundingBox) -> Track:
    return Track(
        track_id=track_id,
        bbox=bbox,
        confidence=0.9,
        room_id="living_room",
        timestamp=timestamp,
        image_size=(120, 160),
    )


def test_pipeline_emits_snapshot_when_quality_exceeds_threshold() -> None:
    analyzer = StubAnalyzer()
    config = FaceCaptureConfig(
        minimum_quality=0.5,
        improvement_margin=0.05,
        min_time_between_snapshots=0.0,
        face_padding=0.0,
    )
    pipeline = FaceCapturePipeline(
        analyzer,
        config=config,
        quality_evaluator=lambda image, candidate: _quality_from_confidence(
            candidate.confidence
        ),
    )

    timestamp = datetime(2024, 1, 1, 12, 0, 0)
    frame = np.full((120, 160, 3), 128, dtype=np.uint8)
    pipeline.add_frame("living_room", timestamp, frame)

    candidate = FaceCandidate(
        bbox=BoundingBox(6.0, 4.0, 24.0, 24.0),
        landmarks=_landmarks(),
        embedding=(0.1, 0.2, 0.3, 0.4),
        confidence=0.9,
    )
    analyzer.enqueue([candidate])
    track = _build_track(
        7,
        timestamp,
        BoundingBox(20.0, 16.0, 48.0, 48.0),
    )

    snapshots = pipeline.process_tracks(
        "living_room",
        "camera_frame",
        timestamp,
        [track],
    )
    assert len(snapshots) == 1
    snapshot = snapshots[0]
    assert snapshot.track_id == 7
    assert snapshot.room_id == "living_room"
    assert snapshot.encoding == "bgr8"
    assert snapshot.embedding == pytest.approx(candidate.embedding)
    assert snapshot.bbox.x == pytest.approx(track.bbox.x + candidate.bbox.x)
    assert snapshot.bbox.y == pytest.approx(track.bbox.y + candidate.bbox.y)
    assert snapshot.face_image.shape[0] == pytest.approx(candidate.bbox.h, rel=0.05)
    assert snapshot.face_image.shape[1] == pytest.approx(candidate.bbox.w, rel=0.05)


def test_pipeline_requires_significant_quality_improvement() -> None:
    analyzer = StubAnalyzer()
    config = FaceCaptureConfig(
        minimum_quality=0.55,
        improvement_margin=0.05,
        min_time_between_snapshots=0.0,
        face_padding=0.0,
    )
    pipeline = FaceCapturePipeline(
        analyzer,
        config=config,
        quality_evaluator=lambda image, candidate: _quality_from_confidence(
            candidate.confidence
        ),
    )
    base_bbox = BoundingBox(30.0, 20.0, 40.0, 40.0)
    timestamp = datetime(2024, 1, 1, 9, 0, 0)
    frame = np.full((140, 140, 3), 90, dtype=np.uint8)

    pipeline.add_frame("living_room", timestamp, frame)
    analyzer.enqueue(
        [
            FaceCandidate(
                bbox=BoundingBox(8.0, 8.0, 24.0, 24.0),
                landmarks=_landmarks(),
                embedding=(0.5, 0.6, 0.7),
                confidence=0.6,
            )
        ]
    )
    track = _build_track(3, timestamp, base_bbox)
    first = pipeline.process_tracks("living_room", "camera_frame", timestamp, [track])
    assert len(first) == 1
    assert first[0].quality.score == pytest.approx(0.6)

    timestamp2 = timestamp + timedelta(seconds=1)
    pipeline.add_frame("living_room", timestamp2, frame)
    analyzer.enqueue(
        [
            FaceCandidate(
                bbox=BoundingBox(8.0, 8.0, 24.0, 24.0),
                landmarks=_landmarks(),
                embedding=(0.5, 0.6, 0.7),
                confidence=0.62,
            )
        ]
    )
    track2 = _build_track(3, timestamp2, base_bbox)
    second = pipeline.process_tracks("living_room", "camera_frame", timestamp2, [track2])
    assert second == []

    timestamp3 = timestamp2 + timedelta(seconds=1)
    pipeline.add_frame("living_room", timestamp3, frame)
    analyzer.enqueue(
        [
            FaceCandidate(
                bbox=BoundingBox(8.0, 8.0, 24.0, 24.0),
                landmarks=_landmarks(),
                embedding=(0.5, 0.6, 0.7),
                confidence=0.72,
            )
        ]
    )
    track3 = _build_track(3, timestamp3, base_bbox)
    third = pipeline.process_tracks("living_room", "camera_frame", timestamp3, [track3])
    assert len(third) == 1
    assert third[0].quality.score == pytest.approx(0.72)
    best = pipeline.get_best_snapshot("living_room", 3)
    assert best is not None
    assert best.quality.score == pytest.approx(0.72)
