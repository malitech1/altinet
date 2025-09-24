"""Face capture pipeline for extracting high-quality face snapshots."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Dict, Iterable, List, Optional, Tuple

import numpy as np

from .types import (
    BoundingBox,
    FaceCandidate,
    FaceLandmarks,
    FaceQuality,
    FaceSnapshot,
    Track,
)

QualityEvaluator = Callable[[np.ndarray, FaceCandidate], FaceQuality]


@dataclass
class FaceCaptureConfig:
    """Configuration parameters governing the face capture pipeline."""

    minimum_quality: float = 0.6
    improvement_margin: float = 0.05
    min_time_between_snapshots: float = 2.0
    frame_history: int = 45
    face_padding: float = 0.15
    frame_tolerance_s: float = 0.1


class FaceCapturePipeline:
    """Match tracked people with face snapshots produced by a detector."""

    def __init__(
        self,
        analyzer,
        config: Optional[FaceCaptureConfig] = None,
        quality_evaluator: Optional[QualityEvaluator] = None,
    ) -> None:
        self.analyzer = analyzer or _NoOpAnalyzer()
        self.config = config or FaceCaptureConfig()
        self.quality_evaluator = quality_evaluator or _default_quality_evaluator
        self._frames: Dict[str, OrderedDict[datetime, np.ndarray]] = {}
        self._best_snapshots: Dict[Tuple[str, int], FaceSnapshot] = {}
        self._last_publish_time: Dict[Tuple[str, int], datetime] = {}

    # ------------------------------------------------------------------
    # Frame management
    # ------------------------------------------------------------------
    def add_frame(self, room_id: str, timestamp: datetime, frame: np.ndarray) -> None:
        """Store a frame for later association with track updates."""

        if not isinstance(frame, np.ndarray):
            raise TypeError("frame must be a numpy.ndarray")
        buffer = self._frames.setdefault(room_id, OrderedDict())
        buffer[timestamp] = np.ascontiguousarray(frame)
        while len(buffer) > self.config.frame_history:
            buffer.popitem(last=False)

    # ------------------------------------------------------------------
    # Track processing
    # ------------------------------------------------------------------
    def process_tracks(
        self,
        room_id: str,
        frame_id: str,
        timestamp: datetime,
        tracks: Iterable[Track],
    ) -> List[FaceSnapshot]:
        """Process ``tracks`` observed in ``room_id`` at ``timestamp``."""

        frame = self._lookup_frame(room_id, timestamp)
        if frame is None:
            return []
        results: List[FaceSnapshot] = []
        image_height, image_width = frame.shape[:2]
        for track in tracks:
            if track.room_id and track.room_id != room_id:
                continue
            crop_bbox = _expand_bbox(
                track.bbox,
                self.config.face_padding,
                float(image_width),
                float(image_height),
            )
            crop = _crop(frame, crop_bbox)
            if crop is None:
                continue
            candidates = list(self.analyzer.detect(crop))
            if not candidates:
                continue
            best_snapshot: Optional[FaceSnapshot] = None
            best_quality: Optional[FaceQuality] = None
            for candidate in candidates:
                translated = _translate_bbox(candidate.bbox, crop_bbox, frame.shape[:2])
                if translated is None:
                    continue
                face_patch = _crop(frame, translated)
                if face_patch is None:
                    continue
                quality = self.quality_evaluator(face_patch, candidate)
                snapshot = FaceSnapshot(
                    track_id=track.track_id,
                    room_id=room_id,
                    timestamp=track.timestamp,
                    frame_id=frame_id,
                    bbox=translated,
                    quality=quality,
                    landmarks=candidate.landmarks,
                    embedding=tuple(float(x) for x in candidate.embedding),
                    face_image=np.ascontiguousarray(face_patch),
                    encoding="bgr8",
                )
                if best_quality is None or quality.score > best_quality.score:
                    best_quality = quality
                    best_snapshot = snapshot
            if best_snapshot is None:
                continue
            if self._should_publish(best_snapshot):
                results.append(best_snapshot)
        return results

    def get_best_snapshot(self, room_id: str, track_id: int) -> Optional[FaceSnapshot]:
        """Return the best known snapshot for ``track_id`` in ``room_id``."""

        return self._best_snapshots.get((room_id, track_id))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _lookup_frame(self, room_id: str, timestamp: datetime) -> Optional[np.ndarray]:
        buffer = self._frames.get(room_id)
        if not buffer:
            return None
        frame = buffer.get(timestamp)
        if frame is not None:
            return frame
        tolerance = max(self.config.frame_tolerance_s, 0.0)
        if tolerance <= 0.0:
            return None
        best_key = None
        best_delta = float("inf")
        for key in buffer.keys():
            delta = abs((timestamp - key).total_seconds())
            if delta < best_delta:
                best_delta = delta
                best_key = key
        if best_key is None or best_delta > tolerance:
            return None
        return buffer[best_key]

    def _should_publish(self, snapshot: FaceSnapshot) -> bool:
        key = (snapshot.room_id, snapshot.track_id)
        best = self._best_snapshots.get(key)
        quality = snapshot.quality.score
        min_quality = self.config.minimum_quality
        if best is None:
            if quality >= min_quality and self._cadence_allows(key, snapshot.timestamp):
                self._best_snapshots[key] = snapshot
                self._last_publish_time[key] = snapshot.timestamp
                return True
            if quality > (best.quality.score if best else float("-inf")):
                self._best_snapshots[key] = snapshot
            return False

        improvement = quality - best.quality.score
        should_update_best = improvement > 0.0
        if quality < min_quality or improvement < self.config.improvement_margin:
            if should_update_best:
                self._best_snapshots[key] = snapshot
            return False
        if not self._cadence_allows(key, snapshot.timestamp):
            if should_update_best:
                self._best_snapshots[key] = snapshot
            return False
        self._best_snapshots[key] = snapshot
        self._last_publish_time[key] = snapshot.timestamp
        return True

    def _cadence_allows(self, key: Tuple[str, int], timestamp: datetime) -> bool:
        interval = max(self.config.min_time_between_snapshots, 0.0)
        if interval <= 0.0:
            return True
        last_time = self._last_publish_time.get(key)
        if last_time is None:
            return True
        return (timestamp - last_time).total_seconds() >= interval


class InsightFaceAnalyzer:
    """Adapter around :mod:`insightface` providing ``FaceCandidate`` objects."""

    def __init__(
        self,
        model_name: str = "buffalo_l",
        model_root: Optional[str] = None,
        det_size: Tuple[int, int] = (640, 640),
        ctx_id: int = 0,
    ) -> None:
        try:  # pragma: no cover - optional dependency
            from insightface.app import FaceAnalysis
        except ImportError as exc:  # pragma: no cover - executed when dependency missing
            raise RuntimeError(
                "insightface is required to use InsightFaceAnalyzer"
            ) from exc
        self._app = FaceAnalysis(name=model_name, root=model_root)
        self._app.prepare(ctx_id=ctx_id, det_size=det_size)

    def detect(self, image: np.ndarray) -> List[FaceCandidate]:  # pragma: no cover - thin wrapper
        faces = self._app.get(image)
        candidates: List[FaceCandidate] = []
        for face in faces:
            bbox = getattr(face, "bbox", None)
            kps = getattr(face, "kps", None)
            embedding = getattr(face, "embedding", None)
            det_score = float(getattr(face, "det_score", 0.0))
            if bbox is None or kps is None:
                continue
            x1, y1, x2, y2 = [float(value) for value in bbox]
            width = max(x2 - x1, 0.0)
            height = max(y2 - y1, 0.0)
            try:
                landmarks = FaceLandmarks(
                    tuple((float(point[0]), float(point[1])) for point in kps)
                )
            except ValueError:
                continue
            embedding_values: Tuple[float, ...]
            if embedding is None:
                embedding_values = tuple()
            else:
                embedding_array = np.asarray(embedding).astype(np.float32).flatten()
                embedding_values = tuple(float(value) for value in embedding_array)
            candidates.append(
                FaceCandidate(
                    bbox=BoundingBox(x1, y1, width, height),
                    landmarks=landmarks,
                    embedding=embedding_values,
                    confidence=det_score,
                )
            )
        return candidates


class _NoOpAnalyzer:
    """Fallback analyzer that never produces faces."""

    def detect(self, image: np.ndarray) -> List[FaceCandidate]:  # pragma: no cover - trivial
        return []


def _expand_bbox(
    bbox: BoundingBox,
    padding_ratio: float,
    image_width: float,
    image_height: float,
) -> BoundingBox:
    pad_w = max(bbox.w * padding_ratio, 0.0)
    pad_h = max(bbox.h * padding_ratio, 0.0)
    x = max(bbox.x - pad_w / 2.0, 0.0)
    y = max(bbox.y - pad_h / 2.0, 0.0)
    w = min(bbox.w + pad_w, image_width - x)
    h = min(bbox.h + pad_h, image_height - y)
    return BoundingBox(x, y, w, h)


def _translate_bbox(
    local_bbox: BoundingBox, offset: BoundingBox, image_size: Tuple[int, int]
) -> Optional[BoundingBox]:
    x = offset.x + local_bbox.x
    y = offset.y + local_bbox.y
    w = local_bbox.w
    h = local_bbox.h
    clipped = _clip_bbox(BoundingBox(x, y, w, h), image_size)
    return clipped


def _clip_bbox(bbox: BoundingBox, image_size: Tuple[int, int]) -> Optional[BoundingBox]:
    height, width = image_size
    x1 = max(bbox.x, 0.0)
    y1 = max(bbox.y, 0.0)
    x2 = min(bbox.x + bbox.w, float(width))
    y2 = min(bbox.y + bbox.h, float(height))
    if x2 <= x1 or y2 <= y1:
        return None
    return BoundingBox(x1, y1, x2 - x1, y2 - y1)


def _crop(image: np.ndarray, bbox: BoundingBox) -> Optional[np.ndarray]:
    x1 = int(round(bbox.x))
    y1 = int(round(bbox.y))
    x2 = int(round(bbox.x + bbox.w))
    y2 = int(round(bbox.y + bbox.h))
    if x2 <= x1 or y2 <= y1:
        return None
    height, width = image.shape[:2]
    x1 = max(x1, 0)
    y1 = max(y1, 0)
    x2 = min(x2, width)
    y2 = min(y2, height)
    if x2 <= x1 or y2 <= y1:
        return None
    return np.ascontiguousarray(image[y1:y2, x1:x2])


def _default_quality_evaluator(
    face_image: np.ndarray, candidate: FaceCandidate
) -> FaceQuality:
    if face_image.size == 0:
        return FaceQuality(score=0.0, sharpness=0.0, brightness=0.0, pose=0.0)
    image = face_image.astype(np.float32)
    if image.ndim == 3:
        gray = image.mean(axis=2)
    else:
        gray = image
    gy, gx = np.gradient(gray)
    magnitude = np.sqrt(gx**2 + gy**2)
    sharpness_raw = float(magnitude.mean())
    sharpness = float(np.tanh(sharpness_raw / 50.0))
    max_value = 255.0 if face_image.dtype != np.float32 else 1.0
    brightness = float(np.clip(gray.mean() / max_value, 0.0, 1.0))
    pose = _estimate_pose(candidate.landmarks)
    score = float(
        0.5 * candidate.confidence + 0.3 * sharpness + 0.15 * brightness + 0.05 * pose
    )
    return FaceQuality(score=score, sharpness=sharpness, brightness=brightness, pose=pose)


def _estimate_pose(landmarks: FaceLandmarks) -> float:
    try:
        left_eye = landmarks.points[0]
        right_eye = landmarks.points[1]
    except (IndexError, TypeError):
        return 0.0
    horizontal = abs(right_eye[0] - left_eye[0])
    vertical = abs(right_eye[1] - left_eye[1])
    if horizontal <= 0.0:
        return 0.0
    tilt_ratio = vertical / horizontal
    return float(max(0.0, 1.0 - min(tilt_ratio, 1.0)))


__all__ = [
    "FaceCaptureConfig",
    "FaceCapturePipeline",
    "InsightFaceAnalyzer",
]
