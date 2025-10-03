"""Utilities for evaluating captured training images against known faces."""

from __future__ import annotations

import base64
import binascii
import io
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

try:  # pragma: no cover - optional dependency for runtime recognition
    from PIL import Image
except ImportError:  # pragma: no cover - handled gracefully at runtime
    Image = None  # type: ignore

from ..models import TrainingImage, TrainingProfile

_FACE_ANALYZER: Optional[object] = None
_TRAINING_EMBEDDING_CACHE: Dict[int, np.ndarray] = {}


@dataclass(frozen=True)
class TrainingTestResult:
    """Represents the outcome of a face matching request."""

    payload: dict
    status: int


def clean_image_data_uri(data_uri: str) -> str | None:
    """Validate that a provided data URI contains base64 encoded image data."""

    if not isinstance(data_uri, str):
        return None

    candidate = data_uri.strip()
    if not candidate.startswith("data:image/"):
        return None

    header, _, base64_data = candidate.partition(",")
    if not base64_data:
        return None

    if ";base64" not in header.lower():
        return None

    try:
        base64.b64decode(base64_data, validate=True)
    except (binascii.Error, ValueError):
        return None

    return f"{header},{base64_data}"


def reset_training_embedding_cache() -> None:
    """Clear cached embeddings so new training data is picked up."""

    _TRAINING_EMBEDDING_CACHE.clear()


def load_image_from_data_uri(data_uri: str) -> Optional[np.ndarray]:
    """Decode a base64 image data URI into an RGB numpy array."""

    if Image is None:
        return None

    cleaned = clean_image_data_uri(data_uri)
    if not cleaned:
        return None

    _, _, base64_data = cleaned.partition(",")
    try:
        raw_bytes = base64.b64decode(base64_data, validate=True)
    except (binascii.Error, ValueError):
        return None

    try:
        with Image.open(io.BytesIO(raw_bytes)) as image:  # type: ignore[union-attr]
            rgb_image = image.convert("RGB")
            return np.array(rgb_image)
    except Exception:  # pragma: no cover - Pillow failures are handled gracefully
        return None


def get_face_analyzer() -> Optional[object]:
    """Return a cached instance of the InsightFace analysis helper."""

    global _FACE_ANALYZER
    if _FACE_ANALYZER is not None:
        return _FACE_ANALYZER

    try:
        from insightface.app import FaceAnalysis  # type: ignore
    except Exception:  # pragma: no cover - optional dependency unavailable
        return None

    try:
        analyzer = FaceAnalysis(name="buffalo_l")
        analyzer.prepare(ctx_id=-1, det_size=(640, 640))
    except Exception:  # pragma: no cover - runtime initialisation failure
        return None

    _FACE_ANALYZER = analyzer
    return _FACE_ANALYZER


def extract_embedding(analyzer: object, image: np.ndarray) -> Optional[np.ndarray]:
    """Generate a normalised embedding for ``image`` using ``analyzer``."""

    if analyzer is None:
        return None

    try:
        faces = analyzer.get(image)  # type: ignore[attr-defined]
    except Exception:
        return None

    if not faces:
        return None

    face = faces[0]
    embedding = getattr(face, "normed_embedding", None)
    if embedding is None:
        return None

    array = np.asarray(embedding, dtype=np.float32)
    if array.size == 0:
        return None

    norm = np.linalg.norm(array)
    if norm == 0:
        return None
    return array / norm


def embedding_for_training_image(
    analyzer: object, image: TrainingImage
) -> Optional[np.ndarray]:
    """Return a cached embedding vector for ``image``."""

    cached = _TRAINING_EMBEDDING_CACHE.get(image.pk)
    if cached is not None:
        return cached

    array = load_image_from_data_uri(image.image_data)
    if array is None:
        return None

    embedding = extract_embedding(analyzer, array)
    if embedding is None:
        return None

    _TRAINING_EMBEDDING_CACHE[image.pk] = embedding
    return embedding


def evaluate_face_test(image_data: str | None) -> TrainingTestResult:
    """Evaluate ``image_data`` against known training profiles."""

    analyzer = get_face_analyzer()
    if analyzer is None or Image is None:
        return TrainingTestResult(
            {
                "success": False,
                "error": "Face recognition components are unavailable on this server.",
            },
            status=503,
        )

    cleaned_image = clean_image_data_uri(image_data) if image_data else None
    if not cleaned_image:
        return TrainingTestResult(
            {"success": False, "error": "Provide a valid captured image."},
            status=400,
        )

    probe_image = load_image_from_data_uri(cleaned_image)
    if probe_image is None:
        return TrainingTestResult(
            {
                "success": False,
                "error": "Unable to decode the captured frame for analysis.",
            },
            status=400,
        )

    probe_embedding = extract_embedding(analyzer, probe_image)
    if probe_embedding is None:
        return TrainingTestResult(
            {
                "success": False,
                "error": "No face was detected in the captured frame.",
            },
            status=422,
        )

    profiles = TrainingProfile.objects.prefetch_related("images").all()
    if not profiles:
        return TrainingTestResult(
            {
                "success": False,
                "error": "No trained profiles are available yet.",
            },
            status=404,
        )

    best_profile: Optional[TrainingProfile] = None
    best_score: float = -1.0

    for profile in profiles:
        for training_image in profile.images.all():
            embedding = embedding_for_training_image(analyzer, training_image)
            if embedding is None:
                continue
            score = float(np.dot(probe_embedding, embedding))
            if score > best_score:
                best_score = score
                best_profile = profile

    threshold = 0.35
    confidence = float(min(max(best_score, 0.0), 1.0))

    if best_profile is None or best_score < threshold:
        return TrainingTestResult(
            {
                "success": True,
                "match": None,
                "confidence": round(confidence, 3),
                "message": "No trained faces matched the capture.",
            },
            status=200,
        )

    return TrainingTestResult(
        {
            "success": True,
            "confidence": round(confidence, 3),
            "match": {
                "id": best_profile.id,
                "full_name": best_profile.full_name,
                "image_count": best_profile.image_count,
                "trained_at": best_profile.trained_at.isoformat()
                if best_profile.trained_at
                else None,
            },
            "message": f"Match found: {best_profile.full_name}.",
        },
        status=200,
    )
