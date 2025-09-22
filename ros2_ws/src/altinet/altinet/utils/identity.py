"""Identity classification helpers used by the identity service."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class IdentityObservation:
    """Normalised view of a person observation."""

    room_id: str
    frame_id: str
    track_id: int
    area_ratio: float
    detection_confidence: float


@dataclass
class IdentityResult:
    """Result returned by :class:`IdentityClassifier`."""

    label: str
    is_user: bool
    confidence: float
    reason: str


@dataclass
class IdentityClassificationConfig:
    """Configuration for :class:`IdentityClassifier`."""

    user_min_area_ratio: float = 0.015
    min_detection_confidence: float = 0.25
    user_confidence_bonus: float = 0.2
    guest_confidence_scale: float = 0.6
    unknown_confidence_scale: float = 0.4
    room_user_area_ratio: Dict[str, float] = field(default_factory=dict)

    def threshold_for_room(self, room_id: str) -> float:
        """Return the area ratio threshold for ``room_id``."""

        return self.room_user_area_ratio.get(room_id, self.user_min_area_ratio)


class IdentityClassifier:
    """Apply simple heuristics to label an observation as user or guest."""

    def __init__(self, config: IdentityClassificationConfig | None = None) -> None:
        self.config = config or IdentityClassificationConfig()

    def classify(self, observation: IdentityObservation) -> IdentityResult:
        """Return an :class:`IdentityResult` for ``observation``."""

        threshold = max(0.0, self.config.threshold_for_room(observation.room_id))
        confidence = _clamp(observation.detection_confidence, 0.0, 1.0)
        area_ratio = max(0.0, observation.area_ratio)

        if confidence < self.config.min_detection_confidence:
            scaled = confidence * self.config.unknown_confidence_scale
            reason = (
                "detection confidence "
                f"{confidence:.2f} below threshold "
                f"{self.config.min_detection_confidence:.2f}"
            )
            return IdentityResult(
                label="unknown",
                is_user=False,
                confidence=_clamp(scaled, 0.0, 1.0),
                reason=reason,
            )

        if area_ratio >= threshold:
            boosted = confidence + self.config.user_confidence_bonus
            reason = (
                f"area ratio {area_ratio:.4f} >= threshold {threshold:.4f}"
            )
            if observation.track_id >= 0:
                reason += f" (track {observation.track_id})"
            return IdentityResult(
                label="user",
                is_user=True,
                confidence=_clamp(boosted, 0.0, 1.0),
                reason=reason,
            )

        scaled = confidence * self.config.guest_confidence_scale
        reason = f"area ratio {area_ratio:.4f} below threshold {threshold:.4f}"
        return IdentityResult(
            label="guest",
            is_user=False,
            confidence=_clamp(scaled, 0.0, 1.0),
            reason=reason,
        )


def compute_area_ratio(width: float, height: float, image_width: int, image_height: int) -> float:
    """Return the ratio of the bounding box area to the image area."""

    if image_width <= 0 or image_height <= 0:
        return 0.0
    area = abs(width) * abs(height)
    denominator = float(image_width) * float(image_height)
    if denominator <= 0.0:
        return 0.0
    return float(area / denominator)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp ``value`` to the ``[minimum, maximum]`` interval."""

    return max(minimum, min(maximum, value))


__all__ = [
    "IdentityObservation",
    "IdentityResult",
    "IdentityClassificationConfig",
    "IdentityClassifier",
    "compute_area_ratio",
]
