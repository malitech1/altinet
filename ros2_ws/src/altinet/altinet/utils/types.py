"""Typed data structures used by Altinet nodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np


@dataclass
class BoundingBox:
    """Axis-aligned bounding box expressed as x, y, width and height.

    Attributes:
        x: X coordinate of the top-left corner in pixels.
        y: Y coordinate of the top-left corner in pixels.
        w: Box width in pixels.
        h: Box height in pixels.
    """

    x: float
    y: float
    w: float
    h: float

    def as_xyxy(self) -> Tuple[float, float, float, float]:
        """Return the bounding box as ``(x1, y1, x2, y2)`` tuple."""

        return self.x, self.y, self.x + self.w, self.y + self.h

    def centroid(self) -> Tuple[float, float]:
        """Return the centroid of the bounding box."""

        return self.x + self.w / 2.0, self.y + self.h / 2.0


@dataclass
class Detection:
    """Represents a single person detection.

    Attributes:
        bbox: Bounding box of the detection in pixel coordinates.
        confidence: Confidence score assigned by the detector.
        room_id: Room identifier associated with the source camera.
        frame_id: Identifier of the originating sensor frame.
        timestamp: UTC timestamp indicating when the frame was captured.
        image_size: Original image dimensions as ``(height, width)``.
    """

    bbox: BoundingBox
    confidence: float
    room_id: str
    frame_id: str
    timestamp: datetime
    image_size: Tuple[int, int]


@dataclass
class Track:
    """Represents a tracked person across frames.

    Attributes:
        track_id: Unique identifier for the track.
        bbox: Current bounding box of the tracked person.
        confidence: Confidence score inherited from the last detection.
        room_id: Room identifier associated with the track.
        timestamp: Timestamp of the most recent update.
        image_size: Dimensions of the frame that produced the update.
        velocity: Estimated pixel velocity (pixels per second) based on
            recent detections.
        hits: Number of successful detection associations.
        age: Number of consecutive updates without a detection match.
    """

    track_id: int
    bbox: BoundingBox
    confidence: float
    room_id: str
    timestamp: datetime
    image_size: Tuple[int, int]
    velocity: Tuple[float, float] = (0.0, 0.0)
    hits: int = 1
    age: int = 0

    def centroid(self) -> Tuple[float, float]:
        """Return the centroid of the tracked bounding box."""

        return self.bbox.centroid()


@dataclass
class FaceLandmarks:
    """Five-point facial landmarks."""

    points: Tuple[Tuple[float, float], ...]

    def __post_init__(self) -> None:
        if len(self.points) != 5:
            raise ValueError("FaceLandmarks requires exactly five points")

    def flatten(self) -> Tuple[float, ...]:
        """Return landmarks as a flat ``(x1, y1, ..., x5, y5)`` tuple."""

        flattened: List[float] = []
        for x, y in self.points:
            flattened.extend([float(x), float(y)])
        return tuple(flattened)


@dataclass
class FaceQuality:
    """Aggregate quality metrics for a face snapshot."""

    score: float
    sharpness: float
    brightness: float
    pose: float


@dataclass
class FaceCandidate:
    """Face candidate returned by an alignment/embedding model."""

    bbox: BoundingBox
    landmarks: FaceLandmarks
    embedding: Tuple[float, ...]
    confidence: float


@dataclass
class FaceSnapshot:
    """Best-quality face snapshot aligned with a tracked person."""

    track_id: int
    room_id: str
    timestamp: datetime
    frame_id: str
    bbox: BoundingBox
    quality: FaceQuality
    landmarks: FaceLandmarks
    embedding: Tuple[float, ...]
    face_image: np.ndarray
    encoding: str = "rgb8"

    def __post_init__(self) -> None:
        if not isinstance(self.face_image, np.ndarray):
            raise TypeError("face_image must be a numpy.ndarray")
        if self.face_image.ndim not in (2, 3):
            raise ValueError("face_image must be 2D or 3D array")

    @property
    def face_height(self) -> int:
        """Height of the aligned face image."""

        return int(self.face_image.shape[0])

    @property
    def face_width(self) -> int:
        """Width of the aligned face image."""

        return int(self.face_image.shape[1])


@dataclass
class RoomPresence:
    """Presence summary for a room."""

    room_id: str
    track_ids: List[int]
    timestamp: datetime

    @property
    def count(self) -> int:
        """Return the number of tracked people in the room."""

        return len(self.track_ids)


@dataclass
class Event:
    """Represents a semantic event emitted by the event manager."""

    type: str
    subject_id: str
    room_id: str
    timestamp: datetime
    payload: Dict[str, object] = field(default_factory=dict)


@dataclass
class LightCommand:
    """Describes the desired state of a light for a specific room."""

    room_id: str
    state: bool
    source: str
    timestamp: datetime


@dataclass
class FaceSnapshot:
    """Metadata published when a new face embedding snapshot is captured."""

    identity_id: str
    embedding_id: str | None
    track_id: int | None
    camera_id: str | None
    captured_at: datetime
    quality: float
    metadata: Dict[str, object] = field(default_factory=dict)


@dataclass
class FaceEnrolmentConfirmation:
    """Confirmation message indicating that an enrolment completed."""

    identity_id: str
    embedding_ids: List[str]
    status: str
    created_at: datetime
    metadata: Dict[str, object] = field(default_factory=dict)


__all__ = [
    "BoundingBox",
    "Detection",
    "Track",
    "FaceLandmarks",
    "FaceQuality",
    "FaceCandidate",
    "FaceSnapshot",
    "RoomPresence",
    "Event",
    "LightCommand",
    "FaceSnapshot",
    "FaceEnrolmentConfirmation",
]
