"""Tracking utilities implementing a ByteTrack-inspired tracker."""

from __future__ import annotations

from dataclasses import dataclass

from typing import Dict, Iterable, List, Tuple

import numpy as np

from .types import Detection, Track


@dataclass
class ByteTrackConfig:
    """Configuration for the multi-object tracker."""

    track_thresh: float = 0.75
    match_thresh: float = 0.7
    max_age: int = 30
    min_hits: int = 3


class ByteTrack:
    """A lightweight implementation of the ByteTrack algorithm."""

    def __init__(self, config: ByteTrackConfig | None = None) -> None:
        self.config = config or ByteTrackConfig()
        self.next_track_id = 1
        self.tracks: Dict[int, Track] = {}

    def update(self, detections: Iterable[Detection]) -> List[Track]:
        """Update the tracker with the provided detections."""

        detections = [
            det for det in detections if det.confidence >= self.config.track_thresh
        ]
        if not self.tracks and not detections:
            return []

        track_ids = list(self.tracks.keys())
        track_boxes = (
            np.array([self.tracks[tid].bbox.as_xyxy() for tid in track_ids])
            if track_ids
            else np.empty((0, 4))
        )
        det_boxes = (
            np.array([det.bbox.as_xyxy() for det in detections])
            if detections
            else np.empty((0, 4))
        )

        matches: List[Tuple[int, int]] = []
        unmatched_tracks = set(track_ids)
        unmatched_detections = set(range(len(detections)))

        if track_boxes.size and det_boxes.size:
            iou_matrix = _iou_matrix(track_boxes, det_boxes)
            while True:
                max_index = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
                max_value = iou_matrix[max_index]
                if max_value < self.config.match_thresh:
                    break
                track_idx, det_idx = max_index
                matches.append((track_idx, det_idx))
                unmatched_tracks.discard(track_ids[track_idx])
                unmatched_detections.discard(det_idx)
                iou_matrix[track_idx, :] = -1.0
                iou_matrix[:, det_idx] = -1.0

        updated_tracks: Dict[int, Track] = {}
        for track_idx, det_idx in matches:
            track_id = track_ids[track_idx]
            track = self.tracks[track_id]
            detection = detections[det_idx]
            velocity = _estimate_velocity(
                track.bbox.centroid(), detection.bbox.centroid()
            )
            updated_tracks[track_id] = Track(
                track_id=track_id,
                bbox=detection.bbox,
                confidence=detection.confidence,
                room_id=detection.room_id,
                timestamp=detection.timestamp,
                image_size=detection.image_size,
                velocity=velocity,
                hits=track.hits + 1,
                age=0,
            )

        for track_id in unmatched_tracks:
            track = self.tracks[track_id]
            if track.age + 1 >= self.config.max_age:
                continue
            updated_tracks[track_id] = Track(
                track_id=track.track_id,
                bbox=track.bbox,
                confidence=track.confidence,
                room_id=track.room_id,
                timestamp=track.timestamp,
                image_size=track.image_size,
                velocity=track.velocity,
                hits=track.hits,
                age=track.age + 1,
            )

        for det_idx in unmatched_detections:
            detection = detections[det_idx]
            track_id = self.next_track_id
            self.next_track_id += 1
            updated_tracks[track_id] = Track(
                track_id=track_id,
                bbox=detection.bbox,
                confidence=detection.confidence,
                room_id=detection.room_id,
                timestamp=detection.timestamp,
                image_size=detection.image_size,
                velocity=(0.0, 0.0),
                hits=1,
                age=0,
            )

        self.tracks = updated_tracks
        return list(self.tracks.values())


def _estimate_velocity(
    previous: Tuple[float, float], current: Tuple[float, float]
) -> Tuple[float, float]:
    """Estimate the velocity vector between two positions."""

    return current[0] - previous[0], current[1] - previous[1]


def _iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Compute IoU matrix between two sets of boxes."""

    iou = np.zeros((len(a), len(b)), dtype=np.float32)
    for i, box_a in enumerate(a):
        for j, box_b in enumerate(b):
            iou[i, j] = _iou(box_a, box_b)
    return iou


def _iou(box1: np.ndarray, box2: np.ndarray) -> float:
    """Intersection over union between two boxes."""

    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter_area + 1e-9
    return inter_area / union


__all__ = ["ByteTrack", "ByteTrackConfig"]
