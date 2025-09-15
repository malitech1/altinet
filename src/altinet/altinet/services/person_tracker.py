"""Track people across frames using IoU and colour histograms."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class _Track:
    bbox: Tuple[int, int, int, int]
    hist: np.ndarray
    track_id: int
    missed: int = 0


class PersonTracker:
    """Track generic person detections and assign persistent IDs.

    The tracker matches incoming ``boxes`` to existing tracks using a
    combination of intersection-over-union and a simple colour histogram for
    re-identification when IoU fails. Tracks that are not matched for several
    consecutive updates are removed.
    """

    def __init__(
        self,
        *,
        iou_threshold: float = 0.3,
        hist_threshold: float = 0.7,
        max_missed: int = 5,
    ) -> None:
        self._iou_threshold = iou_threshold
        self._hist_threshold = hist_threshold
        self._max_missed = max_missed
        self._tracks: List[_Track] = []
        self._next_id = 1

    @staticmethod
    def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        a_x2, a_y2 = ax + aw, ay + ah
        b_x2, b_y2 = bx + bw, by + bh
        inter_x1, inter_y1 = max(ax, bx), max(ay, by)
        inter_x2, inter_y2 = min(a_x2, b_x2), min(a_y2, b_y2)
        inter_w, inter_h = max(0, inter_x2 - inter_x1), max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area == 0:
            return 0.0
        a_area = aw * ah
        b_area = bw * bh
        union_area = a_area + b_area - inter_area
        return inter_area / union_area

    @staticmethod
    def _hist(frame: np.ndarray, box: Tuple[int, int, int, int]) -> np.ndarray:
        x, y, w, h = box
        crop = frame[y : y + h, x : x + w]
        if crop.size == 0:
            return np.zeros(48, dtype=np.float32)
        hist_parts = []
        for c in range(min(3, crop.shape[2])):
            ch_hist, _ = np.histogram(
                crop[:, :, c], bins=16, range=(0, 256), density=True
            )
            hist_parts.append(ch_hist)
        hist = np.concatenate(hist_parts).astype(np.float32)
        norm = np.linalg.norm(hist)
        if norm > 0:
            hist /= norm
        return hist

    def tracks(self) -> List[Tuple[int, Tuple[int, int, int, int]]]:
        """Return the current tracks as ``(id, bbox)`` tuples."""

        return [(t.track_id, t.bbox) for t in self._tracks]

    def update(self, frame, boxes: List[Tuple[int, int, int, int]]) -> List[int]:
        """Update tracker with new detections and return their IDs."""

        matched = [False] * len(self._tracks)
        results: List[int] = []
        for box in boxes:
            # Attempt IoU matching first
            best_iou = 0.0
            best_idx = None
            for i, track in enumerate(self._tracks):
                iou = self._iou(box, track.bbox)
                if iou > best_iou:
                    best_iou, best_idx = iou, i
            if best_idx is not None and best_iou > self._iou_threshold:
                track = self._tracks[best_idx]
                track.bbox = box
                track.hist = self._hist(frame, box)
                track.missed = 0
                matched[best_idx] = True
                results.append(track.track_id)
                continue

            # Compute histogram for re-identification
            hist = self._hist(frame, box)
            best_score = 0.0
            best_hist_idx = None
            for i, track in enumerate(self._tracks):
                if matched[i]:
                    continue
                score = float(np.dot(track.hist, hist))
                if score > best_score:
                    best_score, best_hist_idx = score, i
            if best_hist_idx is not None and best_score > self._hist_threshold:
                track = self._tracks[best_hist_idx]
                track.bbox = box
                track.hist = hist
                track.missed = 0
                matched[best_hist_idx] = True
                results.append(track.track_id)
            else:
                track = _Track(box, hist, self._next_id)
                self._tracks.append(track)
                matched.append(True)
                results.append(self._next_id)
                self._next_id += 1

        # Update unmatched tracks
        new_tracks: List[_Track] = []
        for track, m in zip(self._tracks, matched):
            if m:
                new_tracks.append(track)
            else:
                track.missed += 1
                if track.missed < self._max_missed:
                    new_tracks.append(track)
        self._tracks = new_tracks
        return results
