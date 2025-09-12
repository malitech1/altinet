"""Simple tracking of faces across frames."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from .face_recognition import FaceRecognitionService


@dataclass
class _Track:
    bbox: Tuple[int, int, int, int]
    identity: str
    confidence: float


class FaceTracker:
    """Track faces and remember identities while they remain in frame.

    Parameters
    ----------
    recognizer:
        Service used to identify new faces.
    iou_threshold:
        Minimum intersection-over-union required to match a detection to an
        existing track.
    """

    def __init__(self, recognizer: FaceRecognitionService, *, iou_threshold: float = 0.5) -> None:
        self._recognizer = recognizer
        self._iou_threshold = iou_threshold
        self._tracks: List[_Track] = []

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

    def update(self, frame, boxes: List[Tuple[int, int, int, int]]) -> List[Tuple[str, float]]:
        """Update tracks with new detections.

        Parameters
        ----------
        frame:
            Image from which ``boxes`` were detected. Used to crop faces for
            identification of new tracks.
        boxes:
            List of bounding boxes ``(x, y, w, h)`` for detected faces.

        Returns
        -------
        list of tuple
            Identity and confidence for each box in order of ``boxes``.
        """

        matched = [False] * len(self._tracks)
        results: List[Tuple[str, float]] = []
        for box in boxes:
            match_idx = None
            for i, track in enumerate(self._tracks):
                if self._iou(box, track.bbox) > self._iou_threshold:
                    match_idx = i
                    break
            if match_idx is not None:
                track = self._tracks[match_idx]
                track.bbox = box
                matched[match_idx] = True
                results.append((track.identity, track.confidence))
            else:
                x, y, w, h = box
                face_img = frame[y : y + h, x : x + w]
                identity, confidence = self._recognizer.recognize(face_img)
                self._tracks.append(_Track(box, identity, confidence))
                matched.append(True)
                results.append((identity, confidence))
        self._tracks = [t for t, m in zip(self._tracks, matched) if m]
        return results
