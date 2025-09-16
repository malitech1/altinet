"""Geometry helpers for projecting detections into room coordinates."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

from .config import load_file
from .types import BoundingBox


@dataclass
class RoomGeometry:
    """Holds calibration information for a room."""

    room_id: str
    homography: Optional[np.ndarray]
    roi: Optional[Sequence[Tuple[float, float]]]

    def project_point(self, pixel_point: Tuple[float, float]) -> Tuple[float, float]:
        """Project a pixel point into normalised room coordinates."""

        if self.homography is None:
            return pixel_point

        src = np.array([[pixel_point[0], pixel_point[1], 1.0]], dtype=np.float32).T
        dst = self.homography @ src
        dst /= dst[2, :]
        return float(dst[0, 0]), float(dst[1, 0])

    def contains(self, point: Tuple[float, float]) -> bool:
        """Return ``True`` if a point is within the configured ROI."""

        if not self.roi:
            return True
        polygon = np.asarray(self.roi, dtype=np.float32)
        return _point_in_polygon(point, polygon)


def load_room_geometry(calibration_dir: Path, room_id: str) -> RoomGeometry:
    """Load calibration metadata for a room.

    Args:
        calibration_dir: Directory containing calibration ``*.json`` files.
        room_id: Identifier of the room.

    Returns:
        Loaded :class:`RoomGeometry` instance. When a calibration file is not
        found, a default configuration with ``None`` values is returned to keep
        the pipeline operational in development environments.
    """

    calibration_path = calibration_dir / f"{room_id}.json"
    if not calibration_path.exists():
        return RoomGeometry(room_id=room_id, homography=None, roi=None)

    data = load_file(calibration_path)

    homography = None
    if "homography" in data:
        matrix = np.asarray(data["homography"], dtype=np.float32)
        if matrix.size == 9:
            homography = matrix.reshape((3, 3))

    roi = None
    if "roi" in data and isinstance(data["roi"], Iterable):
        roi = [(float(pt[0]), float(pt[1])) for pt in data["roi"]]

    return RoomGeometry(room_id=room_id, homography=homography, roi=roi)


def normalise_centroid(
    bbox: BoundingBox,
    image_shape: Tuple[int, int],
    geometry: Optional[RoomGeometry] = None,
) -> Tuple[float, float]:
    """Convert a bounding box centroid to normalised room coordinates."""

    cx, cy = bbox.centroid()
    if geometry is not None and geometry.homography is not None:
        cx, cy = geometry.project_point((cx, cy))

    height, width = image_shape[:2]
    return cx / float(width), cy / float(height)


def _point_in_polygon(point: Tuple[float, float], polygon: np.ndarray) -> bool:
    """Return ``True`` when a point is inside the polygon."""

    x, y = point
    num = len(polygon)
    j = num - 1
    inside = False
    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


__all__ = ["RoomGeometry", "load_room_geometry", "normalise_centroid"]
