"""Utilities for mapping floorplan builder data to runtime geometry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, Optional

import numpy as np

from ..utils.geometry import RoomGeometry


class FloorplanAdapter:
    """Load and provide access to floorplan metadata."""

    def __init__(self, floorplan_path: Path) -> None:
        self.floorplan_path = floorplan_path
        self._rooms: Dict[str, RoomGeometry] = {}
        self._load()

    def _load(self) -> None:
        if not self.floorplan_path.exists():
            self._rooms = {}
            return

        with self.floorplan_path.open("r", encoding="utf8") as fh:
            data = json.load(fh)
        rooms = data.get("rooms", [])
        loaded: Dict[str, RoomGeometry] = {}
        for room in rooms:
            room_id = room["id"]
            homography = room.get("homography")
            roi = room.get("polygon")
            loaded[room_id] = RoomGeometry(
                room_id=room_id,
                homography=None if homography is None else _reshape_matrix(homography),
                roi=[tuple(vertex) for vertex in roi] if roi else None,
            )
        self._rooms = loaded

    def get_geometry(self, room_id: str) -> Optional[RoomGeometry]:
        """Return the geometry for ``room_id`` if available."""

        return self._rooms.get(room_id)

    def rooms(self) -> Iterable[str]:
        """Return the identifiers of all rooms with geometry."""

        return self._rooms.keys()


def _reshape_matrix(values):
    if values is None:
        return None
    flat = [float(v) for row in values for v in row]
    if len(flat) != 9:
        return None
    return np.asarray([flat[0:3], flat[3:6], flat[6:9]], dtype=np.float32)


__all__ = ["FloorplanAdapter"]
