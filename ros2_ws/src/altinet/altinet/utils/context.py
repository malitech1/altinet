"""Utilities for tracking live contextual information about rooms."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Iterable, Optional

from .types import Track


@dataclass
class PersonContext:
    """Represents a tracked person inside a room."""

    track_id: int
    last_seen: datetime
    identity_id: Optional[str] = None
    identity_confidence: float = 0.0


@dataclass
class RoomContext:
    """Aggregated room state maintained by :class:`RoomContextManager`."""

    room_id: str
    people: Dict[int, PersonContext] = field(default_factory=dict)
    last_updated: Optional[datetime] = None
    last_motion: Optional[datetime] = None
    brightness: Optional[float] = None
    light_states: Dict[str, bool] = field(default_factory=dict)
    temperature: Optional[float] = None
    metadata: Dict[str, object] = field(default_factory=dict)


class RoomContextManager:
    """Maintain live context about rooms and their occupants."""

    def __init__(self) -> None:
        self._rooms: Dict[str, RoomContext] = {}
        self._track_index: Dict[int, str] = {}

    # ------------------------------------------------------------------
    # Context updates
    # ------------------------------------------------------------------
    def update_presence(
        self,
        room_id: str,
        tracks: Iterable[Track],
        timestamp: datetime,
    ) -> None:
        """Synchronise people present in ``room_id`` with ``tracks``."""

        context = self._rooms.setdefault(room_id, RoomContext(room_id=room_id))
        context.last_updated = timestamp
        active_ids = {track.track_id for track in tracks}
        for track_id in list(context.people.keys()):
            if track_id not in active_ids:
                context.people.pop(track_id, None)
                self._track_index.pop(track_id, None)
        for track in tracks:
            person = context.people.get(track.track_id)
            if person is None:
                person = PersonContext(track_id=track.track_id, last_seen=timestamp)
                context.people[track.track_id] = person
            else:
                person.last_seen = timestamp
            if getattr(track, "identity_id", None):
                person.identity_id = track.identity_id
                person.identity_confidence = getattr(
                    track, "identity_confidence", person.identity_confidence
                )
            elif getattr(track, "identity_confidence", 0.0) > person.identity_confidence:
                person.identity_confidence = track.identity_confidence
            self._track_index[track.track_id] = room_id

    def assign_identity(
        self,
        track_id: int,
        identity_id: str,
        confidence: float,
        timestamp: Optional[datetime] = None,
    ) -> bool:
        """Assign a known identity to ``track_id`` if the track is active."""

        room_id = self._track_index.get(track_id)
        if room_id is None:
            return False
        context = self._rooms.setdefault(room_id, RoomContext(room_id=room_id))
        person = context.people.get(track_id)
        if person is None:
            if timestamp is None:
                timestamp = datetime.utcnow()
            person = PersonContext(track_id=track_id, last_seen=timestamp)
            context.people[track_id] = person
        if timestamp is None:
            timestamp = datetime.utcnow()
        person.identity_id = identity_id
        person.identity_confidence = confidence
        person.last_seen = timestamp
        context.last_updated = timestamp
        return True

    def update_environment(
        self,
        room_id: str,
        *,
        timestamp: Optional[datetime] = None,
        brightness: Optional[float] = None,
        light_states: Optional[Dict[str, bool]] = None,
        temperature: Optional[float] = None,
    ) -> None:
        """Record environmental metrics for ``room_id``."""

        context = self._rooms.setdefault(room_id, RoomContext(room_id=room_id))
        if timestamp is None:
            timestamp = datetime.utcnow()
        context.last_updated = timestamp
        if brightness is not None:
            context.brightness = float(brightness)
        if light_states is not None:
            context.light_states.update(light_states)
        if temperature is not None:
            context.temperature = float(temperature)

    def note_motion(self, room_id: str, timestamp: Optional[datetime] = None) -> None:
        """Record the latest timestamp when motion was observed."""

        context = self._rooms.setdefault(room_id, RoomContext(room_id=room_id))
        context.last_motion = timestamp or datetime.utcnow()

    # ------------------------------------------------------------------
    # Read-only access
    # ------------------------------------------------------------------
    def get_room_context(self, room_id: str) -> RoomContext:
        """Return a defensive copy of the context for ``room_id``."""

        context = self._rooms.setdefault(room_id, RoomContext(room_id=room_id))
        return copy.deepcopy(context)

    def snapshot(self) -> Dict[str, RoomContext]:
        """Return a snapshot of all known rooms."""

        return {room_id: self.get_room_context(room_id) for room_id in self._rooms}


__all__ = ["PersonContext", "RoomContext", "RoomContextManager"]

