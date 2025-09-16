"""Compatibility wrappers for Altinet ROS 2 messages."""

from __future__ import annotations

try:
    from altinet_interfaces.msg import (
        Event,
        PersonDetection,
        PersonDetections,
        PersonTrack,
        PersonTracks,
        RoomPresence,
    )
except ImportError as exc:  # pragma: no cover - requires ROS interfaces
    raise ImportError(
        "altinet_interfaces.msg could not be imported. "
        "Ensure the Altinet ROS 2 interfaces package is built and sourced."
    ) from exc

__all__ = [
    "Event",
    "PersonDetection",
    "PersonDetections",
    "PersonTrack",
    "PersonTracks",
    "RoomPresence",
]
