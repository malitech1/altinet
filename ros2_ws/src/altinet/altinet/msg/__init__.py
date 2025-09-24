"""Compatibility wrappers for Altinet ROS 2 messages."""

from __future__ import annotations

try:
    import altinet_interfaces.msg as _altinet_msgs
except ImportError as exc:  # pragma: no cover - requires ROS interfaces
    raise ImportError(
        "altinet_interfaces.msg could not be imported. "
        "Ensure the Altinet ROS 2 interfaces package is built and sourced."
    ) from exc

Event = _altinet_msgs.Event
PersonDetection = _altinet_msgs.PersonDetection
PersonDetections = _altinet_msgs.PersonDetections
PersonTrack = _altinet_msgs.PersonTrack
PersonTracks = _altinet_msgs.PersonTracks
RoomPresence = _altinet_msgs.RoomPresence
FaceSnapshot = getattr(_altinet_msgs, "FaceSnapshot", None)
FaceEnrolment = getattr(_altinet_msgs, "FaceEnrolment", None)

__all__ = [
    "Event",
    "PersonDetection",
    "PersonDetections",
    "PersonTrack",
    "PersonTracks",
    "RoomPresence",
    "FaceSnapshot",
    "FaceEnrolment",
]
