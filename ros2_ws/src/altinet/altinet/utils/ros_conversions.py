"""Helpers for converting between dataclasses and ROS messages."""

from __future__ import annotations

from typing import Iterable, Optional

from .types import Detection, Event, RoomPresence, Track

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional at test time
    from std_msgs.msg import Header
    from altinet.msg import (
        Event as EventMsg,
        PersonDetection as PersonDetectionMsg,
        PersonDetections as PersonDetectionsMsg,
        PersonTrack as PersonTrackMsg,
        PersonTracks as PersonTracksMsg,
        RoomPresence as RoomPresenceMsg,
    )
    from altinet.srv import ManualLightOverride
except ImportError as exc:  # pragma: no cover - executed when ROS not available
    _ROS_IMPORT_ERROR = exc
    Header = EventMsg = PersonDetectionMsg = PersonDetectionsMsg = PersonTrackMsg = (
        PersonTracksMsg
    ) = RoomPresenceMsg = ManualLightOverride = None


def detections_to_msg(detections: Iterable[Detection], header) -> PersonDetectionsMsg:
    """Convert detections to a ROS message."""

    if PersonDetectionsMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    detections = list(detections)
    msg = PersonDetectionsMsg()
    msg.header = header
    msg.room_id = detections[0].room_id if detections else ""
    msg.detections = [single_detection_to_msg(det, header) for det in detections]
    return msg


def single_detection_to_msg(detection: Detection, header) -> PersonDetectionMsg:
    if PersonDetectionMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    msg = PersonDetectionMsg()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.x = detection.bbox.x
    msg.y = detection.bbox.y
    msg.w = detection.bbox.w
    msg.h = detection.bbox.h
    msg.conf = detection.confidence
    image_height, image_width = detection.image_size
    msg.image_width = int(image_width)
    msg.image_height = int(image_height)
    msg.room_id = detection.room_id
    msg.frame_id = detection.frame_id
    return msg


def tracks_to_msg(tracks: Iterable[Track], header) -> PersonTracksMsg:
    if PersonTracksMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    tracks = list(tracks)
    msg = PersonTracksMsg()
    msg.header = header
    msg.room_id = tracks[0].room_id if tracks else ""
    msg.tracks = [single_track_to_msg(track, header) for track in tracks]
    return msg


def single_track_to_msg(track: Track, header) -> PersonTrackMsg:
    if PersonTrackMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    msg = PersonTrackMsg()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.track_id = track.track_id
    msg.x = track.bbox.x
    msg.y = track.bbox.y
    msg.w = track.bbox.w
    msg.h = track.bbox.h
    cx, cy = track.centroid()
    msg.cx = cx
    msg.cy = cy
    image_height, image_width = track.image_size
    msg.image_width = int(image_width)
    msg.image_height = int(image_height)
    msg.room_id = track.room_id
    return msg


def presence_to_msg(presence: RoomPresence, header) -> RoomPresenceMsg:
    if RoomPresenceMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    msg = RoomPresenceMsg()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.room_id = presence.room_id
    msg.count = presence.count
    msg.track_ids = list(presence.track_ids)
    return msg


def event_to_msg(event: Event, header) -> EventMsg:
    if EventMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    msg = EventMsg()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.type = event.type
    msg.subject_id = event.subject_id
    msg.room_id = event.room_id
    msg.payload_json = json_dumps(event.payload)
    return msg


def json_dumps(payload) -> str:
    import json

    return json.dumps(payload)


__all__ = [
    "detections_to_msg",
    "single_detection_to_msg",
    "tracks_to_msg",
    "single_track_to_msg",
    "presence_to_msg",
    "event_to_msg",
]
