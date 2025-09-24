"""Helpers for converting between dataclasses and ROS messages."""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np

from .types import Detection, Event, FaceSnapshot, RoomPresence, Track

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional at test time
    from std_msgs.msg import Header
    from altinet.msg import (
        Event as EventMsg,
        FaceSnapshot as FaceSnapshotMsg,
        FaceSnapshots as FaceSnapshotsMsg,
        PersonDetection as PersonDetectionMsg,
        PersonDetections as PersonDetectionsMsg,
        PersonTrack as PersonTrackMsg,
        PersonTracks as PersonTracksMsg,
        RoomPresence as RoomPresenceMsg,
    )
    from altinet.srv import ManualLightOverride
except ImportError as exc:  # pragma: no cover - executed when ROS not available
    _ROS_IMPORT_ERROR = exc
    Header = (
        EventMsg
    ) = (
        FaceSnapshotMsg
    ) = (
        FaceSnapshotsMsg
    ) = PersonDetectionMsg = PersonDetectionsMsg = PersonTrackMsg = (
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


def face_snapshot_to_msg(snapshot: FaceSnapshot, header) -> FaceSnapshotMsg:
    if FaceSnapshotMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    msg = FaceSnapshotMsg()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.room_id = snapshot.room_id
    msg.track_id = int(snapshot.track_id)
    msg.x = float(snapshot.bbox.x)
    msg.y = float(snapshot.bbox.y)
    msg.w = float(snapshot.bbox.w)
    msg.h = float(snapshot.bbox.h)
    msg.quality_score = float(snapshot.quality.score)
    msg.sharpness = float(snapshot.quality.sharpness)
    msg.brightness = float(snapshot.quality.brightness)
    msg.pose_score = float(snapshot.quality.pose)
    msg.landmarks = list(snapshot.landmarks.flatten())
    msg.embedding = [float(value) for value in snapshot.embedding]
    face_image = np.ascontiguousarray(snapshot.face_image)
    msg.face_height = snapshot.face_height
    msg.face_width = snapshot.face_width
    msg.face_encoding = snapshot.encoding
    msg.face_data = bytes(face_image.tobytes())
    return msg


def face_snapshots_to_msg(
    snapshots: Iterable[FaceSnapshot], header
) -> FaceSnapshotsMsg:
    if FaceSnapshotsMsg is None:
        message = "ROS messages are not available in this environment"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    snapshots = list(snapshots)
    msg = FaceSnapshotsMsg()
    msg.header = header
    msg.room_id = snapshots[0].room_id if snapshots else ""
    msg.snapshots = [face_snapshot_to_msg(snapshot, header) for snapshot in snapshots]
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
    "face_snapshot_to_msg",
    "face_snapshots_to_msg",
    "presence_to_msg",
    "event_to_msg",
]
