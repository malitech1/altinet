"""Event manager node for generating semantic events."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field, replace
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS 2 is not available
    import rclpy
    from rclpy.node import Node
    from altinet.msg import Event as EventMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
    from altinet.msg import RoomPresence as RoomPresenceMsg
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    EventMsg = PersonTracksMsg = RoomPresenceMsg = None

from ..dashboards.floorplan_adapter import FloorplanAdapter
from ..utils.context import RoomContextManager
from ..utils.geometry import RoomGeometry, load_room_geometry, normalise_centroid
from ..utils.ros_conversions import event_to_msg, presence_to_msg
from ..utils.scene_monitor import SceneActivityConfig, SceneChangeDetector
from ..utils.types import (
    BoundingBox,
    Event as EventModel,
    RoomPresence as RoomPresenceModel,
    Track,
)


@dataclass
class EventManagerConfig:
    """Configuration options for :class:`EventManager`."""

    presence_timeout_s: float = 2.0
    position_threshold: float = 0.05
    scene_activity: SceneActivityConfig = field(default_factory=SceneActivityConfig)


class EventManager:
    """Processes tracks and emits semantic events and presence summaries."""

    def __init__(
        self,
        config: EventManagerConfig,
        geometry_provider,
        *,
        scene_detector: SceneChangeDetector | None = None,
        context_manager: RoomContextManager | None = None,
    ) -> None:
        self.config = config
        self.geometry_provider = geometry_provider
        self.tracks: Dict[int, Track] = {}
        self.last_positions: Dict[int, Tuple[float, float]] = {}
        self.last_seen: Dict[int, datetime] = {}
        self.rooms: set[str] = set()
        self.context = context_manager or RoomContextManager()
        self.scene_detector = scene_detector or SceneChangeDetector(config.scene_activity)
        self._room_presence: Dict[str, bool] = {}

    def update(
        self, tracks: Iterable[Track]
    ) -> Tuple[List[EventModel], List[RoomPresenceModel]]:
        now = datetime.utcnow()
        events: List[EventModel] = []
        room_membership: Dict[str, List[int]] = defaultdict(list)
        tracks_by_room: Dict[str, List[Track]] = defaultdict(list)

        for track in tracks:
            self.tracks[track.track_id] = track
            self.last_seen[track.track_id] = now
            self.rooms.add(track.room_id)
            geometry = self.geometry_provider(track.room_id)
            centroid = normalise_centroid(track.bbox, track.image_size, geometry)
            previous_centroid = self.last_positions.get(track.track_id)
            if track.track_id not in self.last_positions:
                events.append(
                    EventModel(
                        type="ENTRY",
                        subject_id=str(track.track_id),
                        room_id=track.room_id,
                        timestamp=now,
                        payload={"centroid": centroid},
                    )
                )
            elif (
                _distance(previous_centroid, centroid) >= self.config.position_threshold
            ):
                events.append(
                    EventModel(
                        type="POSITION_CHANGE",
                        subject_id=str(track.track_id),
                        room_id=track.room_id,
                        timestamp=now,
                        payload={"centroid": centroid, "previous": previous_centroid},
                    )
                )
            self.last_positions[track.track_id] = centroid
            room_membership[track.room_id].append(track.track_id)
            tracks_by_room[track.room_id].append(track)

        # Handle timeouts
        expired_tracks = [
            track_id
            for track_id, seen in self.last_seen.items()
            if now - seen > timedelta(seconds=self.config.presence_timeout_s)
        ]
        for track_id in expired_tracks:
            track = self.tracks.pop(track_id, None)
            self.last_seen.pop(track_id, None)
            self.last_positions.pop(track_id, None)
            if track is None:
                continue
            events.append(
                EventModel(
                    type="EXIT",
                    subject_id=str(track_id),
                    room_id=track.room_id,
                    timestamp=now,
                    payload={},
                )
            )
            tracks_by_room.setdefault(track.room_id, [])

        presence: List[RoomPresenceModel] = [
            RoomPresenceModel(room_id=room_id, track_ids=ids, timestamp=now)
            for room_id, ids in room_membership.items()
        ]
        for room_id in self.rooms:
            if room_id not in room_membership:
                presence.append(
                    RoomPresenceModel(room_id=room_id, track_ids=[], timestamp=now)
                )
                tracks_by_room.setdefault(room_id, [])

        for room_id, room_tracks in tracks_by_room.items():
            self.context.update_presence(room_id, room_tracks, now)

        for room_id in self.rooms:
            has_people = bool(tracks_by_room.get(room_id))
            was_present = self._room_presence.get(room_id, False)
            if has_people and not was_present:
                self.scene_detector.notify_detection_result(room_id, True)
            elif not has_people and was_present:
                self.scene_detector.notify_detection_result(room_id, False)
            self._room_presence[room_id] = has_people

        return events, presence

    def observe_frame(
        self,
        room_id: str,
        frame: np.ndarray,
        timestamp: Optional[datetime] = None,
    ) -> bool:
        """Observe a camera frame and decide whether to run detection."""

        timestamp = timestamp or datetime.utcnow()
        decision = self.scene_detector.observe(room_id, frame)
        brightness = _estimate_brightness(frame)
        self.context.update_environment(
            room_id,
            timestamp=timestamp,
            brightness=brightness,
        )
        if decision.motion_detected:
            self.context.note_motion(room_id, timestamp)
        return decision.should_detect

    def assign_identity(
        self,
        track_id: int,
        identity_id: str,
        confidence: float,
        timestamp: Optional[datetime] = None,
    ) -> bool:
        """Record that ``track_id`` corresponds to ``identity_id``."""

        timestamp = timestamp or datetime.utcnow()
        track = self.tracks.get(track_id)
        assigned = self.context.assign_identity(
            track_id, identity_id, confidence, timestamp
        )
        if track is None:
            return assigned
        updated = replace(
            track,
            identity_id=identity_id,
            identity_confidence=confidence,
            timestamp=timestamp,
        )
        self.tracks[track_id] = updated
        self.last_seen[track_id] = timestamp
        return True


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _estimate_brightness(frame: np.ndarray) -> float:
    """Return the normalised brightness of ``frame`` in ``[0.0, 1.0]``."""

    if not isinstance(frame, np.ndarray) or frame.size == 0:
        return 0.0
    arr = np.asarray(frame, dtype=np.float32)
    if arr.ndim == 3 and arr.shape[2] > 0:
        arr = arr.mean(axis=2)
    max_value = 255.0 if not np.issubdtype(arr.dtype, np.floating) else 1.0
    if max_value == 0:
        return 0.0
    mean = float(np.mean(arr)) / max_value
    return float(np.clip(mean, 0.0, 1.0))


class EventManagerNode(Node):  # pragma: no cover - requires ROS 2 runtime
    """ROS 2 wrapper around :class:`EventManager`."""

    def __init__(self) -> None:
        super().__init__("event_manager_node")
        self.declare_parameter("calibration_dir", str(Path("assets/calibration")))
        self.declare_parameter(
            "floorplan_path", str(Path("assets/floorplans/floorplan.json"))
        )
        self.declare_parameter("presence_timeout_s", 2.0)
        self.declare_parameter("position_threshold", 0.05)

        calibration_dir = Path(self.get_parameter("calibration_dir").value)
        floorplan_path = Path(self.get_parameter("floorplan_path").value)
        self.floorplan = FloorplanAdapter(floorplan_path)
        config = EventManagerConfig(
            presence_timeout_s=float(self.get_parameter("presence_timeout_s").value),
            position_threshold=float(self.get_parameter("position_threshold").value),
        )

        def geometry_provider(room_id: str) -> Optional[RoomGeometry]:
            geometry = self.floorplan.get_geometry(room_id)
            if geometry is None:
                geometry = load_room_geometry(calibration_dir, room_id)
            return geometry

        self.manager = EventManager(config=config, geometry_provider=geometry_provider)

        self.tracks_sub = self.create_subscription(
            PersonTracksMsg,
            "/altinet/person_tracks",
            self._on_tracks,
            10,
        )
        self.events_pub = self.create_publisher(EventMsg, "/altinet/events", 10)
        self.presence_pub = self.create_publisher(
            RoomPresenceMsg, "/altinet/room_presence", 10
        )

    def _on_tracks(self, msg) -> None:
        now = datetime.utcnow()
        tracks = []
        for track_msg in msg.tracks:
            bbox = BoundingBox(track_msg.x, track_msg.y, track_msg.w, track_msg.h)
            image_height = int(getattr(track_msg, "image_height", 0)) or 1
            image_width = int(getattr(track_msg, "image_width", 0)) or 1
            track = Track(
                track_id=track_msg.track_id,
                bbox=bbox,
                confidence=1.0,
                room_id=msg.room_id,
                timestamp=now,
                image_size=(image_height, image_width),
            )
            tracks.append(track)
        events, presences = self.manager.update(tracks)
        header = msg.header
        for event in events:
            self.events_pub.publish(event_to_msg(event, header))
        for presence in presences:
            self.presence_pub.publish(presence_to_msg(presence, header))


__all__ = ["EventManager", "EventManagerConfig", "EventManagerNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = EventManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
