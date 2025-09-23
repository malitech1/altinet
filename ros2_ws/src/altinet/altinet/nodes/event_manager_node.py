"""Event manager node for generating semantic events."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

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
from ..utils.geometry import RoomGeometry, load_room_geometry, normalise_centroid
from ..utils.ros_conversions import event_to_msg, presence_to_msg
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


class EventManager:
    """Processes tracks and emits semantic events and presence summaries."""

    def __init__(
        self,
        config: EventManagerConfig,
        geometry_provider,
    ) -> None:
        self.config = config
        self.geometry_provider = geometry_provider
        self.tracks: Dict[int, Track] = {}
        self.last_positions: Dict[int, Tuple[float, float]] = {}
        self.last_seen: Dict[int, datetime] = {}
        self.rooms: set[str] = set()

    def update(
        self, tracks: Iterable[Track]
    ) -> Tuple[List[EventModel], List[RoomPresenceModel]]:
        now = datetime.utcnow()
        events: List[EventModel] = []
        room_membership: Dict[str, List[int]] = defaultdict(list)

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

        presence: List[RoomPresenceModel] = [
            RoomPresenceModel(room_id=room_id, track_ids=ids, timestamp=now)
            for room_id, ids in room_membership.items()
        ]
        for room_id in self.rooms:
            if room_id not in room_membership:
                presence.append(
                    RoomPresenceModel(room_id=room_id, track_ids=[], timestamp=now)
                )

        return events, presence


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


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
