"""Bridge node forwarding ROS data to the Django backend."""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional

try:  # pragma: no cover - optional dependency
    import requests
except ImportError:  # pragma: no cover - executed when requests missing
    requests = None

try:  # pragma: no cover - optional dependency
    import websocket
except ImportError:  # pragma: no cover - executed when websocket-client missing
    websocket = None

try:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from altinet.msg import Event as EventMsg
    from altinet.msg import PersonTracks as PersonTracksMsg
    from altinet.msg import RoomPresence as RoomPresenceMsg
except Exception:  # pragma: no cover - executed in tests
    rclpy = None
    Node = object  # type: ignore
    EventMsg = PersonTracksMsg = RoomPresenceMsg = None

from ..utils.config import load_file
from ..utils.types import (
    BoundingBox,
    Event as EventModel,
    RoomPresence as RoomPresenceModel,
    Track,
)


@dataclass
class BridgeConfig:
    """Configuration for the ROS → Django bridge."""

    api_base: str
    ws_url: Optional[str]
    auth_token: Optional[str]
    forward_allowed: bool = True
    max_backoff_s: float = 30.0


class BridgeTransport:
    """Abstract transport for delivering payloads."""

    def send(self, payload: Dict[str, object]) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class RequestsTransport(BridgeTransport):
    """Sends payloads using HTTP POST requests."""

    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        if requests is None:
            raise RuntimeError("The requests package is required for HTTP transport")
        self.session = requests.Session()

    def send(self, payload: Dict[str, object]) -> None:
        url = f"{self.config.api_base.rstrip('/')}/events/"
        headers = {}
        if self.config.auth_token:
            headers["Authorization"] = f"Token {self.config.auth_token}"
        response = self.session.post(url, json=payload, headers=headers, timeout=5)
        response.raise_for_status()


class WebSocketTransport(BridgeTransport):
    """Sends payloads over a persistent WebSocket connection."""

    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        self._connection = None

    def _ensure_connection(self) -> None:
        if self._connection and self._connection.connected:
            return
        headers = []
        if self.config.auth_token:
            headers.append(f"Authorization: Token {self.config.auth_token}")
        self._connection = websocket.create_connection(
            self.config.ws_url, header=headers, timeout=5
        )

    def send(self, payload: Dict[str, object]) -> None:
        if not self.config.ws_url:
            raise RuntimeError("WebSocket URL not configured")
        if websocket is None:
            raise RuntimeError("websocket-client is required for WebSocket transport")
        try:
            self._ensure_connection()
            self._connection.send(json.dumps(payload))
        except Exception:
            if self._connection:
                try:
                    self._connection.close()
                except Exception:  # pragma: no cover - best effort
                    pass
            self._connection = None
            raise


class DjangoBridge:
    """Buffers and forwards events to the Django backend."""

    def __init__(
        self, config: BridgeConfig, transport: Optional[BridgeTransport] = None
    ) -> None:
        self.config = config
        if transport is not None:
            self.transport = transport
        elif config.ws_url:
            self.transport = WebSocketTransport(config)
        else:
            self.transport = RequestsTransport(config)
        self.queue: Deque[Dict[str, object]] = deque()
        self._last_attempt: Optional[float] = None
        self._backoff = 1.0

    def publish_event(
        self, event: EventModel, centroid: Optional[List[float]] = None
    ) -> None:
        payload = {
            "type": event.type,
            "subject_id": event.subject_id,
            "room_id": event.room_id,
            "timestamp": event.timestamp.isoformat(),
            "payload": event.payload,
        }
        if centroid is not None:
            payload["centroid"] = centroid
        self.queue.append(payload)
        self._flush()

    def publish_presence(self, presence: RoomPresenceModel) -> None:
        payload = {
            "type": "PRESENCE",
            "room_id": presence.room_id,
            "timestamp": presence.timestamp.isoformat(),
            "track_ids": presence.track_ids,
        }
        self.queue.append(payload)
        self._flush()

    def publish_tracks(
        self, room_id: str, tracks: Iterable[Track], timestamp: datetime
    ) -> None:
        for track in tracks:
            payload = {
                "type": "TRACK",
                "room_id": room_id,
                "timestamp": timestamp.isoformat(),
                "track_id": track.track_id,
                "centroid": _normalise_centroid(track),
            }
            self.queue.append(payload)
        self._flush()

    def _flush(self) -> None:
        if not self.config.forward_allowed:
            return
        now = time.monotonic()
        if self._last_attempt and now - self._last_attempt < self._backoff:
            return
        while self.queue:
            payload = self.queue[0]
            try:
                self.transport.send(payload)
            except Exception:
                self._last_attempt = time.monotonic()
                self._backoff = min(self._backoff * 2.0, self.config.max_backoff_s)
                break
            else:
                self.queue.popleft()
                self._backoff = 1.0
                self._last_attempt = None


def load_privacy_config(path: Path) -> BridgeConfig:
    data = load_file(path)
    return BridgeConfig(
        api_base=data.get("api_base", "http://localhost:8000/api"),
        ws_url=data.get("ws_url"),
        auth_token=data.get("auth_token"),
        forward_allowed=bool(data.get("forward_allowed", True)),
        max_backoff_s=float(data.get("max_backoff_s", 30.0)),
    )


class Ros2DjangoBridgeNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node that forwards events and presence updates to Django."""

    def __init__(self) -> None:
        super().__init__("ros2_django_bridge")
        self.declare_parameter("privacy_config", str(Path("config/privacy.yaml")))
        privacy_config_path = Path(self.get_parameter("privacy_config").value)
        config = load_privacy_config(privacy_config_path)
        self.bridge = DjangoBridge(config)
        self.event_sub = self.create_subscription(
            EventMsg, "/altinet/events", self._on_event, 10
        )
        self.presence_sub = self.create_subscription(
            RoomPresenceMsg, "/altinet/room_presence", self._on_presence, 10
        )
        self.tracks_sub = self.create_subscription(
            PersonTracksMsg, "/altinet/person_tracks", self._on_tracks, 10
        )

    def _on_event(self, msg: EventMsg) -> None:
        event = EventModel(
            type=msg.type,
            subject_id=msg.subject_id,
            room_id=msg.room_id,
            timestamp=datetime.utcnow(),
            payload=json.loads(msg.payload_json or "{}"),
        )
        self.bridge.publish_event(event)

    def _on_presence(self, msg: RoomPresenceMsg) -> None:
        presence = RoomPresenceModel(
            room_id=msg.room_id,
            track_ids=list(msg.track_ids),
            timestamp=datetime.utcnow(),
        )
        self.bridge.publish_presence(presence)

    def _on_tracks(self, msg: PersonTracksMsg) -> None:
        tracks = []
        for track_msg in msg.tracks:
            bbox = BoundingBox(track_msg.x, track_msg.y, track_msg.w, track_msg.h)
            tracks.append(
                Track(
                    track_id=track_msg.track_id,
                    bbox=bbox,
                    confidence=1.0,
                    room_id=msg.room_id,
                    timestamp=datetime.utcnow(),
                    image_size=(1, 1),
                )
            )
        self.bridge.publish_tracks(msg.room_id, tracks, datetime.utcnow())


def _normalise_centroid(track: Track) -> List[float]:
    width = track.image_size[1] if track.image_size[1] else 1.0
    height = track.image_size[0] if track.image_size[0] else 1.0
    cx, cy = track.centroid()
    return [cx / width, cy / height]


__all__ = [
    "DjangoBridge",
    "BridgeConfig",
    "Ros2DjangoBridgeNode",
    "load_privacy_config",
]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        raise RuntimeError("ROS 2 is not available in this environment")
    rclpy.init(args=args)
    node = Ros2DjangoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
