"""Lighting control node that reacts to presence events."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Dict, List

try:  # pragma: no cover - optional when ROS is unavailable
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from altinet.msg import Event as EventMsg
    from altinet.msg import RoomPresence as RoomPresenceMsg
    from altinet.srv import ManualLightOverride
except Exception:  # pragma: no cover - executed during tests
    rclpy = None
    Node = object  # type: ignore
    ReentrantCallbackGroup = MultiThreadedExecutor = qos_profile_sensor_data = (
        EventMsg
    ) = RoomPresenceMsg = ManualLightOverride = None

from ..drivers.lights import LightDriver
from ..utils.types import (
    Event as EventModel,
    LightCommand,
    RoomPresence as RoomPresenceModel,
)


class LightingController:
    """Applies rule-based lighting logic."""

    def __init__(
        self,
        driver: LightDriver,
        cooldown_s: float = 30.0,
        override_duration_s: float = 300.0,
    ) -> None:
        self.driver = driver
        self.cooldown = timedelta(seconds=cooldown_s)
        self.override_duration = timedelta(seconds=override_duration_s)
        self.room_counts: Dict[str, int] = {}
        self.pending_off: Dict[str, datetime] = {}
        self.overrides: Dict[str, datetime] = {}

    def update_presence(self, presence: RoomPresenceModel) -> None:
        """Update the known person count for a room."""

        self.room_counts[presence.room_id] = presence.count
        if presence.count > 0 and presence.room_id in self.pending_off:
            self.pending_off.pop(presence.room_id, None)

    def handle_event(self, event: EventModel) -> List[LightCommand]:
        """Process an event and return the issued light commands."""

        now = datetime.utcnow()
        override_until = self.overrides.get(event.room_id)
        commands: List[LightCommand] = []
        if override_until and override_until > now:
            return commands

        if event.type == "ENTRY":
            if self.room_counts.get(event.room_id, 0) > 0:
                command = LightCommand(
                    room_id=event.room_id, state=True, source="rules", timestamp=now
                )
                self.driver.set_state(command)
                commands.append(command)
        elif event.type == "EXIT":
            if self.room_counts.get(event.room_id, 0) == 0:
                self.pending_off[event.room_id] = now + self.cooldown
        return commands

    def tick(self) -> List[LightCommand]:
        """Evaluate pending timers and return issued commands."""

        now = datetime.utcnow()
        commands: List[LightCommand] = []
        for room_id, deadline in list(self.pending_off.items()):
            if deadline <= now and self.overrides.get(room_id, datetime.min) <= now:
                command = LightCommand(
                    room_id=room_id, state=False, source="rules", timestamp=now
                )
                self.driver.set_state(command)
                commands.append(command)
                self.pending_off.pop(room_id, None)
        return commands

    def manual_override(self, room_id: str, state: bool) -> LightCommand:
        """Apply a manual override."""

        now = datetime.utcnow()
        self.overrides[room_id] = now + self.override_duration
        command = LightCommand(
            room_id=room_id, state=state, source="manual", timestamp=now
        )
        self.driver.set_state(command)
        return command


class LightingControlNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node orchestrating lighting rules."""

    def __init__(self) -> None:
        super().__init__("lighting_control_node")
        self.declare_parameter("cooldown_s", 30.0)
        self.declare_parameter("override_duration_s", 300.0)
        driver = LightDriver()
        self.controller = LightingController(
            driver,
            cooldown_s=float(self.get_parameter("cooldown_s").value),
            override_duration_s=float(self.get_parameter("override_duration_s").value),
        )
        self.callback_group = ReentrantCallbackGroup()
        self.event_sub = self.create_subscription(
            EventMsg,
            "/altinet/events",
            self._on_event,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.presence_sub = self.create_subscription(
            RoomPresenceMsg,
            "/altinet/room_presence",
            self._on_presence,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(
            1.0, self._on_timer, callback_group=self.callback_group
        )
        self.service = self.create_service(
            ManualLightOverride,
            "/altinet/manual_light_override",
            self._on_manual_override,
            callback_group=self.callback_group,
        )

    def _on_event(self, msg: EventMsg) -> None:
        event = EventModel(
            type=msg.type,
            subject_id=msg.subject_id,
            room_id=msg.room_id,
            timestamp=datetime.utcnow(),
            payload={},
        )
        self.controller.handle_event(event)

    def _on_presence(self, msg: RoomPresenceMsg) -> None:
        presence = RoomPresenceModel(
            room_id=msg.room_id,
            track_ids=list(msg.track_ids),
            timestamp=datetime.utcnow(),
        )
        self.controller.update_presence(presence)

    def _on_timer(self) -> None:
        self.controller.tick()

    def _on_manual_override(self, request, response):
        self.controller.manual_override(request.room_id, request.state)
        response.success = True
        response.message = (
            f"Override applied until {self.controller.overrides[request.room_id]}"
        )
        return response


__all__ = ["LightingController", "LightingControlNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        raise RuntimeError("ROS 2 is not available in this environment")
    rclpy.init(args=args)
    node = LightingControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
