"""Hardware abstraction for controlling room lights."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict

from ..utils.types import LightCommand


@dataclass
class LightState:
    """Current state of a room light."""

    is_on: bool = False
    last_changed: datetime = field(default_factory=datetime.utcnow)
    source: str = "rules"


class LightDriver:
    """Simple in-memory light driver used in tests and development."""

    def __init__(self) -> None:
        self._state: Dict[str, LightState] = {}

    def set_state(self, command: LightCommand) -> None:
        """Apply a :class:`LightCommand` to the light."""

        self._state[command.room_id] = LightState(
            is_on=command.state,
            last_changed=command.timestamp,
            source=command.source,
        )

    def get_state(self, room_id: str) -> LightState:
        """Return the current state for ``room_id``."""

        return self._state.get(room_id, LightState())


__all__ = ["LightDriver", "LightState"]
