"""Compatibility wrappers for Altinet ROS 2 services."""

from __future__ import annotations

try:
    from altinet_interfaces.srv import CheckPersonIdentity, ManualLightOverride
except ImportError as exc:  # pragma: no cover - requires ROS interfaces
    raise ImportError(
        "altinet_interfaces.srv could not be imported. "
        "Ensure the Altinet ROS 2 interfaces package is built and sourced."
    ) from exc

__all__ = ["ManualLightOverride", "CheckPersonIdentity"]
