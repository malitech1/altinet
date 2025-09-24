"""Compatibility wrappers for Altinet ROS 2 services."""

from __future__ import annotations

try:

    import altinet_interfaces.srv as _altinet_srv

    from altinet_interfaces.srv import (
        CheckPersonIdentity,
        ManualLightOverride,
        RequestFaceSnapshot,
    )

except ImportError as exc:  # pragma: no cover - requires ROS interfaces
    raise ImportError(
        "altinet_interfaces.srv could not be imported. "
        "Ensure the Altinet ROS 2 interfaces package is built and sourced."
    ) from exc

ManualLightOverride = _altinet_srv.ManualLightOverride
CheckPersonIdentity = _altinet_srv.CheckPersonIdentity
FaceEnrolment = getattr(_altinet_srv, "FaceEnrolment", None)

__all__ = ["ManualLightOverride", "CheckPersonIdentity", "FaceEnrolment"]

__all__ = ["ManualLightOverride", "CheckPersonIdentity", "RequestFaceSnapshot"]

