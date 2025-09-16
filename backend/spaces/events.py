"""Utility helpers to broadcast camera and calibration events."""

from __future__ import annotations

from typing import Any, Dict

from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer


def broadcast_camera_health(payload: Dict[str, Any]) -> None:
    channel_layer = get_channel_layer()
    if channel_layer is None:  # pragma: no cover - no channel layer configured
        return
    async_to_sync(channel_layer.group_send)(
        "camera_health", {"type": "camera_health_event", "payload": payload}
    )


def broadcast_calibration_progress(camera_id: str, payload: Dict[str, Any]) -> None:
    channel_layer = get_channel_layer()
    if channel_layer is None:  # pragma: no cover
        return
    async_to_sync(channel_layer.group_send)(
        f"calibration_{camera_id}",
        {"type": "calibration_progress_event", "payload": payload},
    )
