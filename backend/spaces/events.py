"""Utility helpers to broadcast camera and calibration events."""

from __future__ import annotations

from typing import Any, Dict

import asyncio
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer


def broadcast_camera_health(payload: Dict[str, Any]) -> None:
    channel_layer = get_channel_layer()
    if channel_layer is None:  # pragma: no cover - no channel layer configured
        return
    _send_group_message(
        channel_layer,
        "camera_health",
        {"type": "camera_health_event", "payload": payload},
    )


def broadcast_calibration_progress(camera_id: str, payload: Dict[str, Any]) -> None:
    channel_layer = get_channel_layer()
    if channel_layer is None:  # pragma: no cover
        return
    _send_group_message(
        channel_layer,
        f"calibration_{camera_id}",
        {"type": "calibration_progress_event", "payload": payload},
    )


def _send_group_message(channel_layer, group: str, message: Dict[str, Any]) -> None:
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        async_to_sync(channel_layer.group_send)(group, message)
    else:  # pragma: no cover - exercised in async tests
        loop.create_task(channel_layer.group_send(group, message))
