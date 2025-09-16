"""Adapters for the ROS2 LocalNode bridge."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

import httpx
from django.conf import settings

logger = logging.getLogger(__name__)


class ROSBridgeClient:
    """Lightweight HTTP client for the ROS bridge facade."""

    def __init__(self, base_url: Optional[str] = None) -> None:
        self.base_url = base_url or settings.ROS_BRIDGE_BASE_URL.rstrip("/")

    def _post(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        url = f"{self.base_url}{path}"
        try:
            response = httpx.post(url, json=payload, timeout=5.0)
            response.raise_for_status()
            return response.json()
        except Exception as exc:  # pragma: no cover - defensive logging
            logger.warning("ROS bridge POST %s failed: %s", url, exc)
            # TODO: replace with real error propagation once bridge is available.
            return {"ok": False, "message": str(exc)}

    def probe_camera(self, camera_id: str) -> Dict[str, Any]:
        return self._post(f"/ros/camera/{camera_id}/probe", {})

    def start_calibration(
        self, camera_id: str, payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        return self._post(f"/ros/camera/{camera_id}/calibrate", payload)

    def cancel_calibration(
        self, camera_id: str, payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        return self._post(f"/ros/camera/{camera_id}/cancel", payload)

    def list_cameras(self) -> Dict[str, Any]:
        return self._post("/ros/camera/list", {})
