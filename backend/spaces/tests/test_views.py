from __future__ import annotations

import pytest
from spaces.models import Camera, CameraCalibrationRun


@pytest.mark.django_db
def test_room_crud_flow(api_client):
    payload = {
        "name": "Living Room",
        "level": 1,
        "origin_x_mm": 0,
        "origin_y_mm": 0,
        "rotation_deg": 0,
        "polygon_mm": [
            {"x_mm": 0, "y_mm": 0},
            {"x_mm": 5000, "y_mm": 0},
            {"x_mm": 5000, "y_mm": 4000},
        ],
        "ceiling_height_mm": 2600,
    }
    response = api_client.post("/api/rooms/", payload, format="json")
    assert response.status_code == 201
    room_id = response.data["id"]
    response = api_client.get(f"/api/rooms/{room_id}/")
    assert response.status_code == 200
    response = api_client.post(
        f"/api/rooms/{room_id}/recenter/",
        {"origin_x_mm": 100, "origin_y_mm": 50, "rotation_deg": 5},
    )
    assert response.status_code == 200


@pytest.mark.django_db
def test_camera_endpoints(monkeypatch, api_client, room):
    class DummyBridge:
        def probe_camera(self, camera_id: str):
            return {"ok": True, "fps": 15, "resolution": "2560x1920"}

        def start_calibration(self, camera_id: str, payload):
            return {"accepted": True}

        def cancel_calibration(self, camera_id: str, payload):
            return {"cancelled": True}

    monkeypatch.setattr("spaces.views.ROSBridgeClient", lambda: DummyBridge())

    response = api_client.post(
        "/api/cameras/",
        {
            "name": "Living Cam",
            "room": str(room.id),
            "make": "Reolink",
            "model": "510A",
            "rtsp_url": "rtsp://user:pass@cam/stream",
            "resolution_w": 2560,
            "resolution_h": 1920,
            "fps": 15,
            "fov_deg": 90,
            "position_mm": {"x_mm": 0, "y_mm": 0, "z_mm": 2500},
            "yaw_deg": 45,
            "pitch_deg": -15,
            "roll_deg": 0,
        },
        format="json",
    )
    assert response.status_code == 201
    camera_id = response.data["id"]

    response = api_client.post(f"/api/cameras/{camera_id}/test-connection/", {})
    assert response.status_code == 200
    assert response.data["ok"] is True

    response = api_client.post(
        f"/api/cameras/{camera_id}/calibration/start/",
        {
            "method": CameraCalibrationRun.Method.CHECKERBOARD,
            "board_spec": {"squaresX": 5},
        },
        format="json",
    )
    assert response.status_code == 202
    run_id = response.data["run"]["id"]

    response = api_client.get(f"/api/cameras/{camera_id}/calibration/runs/")
    assert response.status_code == 200
    assert response.data[0]["id"] == run_id

    response = api_client.post(
        f"/api/cameras/{camera_id}/calibration/cancel/",
        {"run_id": run_id},
        format="json",
    )
    assert response.status_code == 200

    response = api_client.post(
        f"/api/cameras/{camera_id}/calibration/save/",
        {"intrinsics": {"fx": 1.0, "fy": 1.0}, "error_rms": 0.5},
        format="json",
    )
    assert response.status_code == 200
    camera = Camera.objects.get(id=camera_id)
    assert camera.intrinsics["fx"] == 1.0
