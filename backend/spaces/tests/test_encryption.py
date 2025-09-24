from __future__ import annotations

import pytest
from django.db import connection
from spaces.models import Camera


@pytest.mark.django_db
def test_rtsp_url_encrypted_at_rest(room):
    camera = Camera.objects.create(
        name="Room Cam",
        room=room,
        make="Reolink",
        model="510A",
        rtsp_url="rtsp://user:pass@example/stream",
        resolution_w=2560,
        resolution_h=1920,
        fps=15,
        fov_deg=90,
        position_mm={"x_mm": 0, "y_mm": 0, "z_mm": 2500},
        yaw_deg=45,
        pitch_deg=-15,
        roll_deg=0,
    )
    camera.refresh_from_db()
    assert camera.rtsp_url == "rtsp://user:pass@example/stream"
    assert camera.is_rtsp_encrypted is True
    with connection.cursor() as cursor:
        cursor.execute(
            "SELECT rtsp_url FROM spaces_camera WHERE id=%s", [camera.id.hex]
        )
        raw_value = cursor.fetchone()[0]
    assert raw_value != "rtsp://user:pass@example/stream"
    assert raw_value
