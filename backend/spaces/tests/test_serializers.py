from __future__ import annotations

import pytest
from spaces.serializers import RoomSerializer


@pytest.mark.django_db
def test_room_serializer_validates_polygon_orientation():
    serializer = RoomSerializer(
        data={
            "name": "Bedroom",
            "level": 1,
            "origin_x_mm": 0,
            "origin_y_mm": 0,
            "rotation_deg": 0,
            "polygon_mm": [
                {"x_mm": 0, "y_mm": 0},
                {"x_mm": 4000, "y_mm": 0},
                {"x_mm": 4000, "y_mm": 3000},
            ],
            "ceiling_height_mm": 2500,
            "metadata": {},
        }
    )
    assert serializer.is_valid(), serializer.errors
    room = serializer.save()
    assert room.polygon_mm[0] == {"x_mm": 0, "y_mm": 0}
    assert room.polygon_mm[-1] == {"x_mm": 0, "y_mm": 0}


@pytest.mark.django_db
def test_room_serializer_rejects_self_intersecting_polygon():
    serializer = RoomSerializer(
        data={
            "name": "Bad Room",
            "level": 1,
            "origin_x_mm": 0,
            "origin_y_mm": 0,
            "rotation_deg": 0,
            "polygon_mm": [
                {"x_mm": 0, "y_mm": 0},
                {"x_mm": 4, "y_mm": 4},
                {"x_mm": 0, "y_mm": 4},
                {"x_mm": 4, "y_mm": 0},
            ],
            "ceiling_height_mm": 2500,
            "metadata": {},
        }
    )
    assert not serializer.is_valid()
    assert "Polygon must be simple" in serializer.errors["polygon_mm"][0]
