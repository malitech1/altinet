from __future__ import annotations

import sys
from pathlib import Path

import pytest
from rest_framework.test import APIClient

BASE_DIR = Path(__file__).resolve().parents[2]
BACKEND_DIR = BASE_DIR / "backend"
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from spaces.models import Room


@pytest.fixture
def api_client() -> APIClient:
    return APIClient()


@pytest.fixture
def room() -> Room:
    return Room.objects.create(
        name="Test Room",
        level=1,
        origin_x_mm=0,
        origin_y_mm=0,
        rotation_deg=0,
        polygon_mm=[
            {"x_mm": 0, "y_mm": 0},
            {"x_mm": 4000, "y_mm": 0},
            {"x_mm": 4000, "y_mm": 3000},
            {"x_mm": 0, "y_mm": 3000},
            {"x_mm": 0, "y_mm": 0},
        ],
        ceiling_height_mm=2500,
    )
