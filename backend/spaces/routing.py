"""Websocket routing for spaces app."""

from __future__ import annotations

from django.urls import path

from . import consumers

websocket_urlpatterns = [
    path("ws/cameras/", consumers.CameraHealthConsumer.as_asgi()),
    path(
        "ws/calibration/<uuid:camera_id>",
        consumers.CalibrationProgressConsumer.as_asgi(),
    ),
    path("ws/tracks/", consumers.PersonTrackConsumer.as_asgi()),
]
