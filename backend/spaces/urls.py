"""API URL routing for spaces app."""

from __future__ import annotations

from django.urls import include, path
from rest_framework.routers import DefaultRouter

from .views import CameraViewSet, RoomViewSet

router = DefaultRouter()
router.register("rooms", RoomViewSet)
router.register("cameras", CameraViewSet)

urlpatterns = [
    path("", include(router.urls)),
]
