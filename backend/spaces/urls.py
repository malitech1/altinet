"""API URL routing for spaces app."""

from __future__ import annotations

from django.urls import include, path
from rest_framework.routers import DefaultRouter

from .views import (
    CameraViewSet,
    FaceEmbeddingViewSet,
    FaceSnapshotViewSet,
    FloorplanRenderView,
    IdentityViewSet,
    RoomViewSet,
)

router = DefaultRouter()
router.register("rooms", RoomViewSet)
router.register("cameras", CameraViewSet)
router.register("identities", IdentityViewSet)
router.register("face-embeddings", FaceEmbeddingViewSet)
router.register("face-snapshots", FaceSnapshotViewSet)

urlpatterns = [
    path("floorplans/render/", FloorplanRenderView.as_view(), name="floorplan-render"),
    path("", include(router.urls)),
]
