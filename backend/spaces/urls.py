"""API URL routing for spaces app."""

from __future__ import annotations

from django.urls import include, path
from rest_framework.routers import DefaultRouter

from .views import (
    CameraViewSet,
    FaceEmbeddingViewSet,
    FaceSnapshotViewSet,
    IdentityViewSet,
    RoomViewSet,
)
from .auth_views import AuthStatusView, LoginView, LogoutView, RegisterView
from .event_views import EventIngestView

router = DefaultRouter()
router.register("rooms", RoomViewSet)
router.register("cameras", CameraViewSet)
router.register("identities", IdentityViewSet)
router.register("face-embeddings", FaceEmbeddingViewSet)
router.register("face-snapshots", FaceSnapshotViewSet)

urlpatterns = [
    path("auth/status/", AuthStatusView.as_view(), name="auth-status"),
    path("auth/register/", RegisterView.as_view(), name="auth-register"),
    path("auth/login/", LoginView.as_view(), name="auth-login"),
    path("auth/logout/", LogoutView.as_view(), name="auth-logout"),
    path("events/", EventIngestView.as_view(), name="events-ingest"),
    path("", include(router.urls)),
]
