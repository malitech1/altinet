"""URL configuration for Altinet backend."""

from __future__ import annotations

from django.contrib import admin
from django.urls import include, path, re_path
from drf_spectacular.views import SpectacularAPIView, SpectacularSwaggerView

urlpatterns = [
    path("admin/", admin.site.urls),
    path("api/schema/", SpectacularAPIView.as_view(), name="schema"),
    path(
        "api/docs/",
        SpectacularSwaggerView.as_view(url_name="schema"),
        name="api-docs",
    ),
    path("api/", include("spaces.urls")),
]

try:
    from .views import FrontendAppView
except ImportError:  # pragma: no cover
    FrontendAppView = None

if FrontendAppView is not None:
    urlpatterns += [
        path("", FrontendAppView.as_view(), name="frontend"),
        re_path(
            r"^(?!static/|api/|admin/).*",
            FrontendAppView.as_view(),
            name="frontend-catchall",
        ),
    ]
