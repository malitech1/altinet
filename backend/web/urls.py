from __future__ import annotations

from django.urls import path

from . import views

app_name = "web"

urlpatterns = [
    path("", views.home, name="home"),
    path("builder/", views.builder, name="builder"),
    path("settings/", views.settings_view, name="settings"),
]
