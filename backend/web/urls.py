from __future__ import annotations

from django.urls import path

from . import views

app_name = "web"

urlpatterns = [
    path("", views.home, name="home"),
    path("builder/", views.builder, name="builder"),
    path("training/", views.training, name="training"),
    path("settings/", views.settings_view, name="settings"),
    path("api/weather/", views.weather_snapshot, name="weather-snapshot"),
    path("api/training/", views.training_create, name="training-create"),
]
