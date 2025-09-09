"""URL configuration for the Altinet local node GUI."""

from django.urls import path
from . import views

urlpatterns = [
    path("", views.home, name="home"),
    path("map/", views.map_builder, name="map_builder"),
    path("live/", views.live_view, name="live_view"),
]
