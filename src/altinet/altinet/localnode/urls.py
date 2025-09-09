"""URL configuration for the Altinet local node GUI."""

from django.urls import path
from . import views

urlpatterns = [
    path("", views.home, name="home"),
]
