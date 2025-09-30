"""URL routing for the LLM app."""

from __future__ import annotations

from django.urls import path

from .views import HealthView, PromptView

app_name = "llm"

urlpatterns = [
    path("prompt/", PromptView.as_view(), name="prompt"),
    path("health/", HealthView.as_view(), name="health"),
]
