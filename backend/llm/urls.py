"""URL routing for the LLaMA app."""

from __future__ import annotations

from django.urls import path

from .views import HealthView, LlamaPromptView

app_name = "llm"

urlpatterns = [
    path("prompt/", LlamaPromptView.as_view(), name="prompt"),
    path("health/", HealthView.as_view(), name="health"),
]
