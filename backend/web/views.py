from __future__ import annotations

from django.conf import settings
from django.contrib.auth.decorators import login_required
from django.shortcuts import render


@login_required
def home(request):
    """Render the main dashboard once the user is authenticated."""
    return render(
        request,
        "web/home.html",
        {
            "home_model_path": getattr(settings, "HOME_MODEL_STATIC_PATH", "web/models/home.obj"),
        },
    )


@login_required
def builder(request):
    """Display the interactive home builder canvas."""
    return render(request, "web/builder.html")
