from __future__ import annotations

from django.contrib.auth.decorators import login_required
from django.shortcuts import render


@login_required
def home(request):
    """Render the main dashboard once the user is authenticated."""
    return render(request, "web/home.html")
