"""Views for the Altinet local node GUI."""

from django.shortcuts import render


def home(request):
    """Render the landing page for the local node."""
    return render(request, "index.html")
