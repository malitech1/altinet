"""Views for the Altinet local node GUI."""

from __future__ import annotations

from datetime import datetime

from django.shortcuts import redirect, render

from .map_store import load_map, save_map
from django.shortcuts import render
 


def home(request):
    """Render the landing page for the local node."""
    return render(request, "index.html")


def map_builder(request):
    """Display and edit the stored home map."""
    data = load_map()
    nodes = data.get("nodes", [])
    if request.method == "POST":
        name = request.POST.get("name")
        x = request.POST.get("x")
        y = request.POST.get("y")
        try:
            if name and x is not None and y is not None:
                x_f = float(x)
                y_f = float(y)
                # Replace existing node with the same name
                nodes = [n for n in nodes if n.get("name") != name]
                nodes.append({"name": name, "x": x_f, "y": y_f})
                data["nodes"] = nodes
                save_map(data)
        except ValueError:
            pass
        return redirect("map_builder")
    return render(request, "map_builder.html", {"nodes": nodes})


def live_view(request):
    """Render a live view of the home with environment data."""
    context = load_map()
    context.update(
        {
            "now": datetime.now(),
            "temperature": 22,  # Placeholder values
            "occupied": True,
            "mood": "calm",
            "people": [],
            "objects": [],
            "pets": [],
        }
    )
    return render(request, "live_view.html", context)
