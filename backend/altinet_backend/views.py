"""Site-level views for the Altinet Django project."""

from __future__ import annotations

from typing import Any

from django.views.generic import TemplateView


class DashboardView(TemplateView):
    """Render the primary dashboard landing page."""

    template_name = "dashboard/index.html"

    def get_context_data(self, **kwargs: Any) -> dict[str, Any]:
        context = super().get_context_data(**kwargs)
        context.update(
            {
                "primary_links": [
                    {
                        "href": "/spaces/",
                        "label": "Manage Spaces",
                        "description": "Create rooms, configure cameras, and review calibration runs.",
                    },
                    {
                        "href": "/api/docs/",
                        "label": "API Reference",
                        "description": "Explore the REST endpoints backing the Altinet platform.",
                    },
                    {
                        "href": "/admin/",
                        "label": "Admin Site",
                        "description": "Manage Django auth users, permissions, and persisted resources.",
                    },
                ],
                "coming_soon": [
                    {
                        "label": "Presence Monitor",
                        "description": "Real-time view of who is currently detected across all rooms.",
                    },
                    {
                        "label": "Face Gallery",
                        "description": "Browse captured embeddings and snapshots from recent events.",
                    },
                ],
            }
        )
        return context
