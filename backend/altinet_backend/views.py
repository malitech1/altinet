"""Views for serving the compiled frontend through Django."""

from __future__ import annotations

from pathlib import Path

from django.conf import settings
from django.http import FileResponse, Http404
from django.views import View


class FrontendAppView(View):
    """Serve the compiled single-page app index.html from the dist folder."""

    def get(self, request, *args, **kwargs):  # type: ignore[override]
        index_file: Path = settings.FRONTEND_DIST_DIR / "index.html"
        if not index_file.exists():
            raise Http404("Frontend build not found. Run `npm run build` inside frontend/.")

        return FileResponse(index_file.open("rb"))
