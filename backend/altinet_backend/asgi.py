"""ASGI config for Altinet backend."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from channels.routing import ProtocolTypeRouter, URLRouter
from django.core.asgi import get_asgi_application
from django.urls import path

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "altinet_backend.settings")

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

django_asgi_app = get_asgi_application()

from spaces import \
    routing as spaces_routing  # noqa  E402 (import after settings)

application = ProtocolTypeRouter(
    {
        "http": django_asgi_app,
        "websocket": URLRouter(spaces_routing.websocket_urlpatterns),
    }
)
