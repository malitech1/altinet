"""WSGI config for Altinet backend."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from django.core.wsgi import get_wsgi_application

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "altinet_backend.settings")

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

application = get_wsgi_application()
