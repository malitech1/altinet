"""Django settings for the Altinet backend."""

from __future__ import annotations

import base64
import hashlib
import os
from pathlib import Path
from typing import Any, Dict

from django.core.management.utils import get_random_secret_key

BASE_DIR = Path(__file__).resolve().parent.parent

# Core configuration
SECRET_KEY = os.environ.get("DJANGO_SECRET_KEY", get_random_secret_key())
# Default to Django debug mode in development unless explicitly disabled via
# the environment. This keeps `runserver` behaviour (like serving static files
# from `backend/web/static`) without requiring developers to set
# `DJANGO_DEBUG=true`.
DEBUG = os.environ.get("DJANGO_DEBUG", "true").lower() == "true"
ALLOWED_HOSTS: list[str] = os.environ.get("DJANGO_ALLOWED_HOSTS", "*").split(",")

# Applications
INSTALLED_APPS = [
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "rest_framework",
    "django_filters",
    "drf_spectacular",
    "channels",
    "llm",
    "spaces",
    "web",
]

MIDDLEWARE = [
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.common.CommonMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.contrib.auth.middleware.AuthenticationMiddleware",
    "django.contrib.messages.middleware.MessageMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
]

ROOT_URLCONF = "altinet_backend.urls"

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [BASE_DIR / "templates"],
        "APP_DIRS": True,
        "OPTIONS": {
            "context_processors": [
                "django.template.context_processors.debug",
                "django.template.context_processors.request",
                "django.contrib.auth.context_processors.auth",
                "django.contrib.messages.context_processors.messages",
            ],
        },
    }
]

WSGI_APPLICATION = "altinet_backend.wsgi.application"
ASGI_APPLICATION = "altinet_backend.asgi.application"

# Database
if os.environ.get("POSTGRES_DB"):
    DATABASES: Dict[str, Dict[str, Any]] = {
        "default": {
            "ENGINE": "django.db.backends.postgresql",
            "NAME": os.environ.get("POSTGRES_DB"),
            "USER": os.environ.get("POSTGRES_USER", "postgres"),
            "PASSWORD": os.environ.get("POSTGRES_PASSWORD", "postgres"),
            "HOST": os.environ.get("POSTGRES_HOST", "localhost"),
            "PORT": os.environ.get("POSTGRES_PORT", "5432"),
        }
    }
else:
    DATABASES = {
        "default": {
            "ENGINE": "django.db.backends.sqlite3",
            "NAME": BASE_DIR / "db.sqlite3",
        }
    }

# Password validation
AUTH_PASSWORD_VALIDATORS = [
    {
        "NAME": "django.contrib.auth.password_validation.UserAttributeSimilarityValidator"
    },
    {"NAME": "django.contrib.auth.password_validation.MinimumLengthValidator"},
    {"NAME": "django.contrib.auth.password_validation.CommonPasswordValidator"},
    {"NAME": "django.contrib.auth.password_validation.NumericPasswordValidator"},
]

LANGUAGE_CODE = "en-us"
TIME_ZONE = "UTC"
USE_I18N = True
USE_TZ = True

STATIC_URL = "/static/"
STATIC_ROOT = BASE_DIR / "static"
DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"

LOGIN_REDIRECT_URL = "web:home"
LOGOUT_REDIRECT_URL = "login"

# Path to the OBJ model displayed on the dashboard viewer. Overridable via
# ALTINET_HOME_MODEL environment variable so Blender exports can update the
# served asset without touching templates.
HOME_MODEL_STATIC_PATH = os.environ.get("ALTINET_HOME_MODEL", "web/models/home.obj")

# Floorplan automation
REPO_ROOT = BASE_DIR.parent
_default_plan_path = REPO_ROOT / "assets" / "floorplans" / "latest.json"
_default_obj_path = BASE_DIR / "web" / "static" / HOME_MODEL_STATIC_PATH
FLOORPLAN_PLAN_STORAGE_PATH = Path(
    os.environ.get("ALTINET_FLOORPLAN_PLAN", str(_default_plan_path))
)
FLOORPLAN_OBJ_OUTPUT_PATH = Path(
    os.environ.get("ALTINET_FLOORPLAN_OBJ", str(_default_obj_path))
)
FLOORPLAN_WALL_HEIGHT = float(os.environ.get("ALTINET_FLOORPLAN_WALL_HEIGHT", "3.0"))
FLOORPLAN_WALL_THICKNESS = float(
    os.environ.get("ALTINET_FLOORPLAN_WALL_THICKNESS", "0.2")
)
FLOORPLAN_STOREY_HEIGHT = float(os.environ.get("ALTINET_FLOORPLAN_STOREY_HEIGHT", "3.2"))
FLOORPLAN_FLOOR_THICKNESS = float(
    os.environ.get("ALTINET_FLOORPLAN_FLOOR_THICKNESS", "0.15")
)
FLOORPLAN_FALLBACK_WIDTH = float(
    os.environ.get("ALTINET_FLOORPLAN_FALLBACK_WIDTH", "10.0")
)
FLOORPLAN_FALLBACK_DEPTH = float(
    os.environ.get("ALTINET_FLOORPLAN_FALLBACK_DEPTH", "8.0")
)

# REST Framework
REST_FRAMEWORK = {
    "DEFAULT_SCHEMA_CLASS": "drf_spectacular.openapi.AutoSchema",
    "DEFAULT_FILTER_BACKENDS": [
        "django_filters.rest_framework.DjangoFilterBackend",
        "rest_framework.filters.OrderingFilter",
        "rest_framework.filters.SearchFilter",
    ],
    "DEFAULT_PAGINATION_CLASS": "rest_framework.pagination.PageNumberPagination",
    "PAGE_SIZE": 25,
}

SPECTACULAR_SETTINGS = {
    "TITLE": "Altinet Spaces API",
    "DESCRIPTION": "Rooms, cameras and calibration management APIs for the Altinet stack.",
    "VERSION": "1.0.0",
    "SERVE_PERMISSIONS": [],
}

# Channels (in-memory layer is fine for dev/testing)
CHANNEL_LAYERS = {
    "default": {
        "BACKEND": "channels.layers.InMemoryChannelLayer",
    }
}

# Encryption
FERNET_SECRET = os.environ.get("FERNET_SECRET_KEY", SECRET_KEY)


def derive_fernet_key(secret: str) -> bytes:
    digest = hashlib.sha256(secret.encode("utf-8")).digest()
    return base64.urlsafe_b64encode(digest)


FERNET_DERIVED_KEY = derive_fernet_key(FERNET_SECRET)

# ROS bridge endpoints
ROS_BRIDGE_BASE_URL = os.environ.get("ROS_BRIDGE_URL", "http://localhost:8001")
ROS_BRIDGE_WS_URL = os.environ.get("ROS_BRIDGE_WS_URL", "ws://localhost:8001")

# Offline LLaMA configuration
_default_llama_model = BASE_DIR.parent / "assets" / "models" / "tinyllama-1.1b-chat-v1.0.Q2_K.gguf"
_configured_llama_model = os.environ.get("ALTINET_LLAMA_MODEL_PATH")
if _configured_llama_model:
    LLAMA_MODEL_PATH = _configured_llama_model
elif _default_llama_model.exists():
    LLAMA_MODEL_PATH = str(_default_llama_model)
else:
    LLAMA_MODEL_PATH = ""
LLAMA_CONTEXT_WINDOW = int(os.environ.get("ALTINET_LLAMA_CONTEXT", "2048"))
LLAMA_MAX_TOKENS = int(os.environ.get("ALTINET_LLAMA_MAX_TOKENS", "256"))
LLAMA_TEMPERATURE = float(os.environ.get("ALTINET_LLAMA_TEMPERATURE", "0.7"))
_llama_stop_sequences = os.environ.get("ALTINET_LLAMA_STOP_SEQUENCES", "")
if _llama_stop_sequences:
    LLAMA_STOP_SEQUENCES = tuple(
        sequence.strip()
        for sequence in _llama_stop_sequences.split("|")
        if sequence.strip()
    )
else:
    LLAMA_STOP_SEQUENCES: tuple[str, ...] = tuple()
