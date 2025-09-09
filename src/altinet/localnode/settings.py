"""Minimal Django settings for the Altinet local node GUI."""

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

SECRET_KEY = "dev-secret-key"
DEBUG = True
ROOT_URLCONF = "altinet.localnode.urls"

ALLOWED_HOSTS = ["*"]

INSTALLED_APPS = [
    "django.contrib.staticfiles",
]

MIDDLEWARE = [
    "django.middleware.common.CommonMiddleware",
]

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [BASE_DIR / "templates"],
        "APP_DIRS": True,
        "OPTIONS": {
            "context_processors": [],
        },
    }
]

STATIC_URL = "/static/"
