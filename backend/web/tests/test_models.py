import pytest
from django.db import connection
from django.utils import timezone

from web.models import SystemSettings, TrainingProfile


@pytest.mark.django_db
def test_load_returns_defaults_when_table_missing(monkeypatch):
    monkeypatch.setattr(connection.introspection, "table_names", lambda: [])

    settings = SystemSettings.load()

    assert settings.pk is None
    assert settings.site_name == "Altinet"
    assert settings.maintenance_mode is False
    assert settings.home_address == ""


@pytest.mark.django_db
def test_mark_trained_sets_timestamp(monkeypatch):
    frozen_time = timezone.now()
    monkeypatch.setattr(timezone, "now", lambda: frozen_time)

    profile = TrainingProfile.objects.create(full_name="Test User")

    assert profile.trained_at is None

    profile.mark_trained()

    profile.refresh_from_db()
    assert profile.trained_at == frozen_time
