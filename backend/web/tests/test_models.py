import pytest
from django.db import connection

from web.models import SystemSettings


@pytest.mark.django_db
def test_load_returns_defaults_when_table_missing(monkeypatch):
    monkeypatch.setattr(connection.introspection, "table_names", lambda: [])

    settings = SystemSettings.load()

    assert settings.pk is None
    assert settings.site_name == "Altinet"
    assert settings.maintenance_mode is False
    assert settings.home_address == ""
