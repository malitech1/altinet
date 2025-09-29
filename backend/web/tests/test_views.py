from __future__ import annotations

import pytest
from django.contrib.auth.models import User
from django.test import Client
from django.urls import reverse

from web.models import SystemSettings


pytestmark = pytest.mark.django_db


def test_home_requires_authentication(client: Client) -> None:
    response = client.get(reverse("web:home"))
    assert response.status_code == 302
    assert reverse("login") in response.url


def test_home_renders_for_authenticated_user(client: Client, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("web.views.fetch_weather_snapshot", lambda _: {})

    user = User.objects.create_user(username="tester", password="password123")
    client.force_login(user)

    response = client.get(reverse("web:home"))

    assert response.status_code == 200
    assert b"Welcome" in response.content


def test_settings_requires_authentication(client: Client) -> None:
    response = client.get(reverse("web:settings"))

    assert response.status_code == 302
    assert reverse("login") in response.url


def test_user_can_update_profile_information(client: Client) -> None:
    user = User.objects.create_user(
        username="tester",
        password="password123",
        email="tester@example.com",
    )
    client.force_login(user)

    response = client.post(
        reverse("web:settings"),
        {
            "user-username": "tester",
            "user-first_name": "Test",
            "user-last_name": "User",
            "user-email": "new-email@example.com",
            "save_user": "1",
        },
        follow=True,
    )

    user.refresh_from_db()

    assert response.status_code == 200
    assert user.first_name == "Test"
    assert user.email == "new-email@example.com"


def test_operator_can_adjust_system_settings(client: Client) -> None:
    user = User.objects.create_user(username="admin", password="password123")
    client.force_login(user)

    response = client.post(
        reverse("web:settings"),
        {
            "system-site_name": "Altinet HQ",
            "system-support_email": "support@example.com",
            "system-maintenance_mode": "on",
            "system-default_theme": "dark",
            "system-home_address": "1600 Pennsylvania Avenue NW, Washington, DC",
            "save_system": "1",
        },
        follow=True,
    )

    settings_obj = SystemSettings.load()

    assert response.status_code == 200
    assert settings_obj.site_name == "Altinet HQ"
    assert settings_obj.support_email == "support@example.com"
    assert settings_obj.maintenance_mode is True
    assert settings_obj.default_theme == "dark"
    assert (
        settings_obj.home_address == "1600 Pennsylvania Avenue NW, Washington, DC"
    )
