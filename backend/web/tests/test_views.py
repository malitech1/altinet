from __future__ import annotations

import json

import numpy as np
import pytest
from django.contrib.auth.models import User
from django.test import Client
from django.urls import reverse

from web.models import SystemSettings, TrainingImage, TrainingProfile


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
    assert b"home-viewer" in response.content


def test_weather_snapshot_endpoint_returns_data(
    client: Client, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "web.views.fetch_weather_snapshot",
        lambda _: {
            "outside_temperature_c": 25.2,
            "outside_humidity": 68,
            "weather_summary": "Sunny",
            "wind_speed_kmh": 12.5,
            "wind_direction_deg": 180,
            "air_quality_index": 47,
        },
    )

    user = User.objects.create_user(username="weather", password="password123")
    client.force_login(user)

    response = client.get(reverse("web:weather-snapshot"))

    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["data"]["wind_direction_cardinal"] == "S"
    assert payload["data"]["wind_direction_deg"] == 180


def test_weather_snapshot_endpoint_handles_empty_payload(
    client: Client, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("web.views.fetch_weather_snapshot", lambda _: {})

    user = User.objects.create_user(username="empty", password="password123")
    client.force_login(user)

    response = client.get(reverse("web:weather-snapshot"))

    assert response.status_code == 503


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


def test_training_requires_authentication(client: Client) -> None:
    response = client.get(reverse("web:training"))

    assert response.status_code == 302
    assert reverse("login") in response.url


def test_training_page_lists_existing_profiles(client: Client) -> None:
    user = User.objects.create_user(username="trainer", password="password123")
    client.force_login(user)

    profile = TrainingProfile.objects.create(full_name="Alex Johnson", notes="Security")
    TrainingImage.objects.create(
        profile=profile,
        image_data="data:image/png;base64,dGVzdA==",
    )
    profile.mark_trained()

    response = client.get(reverse("web:training"))

    assert response.status_code == 200
    assert b"Enroll a new user" in response.content
    assert b"Alex Johnson" in response.content


def test_training_endpoint_creates_profile(client: Client) -> None:
    user = User.objects.create_user(username="coach", password="password123")
    client.force_login(user)

    payload = {
        "full_name": "Jordan Smith",
        "notes": "Engineering",
        "images": [
            "data:image/png;base64,dGVzdA==",
            "data:image/jpeg;base64,dGVzdDI=",
        ],
    }

    response = client.post(
        reverse("web:training-create"),
        data=json.dumps(payload),
        content_type="application/json",
    )

    assert response.status_code == 201
    data = response.json()
    assert data["success"] is True
    profile = TrainingProfile.objects.get(full_name="Jordan Smith")
    assert profile.notes == "Engineering"
    assert profile.trained_at is not None
    assert profile.images.count() == 2


def test_training_endpoint_validates_payload(client: Client) -> None:
    user = User.objects.create_user(username="validator", password="password123")
    client.force_login(user)

    response = client.post(
        reverse("web:training-create"),
        data=json.dumps({"full_name": "Taylor"}),
        content_type="application/json",
    )

    assert response.status_code == 400


def test_training_test_requires_face_components(
    client: Client, monkeypatch: pytest.MonkeyPatch
) -> None:
    user = User.objects.create_user(username="tester", password="password123")
    client.force_login(user)

    monkeypatch.setattr("web.services.face_training.get_face_analyzer", lambda: None)

    response = client.post(
        reverse("web:training-test"),
        data=json.dumps({"image": "data:image/png;base64,AAAA"}),
        content_type="application/json",
    )

    assert response.status_code == 503


def test_training_test_returns_match(
    client: Client, monkeypatch: pytest.MonkeyPatch
) -> None:
    user = User.objects.create_user(username="match", password="password123")
    client.force_login(user)

    profile = TrainingProfile.objects.create(full_name="Jordan Smith")
    TrainingImage.objects.create(
        profile=profile,
        image_data="data:image/png;base64,dHJhaW4=",
    )

    embeddings = {
        "data:image/png;base64,dHJhaW4=": np.array([1.0, 0.0], dtype=float),
        "data:image/png;base64,cHJvYmU=": np.array([1.0, 0.0], dtype=float),
    }

    monkeypatch.setattr("web.services.face_training.get_face_analyzer", lambda: object())
    monkeypatch.setattr("web.services.face_training.load_image_from_data_uri", lambda data: data)
    monkeypatch.setattr(
        "web.services.face_training.extract_embedding", lambda analyzer, image: embeddings.get(image)
    )
    monkeypatch.setattr(
        "web.services.face_training.embedding_for_training_image",
        lambda analyzer, image: embeddings.get(image.image_data),
    )

    response = client.post(
        reverse("web:training-test"),
        data=json.dumps({"image": "data:image/png;base64,cHJvYmU="}),
        content_type="application/json",
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["match"]["full_name"] == "Jordan Smith"
    assert payload["message"].startswith("Match found")


def test_training_test_handles_no_match(
    client: Client, monkeypatch: pytest.MonkeyPatch
) -> None:
    user = User.objects.create_user(username="nomatch", password="password123")
    client.force_login(user)

    profile = TrainingProfile.objects.create(full_name="Alex Johnson")
    TrainingImage.objects.create(
        profile=profile,
        image_data="data:image/png;base64,dHJhaW4=",
    )

    embeddings = {
        "data:image/png;base64,dHJhaW4=": np.array([0.0, 1.0], dtype=float),
        "data:image/png;base64,cHJvYmU=": np.array([1.0, 0.0], dtype=float),
    }

    monkeypatch.setattr("web.services.face_training.get_face_analyzer", lambda: object())
    monkeypatch.setattr("web.services.face_training.load_image_from_data_uri", lambda data: data)
    monkeypatch.setattr(
        "web.services.face_training.extract_embedding", lambda analyzer, image: embeddings.get(image)
    )
    monkeypatch.setattr(
        "web.services.face_training.embedding_for_training_image",
        lambda analyzer, image: embeddings.get(image.image_data),
    )

    response = client.post(
        reverse("web:training-test"),
        data=json.dumps({"image": "data:image/png;base64,cHJvYmU="}),
        content_type="application/json",
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["match"] is None
    assert payload["message"] == "No trained faces matched the capture."
