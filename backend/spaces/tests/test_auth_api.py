from __future__ import annotations

import pytest
from django.contrib.auth import get_user_model


@pytest.mark.django_db
def test_auth_status_reports_user_count(api_client):
    response = api_client.get("/api/auth/status/")
    assert response.status_code == 200
    assert response.data["has_users"] is False
    assert response.data["is_authenticated"] is False

    User = get_user_model()
    User.objects.create_user(username="alice", password="secret123")

    response = api_client.get("/api/auth/status/")
    assert response.status_code == 200
    assert response.data["has_users"] is True


@pytest.mark.django_db
def test_register_creates_first_user(api_client):
    payload = {
        "username": "operator",
        "password": "supersecret",
        "email": "ops@example.com",
    }
    response = api_client.post("/api/auth/register/", payload, format="json")
    assert response.status_code == 201
    assert response.data["username"] == "operator"

    # Additional registration attempts should be rejected once a user exists.
    response = api_client.post("/api/auth/register/", payload, format="json")
    assert response.status_code == 403


@pytest.mark.django_db
def test_login_and_logout_flow(api_client):
    User = get_user_model()
    User.objects.create_user(username="alice", password="secret123")

    login_response = api_client.post(
        "/api/auth/login/",
        {"username": "alice", "password": "secret123"},
        format="json",
    )
    assert login_response.status_code == 200
    assert login_response.data["username"] == "alice"

    status_response = api_client.get("/api/auth/status/")
    assert status_response.data["is_authenticated"] is True

    logout_response = api_client.post("/api/auth/logout/")
    assert logout_response.status_code == 204

    status_response = api_client.get("/api/auth/status/")
    assert status_response.data["is_authenticated"] is False

