from __future__ import annotations

import pytest
from django.contrib.auth.models import User
from django.test import Client
from django.urls import reverse


pytestmark = pytest.mark.django_db


def test_home_requires_authentication(client: Client) -> None:
    response = client.get(reverse("web:home"))
    assert response.status_code == 302
    assert reverse("login") in response.url


def test_home_renders_for_authenticated_user(client: Client) -> None:
    user = User.objects.create_user(username="tester", password="password123")
    client.force_login(user)

    response = client.get(reverse("web:home"))

    assert response.status_code == 200
    assert b"Welcome" in response.content
