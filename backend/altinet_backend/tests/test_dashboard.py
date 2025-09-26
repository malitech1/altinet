from __future__ import annotations

from django.urls import resolve


def test_dashboard_view_renders_with_expected_content(client):
    response = client.get("/")
    assert response.status_code == 200
    html = response.content.decode("utf-8")
    assert "Altinet Dashboard" in html
    assert "Manage Spaces" in html
    assert "API Reference" in html


def test_dashboard_url_maps_to_dashboard_view():
    match = resolve("/")
    assert match.view_name == "dashboard"
