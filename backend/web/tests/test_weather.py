from __future__ import annotations

import json

import httpx
import pytest

from web.weather import fetch_weather_snapshot


class MockResponse:
    def __init__(self, *, text: str, status_code: int = 200, url: str = "https://example.com") -> None:
        self.text = text
        self.status_code = status_code
        self.url = url

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            request = httpx.Request("GET", self.url)
            raise httpx.HTTPStatusError(
                "error", request=request, response=httpx.Response(self.status_code, request=request)
            )


def _build_sample_html(payload: dict) -> str:
    return (
        "<html><head></head><body>"
        f"<script id=\"__NEXT_DATA__\" type=\"application/json\">{json.dumps(payload)}</script>"
        "</body></html>"
    )


def test_fetch_weather_snapshot_parses_scraped_payload(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = {
        "props": {
            "pageProps": {
                "dataStore": {
                    "wxObservation": {
                        "current": {
                            "temperature": 21.4,
                            "relativeHumidity": 63,
                            "windSpeed": 18.6,
                            "windDirection": 140,
                            "windDirectionCardinal": "SE",
                            "wxPhraseLong": "Partly cloudy",
                        }
                    },
                    "airQuality": {
                        "details": {"airQualityIndex": 41.6}
                    },
                },
                "meta": {"timeZoneId": "Australia/Brisbane"},
            }
        }
    }

    html = _build_sample_html(payload)

    def fake_get(url: str, headers: dict | None = None, timeout: float | None = None):
        assert "weather.com" in url
        return MockResponse(text=html, url=url)

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    snapshot = fetch_weather_snapshot("Anywhere")

    assert snapshot == {
        "outside_temperature_c": pytest.approx(21.4),
        "outside_humidity": 63,
        "weather_summary": "Partly cloudy",
        "wind_speed_kmh": pytest.approx(18.6),
        "wind_direction_deg": pytest.approx(140.0),
        "air_quality_index": 42,
        "location_timezone": "Australia/Brisbane",
    }


def test_fetch_weather_snapshot_returns_empty_when_no_data(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_get(url: str, headers: dict | None = None, timeout: float | None = None):
        return MockResponse(text="<html><body>No weather here</body></html>")

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    assert fetch_weather_snapshot("Anywhere") == {}


def test_fetch_weather_snapshot_handles_http_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_get(url: str, headers: dict | None = None, timeout: float | None = None):
        return MockResponse(text="", status_code=500)

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    assert fetch_weather_snapshot("Anywhere") == {}
