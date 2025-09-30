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


def _build_sample_html(payload: dict, *, id_with_whitespace: bool = False) -> str:
    if id_with_whitespace:
        id_attribute = 'id = "__NEXT_DATA__"'
    else:
        id_attribute = 'id="__NEXT_DATA__"'

    return (
        "<html><head></head><body>"
        f"<script crossorigin=\"anonymous\" {id_attribute} type=\"application/json\">{json.dumps(payload)}</script>"
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


def test_fetch_weather_snapshot_handles_whitespace_around_id_equals(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = {
        "props": {
            "pageProps": {
                "dataStore": {
                    "wxObservation": {
                        "current": {
                            "temperature": 19.2,
                            "relativeHumidity": 70,
                            "windSpeed": 12.4,
                            "windDirection": 200,
                            "wxPhraseShort": "Cloudy",
                        }
                    }
                }
            }
        }
    }

    html = _build_sample_html(payload, id_with_whitespace=True)

    def fake_get(url: str, headers: dict | None = None, timeout: float | None = None):
        assert "weather.com" in url
        return MockResponse(text=html, url=url)

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    snapshot = fetch_weather_snapshot("Anywhere")

    assert snapshot["outside_temperature_c"] == pytest.approx(19.2)
    assert snapshot["outside_humidity"] == 70
    assert snapshot["weather_summary"] == "Cloudy"
    assert snapshot["wind_speed_kmh"] == pytest.approx(12.4)
    assert snapshot["wind_direction_deg"] == pytest.approx(200.0)


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
