from __future__ import annotations

import httpx
import pytest

from web.weather import fetch_weather_snapshot


class MockResponse:
    def __init__(self, payload: dict, status_code: int = 200, url: str = "https://example.com"):
        self._payload = payload
        self.status_code = status_code
        self.url = url

    def json(self) -> dict:
        return self._payload

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            request = httpx.Request("GET", self.url)
            raise httpx.HTTPStatusError(
                "error", request=request, response=httpx.Response(self.status_code, request=request)
            )


@pytest.mark.parametrize("address", ["", "   "])
def test_fetch_weather_snapshot_returns_empty_for_blank_address(address: str) -> None:
    assert fetch_weather_snapshot(address) == {}


def test_fetch_weather_snapshot_parses_weather(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_get(url: str, params: dict | None = None, timeout: float | None = None):
        if "geocoding" in url:
            return MockResponse(
                {
                    "results": [
                        {
                            "latitude": 51.5,
                            "longitude": -0.1,
                            "timezone": "Europe/London",
                        }
                    ]
                }
            )
        if "forecast" in url:
            return MockResponse(
                {
                    "current_weather": {
                        "temperature": 20.1,
                        "weathercode": 2,
                        "windspeed": 12.3,
                        "winddirection": 225,
                        "time": "2024-01-01T12:00",
                    },
                    "hourly": {
                        "time": ["2024-01-01T12:00"],
                        "relativehumidity_2m": [55],
                    },
                }
            )
        return MockResponse(
            {
                "hourly": {
                    "time": ["2024-01-01T12:00"],
                    "us_aqi": [42],
                }
            }
        )

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    snapshot = fetch_weather_snapshot("London")

    assert snapshot == {
        "outside_temperature_c": 20.1,
        "outside_humidity": 55,
        "weather_summary": "Partly cloudy",
        "wind_speed_kmh": 12.3,
        "wind_direction_deg": 225,
        "air_quality_index": 42,
    }


def test_fetch_weather_snapshot_handles_http_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_get(url: str, params: dict | None = None, timeout: float | None = None):
        return MockResponse({}, status_code=500)

    monkeypatch.setattr("web.weather.httpx.get", fake_get)

    assert fetch_weather_snapshot("London") == {}
