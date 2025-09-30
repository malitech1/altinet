"""Utilities for fetching weather data for the dashboard."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import httpx

# Weather code mapping based on the Open-Meteo documentation:
# https://open-meteo.com/en/docs#latitude=52.52&longitude=13.41&hourly=temperature_2m
WEATHER_CODE_SUMMARY: Dict[int, str] = {
    0: "Clear sky",
    1: "Mainly clear",
    2: "Partly cloudy",
    3: "Overcast",
    45: "Fog",
    48: "Depositing rime fog",
    51: "Light drizzle",
    53: "Moderate drizzle",
    55: "Dense drizzle",
    56: "Light freezing drizzle",
    57: "Dense freezing drizzle",
    61: "Slight rain",
    63: "Moderate rain",
    65: "Heavy rain",
    66: "Light freezing rain",
    67: "Heavy freezing rain",
    71: "Slight snow fall",
    73: "Moderate snow fall",
    75: "Heavy snow fall",
    77: "Snow grains",
    80: "Slight rain showers",
    81: "Moderate rain showers",
    82: "Violent rain showers",
    85: "Slight snow showers",
    86: "Heavy snow showers",
    95: "Thunderstorm",
    96: "Thunderstorm with slight hail",
    99: "Thunderstorm with heavy hail",
}


@dataclass
class WeatherSnapshot:
    """Parsed data returned by the Open-Meteo APIs."""

    temperature_c: Optional[float] = None
    humidity_percent: Optional[int] = None
    summary: Optional[str] = None
    wind_speed_kmh: Optional[float] = None
    wind_direction_deg: Optional[float] = None
    air_quality_index: Optional[int] = None

    def as_environment_fields(self) -> Dict[str, Any]:
        """Convert the snapshot into the context keys used by the UI."""

        return {
            "outside_temperature_c": self.temperature_c,
            "outside_humidity": self.humidity_percent,
            "weather_summary": self.summary,
            "wind_speed_kmh": self.wind_speed_kmh,
            "wind_direction_deg": self.wind_direction_deg,
            "air_quality_index": self.air_quality_index,
        }


def _select_hourly_value(hourly: Dict[str, Any], key: str, timestamp: str) -> Optional[Any]:
    """Return the hourly value for the provided timestamp if it exists."""

    if not hourly:
        return None

    times = hourly.get("time") or []
    values = hourly.get(key) or []
    try:
        index = times.index(timestamp)
    except ValueError:
        return None

    if index >= len(values):
        return None

    return values[index]


def _parse_weather_summary(code: Optional[int]) -> Optional[str]:
    if code is None:
        return None
    return WEATHER_CODE_SUMMARY.get(code, "Unknown conditions")


def fetch_weather_snapshot(address: str) -> Dict[str, Any]:
    """Fetch weather information for the provided address.

    The Open-Meteo API is used because it doesn't require API keys and offers free
    geocoding as well as weather and air quality data. All errors are swallowed so
    the dashboard can continue to render even if the external service is
    unavailable.
    """

    if not address or not address.strip():
        return {}

    try:
        geocode_response = httpx.get(
            "https://geocoding-api.open-meteo.com/v1/search",
            params={"name": address, "count": 1},
            timeout=5.0,
        )
        geocode_response.raise_for_status()
        geocode_data = geocode_response.json()
        results = geocode_data.get("results")
        if not results:
            return {}

        location = results[0]
        latitude = location.get("latitude")
        longitude = location.get("longitude")
        timezone = location.get("timezone") or "auto"
        if latitude is None or longitude is None:
            return {}

        weather_response = httpx.get(
            "https://api.open-meteo.com/v1/forecast",
            params={
                "latitude": latitude,
                "longitude": longitude,
                "current_weather": "true",
                "hourly": "relativehumidity_2m",
                "timezone": timezone,
            },
            timeout=5.0,
        )
        weather_response.raise_for_status()
        weather_data = weather_response.json()
        current_weather = weather_data.get("current_weather") or {}
        timestamp = current_weather.get("time")
        hourly = weather_data.get("hourly") or {}

        snapshot = WeatherSnapshot(
            temperature_c=current_weather.get("temperature"),
            humidity_percent=_select_hourly_value(hourly, "relativehumidity_2m", timestamp)
            if timestamp
            else None,
            summary=_parse_weather_summary(current_weather.get("weathercode")),
            wind_speed_kmh=current_weather.get("windspeed"),
            wind_direction_deg=current_weather.get("winddirection"),
        )

        air_quality_response = httpx.get(
            "https://air-quality-api.open-meteo.com/v1/air-quality",
            params={
                "latitude": latitude,
                "longitude": longitude,
                "hourly": "us_aqi",
                "timezone": timezone,
                "past_days": 0,
                "forecast_days": 1,
            },
            timeout=5.0,
        )
        air_quality_response.raise_for_status()
        air_quality_data = air_quality_response.json()
        aq_hourly = air_quality_data.get("hourly") or {}
        if timestamp:
            snapshot.air_quality_index = _select_hourly_value(aq_hourly, "us_aqi", timestamp)
        else:
            values = aq_hourly.get("us_aqi") or []
            snapshot.air_quality_index = values[0] if values else None

        return snapshot.as_environment_fields()

    except (httpx.HTTPError, KeyError, ValueError):
        return {}
