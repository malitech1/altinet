"""Utilities for fetching weather data for the dashboard."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

import httpx

DEFAULT_WEATHER_URL = "https://weather.com/en-AU/weather/today/l/-27.61,153.33?par=altinet"

REQUEST_HEADERS = {
    "User-Agent": (
        "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) "
        "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
    )
}


@dataclass
class WeatherSnapshot:
    """Parsed weather data scraped from the provider."""

    temperature_c: Optional[float] = None
    humidity_percent: Optional[int] = None
    summary: Optional[str] = None
    wind_speed_kmh: Optional[float] = None
    wind_direction_deg: Optional[float] = None
    air_quality_index: Optional[int] = None
    timezone_name: Optional[str] = None

    def as_environment_fields(self) -> Dict[str, Any]:
        """Convert the snapshot into the context keys used by the UI."""

        fields = {
            "outside_temperature_c": self.temperature_c,
            "outside_humidity": self.humidity_percent,
            "weather_summary": self.summary,
            "wind_speed_kmh": self.wind_speed_kmh,
            "wind_direction_deg": self.wind_direction_deg,
            "air_quality_index": self.air_quality_index,
        }

        if self.timezone_name:
            fields["location_timezone"] = self.timezone_name

        return fields


def _coerce_scalar(value: Any) -> Any:
    """Extract a scalar value from nested structures returned by the site."""

    if isinstance(value, dict):
        for key in ("value", "values", "min", "max", "amount", "number"):
            if key in value:
                result = _coerce_scalar(value[key])
                if result is not None:
                    return result
        return None

    if isinstance(value, (list, tuple)):
        for item in value:
            result = _coerce_scalar(item)
            if result is not None:
                return result
        return None

    return value


def _to_float(value: Any) -> Optional[float]:
    scalar = _coerce_scalar(value)
    if scalar is None:
        return None
    try:
        numeric = float(scalar)
    except (TypeError, ValueError):
        return None
    if math.isnan(numeric) or math.isinf(numeric):
        return None
    return numeric


def _to_int(value: Any) -> Optional[int]:
    scalar = _coerce_scalar(value)
    if scalar is None:
        return None
    try:
        numeric = int(round(float(scalar)))
    except (TypeError, ValueError):
        return None
    return numeric


def _extract_next_data(html: str) -> Optional[Dict[str, Any]]:
    """Extract the Next.js data blob from the rendered HTML."""

    script_pattern = re.compile(
        r"<script\b[^>]*\bid=(\"|')__NEXT_DATA__\1[^>]*>",
        flags=re.IGNORECASE,
    )
    match = script_pattern.search(html)
    if not match:
        return None

    start = match.end()
    closing_pattern = re.compile(r"</script\s*>", flags=re.IGNORECASE)
    closing_match = closing_pattern.search(html, start)
    if not closing_match:
        return None

    payload = html[start:closing_match.start()]
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return None


def _find_dict_with_keys(data: Any, required: Iterable[str]) -> Optional[Dict[str, Any]]:
    """Search the nested JSON for a dictionary containing the provided keys."""

    stack = [data]
    required_set = set(required)

    while stack:
        current = stack.pop()
        if isinstance(current, dict):
            if required_set.issubset(current.keys()):
                return current
            stack.extend(current.values())
        elif isinstance(current, (list, tuple)):
            stack.extend(current)

    return None


def _find_first_value_for_key(data: Any, key: str) -> Any:
    """Return the first value encountered for ``key`` in the nested JSON."""

    stack = [data]

    while stack:
        current = stack.pop()
        if isinstance(current, dict):
            if key in current:
                return current[key]
            stack.extend(current.values())
        elif isinstance(current, (list, tuple)):
            stack.extend(current)

    return None


def _coerce_summary(payload: Dict[str, Any]) -> Optional[str]:
    for key in ("wxPhraseLong", "wxPhraseShort", "phrase", "narrative"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _derive_timezone(data: Dict[str, Any]) -> str:
    candidate = _find_first_value_for_key(data, "timeZoneId")
    if isinstance(candidate, str) and candidate.strip():
        return candidate.strip()

    candidate = _find_first_value_for_key(data, "timeZone")
    if isinstance(candidate, str) and candidate.strip():
        return candidate.strip()

    return "Australia/Brisbane"


def fetch_weather_snapshot(address: str) -> Dict[str, Any]:
    """Scrape outdoor weather data for Macleay Island in Brisbane."""

    del address  # The dashboard always displays Macleay Island conditions.

    try:
        response = httpx.get(DEFAULT_WEATHER_URL, headers=REQUEST_HEADERS, timeout=10.0)
        response.raise_for_status()
    except httpx.HTTPError:
        return {}

    data = _extract_next_data(response.text)
    if not data:
        return {}

    observation = _find_dict_with_keys(
        data,
        {"temperature", "relativeHumidity", "windSpeed", "windDirection"},
    )
    if not observation:
        return {}

    snapshot = WeatherSnapshot(
        temperature_c=_to_float(observation.get("temperature")),
        humidity_percent=_to_int(observation.get("relativeHumidity")),
        summary=_coerce_summary(observation),
        wind_speed_kmh=_to_float(observation.get("windSpeed")),
        wind_direction_deg=_to_float(observation.get("windDirection")),
        timezone_name=_derive_timezone(data),
    )

    air_quality = _find_first_value_for_key(data, "airQualityIndex")
    snapshot.air_quality_index = _to_int(air_quality)

    return snapshot.as_environment_fields()
