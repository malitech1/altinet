"""Configuration loading helpers with graceful YAML fallbacks."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

try:  # pragma: no cover - optional dependency
    import yaml
except ImportError:  # pragma: no cover - executed when PyYAML unavailable
    yaml = None


def load_file(path: Path) -> Dict[str, Any]:
    """Load a configuration file supporting YAML or JSON."""

    text = path.read_text(encoding="utf8")
    if yaml is not None:
        return yaml.safe_load(text) or {}
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return _parse_simple_yaml(text)


def _parse_simple_yaml(text: str) -> Dict[str, Any]:
    """Parse a limited subset of YAML (key/value pairs)."""

    result: Dict[str, Any] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        result[key.strip()] = _coerce_value(value.strip())
    return result


def _coerce_value(value: str) -> Any:
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


__all__ = ["load_file"]
