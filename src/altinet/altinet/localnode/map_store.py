"""Utilities for loading and saving the home map used by the GUI."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

# Default file for persisting the map. Stored alongside this module so the
# local node can keep state without requiring a database.
MAP_FILE = Path(__file__).with_name("home_map.json")


def load_map(path: Optional[Path] = None) -> Dict[str, Any]:
    """Load the home map data from *path*.

    If *path* is not provided, the module-level :data:`MAP_FILE` is used.
    Returns a dictionary with at least a ``nodes`` key containing a list of
    node definitions.
    """
    target = path or MAP_FILE
    if target.exists():
        with target.open("r", encoding="utf-8") as f:
            return json.load(f)
    return {"nodes": []}


def save_map(data: Dict[str, Any], path: Optional[Path] = None) -> None:
    """Persist *data* to *path*.

    If *path* is not provided, :data:`MAP_FILE` is used. The file is written in
    UTF-8 encoded JSON format.
    """
    target = path or MAP_FILE
    with target.open("w", encoding="utf-8") as f:
        json.dump(data, f)
