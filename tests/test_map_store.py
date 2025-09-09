from pathlib import Path

from altinet.localnode.map_store import load_map, save_map


def test_save_and_load_map(tmp_path: Path) -> None:
    data = {"nodes": [{"name": "A", "x": 1.0, "y": 2.0}]}
    path = tmp_path / "map.json"
    save_map(data, path)
    loaded = load_map(path)
    assert loaded == data
