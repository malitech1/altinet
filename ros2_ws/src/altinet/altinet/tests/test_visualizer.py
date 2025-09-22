"""Tests for timestamp filtering in :mod:`altinet.nodes.visualizer_node`."""

from __future__ import annotations

from typing import Any, Optional, Sequence

from altinet.nodes.visualizer_node import VisualizerNode


class _FakeDuration:
    """Minimal duration object mimicking ROS 2 ``Duration``."""

    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class _FakeTime:
    """Time stub returning a configurable duration when subtracted."""

    def __init__(self, seconds: float) -> None:
        self._nanoseconds = int(seconds * 1e9)

    def __sub__(self, other: "_FakeTime") -> _FakeDuration:
        return _FakeDuration(self._nanoseconds - other._nanoseconds)


def _filter(
    tolerance_ns: Optional[int],
    entities: Sequence[Any],
    stamp: Optional[_FakeTime],
    image_stamp: _FakeTime,
):
    node = VisualizerNode.__new__(VisualizerNode)
    node._timestamp_tolerance_ns = tolerance_ns  # type: ignore[attr-defined]
    return node._filter_by_timestamp(entities, stamp, image_stamp)


def test_filter_by_timestamp_accepts_within_tolerance():
    entities = [object()]
    stamp = _FakeTime(0.0)
    image_stamp = _FakeTime(1.0)  # 1 second later
    result = _filter(int(1.5 * 1e9), entities, stamp, image_stamp)
    assert result == list(entities)


def test_filter_by_timestamp_drops_stale_entities():
    entities = [object()]
    stamp = _FakeTime(0.0)
    image_stamp = _FakeTime(1.0)  # 1 second later
    result = _filter(int(0.5 * 1e9), entities, stamp, image_stamp)
    assert result == []


def test_filter_by_timestamp_disables_filtering_for_non_positive():
    entities = [object()]
    stamp = _FakeTime(0.0)
    image_stamp = _FakeTime(10.0)
    result = _filter(None, entities, stamp, image_stamp)
    assert result == list(entities)
    # Filtering disabled even when timestamp is unavailable
    result_without_stamp = _filter(None, entities, None, image_stamp)
    assert result_without_stamp == list(entities)
