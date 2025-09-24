"""Utilities for monitoring camera scenes and detecting motion events."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

import numpy as np


@dataclass
class SceneActivityConfig:
    """Configuration parameters controlling motion-triggered detection."""

    motion_threshold: float = 0.02
    min_trigger_interval_s: float = 0.5
    checks_after_motion: int = 3
    checks_while_tracking: int = 6


@dataclass
class SceneDecision:
    """Result of evaluating a frame for motion."""

    should_detect: bool
    motion_detected: bool
    motion_score: float


@dataclass
class _SceneState:
    """Internal per-room state for :class:`SceneChangeDetector`."""

    previous_frame: Optional[np.ndarray] = None
    pending_checks: int = 0
    tracking_checks: int = 0
    last_motion_time: float = 0.0
    motion_score: float = 0.0
    tracking_active: bool = False


def _to_float_gray(frame: np.ndarray) -> np.ndarray:
    """Convert ``frame`` to a contiguous float32 grayscale array."""

    if not isinstance(frame, np.ndarray):
        raise TypeError("frame must be a numpy.ndarray")
    gray = frame
    if gray.ndim == 3:
        gray = gray.mean(axis=2)
    if not np.issubdtype(gray.dtype, np.floating):
        gray = gray.astype(np.float32)
    else:
        gray = np.asarray(gray, dtype=np.float32)
    if not gray.flags.c_contiguous:
        gray = np.ascontiguousarray(gray)
    return gray


class SceneChangeDetector:
    """Detect scene changes and control when person detection should run."""

    def __init__(
        self,
        config: SceneActivityConfig | None = None,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.config = config or SceneActivityConfig()
        self._clock: Callable[[], float] = clock or __import__("time").monotonic
        self._states: Dict[str, _SceneState] = {}

    def observe(self, room_id: str, frame: np.ndarray) -> SceneDecision:
        """Inspect ``frame`` and decide whether to trigger detection."""

        state = self._states.setdefault(room_id, _SceneState())
        gray = _to_float_gray(frame)
        motion_detected = False
        score = 0.0
        if state.previous_frame is not None and state.previous_frame.shape == gray.shape:
            diff = np.abs(gray - state.previous_frame)
            if diff.size:
                score = float(np.mean(diff) / 255.0)
                motion_detected = score >= self.config.motion_threshold
        state.previous_frame = gray
        now = self._clock()
        should_detect = False
        if motion_detected:
            if now - state.last_motion_time >= self.config.min_trigger_interval_s:
                state.pending_checks = max(
                    state.pending_checks, self.config.checks_after_motion
                )
                state.last_motion_time = now
            else:
                state.pending_checks = max(1, state.pending_checks)
            state.motion_score = score
        if state.pending_checks > 0:
            should_detect = True
            state.pending_checks -= 1
        elif state.tracking_active and state.tracking_checks > 0:
            should_detect = True
            state.tracking_checks -= 1
            if state.tracking_checks == 0:
                state.tracking_active = False
        else:
            state.motion_score = score
        return SceneDecision(
            should_detect=should_detect,
            motion_detected=motion_detected,
            motion_score=score,
        )

    def notify_detection_result(self, room_id: str, person_found: bool) -> None:
        """Update tracking state based on the latest detection outcome."""

        state = self._states.setdefault(room_id, _SceneState())
        if person_found:
            if not state.tracking_active:
                state.tracking_active = True
                state.tracking_checks = max(0, self.config.checks_while_tracking)
        else:
            state.tracking_active = False
            state.tracking_checks = 0


__all__ = ["SceneActivityConfig", "SceneChangeDetector", "SceneDecision"]

