"""Offline text-to-speech service."""

from __future__ import annotations

from typing import Any, Optional


class OfflineSpeechService:
    """Convert text to speech locally using ``pyttsx3``.

    The service wraps a ``pyttsx3`` engine instance to synthesise human-like
    speech without needing internet connectivity. A pre-initialised engine can
    be provided to ease testing.
    """

    def __init__(self, *, engine: Optional[Any] = None) -> None:
        if engine is not None:
            self._engine = engine
        else:  # pragma: no cover - optional dependency
            import pyttsx3

            self._engine = pyttsx3.init()

    def speak(self, text: str) -> None:
        """Vocalise ``text`` using the configured engine."""

        self._engine.say(text)
        self._engine.runAndWait()

