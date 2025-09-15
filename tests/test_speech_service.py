"""Tests for the offline speech synthesis service."""

import sys
from types import SimpleNamespace

from altinet.services.speech import OfflineSpeechService


class DummyEngine:
    """Simple stub of a ``pyttsx3`` engine."""

    def __init__(self) -> None:
        self.spoken: list[str] = []
        self.ran = False

    def say(self, text: str) -> None:
        self.spoken.append(text)

    def runAndWait(self) -> None:  # noqa: N802 - library method name
        self.ran = True


def test_speak_uses_provided_engine() -> None:
    engine = DummyEngine()
    service = OfflineSpeechService(engine=engine)
    service.speak("hello")
    assert engine.spoken == ["hello"]
    assert engine.ran


def test_speak_initialises_pyttsx3(monkeypatch) -> None:
    dummy = DummyEngine()
    fake_module = SimpleNamespace(init=lambda: dummy)
    monkeypatch.setitem(sys.modules, "pyttsx3", fake_module)
    service = OfflineSpeechService()
    service.speak("hi")
    assert dummy.spoken == ["hi"]
    assert dummy.ran

