"""Tests for the working memory module."""

from altinet.services.memory import WorkingMemory


def test_build_prompt_with_context_and_people() -> None:
    wm = WorkingMemory()
    wm.add_person("Alice", location="kitchen")
    wm.update_context(temperature_c=21.5, location="kitchen")
    prompt = wm.build_prompt()
    assert "Alice" in prompt
    assert "21.5" in prompt
    assert "kitchen" in prompt


def test_build_prompt_no_people() -> None:
    wm = WorkingMemory()
    wm.update_context(temperature_c=18.0, location="office")
    prompt = wm.build_prompt()
    assert "No people are currently detected." in prompt
    assert "office" in prompt
