"""Tests for the offline LLM service."""

from altinet.services.llm import OfflineLLMService


class FakeLlama:
    """Minimal stub of ``llama_cpp.Llama`` for testing."""

    def __init__(self) -> None:
        self.last_prompt = None

    def __call__(self, prompt: str, *, max_tokens: int, temperature: float, stop: list[str]):
        self.last_prompt = prompt
        # Return a deterministic JSON action
        return {"choices": [{"text": '{"action": "greet", "arguments": {"name": "Bob"}}'}]}


def test_build_prompt_includes_sections() -> None:
    prompt = OfflineLLMService.build_prompt("sunny", "likes tea", "say hello")
    assert "Environment:" in prompt
    assert "User preferences:" in prompt
    assert "User request:" in prompt


def test_choose_action_parses_model_output() -> None:
    fake = FakeLlama()
    service = OfflineLLMService(model_path="dummy", llama_client=fake)
    result = service.choose_action("rainy", "prefers coffee", "greet Bob")
    assert result == {"action": "greet", "arguments": {"name": "Bob"}}
    assert "Environment:\nrainy" in fake.last_prompt
