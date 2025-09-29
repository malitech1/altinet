from __future__ import annotations

import pytest
from django.core.exceptions import ImproperlyConfigured
from django.urls import reverse

from llm.services import reset_llama_cache


@pytest.fixture(autouse=True)
def clear_llama_cache():
    reset_llama_cache()
    yield
    reset_llama_cache()


@pytest.fixture
def prompt_url() -> str:
    return reverse("llm:prompt")


@pytest.fixture
def health_url() -> str:
    return reverse("llm:health")


class DummyLlama:
    def __init__(self):
        self.prompts: list[str] = []

    def generate(self, prompt: str, **kwargs):
        self.prompts.append(prompt)
        temperature = kwargs.get("temperature")
        token_info = kwargs.get("max_tokens")
        return f"prompt={prompt};temp={temperature};tokens={token_info}"


@pytest.fixture
def dummy_llama(monkeypatch):
    llama = DummyLlama()
    monkeypatch.setattr("llm.views.get_default_llama", lambda: llama)
    return llama


@pytest.mark.django_db
def test_prompt_endpoint_returns_response(client, prompt_url, dummy_llama):
    payload = {"prompt": "Hello", "temperature": 0.5, "max_tokens": 128}
    response = client.post(prompt_url, payload, content_type="application/json")

    assert response.status_code == 200
    data = response.json()
    assert data["response"] == "prompt=Hello;temp=0.5;tokens=128"
    assert dummy_llama.prompts == ["Hello"]


@pytest.mark.django_db
def test_prompt_endpoint_requires_prompt(client, prompt_url):
    response = client.post(prompt_url, {}, content_type="application/json")

    assert response.status_code == 400
    assert "prompt" in response.json()


@pytest.mark.django_db
def test_prompt_endpoint_returns_service_unavailable_when_unconfigured(
    client, prompt_url, monkeypatch
):
    def raise_config_error():
        raise ImproperlyConfigured("missing model")

    monkeypatch.setattr("llm.views.get_default_llama", raise_config_error)

    response = client.post(prompt_url, {"prompt": "Hello"}, content_type="application/json")

    assert response.status_code == 503
    payload = response.json()
    assert payload["detail"] == "missing model"


@pytest.mark.django_db
def test_health_endpoint_ok(client, health_url, dummy_llama):
    response = client.get(health_url)
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


@pytest.mark.django_db
def test_health_endpoint_reports_error(client, health_url, monkeypatch):
    def raise_config_error():
        raise ImproperlyConfigured("missing model")

    monkeypatch.setattr("llm.views.get_default_llama", raise_config_error)

    response = client.get(health_url)

    assert response.status_code == 503
    assert response.json() == {"status": "error", "detail": "missing model"}
