"""Services for interacting with the OpenAI API."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable, Optional, Sequence

from django.conf import settings
from django.core.exceptions import ImproperlyConfigured

try:  # pragma: no cover - optional dependency
    from openai import OpenAI
except ImportError:  # pragma: no cover - optional dependency
    OpenAI = None  # type: ignore


@dataclass
class OpenAIConfig:
    """Configuration for the OpenAI chat completion service."""

    api_key: str
    model: str
    max_tokens: int
    temperature: float
    stop_sequences: Sequence[str]
    base_url: Optional[str] = None


class OpenAIChatService:
    """Thin wrapper around :class:`openai.OpenAI` with sane defaults."""

    def __init__(
        self,
        config: OpenAIConfig,
        client: Optional[OpenAI] = None,
    ) -> None:
        if not config.api_key:
            raise ImproperlyConfigured("OPENAI_API_KEY is not configured")

        if client is None:
            if OpenAI is None:
                raise ImproperlyConfigured(
                    "openai is not installed. Add it to requirements to use the hosted model.",
                )
            init_kwargs = {"api_key": config.api_key}
            if config.base_url:
                init_kwargs["base_url"] = config.base_url
            client = OpenAI(**init_kwargs)

        self._client = client
        self._model = config.model
        self._max_tokens = config.max_tokens
        self._temperature = config.temperature
        self._stop_sequences = list(config.stop_sequences)

    def generate(
        self,
        prompt: str,
        *,
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
        stop_sequences: Optional[Iterable[str]] = None,
    ) -> str:
        """Generate a completion for ``prompt`` and return the raw text."""

        stops = list(stop_sequences) if stop_sequences is not None else self._stop_sequences

        response = self._client.chat.completions.create(
            model=self._model,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=max_tokens or self._max_tokens,
            temperature=self._temperature if temperature is None else temperature,
            stop=stops or None,
        )
        choice = response.choices[0]
        message = getattr(choice, "message", None)
        if message is None:
            return ""
        content = getattr(message, "content", "")
        return (content or "").strip()


def _build_config() -> OpenAIConfig:
    base_url = settings.OPENAI_BASE_URL or None

    return OpenAIConfig(
        api_key=settings.OPENAI_API_KEY,
        model=settings.OPENAI_MODEL,
        max_tokens=settings.OPENAI_MAX_TOKENS,
        temperature=settings.OPENAI_TEMPERATURE,
        stop_sequences=settings.OPENAI_STOP_SEQUENCES,
        base_url=base_url,
    )


@lru_cache(maxsize=1)
def get_default_openai_client() -> OpenAIChatService:
    """Return a shared :class:`OpenAIChatService` instance for the service."""

    return OpenAIChatService(_build_config())


def reset_openai_cache() -> None:
    """Reset the cached OpenAI client instance (useful for tests)."""

    get_default_openai_client.cache_clear()
