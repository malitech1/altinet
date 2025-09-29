"""Services for interacting with an offline LLaMA model."""

from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable, Optional, Sequence

from django.conf import settings
from django.core.exceptions import ImproperlyConfigured

try:  # pragma: no cover - optional dependency
    from llama_cpp import Llama  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    Llama = None  # type: ignore


@dataclass
class OfflineLlamaConfig:
    """Configuration for the offline LLaMA model."""

    model_path: str
    context_window: int
    temperature: float
    stop_sequences: Sequence[str]


class OfflineLlama:
    """Thin wrapper around :class:`llama_cpp.Llama` with sane defaults."""

    def __init__(
        self,
        config: OfflineLlamaConfig,
        backend: Optional["Llama"] = None,
    ) -> None:
        if backend is None:
            if Llama is None:
                raise ImproperlyConfigured(
                    "llama-cpp-python is not installed. Add it to requirements to use the offline model.",
                )
            if not os.path.exists(config.model_path):
                raise ImproperlyConfigured(
                    f"LLaMA model path '{config.model_path}' does not exist."
                )
            backend = Llama(
                model_path=config.model_path,
                n_ctx=config.context_window,
                logits_all=False,
            )

        self._backend = backend
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

        kwargs = {
            "max_tokens": max_tokens or settings.LLAMA_MAX_TOKENS,
            "temperature": temperature if temperature is not None else self._temperature,
        }
        stops = list(stop_sequences) if stop_sequences is not None else self._stop_sequences
        if stops:
            kwargs["stop"] = stops

        completion = self._backend(
            prompt,
            **kwargs,
        )
        choice = completion["choices"][0]
        text = choice.get("text") or choice.get("message", {}).get("content", "")
        return text.strip()


def _build_config() -> OfflineLlamaConfig:
    if not settings.LLAMA_MODEL_PATH:
        raise ImproperlyConfigured("LLAMA_MODEL_PATH is not configured")

    return OfflineLlamaConfig(
        model_path=settings.LLAMA_MODEL_PATH,
        context_window=settings.LLAMA_CONTEXT_WINDOW,
        temperature=settings.LLAMA_TEMPERATURE,
        stop_sequences=settings.LLAMA_STOP_SEQUENCES,
    )


@lru_cache(maxsize=1)
def get_default_llama() -> OfflineLlama:
    """Return a shared :class:`OfflineLlama` instance for the service."""

    return OfflineLlama(_build_config())


def reset_llama_cache() -> None:
    """Reset the cached LLaMA instance (useful for tests)."""

    get_default_llama.cache_clear()
