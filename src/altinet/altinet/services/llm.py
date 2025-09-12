"""Offline LLM service using llama.cpp."""

from __future__ import annotations

import json
from typing import Any, Dict, Optional


class OfflineLLMService:
    """Interface to a local LLaMA model via llama.cpp.

    The service loads a model from ``model_path`` using ``llama-cpp-python``.
    It can be queried with environment descriptions, user preferences and a
    specific request. The model is expected to return a JSON object describing
    an action and its arguments.
    """

    def __init__(
        self,
        model_path: str,
        *,
        llama_client: Optional[Any] = None,
        n_ctx: int = 2048,
        temperature: float = 0.2,
        max_tokens: int = 256,
    ) -> None:
        """Create the service.

        Parameters
        ----------
        model_path:
            Path to the LLaMA model weights.
        llama_client:
            Optional pre-initialised ``llama_cpp.Llama`` instance. Supplying
            this allows tests to stub out the heavy model dependency.
        n_ctx:
            Context window passed to the underlying model when ``llama_client``
            is not provided.
        temperature:
            Sampling temperature used for generation.
        max_tokens:
            Maximum number of tokens to generate.
        """

        if llama_client is not None:
            self._llama = llama_client
        else:  # pragma: no cover - requires the optional dependency
            from llama_cpp import Llama

            self._llama = Llama(model_path=model_path, n_ctx=n_ctx)

        self._temperature = temperature
        self._max_tokens = max_tokens

    @staticmethod
    def build_prompt(environment: str, user_info: str, user_request: str) -> str:
        """Construct a prompt for the model.

        The prompt includes a summary of the environment, known user
        preferences and the current request. The model is instructed to respond
        with a JSON object containing ``action`` and ``arguments`` fields.
        """

        return (
            f"Environment:\n{environment}\n\n"
            f"User preferences:\n{user_info}\n\n"
            f"User request:\n{user_request}\n\n"
            "Respond with a JSON object containing an 'action' key and an "
            "'arguments' object."
        )

    def choose_action(self, environment: str, user_info: str, user_request: str) -> Dict[str, Any]:
        """Query the model and return the parsed action description."""

        prompt = self.build_prompt(environment, user_info, user_request)
        result = self._llama(
            prompt,
            max_tokens=self._max_tokens,
            temperature=self._temperature,
            stop=["</s>"],
        )
        text = result["choices"][0]["text"].strip()
        return json.loads(text)
