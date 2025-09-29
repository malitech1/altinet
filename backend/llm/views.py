"""API views for the offline LLaMA model."""

from __future__ import annotations

from django.core.exceptions import ImproperlyConfigured
from rest_framework import permissions, status
from rest_framework.request import Request
from rest_framework.response import Response
from rest_framework.views import APIView

from .serializers import PromptRequestSerializer, PromptResponseSerializer
from .services import get_default_llama


class LlamaPromptView(APIView):
    """Accept a prompt and return the generated response."""

    permission_classes = [permissions.AllowAny]

    def post(self, request: Request) -> Response:
        serializer = PromptRequestSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        try:
            llama = self._get_llama()
        except ImproperlyConfigured as exc:
            return Response(
                {"detail": str(exc)},
                status=status.HTTP_503_SERVICE_UNAVAILABLE,
            )
        prompt = serializer.validated_data["prompt"]
        max_tokens = serializer.validated_data.get("max_tokens")
        temperature = serializer.validated_data.get("temperature")

        response_text = llama.generate(
            prompt,
            max_tokens=max_tokens,
            temperature=temperature,
        )

        response_serializer = PromptResponseSerializer({"response": response_text})
        return Response(response_serializer.data, status=status.HTTP_200_OK)

    def _get_llama(self):
        return get_default_llama()


class HealthView(APIView):
    """Simple health endpoint for readiness checks."""

    permission_classes = [permissions.AllowAny]

    def get(self, request: Request) -> Response:
        try:
            get_default_llama()
        except ImproperlyConfigured as exc:
            return Response(
                {"status": "error", "detail": str(exc)},
                status=status.HTTP_503_SERVICE_UNAVAILABLE,
            )
        return Response({"status": "ok"}, status=status.HTTP_200_OK)
