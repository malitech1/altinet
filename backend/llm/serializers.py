"""Serializers for LLaMA endpoints."""

from __future__ import annotations

from rest_framework import serializers


class PromptRequestSerializer(serializers.Serializer):
    prompt = serializers.CharField(max_length=4096, allow_blank=False)
    max_tokens = serializers.IntegerField(min_value=1, max_value=4096, required=False)
    temperature = serializers.FloatField(min_value=0.0, max_value=2.0, required=False)


class PromptResponseSerializer(serializers.Serializer):
    response = serializers.CharField()
