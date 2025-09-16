"""Encryption helpers for sensitive camera credentials."""

from __future__ import annotations

from functools import lru_cache

from cryptography.fernet import Fernet, InvalidToken
from django.conf import settings


@lru_cache
def _get_fernet() -> Fernet:
    return Fernet(settings.FERNET_DERIVED_KEY)


def encrypt(value: str) -> str:
    """Encrypt the provided value using Fernet."""
    fernet = _get_fernet()
    return fernet.encrypt(value.encode("utf-8")).decode("utf-8")


def decrypt(value: str) -> str:
    """Decrypt the provided value using Fernet."""
    fernet = _get_fernet()
    try:
        return fernet.decrypt(value.encode("utf-8")).decode("utf-8")
    except InvalidToken as exc:  # pragma: no cover - defensive
        raise ValueError("Unable to decrypt value") from exc
