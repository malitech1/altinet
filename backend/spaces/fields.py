"""Custom model fields for the spaces app."""

from __future__ import annotations

from typing import Any

from django.db import models

from . import encryption


class EncryptedTextField(models.TextField):
    """Model field that transparently encrypts/decrypts text values."""

    description = "Text field encrypted at rest using Fernet"

    def from_db_value(self, value: Any, expression: Any, connection: Any) -> Any:
        if value is None:
            return value
        return encryption.decrypt(value)

    def get_prep_value(self, value: Any) -> Any:
        if value in (None, ""):
            return value
        return encryption.encrypt(str(value))

    def value_to_string(self, obj: models.Model) -> str:
        value = self.value_from_object(obj)
        return value or ""
