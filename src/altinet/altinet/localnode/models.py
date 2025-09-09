"""Database models for the local node GUI.

These models provide a minimal schema for storing users and their
system preferences.  The database is configured in
:mod:`altinet.localnode.settings` using SQLite.
"""

from django.db import models


class User(models.Model):
    """A minimal user profile."""

    username = models.CharField(max_length=150, unique=True)
    email = models.EmailField(blank=True)

    def __str__(self) -> str:  # pragma: no cover - human readable representation
        return self.username


class SystemPreference(models.Model):
    """A key-value preference tied to a :class:`User`."""

    user = models.ForeignKey(User, related_name="preferences", on_delete=models.CASCADE)
    key = models.CharField(max_length=100)
    value = models.CharField(max_length=255)

    class Meta:
        unique_together = ("user", "key")

    def __str__(self) -> str:  # pragma: no cover - human readable representation
        return f"{self.user}: {self.key}={self.value}"
