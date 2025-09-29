from __future__ import annotations

from django.db import models


class SystemSettings(models.Model):
    """Singleton model storing operator-adjustable settings exposed in the UI."""

    THEME_CHOICES = [
        ("light", "Light"),
        ("dark", "Dark"),
        ("system", "Match system"),
    ]

    site_name = models.CharField(max_length=120, default="Altinet")
    support_email = models.EmailField(blank=True)
    maintenance_mode = models.BooleanField(default=False)
    default_theme = models.CharField(max_length=20, choices=THEME_CHOICES, default="system")

    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        verbose_name = "System settings"
        verbose_name_plural = "System settings"

    def __str__(self) -> str:  # pragma: no cover - human readable representation
        return "System configuration"

    def save(self, *args, **kwargs):
        # Force a single instance row. pk=1 is sufficient for our lightweight needs.
        self.pk = 1
        super().save(*args, **kwargs)

    @classmethod
    def load(cls) -> "SystemSettings":
        obj, _ = cls.objects.get_or_create(pk=1)
        return obj
