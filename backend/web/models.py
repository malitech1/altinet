from __future__ import annotations

from django.db import connection, models
from django.db.utils import OperationalError, ProgrammingError


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
    home_address = models.CharField(
        max_length=255,
        blank=True,
        help_text="Street address used to fetch local weather conditions.",
    )

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
        """Return the singleton settings instance if the table exists.

        The settings page is visited very early during new installations.
        When the initial migrations haven't been applied yet the backing
        database table is missing which used to raise an OperationalError and
        crash the whole view.  Instead of propagating the error we return an
        unsaved instance populated with model defaults so the UI can render
        and guide the operator through the remainder of the setup process.
        """

        try:
            table_names = connection.introspection.table_names()
        except (OperationalError, ProgrammingError):
            return cls()

        if cls._meta.db_table not in table_names:
            return cls()

        obj, _ = cls.objects.get_or_create(pk=1)
        return obj
