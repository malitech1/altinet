"""Global runtime settings for the Altinet project."""

from __future__ import annotations

import logging
from dataclasses import dataclass


@dataclass
class Settings:
    """Runtime configuration flags."""

    debug: bool = True


settings = Settings()


def configure_logging() -> None:
    """Configure root logging based on :mod:`settings`.

    When ``settings.debug`` is ``True`` logs are emitted to the console at
    ``DEBUG`` level. Otherwise, logs are written to ``altinet.log`` with level
    ``INFO``.
    """

    if logging.getLogger().handlers:
        return
    level = logging.DEBUG if settings.debug else logging.INFO
    handler: logging.Handler
    if settings.debug:
        handler = logging.StreamHandler()
    else:
        handler = logging.FileHandler("altinet.log")
    logging.basicConfig(
        level=level,
        handlers=[handler],
        format="%(asctime)s %(name)s [%(levelname)s] %(message)s",
    )
