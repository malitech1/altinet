"""Logging helpers for Altinet.

Provides a ``get_logger`` function that returns a configured logger with a
standard format. The logger uses a :class:`~logging.StreamHandler` so log output
is visible on the console by default.
"""

from __future__ import annotations

import logging


def get_logger(name: str) -> logging.Logger:
    """Return a configured :class:`logging.Logger` instance.

    Parameters
    ----------
    name:
        Name of the logger, typically ``__name__`` of the caller.

    Returns
    -------
    logging.Logger
        Configured logger with a simple console handler attached.
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    return logger

