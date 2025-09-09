"""Startup script for the Altinet local node GUI."""

import os


def start_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    """Start the Django development server for the local node GUI.

    Parameters
    ----------
    host:
        Hostname to bind the server to.
    port:
        Port number for the server.
    """
    try:
        from django.core.management import call_command
        import django
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("Django must be installed to run the local node GUI") from exc

    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "altinet.localnode.settings")
    django.setup()
    call_command("runserver", f"{host}:{port}")
