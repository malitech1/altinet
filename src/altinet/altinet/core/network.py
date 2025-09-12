"""Networking primitives for Altinet."""

from dataclasses import dataclass
import subprocess

from altinet.utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class NetworkNode:
    """Represents a node in the Altinet network."""

    address: str

    def connect(self, other: "NetworkNode") -> None:
        """Placeholder method to establish a connection to another node."""
        logger.info("Connecting %s to %s", self.address, other.address)
        raise NotImplementedError


def open_network_console() -> None:
    """Launch the system network console for managing Wi-Fi connections.

    This function attempts to start the ``nmtui`` program, providing a text
    user interface for viewing and controlling NetworkManager connections. A
    ``RuntimeError`` is raised if ``nmtui`` is not available or fails to run.
    """

    try:
        subprocess.run(["nmtui"], check=True)
    except FileNotFoundError as exc:  # pragma: no cover - external dependency
        raise RuntimeError(
            "nmtui command not found; ensure NetworkManager's text UI is installed"
        ) from exc
    except subprocess.CalledProcessError as exc:  # pragma: no cover - runtime
        raise RuntimeError("Failed to launch network console") from exc


if __name__ == "__main__":  # pragma: no cover - CLI behaviour
    open_network_console()
