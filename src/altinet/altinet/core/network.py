"""Networking primitives for Altinet."""

from dataclasses import dataclass

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
