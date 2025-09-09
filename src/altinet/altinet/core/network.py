"""Networking primitives for Altinet."""

from dataclasses import dataclass


@dataclass
class NetworkNode:
    """Represents a node in the Altinet network."""

    address: str

    def connect(self, other: "NetworkNode") -> None:
        """Placeholder method to establish a connection to another node."""
        raise NotImplementedError
