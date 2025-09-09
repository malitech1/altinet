"""Working memory and prompt construction utilities."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional


@dataclass
class Person:
    """Represents an observed person."""

    name: str
    location: Optional[str] = None


@dataclass
class EnvironmentContext:
    """Contextual information about the environment."""

    time: datetime
    temperature_c: Optional[float] = None
    location: Optional[str] = None


@dataclass
class WorkingMemory:
    """Collects details about people and environment context."""

    people: List[Person] = field(default_factory=list)
    context: Optional[EnvironmentContext] = None

    def add_person(self, name: str, location: Optional[str] = None) -> None:
        """Add a person to the working memory."""

        self.people.append(Person(name=name, location=location))

    def update_context(
        self, *, temperature_c: Optional[float] = None, location: Optional[str] = None
    ) -> None:
        """Update the environmental context using the current time."""

        self.context = EnvironmentContext(
            time=datetime.now(), temperature_c=temperature_c, location=location
        )

    def build_prompt(self) -> str:
        """Return a prompt string summarising the current memory."""

        pieces: List[str] = []
        if self.context:
            time_str = self.context.time.strftime("%Y-%m-%d %H:%M:%S")
            temp = (
                f"{self.context.temperature_c:.1f}\u00b0C"
                if self.context.temperature_c is not None
                else "unknown temperature"
            )
            loc = self.context.location or "unknown location"
            pieces.append(f"The current time is {time_str} at {loc} with {temp}.")
        if self.people:
            people_desc = ", ".join(
                f"{p.name}" + (f" in {p.location}" if p.location else "")
                for p in self.people
            )
            pieces.append(f"People present: {people_desc}.")
        else:
            pieces.append("No people are currently detected.")
        return " ".join(pieces)
