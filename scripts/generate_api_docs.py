"""Generate simple API reference Markdown files."""

from __future__ import annotations

import inspect
from pathlib import Path
import sys
from typing import Iterable

MODULES = [
    "altinet.nodes.detector_node",
    "altinet.nodes.tracker_node",
    "altinet.nodes.event_manager_node",
    "altinet.nodes.lighting_control_node",
    "altinet.nodes.ros2_django_bridge_node",
    "altinet.utils.models",
    "altinet.utils.tracking",
    "altinet.utils.geometry",
]


def iter_public_members(module) -> Iterable[tuple[str, object]]:
    for name, member in inspect.getmembers(module):
        if name.startswith("_"):
            continue
        yield name, member


def render_module(module_name: str) -> str:
    module = __import__(module_name, fromlist=["__name__"])
    lines = [f"# {module_name}"]
    doc = inspect.getdoc(module)
    if doc:
        lines.append(doc)
    for name, member in iter_public_members(module):
        if inspect.isclass(member) or inspect.isfunction(member):
            lines.append(f"## {name}")
            member_doc = inspect.getdoc(member)
            if member_doc:
                lines.append(member_doc)
    return "\n\n".join(lines) + "\n"


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    src = root / "ros2_ws" / "src"
    sys.path.insert(0, str(src))
    sys.path.insert(0, str(src / "altinet"))
    output_dir = Path("docs/api")
    output_dir.mkdir(parents=True, exist_ok=True)
    for module_name in MODULES:
        content = render_module(module_name)
        outfile = output_dir / f"{module_name.replace('.', '_')}.md"
        outfile.write_text(content, encoding="utf8")


if __name__ == "__main__":
    main()
