#!/usr/bin/env python
"""Django's command-line utility for administrative tasks."""
import os
import sys
from pathlib import Path


def main() -> None:
    """Run administrative tasks."""
    project_dir = Path(__file__).resolve().parent
    if str(project_dir) not in sys.path:
        sys.path.insert(0, str(project_dir))
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "altinet_backend.settings")
    from django.core.management import execute_from_command_line

    execute_from_command_line(sys.argv)


if __name__ == "__main__":
    main()
