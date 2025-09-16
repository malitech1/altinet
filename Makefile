PYTHONPATH := ros2_ws/src

.PHONY: docs test lint format mypy

docs:
	PYTHONPATH=$(PYTHONPATH) python scripts/generate_api_docs.py

format:
	black ros2_ws/src/altinet/altinet

lint:
	ruff check ros2_ws/src/altinet/altinet

mypy:
	PYTHONPATH=$(PYTHONPATH) mypy ros2_ws/src/altinet/altinet

test:
	PYTHONPATH=$(PYTHONPATH) pytest
