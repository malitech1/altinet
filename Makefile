PYTHON ?= python
DJANGO_MANAGE = $(PYTHON) backend/manage.py

.PHONY: setup run test lint format mypy seed docs

setup:
$(PYTHON) -m pip install -r requirements.txt

run:
$(DJANGO_MANAGE) migrate
$(DJANGO_MANAGE) runserver 0.0.0.0:8000

test:
pytest

lint:
flake8 backend

format:
black backend
isort backend

mypy:
mypy backend

seed:
$(DJANGO_MANAGE) loaddata backend/spaces/fixtures/demo_spaces.json

docs:
$(DJANGO_MANAGE) spectacular --format openapi-json --output docs/api/schema.json
