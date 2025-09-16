PYTHON ?= python
DJANGO_MANAGE = $(PYTHON) backend/manage.py

.PHONY: setup run test lint format mypy seed docs

setup:
$(PYTHON) -m pip install -r requirements.txt
@if [ -f frontend/package.json ]; then cd frontend && npm install; fi

run:
$(DJANGO_MANAGE) migrate
$(DJANGO_MANAGE) runserver 0.0.0.0:8000

test:
pytest

lint:
flake8 backend
@if [ -d frontend ]; then cd frontend && npm run lint --if-present; fi

format:
black backend
isort backend

mypy:
mypy backend

seed:
$(DJANGO_MANAGE) loaddata backend/spaces/fixtures/demo_spaces.json

docs:
$(DJANGO_MANAGE) spectacular --format openapi-json --output docs/api/schema.json
