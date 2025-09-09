#!/usr/bin/env bash
set -e

python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "Virtual environment created in .venv"
echo "Activate it with 'source .venv/bin/activate'"
