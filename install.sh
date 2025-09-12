#!/usr/bin/env bash
set -e

REQUIRED_PYTHON="3.10"
PYTHON_BIN="python${REQUIRED_PYTHON}"

if command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  INSTALLED_VERSION=$($PYTHON_BIN -V 2>&1 | awk '{print $2}' | cut -d'.' -f1,2)
else
  INSTALLED_VERSION=""
fi

if [ "$INSTALLED_VERSION" != "$REQUIRED_PYTHON" ]; then
  echo "Python $REQUIRED_PYTHON is required."
  if command -v apt-get >/dev/null 2>&1; then
    echo "Installing Python $REQUIRED_PYTHON..."
    sudo apt-get update
    sudo apt-get install -y "python${REQUIRED_PYTHON}"
  else
    echo "Please install Python $REQUIRED_PYTHON manually."
    exit 1
  fi
fi

PYTHON_BIN="python${REQUIRED_PYTHON}"

$PYTHON_BIN -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "Virtual environment created in .venv"
echo "Activate it with 'source .venv/bin/activate'"

if command -v blender >/dev/null 2>&1; then
  blender -b -noaudio --python scripts/generate_floorplan.py -- \
    --out assets/floorplans/basic_floorplan.blend \
    --width 10 --depth 8 --wall_height 3 --wall_thickness 0.2
else
  echo "Blender not found. To generate the floorplan manually, run:"
  echo "  blender -b -noaudio --python scripts/generate_floorplan.py -- \\
    --out assets/floorplans/basic_floorplan.blend --width 10 --depth 8 --wall_height 3 --wall_thickness 0.2"
fi
