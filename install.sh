#!/usr/bin/env bash
set -e

python -m venv .venv
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
