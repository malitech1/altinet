.PHONY: floorplan

floorplan:
	blender -b -noaudio --python scripts/generate_floorplan.py -- \
	  --out assets/floorplans/basic_floorplan.blend \
	  --width 10 --depth 8 --wall_height 3 --wall_thickness 0.2
