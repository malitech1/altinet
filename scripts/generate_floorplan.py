#!/usr/bin/env python3
"""Generate a simple 3D floorplan using Blender's Python API."""

import argparse
import math
import os
import sys

import bpy


def parse_args():
    """Parse command-line arguments passed after `--` by Blender."""
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    parser = argparse.ArgumentParser(description="Generate a simple 3D floorplan.")
    parser.add_argument("--out", default="assets/floorplans/basic_floorplan.blend", help="Output .blend file path")
    parser.add_argument("--width", type=float, default=10.0, help="Floor width in meters")
    parser.add_argument("--depth", type=float, default=8.0, help="Floor depth in meters")
    parser.add_argument("--wall_height", type=float, default=3.0, help="Wall height in meters")
    parser.add_argument("--wall_thickness", type=float, default=0.2, help="Wall thickness in meters")
    return parser.parse_args(argv)


def clear_scene():
    """Remove all objects and data blocks from the current scene."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in bpy.data.meshes:
        bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        bpy.data.materials.remove(block)


def create_material(name, color):
    """Create a basic diffuse material."""
    mat = bpy.data.materials.new(name=name)
    mat.diffuse_color = (*color, 1.0)
    return mat


def add_floor(width, depth, material):
    """Add a rectangular floor plane."""
    bpy.ops.mesh.primitive_plane_add(size=1)
    floor = bpy.context.active_object
    floor.name = "Floor"
    floor.scale = (width / 2, depth / 2, 1)
    floor.data.materials.append(material)
    return floor


def add_wall(length, thickness, height, location, material):
    """Add a single wall segment as a scaled cube."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    wall = bpy.context.active_object
    wall.name = "Wall"
    wall.scale = (length / 2, thickness / 2, height / 2)
    wall.data.materials.append(material)
    return wall


def add_walls(width, depth, wall_height, wall_thickness, material):
    """Add four perimeter walls around the floor plane."""
    z = wall_height / 2
    d = depth / 2 + wall_thickness / 2
    w = width / 2 + wall_thickness / 2
    add_wall(width, wall_thickness, wall_height, (0, d, z), material)   # North
    add_wall(width, wall_thickness, wall_height, (0, -d, z), material)  # South
    add_wall(wall_thickness, depth, wall_height, (w, 0, z), material)   # East
    add_wall(wall_thickness, depth, wall_height, (-w, 0, z), material)  # West


def add_camera(width, depth, wall_height):
    """Create an isometric-style orthographic camera."""
    cam_data = bpy.data.cameras.new("Camera")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = max(width, depth) * 1.5

    cam = bpy.data.objects.new("Camera", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = (width, -depth, wall_height * 2)
    cam.rotation_euler = (math.radians(60), 0, math.radians(45))
    bpy.context.scene.camera = cam


def add_sunlight():
    """Add a sun lamp for basic illumination."""
    light_data = bpy.data.lights.new(name="Sun", type="SUN")
    light = bpy.data.objects.new(name="Sun", object_data=light_data)
    bpy.context.collection.objects.link(light)
    light.rotation_euler = (math.radians(60), 0, math.radians(45))


def main():
    args = parse_args()

    clear_scene()
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    floor_mat = create_material("FloorMaterial", (0.8, 0.8, 0.8))
    wall_mat = create_material("WallMaterial", (0.9, 0.9, 0.9))

    add_floor(args.width, args.depth, floor_mat)
    add_walls(args.width, args.depth, args.wall_height, args.wall_thickness, wall_mat)
    add_camera(args.width, args.depth, args.wall_height)
    add_sunlight()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    bpy.ops.wm.save_mainfile(filepath=args.out)
    print(f"Floorplan saved to {args.out}")


if __name__ == "__main__":
    main()
