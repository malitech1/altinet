#!/usr/bin/env python3
"""Generate a 3D floorplan using Blender's Python API."""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Iterable, Sequence

import bpy


@dataclass
class Bounds:
    """Simple container describing 2D bounds in builder space."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def depth(self) -> float:
        return self.max_y - self.min_y

    @property
    def centre_x(self) -> float:
        return (self.min_x + self.max_x) / 2

    @property
    def centre_y(self) -> float:
        return (self.min_y + self.max_y) / 2


def parse_args():
    """Parse command-line arguments passed after `--` by Blender."""
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    parser = argparse.ArgumentParser(description="Generate a 3D floorplan from the Altinet builder export.")
    parser.add_argument("--out", default="assets/floorplans/generated_floorplan.blend", help="Output .blend file path")
    parser.add_argument("--plan", help="Path to an exported floorplan JSON from the web builder")
    parser.add_argument("--obj-out", help="Optional OBJ path for the dashboard viewer")
    parser.add_argument("--wall-height", type=float, default=3.0, help="Wall height in metres")
    parser.add_argument("--wall-thickness", type=float, default=0.2, help="Wall thickness in metres")
    parser.add_argument("--storey-height", type=float, default=3.2, help="Vertical spacing between storeys in metres")
    parser.add_argument("--floor-thickness", type=float, default=0.15, help="Thickness of each floor slab in metres")
    parser.add_argument(
        "--unit-scale",
        type=float,
        default=0.5,
        help="Metres per grid unit if the plan export does not contain unitScale",
    )
    parser.add_argument("--fallback-width", type=float, default=10.0, help="Fallback floor width when no plan is provided")
    parser.add_argument("--fallback-depth", type=float, default=8.0, help="Fallback floor depth when no plan is provided")
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


def add_floor_slab(width, depth, thickness, location, material, name="Floor"):
    """Add a rectangular floor slab represented as a scaled cube."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=(location[0], location[1], location[2] - thickness / 2))
    floor = bpy.context.active_object
    floor.name = name
    floor.scale = (width / 2, depth / 2, thickness / 2)
    floor.data.materials.append(material)
    return floor


def add_wall(length, thickness, height, location, material, rotation=0.0, name="Wall"):
    """Add a single wall segment as a scaled cube with optional rotation."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    wall = bpy.context.active_object
    wall.name = name
    wall.scale = (length / 2, thickness / 2, height / 2)
    wall.rotation_euler = (0, 0, rotation)
    wall.data.materials.append(material)
    return wall


def add_camera(bounds: Bounds, wall_height: float, storey_height: float):
    """Create an isometric-style orthographic camera centred on the plan."""
    cam_data = bpy.data.cameras.new("Camera")
    cam_data.type = "ORTHO"
    radius = max(bounds.width, bounds.depth) * 1.2 / 2
    cam_data.ortho_scale = max(bounds.width, bounds.depth) * 1.4

    cam = bpy.data.objects.new("Camera", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = (
        radius * math.sqrt(2),
        -radius * math.sqrt(2),
        wall_height + storey_height,
    )
    cam.rotation_euler = (math.radians(60), 0, math.radians(45))
    bpy.context.scene.camera = cam


def add_sunlight():
    """Add a sun lamp for basic illumination."""
    light_data = bpy.data.lights.new(name="Sun", type="SUN")
    light = bpy.data.objects.new(name="Sun", object_data=light_data)
    bpy.context.collection.objects.link(light)
    light.rotation_euler = (math.radians(60), 0, math.radians(45))


def load_plan(path: str) -> dict:
    """Load a builder export JSON."""
    with open(path, "r", encoding="utf-8") as handle:
        plan = json.load(handle)
    if not isinstance(plan, dict) or "levels" not in plan:
        raise ValueError("Plan export is missing levels")
    levels = plan["levels"]
    if not isinstance(levels, Sequence) or len(levels) == 0:
        raise ValueError("Plan export must contain at least one level")
    return plan


def level_bounds(level: dict, grid_size: float) -> Bounds:
    """Calculate 2D bounds for a single level."""
    xs: list[float] = []
    ys: list[float] = []

    for wall in level.get("walls", []):
        for key in ("start", "end"):
            point = wall.get(key)
            if isinstance(point, dict):
                xs.append(float(point.get("x", 0)))
                ys.append(float(point.get("y", 0)))
    for room in level.get("rooms", []):
        x = float(room.get("x", 0))
        y = float(room.get("y", 0))
        width = float(room.get("width", grid_size))
        height = float(room.get("height", grid_size))
        xs.extend([x, x + width])
        ys.extend([y, y + height])

    if not xs or not ys:
        return Bounds(0.0, grid_size, 0.0, grid_size)

    return Bounds(min(xs), max(xs), min(ys), max(ys))


def aggregate_bounds(levels: Iterable[dict], grid_size: float) -> Bounds:
    """Combine bounds from multiple levels to determine the overall extents."""
    all_bounds = [level_bounds(level, grid_size) for level in levels]
    return Bounds(
        min(bound.min_x for bound in all_bounds),
        max(bound.max_x for bound in all_bounds),
        min(bound.min_y for bound in all_bounds),
        max(bound.max_y for bound in all_bounds),
    )


def convert_point(point: dict, bounds: Bounds, grid_size: float, unit_scale: float) -> tuple[float, float]:
    """Convert canvas coordinates to Blender world coordinates."""
    x = float(point.get("x", 0.0))
    y = float(point.get("y", 0.0))
    world_x = ((x - bounds.centre_x) / grid_size) * unit_scale
    world_y = -((y - bounds.centre_y) / grid_size) * unit_scale
    return world_x, world_y


def build_from_plan(plan: dict, args, floor_mat, wall_mat):
    """Create geometry in Blender based on the exported plan."""
    grid_size = float(plan.get("gridSize", 30))
    unit_scale = float(plan.get("unitScale", args.unit_scale))
    levels = plan.get("levels", [])
    plan_bounds = aggregate_bounds(levels, grid_size)

    for index, level in enumerate(levels):
        level_name = level.get("name") or f"Level {index + 1}"
        base_height = index * args.storey_height

        for room in level.get("rooms", []):
            centre_pixel = {
                "x": room.get("x", 0) + room.get("width", 0) / 2,
                "y": room.get("y", 0) + room.get("height", 0) / 2,
            }
            centre_x, centre_y = convert_point(centre_pixel, plan_bounds, grid_size, unit_scale)
            width = float(room.get("width", grid_size)) / grid_size * unit_scale
            depth = float(room.get("height", grid_size)) / grid_size * unit_scale
            add_floor_slab(
                width,
                depth,
                args.floor_thickness,
                (centre_x, centre_y, base_height),
                floor_mat,
                name=f"{level_name} - {room.get('name', 'Room')} floor",
            )

        for wall in level.get("walls", []):
            start = wall.get("start", {})
            end = wall.get("end", {})
            start_x, start_y = convert_point(start, plan_bounds, grid_size, unit_scale)
            end_x, end_y = convert_point(end, plan_bounds, grid_size, unit_scale)
            length = math.hypot(end_x - start_x, end_y - start_y)
            if length == 0:
                continue
            centre_x = (start_x + end_x) / 2
            centre_y = (start_y + end_y) / 2
            rotation = math.atan2(end_y - start_y, end_x - start_x)
            add_wall(
                length,
                args.wall_thickness,
                args.wall_height,
                (centre_x, centre_y, base_height + args.wall_height / 2),
                wall_mat,
                rotation=rotation,
                name=f"{level_name} wall",
            )

    add_camera(plan_bounds, args.wall_height, args.storey_height)
    add_sunlight()


def build_fallback(args, floor_mat, wall_mat):
    """Fallback geometry when no plan export is provided."""
    add_floor_slab(
        args.fallback_width,
        args.fallback_depth,
        args.floor_thickness,
        (0, 0, 0),
        floor_mat,
        name="Fallback floor",
    )
    add_wall(
        args.fallback_width,
        args.wall_thickness,
        args.wall_height,
        (0, args.fallback_depth / 2, args.wall_height / 2),
        wall_mat,
        name="North wall",
    )
    add_wall(
        args.fallback_width,
        args.wall_thickness,
        args.wall_height,
        (0, -args.fallback_depth / 2, args.wall_height / 2),
        wall_mat,
        name="South wall",
    )
    add_wall(
        args.wall_thickness,
        args.fallback_depth,
        args.wall_height,
        (args.fallback_width / 2, 0, args.wall_height / 2),
        wall_mat,
        name="East wall",
    )
    add_wall(
        args.wall_thickness,
        args.fallback_depth,
        args.wall_height,
        (-args.fallback_width / 2, 0, args.wall_height / 2),
        wall_mat,
        name="West wall",
    )
    add_camera(Bounds(-args.fallback_width / 2, args.fallback_width / 2, -args.fallback_depth / 2, args.fallback_depth / 2), args.wall_height, args.storey_height)
    add_sunlight()


def export_obj(path: str):
    """Export the current scene to OBJ."""
    bpy.ops.export_scene.obj(filepath=path, use_selection=False, use_materials=False)
    print(f"OBJ exported to {path}")


def main():
    args = parse_args()

    clear_scene()
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    floor_mat = create_material("FloorMaterial", (0.82, 0.82, 0.82))
    wall_mat = create_material("WallMaterial", (0.92, 0.92, 0.92))

    if args.plan:
        plan = load_plan(args.plan)
        build_from_plan(plan, args, floor_mat, wall_mat)
    else:
        build_fallback(args, floor_mat, wall_mat)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    bpy.ops.wm.save_mainfile(filepath=args.out)
    print(f"Floorplan saved to {args.out}")

    if args.obj_out:
        os.makedirs(os.path.dirname(args.obj_out), exist_ok=True)
        export_obj(args.obj_out)


if __name__ == "__main__":
    main()
