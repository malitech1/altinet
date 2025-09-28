"""Helpers for generating OBJ meshes from builder floorplans."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


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


def _level_bounds(level: dict, grid_size: float) -> Bounds:
    xs: List[float] = []
    ys: List[float] = []

    for wall in level.get("walls", []):
        for key in ("start", "end"):
            point = wall.get(key)
            if isinstance(point, dict):
                xs.append(float(point.get("x", 0.0)))
                ys.append(float(point.get("y", 0.0)))

    for room in level.get("rooms", []):
        x = float(room.get("x", 0.0))
        y = float(room.get("y", 0.0))
        width = float(room.get("width", grid_size))
        height = float(room.get("height", grid_size))
        xs.extend([x, x + width])
        ys.extend([y, y + height])

    if not xs or not ys:
        return Bounds(0.0, grid_size, 0.0, grid_size)

    return Bounds(min(xs), max(xs), min(ys), max(ys))


def _aggregate_bounds(levels: Iterable[dict], grid_size: float) -> Bounds:
    level_bounds = [_level_bounds(level, grid_size) for level in levels]
    return Bounds(
        min(bound.min_x for bound in level_bounds),
        max(bound.max_x for bound in level_bounds),
        min(bound.min_y for bound in level_bounds),
        max(bound.max_y for bound in level_bounds),
    )


def _convert_point(
    point: dict, bounds: Bounds, grid_size: float, unit_scale: float
) -> Tuple[float, float]:
    x = float(point.get("x", 0.0))
    y = float(point.get("y", 0.0))
    world_x = ((x - bounds.centre_x) / grid_size) * unit_scale
    world_y = -((y - bounds.centre_y) / grid_size) * unit_scale
    return world_x, world_y


class FloorplanGenerator:
    """Generate a lightweight OBJ representation of a builder floorplan."""

    def __init__(
        self,
        plan: dict,
        *,
        obj_output_path: Path,
        plan_storage_path: Path | None = None,
        wall_height: float = 3.0,
        wall_thickness: float = 0.2,
        storey_height: float = 3.2,
        floor_thickness: float = 0.15,
        fallback_width: float = 10.0,
        fallback_depth: float = 8.0,
    ) -> None:
        self.plan = plan
        self.obj_output_path = obj_output_path
        self.plan_storage_path = plan_storage_path
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.storey_height = storey_height
        self.floor_thickness = floor_thickness
        self.fallback_width = fallback_width
        self.fallback_depth = fallback_depth

    def generate(self) -> dict:
        """Render the OBJ and persist the canonical plan JSON."""

        levels: Sequence[dict] = self.plan.get("levels", [])
        if not isinstance(levels, Sequence) or len(levels) == 0:
            raise ValueError("Floorplan must contain at least one level")

        grid_size = float(self.plan.get("gridSize", 30.0))
        unit_scale = float(self.plan.get("unitScale", 0.5))

        has_geometry = any(
            (level.get("rooms") or level.get("walls")) for level in levels
        )

        vertices: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int, int]] = []

        if has_geometry:
            bounds = _aggregate_bounds(levels, grid_size)
            room_count = 0
            wall_count = 0
            for index, level in enumerate(levels):
                level_index = int(level.get("index", index))
                base_height = level_index * self.storey_height

                for room in level.get("rooms", []):
                    room_count += 1
                    width_units = float(room.get("width", grid_size)) / grid_size
                    depth_units = float(room.get("height", grid_size)) / grid_size
                    width = width_units * unit_scale
                    depth = depth_units * unit_scale
                    centre_x = float(room.get("x", 0.0)) + float(
                        room.get("width", grid_size)
                    ) / 2
                    centre_y = float(room.get("y", 0.0)) + float(
                        room.get("height", grid_size)
                    ) / 2
                    world_x, world_y = _convert_point(
                        {"x": centre_x, "y": centre_y},
                        bounds,
                        grid_size,
                        unit_scale,
                    )
                    self._add_aligned_prism(
                        vertices,
                        faces,
                        world_x,
                        world_y,
                        width,
                        depth,
                        base_height,
                        max(self.floor_thickness, 0.01),
                    )

                for wall in level.get("walls", []):
                    start = wall.get("start", {})
                    end = wall.get("end", {})
                    start_pt = _convert_point(start, bounds, grid_size, unit_scale)
                    end_pt = _convert_point(end, bounds, grid_size, unit_scale)
                    if start_pt == end_pt:
                        continue
                    wall_count += 1
                    self._add_wall_prism(
                        vertices,
                        faces,
                        start_pt,
                        end_pt,
                        base_height,
                        max(self.wall_height, 0.1),
                        max(self.wall_thickness, 0.05),
                    )
        else:
            bounds = Bounds(-0.5, 0.5, -0.5, 0.5)
            room_count = 0
            wall_count = 0
            self._add_aligned_prism(
                vertices,
                faces,
                0.0,
                0.0,
                self.fallback_width,
                self.fallback_depth,
                0.0,
                max(self.floor_thickness, 0.01),
            )

        self._write_obj(vertices, faces)
        if self.plan_storage_path is not None:
            self.plan_storage_path.parent.mkdir(parents=True, exist_ok=True)
            with self.plan_storage_path.open("w", encoding="utf-8") as handle:
                json.dump(self.plan, handle, indent=2, ensure_ascii=False)

        return {
            "levels": len(levels),
            "rooms": room_count,
            "walls": wall_count,
            "obj_path": str(self.obj_output_path),
            "plan_path": str(self.plan_storage_path) if self.plan_storage_path else None,
            "vertices": len(vertices),
            "faces": len(faces),
            "bounds": {
                "width": bounds.width,
                "depth": bounds.depth,
            },
        }

    def _add_aligned_prism(
        self,
        vertices: List[Tuple[float, float, float]],
        faces: List[Tuple[int, int, int, int]],
        centre_x: float,
        centre_y: float,
        width: float,
        depth: float,
        base_height: float,
        height: float,
    ) -> None:
        half_w = width / 2
        half_d = depth / 2
        corners = [
            (centre_x - half_w, centre_y - half_d),
            (centre_x + half_w, centre_y - half_d),
            (centre_x + half_w, centre_y + half_d),
            (centre_x - half_w, centre_y + half_d),
        ]
        self._add_prism(vertices, faces, corners, base_height, height)

    def _add_wall_prism(
        self,
        vertices: List[Tuple[float, float, float]],
        faces: List[Tuple[int, int, int, int]],
        start: Tuple[float, float],
        end: Tuple[float, float],
        base_height: float,
        wall_height: float,
        wall_thickness: float,
    ) -> None:
        start_x, start_y = start
        end_x, end_y = end
        dx = end_x - start_x
        dy = end_y - start_y
        length = math.hypot(dx, dy)
        if length == 0:
            return
        ux = dx / length
        uy = dy / length
        px = -uy
        py = ux
        half_len = length / 2
        half_thick = wall_thickness / 2
        mid_x = (start_x + end_x) / 2
        mid_y = (start_y + end_y) / 2
        corners = [
            (
                mid_x - ux * half_len - px * half_thick,
                mid_y - uy * half_len - py * half_thick,
            ),
            (
                mid_x + ux * half_len - px * half_thick,
                mid_y + uy * half_len - py * half_thick,
            ),
            (
                mid_x + ux * half_len + px * half_thick,
                mid_y + uy * half_len + py * half_thick,
            ),
            (
                mid_x - ux * half_len + px * half_thick,
                mid_y - uy * half_len + py * half_thick,
            ),
        ]
        self._add_prism(vertices, faces, corners, base_height, wall_height)

    def _add_prism(
        self,
        vertices: List[Tuple[float, float, float]],
        faces: List[Tuple[int, int, int, int]],
        base_corners: Sequence[Tuple[float, float]],
        base_height: float,
        height: float,
    ) -> None:
        start_index = len(vertices) + 1
        for x, y in base_corners:
            vertices.append((x, y, base_height))
        for x, y in base_corners:
            vertices.append((x, y, base_height + height))
        faces.extend(
            [
                (start_index, start_index + 1, start_index + 2, start_index + 3),
                (
                    start_index + 4,
                    start_index + 5,
                    start_index + 6,
                    start_index + 7,
                ),
                (
                    start_index,
                    start_index + 1,
                    start_index + 5,
                    start_index + 4,
                ),
                (
                    start_index + 1,
                    start_index + 2,
                    start_index + 6,
                    start_index + 5,
                ),
                (
                    start_index + 2,
                    start_index + 3,
                    start_index + 7,
                    start_index + 6,
                ),
                (
                    start_index + 3,
                    start_index,
                    start_index + 4,
                    start_index + 7,
                ),
            ]
        )

    def _write_obj(
        self, vertices: Sequence[Tuple[float, float, float]], faces: Sequence[Tuple[int, int, int, int]]
    ) -> None:
        self.obj_output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.obj_output_path.open("w", encoding="utf-8") as handle:
            handle.write("# Altinet autogenerated floorplan\n")
            for x, y, z in vertices:
                handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for face in faces:
                handle.write("f " + " ".join(str(index) for index in face) + "\n")
