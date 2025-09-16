"""Serializers for rooms, cameras and calibration runs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence

from django.utils import timezone
from rest_framework import serializers

from .models import Camera, CameraCalibrationRun, Room


@dataclass
class PolygonPoint:
    x_mm: int
    y_mm: int

    def as_dict(self) -> Dict[str, int]:
        return {"x_mm": self.x_mm, "y_mm": self.y_mm}


def _orientation(a: PolygonPoint, b: PolygonPoint, c: PolygonPoint) -> int:
    value = (b.y_mm - a.y_mm) * (c.x_mm - b.x_mm) - (b.x_mm - a.x_mm) * (
        c.y_mm - b.y_mm
    )
    if value == 0:
        return 0
    return 1 if value > 0 else 2


def _segments_intersect(
    p1: PolygonPoint, q1: PolygonPoint, p2: PolygonPoint, q2: PolygonPoint
) -> bool:
    if p1 == p2 or p1 == q2 or q1 == p2 or q1 == q2:
        return False
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)
    if o1 != o2 and o3 != o4:
        return True
    return False


def _is_simple_polygon(points: Sequence[PolygonPoint]) -> bool:
    edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
    for i, (a1, a2) in enumerate(edges):
        for j, (b1, b2) in enumerate(edges):
            if abs(i - j) <= 1:
                continue
            if i == 0 and j == len(edges) - 1:
                continue
            if _segments_intersect(a1, a2, b1, b2):
                return False
    return True


def _polygon_area(points: Sequence[PolygonPoint]) -> float:
    area = 0.0
    for i in range(len(points) - 1):
        area += points[i].x_mm * points[i + 1].y_mm
        area -= points[i + 1].x_mm * points[i].y_mm
    return area / 2.0


def _ensure_closed(points: List[PolygonPoint]) -> List[PolygonPoint]:
    if points[0] != points[-1]:
        points.append(points[0])
    return points


def _ensure_clockwise(points: List[PolygonPoint]) -> List[PolygonPoint]:
    area = _polygon_area(points)
    if area > 0:
        return list(reversed(points))
    return points


class RoomSerializer(serializers.ModelSerializer):
    polygon_mm = serializers.ListField(child=serializers.DictField(), allow_empty=False)

    class Meta:
        model = Room
        fields = [
            "id",
            "name",
            "level",
            "origin_x_mm",
            "origin_y_mm",
            "rotation_deg",
            "polygon_mm",
            "ceiling_height_mm",
            "metadata",
            "created_at",
            "updated_at",
        ]
        read_only_fields = ("created_at", "updated_at")

    def validate_polygon_mm(
        self, value: Iterable[Dict[str, Any]]
    ) -> List[Dict[str, int]]:
        try:
            points = [
                PolygonPoint(int(point["x_mm"]), int(point["y_mm"])) for point in value
            ]
        except (KeyError, TypeError, ValueError) as exc:
            raise serializers.ValidationError(
                "Each vertex must contain integer x_mm/y_mm"
            ) from exc
        if len(points) < 3:
            raise serializers.ValidationError(
                "Polygon must contain at least three vertices"
            )
        closed_points = _ensure_closed(points.copy())
        if not _is_simple_polygon(closed_points):
            raise serializers.ValidationError(
                "Polygon must be simple (no self-intersections)"
            )
        area = abs(_polygon_area(closed_points))
        if area < 1:
            raise serializers.ValidationError("Polygon area must be greater than zero")
        clockwise = _ensure_clockwise(closed_points)
        return [p.as_dict() for p in clockwise]


class RoomRecenterSerializer(serializers.Serializer):
    origin_x_mm = serializers.IntegerField()
    origin_y_mm = serializers.IntegerField()
    rotation_deg = serializers.FloatField()


class CameraSerializer(serializers.ModelSerializer):
    room_name = serializers.CharField(source="room.name", read_only=True)
    rtsp_url = serializers.CharField(write_only=True, required=False, allow_blank=True)
    rtsp_url_masked = serializers.SerializerMethodField()

    class Meta:
        model = Camera
        fields = [
            "id",
            "name",
            "room",
            "room_name",
            "make",
            "model",
            "rtsp_url",
            "rtsp_url_masked",
            "is_rtsp_encrypted",
            "webrtc_url",
            "resolution_w",
            "resolution_h",
            "fps",
            "fov_deg",
            "position_mm",
            "yaw_deg",
            "pitch_deg",
            "roll_deg",
            "intrinsics",
            "calibration_state",
            "last_health",
            "last_seen_at",
            "metadata",
            "created_at",
            "updated_at",
        ]
        read_only_fields = ("created_at", "updated_at", "is_rtsp_encrypted")

    def validate_position_mm(self, value: Dict[str, Any]) -> Dict[str, int]:
        required_keys = {"x_mm", "y_mm", "z_mm"}
        if not required_keys.issubset(value.keys()):
            raise serializers.ValidationError(
                "Position must include x_mm, y_mm and z_mm"
            )
        return {key: int(value[key]) for key in required_keys}

    def get_rtsp_url_masked(self, obj: Camera) -> str:
        if obj.is_rtsp_encrypted:
            return "********"
        return ""

    def to_internal_value(self, data: Any) -> Any:
        internal = super().to_internal_value(data)
        if "position_mm" not in internal and not self.partial:
            internal["position_mm"] = {"x_mm": 0, "y_mm": 0, "z_mm": 0}
        return internal

    def create(self, validated_data: Dict[str, Any]) -> Camera:
        rtsp = validated_data.pop("rtsp_url", "")
        camera = super().create(validated_data)
        if rtsp:
            camera.rtsp_url = rtsp
            camera.save(update_fields=["rtsp_url", "is_rtsp_encrypted", "updated_at"])
        return camera

    def update(self, instance: Camera, validated_data: Dict[str, Any]) -> Camera:
        rtsp = validated_data.pop("rtsp_url", None)
        camera = super().update(instance, validated_data)
        if rtsp is not None:
            camera.rtsp_url = rtsp
            camera.save(update_fields=["rtsp_url", "is_rtsp_encrypted", "updated_at"])
        return camera


class CameraTestConnectionSerializer(serializers.Serializer):
    ok = serializers.BooleanField()
    fps = serializers.IntegerField(required=False)
    resolution = serializers.CharField(required=False)
    message = serializers.CharField(required=False)


class CameraCalibrationRunSerializer(serializers.ModelSerializer):
    camera_name = serializers.CharField(source="camera.name", read_only=True)

    class Meta:
        model = CameraCalibrationRun
        fields = [
            "id",
            "camera",
            "camera_name",
            "status",
            "method",
            "board_spec",
            "error_rms",
            "started_at",
            "finished_at",
            "logs",
            "created_at",
            "updated_at",
        ]
        read_only_fields = ("created_at", "updated_at")


class CalibrationStartSerializer(serializers.Serializer):
    method = serializers.ChoiceField(choices=CameraCalibrationRun.Method.choices)
    board_spec = serializers.DictField()


class CalibrationCancelSerializer(serializers.Serializer):
    run_id = serializers.UUIDField(required=False)


class CalibrationProgressSerializer(serializers.Serializer):
    run_id = serializers.UUIDField()
    pct = serializers.FloatField()
    msg = serializers.CharField()


class SaveIntrinsicsSerializer(serializers.Serializer):
    intrinsics = serializers.DictField()
    error_rms = serializers.FloatField(required=False)

    def save(self) -> Camera:
        camera: Camera = self.context["camera"]
        camera.intrinsics = self.validated_data["intrinsics"]
        if "error_rms" in self.validated_data:
            camera.metadata = {
                **camera.metadata,
                "last_calibration_error": self.validated_data["error_rms"],
            }
        camera.calibration_state = Camera.CalibrationState.CALIBRATED
        camera.save(
            update_fields=["intrinsics", "metadata", "calibration_state", "updated_at"]
        )
        return camera
