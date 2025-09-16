"""Database models for rooms, cameras and calibration runs."""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from typing import Iterable, List, Sequence

from django.core.validators import MaxValueValidator, MinValueValidator
from django.db import models

from .fields import EncryptedTextField


class TimeStampedModel(models.Model):
    """Abstract model that provides created/updated timestamps."""

    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        abstract = True


@dataclass
class Point:
    x_mm: int
    y_mm: int


class Room(TimeStampedModel):
    """A physical room/floorplan polygon."""

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    name = models.CharField(max_length=120)
    level = models.IntegerField(blank=True, null=True)
    origin_x_mm = models.IntegerField(default=0)
    origin_y_mm = models.IntegerField(default=0)
    rotation_deg = models.FloatField(default=0.0)
    polygon_mm = models.JSONField(default=list)
    ceiling_height_mm = models.IntegerField(validators=[MinValueValidator(1000)])
    metadata = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["name"]

    def __str__(self) -> str:  # pragma: no cover - readable repr
        return f"Room<{self.name}>"

    @property
    def polygon_points(self) -> List[Point]:
        points: List[Point] = []
        for item in self.polygon_mm or []:
            points.append(Point(int(item["x_mm"]), int(item["y_mm"])))
        return points


class Camera(TimeStampedModel):
    """Camera metadata and placement configuration."""

    class CalibrationState(models.TextChoices):
        UNSET = "UNSET"
        PENDING = "PENDING"
        IN_PROGRESS = "IN_PROGRESS"
        CALIBRATED = "CALIBRATED"
        FAILED = "FAILED"

    class HealthState(models.TextChoices):
        UNKNOWN = "UNKNOWN"
        OK = "OK"
        DEGRADED = "DEGRADED"
        OFFLINE = "OFFLINE"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    name = models.CharField(max_length=120)
    room = models.ForeignKey(Room, related_name="cameras", on_delete=models.CASCADE)
    make = models.CharField(max_length=120)
    model = models.CharField(max_length=120)
    rtsp_url = EncryptedTextField()
    is_rtsp_encrypted = models.BooleanField(default=False)
    webrtc_url = models.URLField(blank=True, null=True)
    resolution_w = models.IntegerField(validators=[MinValueValidator(1)])
    resolution_h = models.IntegerField(validators=[MinValueValidator(1)])
    fps = models.IntegerField(validators=[MinValueValidator(1), MaxValueValidator(240)])
    fov_deg = models.FloatField(
        validators=[MinValueValidator(1.0), MaxValueValidator(179.0)]
    )
    position_mm = models.JSONField(default=dict)
    yaw_deg = models.FloatField(default=0.0)
    pitch_deg = models.FloatField(default=0.0)
    roll_deg = models.FloatField(default=0.0)
    intrinsics = models.JSONField(blank=True, null=True)
    calibration_state = models.CharField(
        max_length=20, choices=CalibrationState.choices, default=CalibrationState.UNSET
    )
    last_health = models.CharField(
        max_length=20, choices=HealthState.choices, default=HealthState.UNKNOWN
    )
    last_seen_at = models.DateTimeField(blank=True, null=True)
    metadata = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["name"]

    def save(self, *args, **kwargs) -> None:
        if self.rtsp_url:
            self.is_rtsp_encrypted = True
        super().save(*args, **kwargs)

    def __str__(self) -> str:  # pragma: no cover
        return f"Camera<{self.name}>"


class CameraCalibrationRun(TimeStampedModel):
    """Individual calibration runs for a camera."""

    class Status(models.TextChoices):
        PENDING = "PENDING"
        RUNNING = "RUNNING"
        SUCCEEDED = "SUCCEEDED"
        FAILED = "FAILED"
        CANCELLED = "CANCELLED"

    class Method(models.TextChoices):
        CHARUCO = "CHARUCO"
        CHECKERBOARD = "CHECKERBOARD"
        APRILTAG = "APRILTAG"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    camera = models.ForeignKey(
        Camera, related_name="calibration_runs", on_delete=models.CASCADE
    )
    status = models.CharField(
        max_length=20, choices=Status.choices, default=Status.PENDING
    )
    method = models.CharField(max_length=20, choices=Method.choices)
    board_spec = models.JSONField()
    error_rms = models.FloatField(blank=True, null=True)
    started_at = models.DateTimeField(blank=True, null=True)
    finished_at = models.DateTimeField(blank=True, null=True)
    logs = models.TextField(blank=True)

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:  # pragma: no cover
        return f"CalibrationRun<{self.camera_id} {self.status}>"
