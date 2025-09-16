from __future__ import annotations

import uuid

import django.db.models.deletion
import spaces.fields
import spaces.models
from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = []

    operations = [
        migrations.CreateModel(
            name="Room",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4,
                        editable=False,
                        primary_key=True,
                        serialize=False,
                    ),
                ),
                ("name", models.CharField(max_length=120)),
                ("level", models.IntegerField(blank=True, null=True)),
                ("origin_x_mm", models.IntegerField(default=0)),
                ("origin_y_mm", models.IntegerField(default=0)),
                ("rotation_deg", models.FloatField(default=0.0)),
                ("polygon_mm", models.JSONField(default=list)),
                ("ceiling_height_mm", models.IntegerField()),
                ("metadata", models.JSONField(blank=True, default=dict)),
            ],
            options={
                "ordering": ["name"],
            },
        ),
        migrations.CreateModel(
            name="Camera",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4,
                        editable=False,
                        primary_key=True,
                        serialize=False,
                    ),
                ),
                ("name", models.CharField(max_length=120)),
                ("make", models.CharField(max_length=120)),
                ("model", models.CharField(max_length=120)),
                ("rtsp_url", spaces.fields.EncryptedTextField()),
                ("is_rtsp_encrypted", models.BooleanField(default=False)),
                ("webrtc_url", models.URLField(blank=True, null=True)),
                ("resolution_w", models.IntegerField()),
                ("resolution_h", models.IntegerField()),
                ("fps", models.IntegerField()),
                ("fov_deg", models.FloatField()),
                ("position_mm", models.JSONField(default=dict)),
                ("yaw_deg", models.FloatField(default=0.0)),
                ("pitch_deg", models.FloatField(default=0.0)),
                ("roll_deg", models.FloatField(default=0.0)),
                ("intrinsics", models.JSONField(blank=True, null=True)),
                (
                    "calibration_state",
                    models.CharField(
                        choices=[
                            ("UNSET", "Unset"),
                            ("PENDING", "Pending"),
                            ("IN_PROGRESS", "In Progress"),
                            ("CALIBRATED", "Calibrated"),
                            ("FAILED", "Failed"),
                        ],
                        default="UNSET",
                        max_length=20,
                    ),
                ),
                (
                    "last_health",
                    models.CharField(
                        choices=[
                            ("UNKNOWN", "Unknown"),
                            ("OK", "Ok"),
                            ("DEGRADED", "Degraded"),
                            ("OFFLINE", "Offline"),
                        ],
                        default="UNKNOWN",
                        max_length=20,
                    ),
                ),
                ("last_seen_at", models.DateTimeField(blank=True, null=True)),
                ("metadata", models.JSONField(blank=True, default=dict)),
                (
                    "room",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="cameras",
                        to="spaces.room",
                    ),
                ),
            ],
            options={
                "ordering": ["name"],
            },
        ),
        migrations.CreateModel(
            name="CameraCalibrationRun",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4,
                        editable=False,
                        primary_key=True,
                        serialize=False,
                    ),
                ),
                (
                    "status",
                    models.CharField(
                        choices=[
                            ("PENDING", "Pending"),
                            ("RUNNING", "Running"),
                            ("SUCCEEDED", "Succeeded"),
                            ("FAILED", "Failed"),
                            ("CANCELLED", "Cancelled"),
                        ],
                        default="PENDING",
                        max_length=20,
                    ),
                ),
                (
                    "method",
                    models.CharField(
                        choices=[
                            ("CHARUCO", "Charuco"),
                            ("CHECKERBOARD", "Checkerboard"),
                            ("APRILTAG", "Apriltag"),
                        ],
                        max_length=20,
                    ),
                ),
                ("board_spec", models.JSONField()),
                ("error_rms", models.FloatField(blank=True, null=True)),
                ("started_at", models.DateTimeField(blank=True, null=True)),
                ("finished_at", models.DateTimeField(blank=True, null=True)),
                ("logs", models.TextField(blank=True)),
                (
                    "camera",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="calibration_runs",
                        to="spaces.camera",
                    ),
                ),
            ],
            options={
                "ordering": ["-created_at"],
            },
        ),
    ]
