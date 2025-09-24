from __future__ import annotations

import uuid

import django.db.models.deletion
from django.db import migrations, models
from django.utils import timezone


class Migration(migrations.Migration):
    dependencies = [
        ("spaces", "0001_initial"),
    ]

    operations = [
        migrations.CreateModel(
            name="Identity",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4, editable=False, primary_key=True, serialize=False
                    ),
                ),
                ("display_name", models.CharField(max_length=120)),
                (
                    "external_id",
                    models.CharField(blank=True, max_length=120, null=True, unique=True),
                ),
                ("is_active", models.BooleanField(default=True)),
                ("metadata", models.JSONField(blank=True, default=dict)),
            ],
            options={"ordering": ["display_name"]},
        ),
        migrations.CreateModel(
            name="FaceEmbedding",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4, editable=False, primary_key=True, serialize=False
                    ),
                ),
                ("embedding_id", models.CharField(max_length=64, unique=True)),
                ("vector", models.JSONField(default=list)),
                ("quality", models.FloatField(default=0.0)),
                ("track_id", models.IntegerField(blank=True, null=True)),
                ("captured_at", models.DateTimeField(default=timezone.now)),
                ("metadata", models.JSONField(blank=True, default=dict)),
                (
                    "camera",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="face_embeddings",
                        to="spaces.camera",
                    ),
                ),
                (
                    "identity",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="embeddings",
                        to="spaces.identity",
                    ),
                ),
            ],
            options={"ordering": ["-created_at"]},
        ),
        migrations.CreateModel(
            name="FaceSnapshot",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "id",
                    models.UUIDField(
                        default=uuid.uuid4, editable=False, primary_key=True, serialize=False
                    ),
                ),
                ("track_id", models.IntegerField(blank=True, null=True)),
                ("captured_at", models.DateTimeField(default=timezone.now)),
                ("quality", models.FloatField(default=0.0)),
                ("metadata", models.JSONField(blank=True, default=dict)),
                (
                    "camera",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="face_snapshots",
                        to="spaces.camera",
                    ),
                ),
                (
                    "embedding",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="snapshots",
                        to="spaces.faceembedding",
                    ),
                ),
                (
                    "identity",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="snapshots",
                        to="spaces.identity",
                    ),
                ),
            ],
            options={"ordering": ["-captured_at"]},
        ),
    ]
