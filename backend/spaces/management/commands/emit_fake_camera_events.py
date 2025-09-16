from __future__ import annotations

import random
import time
from typing import Any

from django.core.management.base import BaseCommand
from django.utils import timezone

from ...events import broadcast_calibration_progress, broadcast_camera_health
from ...models import Camera, CameraCalibrationRun


class Command(BaseCommand):
    help = "Emit fake camera health and calibration progress events for development"

    def add_arguments(self, parser) -> None:  # pragma: no cover - CLI plumbing
        parser.add_argument("--iterations", type=int, default=5)
        parser.add_argument("--sleep", type=float, default=1.0)

    def handle(self, *args: Any, **options: Any) -> None:
        iterations = options["iterations"]
        sleep = options["sleep"]
        for _ in range(iterations):
            for camera in Camera.objects.all():
                broadcast_camera_health(
                    {
                        "camera_id": str(camera.id),
                        "last_health": random.choice(
                            [state for state, _ in Camera.HealthState.choices]
                        ),
                        "last_seen_at": timezone.now().isoformat(),
                    }
                )
                latest_run = camera.calibration_runs.filter(
                    status=CameraCalibrationRun.Status.RUNNING
                ).first()
                if latest_run:
                    broadcast_calibration_progress(
                        str(camera.id),
                        {
                            "run_id": str(latest_run.id),
                            "pct": random.random() * 100,
                            "msg": random.choice(
                                ["Collecting frames", "Solving", "Validating"]
                            ),
                        },
                    )
            time.sleep(sleep)
        self.stdout.write(self.style.SUCCESS("Fake events emitted"))
