from __future__ import annotations

import json

from django.conf import settings
from django.contrib import messages
from django.contrib.auth.decorators import login_required
from django.http import JsonResponse
from django.shortcuts import redirect, render
from django.urls import reverse
from django.utils import timezone
from django.views.decorators.http import require_POST

from .forms import SystemSettingsForm, UserSettingsForm
from .models import SystemSettings, TrainingImage, TrainingProfile
from .services.face_training import (
    TrainingTestResult,
    clean_image_data_uri,
    evaluate_face_test,
    reset_training_embedding_cache,
)
from .weather import fetch_weather_snapshot


def _wind_direction_to_cardinal(degrees: float | None) -> str | None:
    """Convert a wind direction in degrees to a cardinal label."""

    if degrees is None:
        return None

    cardinal_points = [
        "N",
        "NE",
        "E",
        "SE",
        "S",
        "SW",
        "W",
        "NW",
    ]
    index = int((degrees % 360) / 45 + 0.5) % len(cardinal_points)
    return cardinal_points[index]


@login_required
def home(request):
    """Render the main dashboard once the user is authenticated."""
    system_settings = SystemSettings.load()
    base_now = timezone.now()
    local_now = timezone.localtime(base_now)

    environment_snapshot = {
        "people_present": 0,
        "people_list": [],
        "local_time": local_now,
        "outside_temperature_c": None,
        "outside_humidity": None,
        "average_indoor_temperature_c": None,
        "weather_summary": None,
        "wind_speed_kmh": None,
        "wind_direction_deg": None,
        "wind_direction_cardinal": None,
        "air_quality_index": None,
        "energy_usage_kw": None,
    }

    dashboard_metrics = [
        {
            "label": "People detected",
            "value": str(environment_snapshot["people_present"]),
            "detail": "Across all monitored rooms",
        },
        {
            "label": "Local time",
            "value": local_now.strftime("%I:%M %p").lstrip("0"),
            "detail": local_now.strftime("%A, %B %d").replace(" 0", " "),
        },
        {
            "label": "Outside temperature",
            "value": (
                f"{environment_snapshot['outside_temperature_c']:.1f}°C"
                if environment_snapshot["outside_temperature_c"] is not None
                else "—"
            ),
            "detail": (
                "Latest reading from weather service"
                if environment_snapshot["outside_temperature_c"] is not None
                else "Awaiting sensor data"
            ),
        },
        {
            "label": "Humidity",
            "value": (
                f"{environment_snapshot['outside_humidity']}%"
                if environment_snapshot["outside_humidity"] is not None
                else "—"
            ),
            "detail": (
                "Outdoor relative humidity"
                if environment_snapshot["outside_humidity"] is not None
                else "Awaiting sensor data"
            ),
        },
        {
            "label": "Weather",
            "value": environment_snapshot["weather_summary"] or "—",
            "detail": (
                "Live conditions from weather integration"
                if environment_snapshot["weather_summary"]
                else "Awaiting sensor data"
            ),
        },
        {
            "label": "Wind speed",
            "value": (
                f"{environment_snapshot['wind_speed_kmh']:.1f} km/h"
                if environment_snapshot["wind_speed_kmh"] is not None
                else "—"
            ),
            "detail": (
                "Measured at nearby weather station"
                if environment_snapshot["wind_speed_kmh"] is not None
                else "Awaiting sensor data"
            ),
        },
        {
            "label": "Air quality",
            "value": (
                f"AQI {environment_snapshot['air_quality_index']}"
                if environment_snapshot["air_quality_index"] is not None
                else "—"
            ),
            "detail": (
                "EPA standard air quality index"
                if environment_snapshot["air_quality_index"] is not None
                else "Awaiting sensor data"
            ),
        },
        {
            "label": "Energy usage",
            "value": (
                f"{environment_snapshot['energy_usage_kw']:.2f} kW"
                if environment_snapshot["energy_usage_kw"] is not None
                else "—"
            ),
            "detail": (
                "Whole-home consumption"
                if environment_snapshot["energy_usage_kw"] is not None
                else "Awaiting sensor data"
            ),
        },
    ]

    return render(
        request,
        "web/home.html",
        {
            "home_model_path": getattr(settings, "HOME_MODEL_STATIC_PATH", "web/models/home.obj"),
            "environment_snapshot": environment_snapshot,
            "dashboard_metrics": dashboard_metrics,
            "weather_endpoint": reverse("web:weather-snapshot"),
        },
    )


@login_required
def builder(request):
    """Display the interactive home builder canvas."""
    return render(request, "web/builder.html")


@login_required
def weather_snapshot(request):
    """Return the latest scraped weather information for Macleay Island."""

    system_settings = SystemSettings.load()
    snapshot = fetch_weather_snapshot(system_settings.home_address)
    if snapshot:
        snapshot["wind_direction_cardinal"] = _wind_direction_to_cardinal(
            snapshot.get("wind_direction_deg")
        )
        return JsonResponse(
            {
                "success": True,
                "data": snapshot,
                "retrieved_at": timezone.now().isoformat(),
            }
        )

    return JsonResponse({"success": False}, status=503)


@login_required
def training(request):
    """Render the training workspace for capturing and enrolling faces."""

    profiles = TrainingProfile.objects.prefetch_related("images").all()

    return render(
        request,
        "web/training.html",
        {
            "profiles": profiles,
            "training_endpoint": reverse("web:training-create"),
            "testing_endpoint": reverse("web:training-test"),
        },
    )


@login_required
@require_POST
def training_create(request):
    """Create a new training profile from captured image data."""

    try:
        payload = json.loads(request.body.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return JsonResponse(
            {"success": False, "error": "Invalid JSON payload."}, status=400
        )

    full_name = (payload.get("full_name") or "").strip()
    notes = (payload.get("notes") or "").strip()
    raw_images = payload.get("images")

    if not full_name:
        return JsonResponse(
            {"success": False, "error": "A name is required to train a profile."},
            status=400,
        )

    if not isinstance(raw_images, list):
        return JsonResponse(
            {
                "success": False,
                "error": "Images must be supplied as a list of data URIs.",
            },
            status=400,
        )

    cleaned_images: list[str] = []
    for data_uri in raw_images:
        cleaned = clean_image_data_uri(data_uri)
        if cleaned:
            cleaned_images.append(cleaned)

    if not cleaned_images:
        return JsonResponse(
            {
                "success": False,
                "error": "Select at least one valid captured image before training.",
            },
            status=400,
        )

    profile = TrainingProfile.objects.create(full_name=full_name, notes=notes)

    TrainingImage.objects.bulk_create(
        [
            TrainingImage(
                profile=profile,
                image_data=image,
                display_order=index,
            )
            for index, image in enumerate(cleaned_images)
        ]
    )

    profile.mark_trained()
    reset_training_embedding_cache()

    return JsonResponse(
        {
            "success": True,
            "profile": {
                "id": profile.id,
                "full_name": profile.full_name,
                "trained_at": profile.trained_at.isoformat() if profile.trained_at else None,
                "image_count": profile.image_count,
            },
            "message": f"Training complete for {profile.full_name}.",
        },
        status=201,
    )


@login_required
@require_POST
def training_test(request):
    """Evaluate a captured frame against trained profiles."""

    try:
        payload = json.loads(request.body.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return JsonResponse({"success": False, "error": "Invalid JSON payload."}, status=400)

    raw_image = payload.get("image")
    result: TrainingTestResult = evaluate_face_test(raw_image)
    return JsonResponse(result.payload, status=result.status)


@login_required
def settings_view(request):
    """Allow operators to manage personal and core system settings."""

    user_form = UserSettingsForm(
        request.POST or None,
        instance=request.user,
        prefix="user",
    )
    system_settings = SystemSettings.load()
    system_form = SystemSettingsForm(
        request.POST or None,
        instance=system_settings,
        prefix="system",
    )

    if request.method == "POST":
        if "save_user" in request.POST:
            if user_form.is_valid():
                user_form.save()
                messages.success(request, "Your profile information has been updated.")
                return redirect("web:settings")
            messages.error(request, "Please correct the errors in your profile details.")
        elif "save_system" in request.POST:
            if system_form.is_valid():
                system_form.save()
                messages.success(request, "System settings saved successfully.")
                return redirect("web:settings")
            messages.error(request, "Please correct the highlighted system settings.")

    return render(
        request,
        "web/settings.html",
        {
            "user_form": user_form,
            "system_form": system_form,
        },
    )
