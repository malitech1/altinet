from __future__ import annotations

import base64
import base64
import binascii
import io
import json
from typing import Dict, Optional

from django.conf import settings
from django.contrib import messages
from django.contrib.auth.decorators import login_required
from django.http import JsonResponse
from django.shortcuts import redirect, render
from django.urls import reverse
from django.utils import timezone
from django.views.decorators.http import require_POST

import numpy as np

try:  # pragma: no cover - optional dependency for runtime recognition
    from PIL import Image
except ImportError:  # pragma: no cover - handled gracefully at runtime
    Image = None  # type: ignore

from .forms import SystemSettingsForm, UserSettingsForm
from .models import SystemSettings, TrainingImage, TrainingProfile
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


def _clean_image_data_uri(data_uri: str) -> str | None:
    """Validate that a provided data URI contains base64 encoded image data."""

    if not isinstance(data_uri, str):
        return None

    candidate = data_uri.strip()
    if not candidate.startswith("data:image/"):
        return None

    header, _, base64_data = candidate.partition(",")
    if not base64_data:
        return None

    if ";base64" not in header.lower():
        return None

    try:
        base64.b64decode(base64_data, validate=True)
    except (binascii.Error, ValueError):
        return None

    return f"{header},{base64_data}"


_FACE_ANALYZER: Optional[object] = None
_TRAINING_EMBEDDING_CACHE: Dict[int, np.ndarray] = {}


def _reset_training_embedding_cache() -> None:
    """Clear cached embeddings so new training data is picked up."""

    _TRAINING_EMBEDDING_CACHE.clear()


def _load_image_from_data_uri(data_uri: str) -> Optional[np.ndarray]:
    """Decode a base64 image data URI into an RGB numpy array."""

    if Image is None:
        return None

    cleaned = _clean_image_data_uri(data_uri)
    if not cleaned:
        return None

    _, _, base64_data = cleaned.partition(",")
    try:
        raw_bytes = base64.b64decode(base64_data, validate=True)
    except (binascii.Error, ValueError):
        return None

    try:
        with Image.open(io.BytesIO(raw_bytes)) as image:  # type: ignore[union-attr]
            rgb_image = image.convert("RGB")
            return np.array(rgb_image)
    except Exception:
        return None


def _get_face_analyzer() -> Optional[object]:
    """Return a cached instance of the InsightFace analysis helper."""

    global _FACE_ANALYZER
    if _FACE_ANALYZER is not None:
        return _FACE_ANALYZER

    try:
        from insightface.app import FaceAnalysis  # type: ignore
    except Exception:  # pragma: no cover - optional dependency unavailable
        return None

    try:
        analyzer = FaceAnalysis(name="buffalo_l")
        analyzer.prepare(ctx_id=-1, det_size=(640, 640))
    except Exception:
        return None

    _FACE_ANALYZER = analyzer
    return _FACE_ANALYZER


def _extract_embedding(analyzer: object, image: np.ndarray) -> Optional[np.ndarray]:
    """Generate a normalised embedding for ``image`` using ``analyzer``."""

    if analyzer is None:
        return None

    try:
        faces = analyzer.get(image)  # type: ignore[attr-defined]
    except Exception:
        return None

    if not faces:
        return None

    face = faces[0]
    embedding = getattr(face, "normed_embedding", None)
    if embedding is None:
        return None

    array = np.asarray(embedding, dtype=np.float32)
    if array.size == 0:
        return None

    norm = np.linalg.norm(array)
    if norm == 0:
        return None
    return array / norm


def _embedding_for_training_image(analyzer: object, image: TrainingImage) -> Optional[np.ndarray]:
    """Return a cached embedding vector for ``image``."""

    cached = _TRAINING_EMBEDDING_CACHE.get(image.pk)
    if cached is not None:
        return cached

    array = _load_image_from_data_uri(image.image_data)
    if array is None:
        return None

    embedding = _extract_embedding(analyzer, array)
    if embedding is None:
        return None

    _TRAINING_EMBEDDING_CACHE[image.pk] = embedding
    return embedding


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
        cleaned = _clean_image_data_uri(data_uri)
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
    _reset_training_embedding_cache()

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

    analyzer = _get_face_analyzer()
    if analyzer is None or Image is None:
        return JsonResponse(
            {
                "success": False,
                "error": "Face recognition components are unavailable on this server.",
            },
            status=503,
        )

    try:
        payload = json.loads(request.body.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return JsonResponse({"success": False, "error": "Invalid JSON payload."}, status=400)

    raw_image = payload.get("image")
    cleaned_image = _clean_image_data_uri(raw_image)
    if not cleaned_image:
        return JsonResponse(
            {"success": False, "error": "Provide a valid captured image."}, status=400
        )

    probe_image = _load_image_from_data_uri(cleaned_image)
    if probe_image is None:
        return JsonResponse(
            {
                "success": False,
                "error": "Unable to decode the captured frame for analysis.",
            },
            status=400,
        )

    probe_embedding = _extract_embedding(analyzer, probe_image)
    if probe_embedding is None:
        return JsonResponse(
            {
                "success": False,
                "error": "No face was detected in the captured frame.",
            },
            status=422,
        )

    profiles = TrainingProfile.objects.prefetch_related("images").all()
    if not profiles:
        return JsonResponse(
            {
                "success": False,
                "error": "No trained profiles are available yet.",
            },
            status=404,
        )

    best_profile: Optional[TrainingProfile] = None
    best_score: float = -1.0

    for profile in profiles:
        for training_image in profile.images.all():
            embedding = _embedding_for_training_image(analyzer, training_image)
            if embedding is None:
                continue
            score = float(np.dot(probe_embedding, embedding))
            if score > best_score:
                best_score = score
                best_profile = profile

    threshold = 0.35
    confidence = float(min(max(best_score, 0.0), 1.0))

    if best_profile is None or best_score < threshold:
        return JsonResponse(
            {
                "success": True,
                "match": None,
                "confidence": round(confidence, 3),
                "message": "No trained faces matched the capture.",
            }
        )

    return JsonResponse(
        {
            "success": True,
            "confidence": round(confidence, 3),
            "match": {
                "id": best_profile.id,
                "full_name": best_profile.full_name,
                "image_count": best_profile.image_count,
                "trained_at": best_profile.trained_at.isoformat()
                if best_profile.trained_at
                else None,
            },
            "message": f"Match found: {best_profile.full_name}.",
        }
    )


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
