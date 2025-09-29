from __future__ import annotations

from django.conf import settings
from django.contrib import messages
from django.contrib.auth.decorators import login_required
from django.shortcuts import redirect, render
from django.utils import timezone

from .forms import SystemSettingsForm, UserSettingsForm
from .models import SystemSettings
from .weather import fetch_weather_snapshot


@login_required
def home(request):
    """Render the main dashboard once the user is authenticated."""
    local_now = timezone.localtime()

    environment_snapshot = {
        "people_present": 0,
        "local_time": local_now,
        "outside_temperature_c": None,
        "outside_humidity": None,
        "weather_summary": None,
        "wind_speed_kmh": None,
        "air_quality_index": None,
        "energy_usage_kw": None,
    }

    system_settings = SystemSettings.load()
    weather_snapshot = fetch_weather_snapshot(system_settings.home_address)
    environment_snapshot.update(weather_snapshot)

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
        },
    )


@login_required
def builder(request):
    """Display the interactive home builder canvas."""
    return render(request, "web/builder.html")


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
