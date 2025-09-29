from __future__ import annotations

from django.conf import settings
from django.contrib import messages
from django.contrib.auth.decorators import login_required
from django.shortcuts import redirect, render

from .forms import SystemSettingsForm, UserSettingsForm
from .models import SystemSettings


@login_required
def home(request):
    """Render the main dashboard once the user is authenticated."""
    return render(
        request,
        "web/home.html",
        {
            "home_model_path": getattr(settings, "HOME_MODEL_STATIC_PATH", "web/models/home.obj"),
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
