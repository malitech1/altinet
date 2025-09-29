from __future__ import annotations

from django import forms
from django.contrib.auth import get_user_model

from .models import SystemSettings


class UserSettingsForm(forms.ModelForm):
    """Allow authenticated users to update their core profile information."""

    class Meta:
        model = get_user_model()
        fields = ["username", "first_name", "last_name", "email"]
        widgets = {
            "username": forms.TextInput(attrs={"class": "form-control"}),
            "first_name": forms.TextInput(attrs={"class": "form-control"}),
            "last_name": forms.TextInput(attrs={"class": "form-control"}),
            "email": forms.EmailInput(attrs={"class": "form-control"}),
        }


class SystemSettingsForm(forms.ModelForm):
    """Expose a curated subset of system configuration options."""

    class Meta:
        model = SystemSettings
        fields = [
            "site_name",
            "support_email",
            "maintenance_mode",
            "default_theme",
            "home_address",
        ]
        widgets = {
            "site_name": forms.TextInput(attrs={"class": "form-control"}),
            "support_email": forms.EmailInput(attrs={"class": "form-control"}),
            "maintenance_mode": forms.CheckboxInput(attrs={"class": "form-check-input"}),
            "default_theme": forms.Select(attrs={"class": "form-select"}),
            "home_address": forms.TextInput(
                attrs={"class": "form-control", "placeholder": "221B Baker Street, London"}
            ),
        }
