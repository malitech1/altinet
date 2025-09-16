from __future__ import annotations

from django.contrib import admin

from .models import Camera, CameraCalibrationRun, Room


@admin.register(Room)
class RoomAdmin(admin.ModelAdmin):
    list_display = ("name", "level", "ceiling_height_mm")
    search_fields = ("name",)


@admin.register(Camera)
class CameraAdmin(admin.ModelAdmin):
    list_display = ("name", "room", "last_health", "calibration_state")
    search_fields = ("name", "make", "model")
    list_filter = ("last_health", "calibration_state")


@admin.register(CameraCalibrationRun)
class CameraCalibrationRunAdmin(admin.ModelAdmin):
    list_display = ("camera", "status", "method", "created_at")
    list_filter = ("status", "method")
