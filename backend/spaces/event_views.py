"""Views for ingesting real-time events from ROS bridge."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Sequence

from rest_framework import status
from rest_framework.permissions import AllowAny
from rest_framework.request import Request
from rest_framework.response import Response
from rest_framework.views import APIView

from .events import broadcast_person_track, broadcast_room_presence


class EventIngestView(APIView):
    """Accept events from the ROS→Django bridge and fan them out."""

    permission_classes = [AllowAny]
    authentication_classes: Sequence[type] = []

    def post(self, request: Request) -> Response:
        event_type = (request.data.get("type") or "").upper()
        if event_type == "TRACK":
            return self._handle_track(request.data)
        if event_type == "PRESENCE":
            return self._handle_presence(request.data)
        # Other event types are accepted but ignored for now to keep the
        # pipeline extensible.
        return Response({"status": "ignored"}, status=status.HTTP_202_ACCEPTED)

    def _handle_track(self, data: Dict[str, Any]) -> Response:
        room_id = data.get("room_id")
        track_id = data.get("track_id")
        centroid = data.get("centroid")
        if room_id is None or track_id is None or not isinstance(centroid, Iterable):
            return Response(
                {"detail": "room_id, track_id and centroid are required."},
                status=status.HTTP_400_BAD_REQUEST,
            )
        try:
            coords = [float(value) for value in centroid][:2]
        except (TypeError, ValueError):
            return Response(
                {"detail": "centroid must contain numeric values."},
                status=status.HTTP_400_BAD_REQUEST,
            )
        payload = {
            "event": "TRACK",
            "room_id": str(room_id),
            "track_id": str(track_id),
            "centroid": coords,
            "timestamp": data.get("timestamp"),
        }
        broadcast_person_track(payload)
        return Response({"status": "ok"}, status=status.HTTP_202_ACCEPTED)

    def _handle_presence(self, data: Dict[str, Any]) -> Response:
        room_id = data.get("room_id")
        track_ids = data.get("track_ids")
        if room_id is None or not isinstance(track_ids, list):
            return Response(
                {"detail": "room_id and track_ids are required."},
                status=status.HTTP_400_BAD_REQUEST,
            )
        payload = {
            "event": "PRESENCE",
            "room_id": str(room_id),
            "track_ids": [str(track) for track in track_ids],
            "timestamp": data.get("timestamp"),
        }
        broadcast_room_presence(payload)
        return Response({"status": "ok"}, status=status.HTTP_202_ACCEPTED)


__all__ = ["EventIngestView"]

