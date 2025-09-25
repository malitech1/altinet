"""Channels consumers for live camera updates."""

from __future__ import annotations

from channels.generic.websocket import AsyncJsonWebsocketConsumer


class CameraHealthConsumer(AsyncJsonWebsocketConsumer):
    group_name = "camera_health"

    async def connect(self) -> None:
        await self.channel_layer.group_add(self.group_name, self.channel_name)
        await self.accept()

    async def disconnect(
        self, close_code: int
    ) -> None:  # pragma: no cover - connection teardown
        await self.channel_layer.group_discard(self.group_name, self.channel_name)

    async def camera_health_event(self, event):
        await self.send_json(event["payload"])


class CalibrationProgressConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        camera_id = self.scope["url_route"]["kwargs"]["camera_id"]
        self.group_name = f"calibration_{camera_id}"
        await self.channel_layer.group_add(self.group_name, self.channel_name)
        await self.accept()

    async def disconnect(self, close_code: int) -> None:  # pragma: no cover
        await self.channel_layer.group_discard(self.group_name, self.channel_name)

    async def calibration_progress_event(self, event):
        await self.send_json(event["payload"])


class PersonTrackConsumer(AsyncJsonWebsocketConsumer):
    group_names = ["person_tracks"]

    async def connect(self) -> None:
        for group in self.group_names:
            await self.channel_layer.group_add(group, self.channel_name)
        await self.accept()

    async def disconnect(self, close_code: int) -> None:  # pragma: no cover
        for group in self.group_names:
            await self.channel_layer.group_discard(group, self.channel_name)

    async def person_track_event(self, event):
        await self.send_json(event["payload"])

    async def room_presence_event(self, event):
        await self.send_json(event["payload"])
