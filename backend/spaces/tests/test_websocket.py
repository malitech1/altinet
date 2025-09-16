from __future__ import annotations

import pytest
from altinet_backend.asgi import application
from channels.testing import WebsocketCommunicator
from spaces.events import broadcast_camera_health


@pytest.mark.asyncio
async def test_camera_health_websocket_sends_event():
    communicator = WebsocketCommunicator(application, "/ws/cameras/")
    connected, _ = await communicator.connect()
    assert connected
    broadcast_camera_health(
        {"camera_id": "123", "last_health": "OK", "last_seen_at": "now"}
    )
    message = await communicator.receive_json_from()
    assert message["camera_id"] == "123"
    await communicator.disconnect()
