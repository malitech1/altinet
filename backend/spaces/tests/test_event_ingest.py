from __future__ import annotations

from unittest import mock

import pytest


@pytest.mark.django_db
def test_track_events_are_broadcast(api_client):
    payload = {
        "type": "TRACK",
        "room_id": "living_room",
        "track_id": 1,
        "centroid": [0.4, 0.6],
        "timestamp": "2023-11-10T12:00:00Z",
    }
    with mock.patch("spaces.event_views.broadcast_person_track") as broadcast:
        response = api_client.post("/api/events/", payload, format="json")
    assert response.status_code == 202
    broadcast.assert_called_once()
    message = broadcast.call_args.args[0]
    assert message["room_id"] == "living_room"
    assert message["centroid"] == [0.4, 0.6]


@pytest.mark.django_db
def test_presence_events_are_broadcast(api_client):
    payload = {
        "type": "PRESENCE",
        "room_id": "kitchen",
        "track_ids": [1, 2, 3],
        "timestamp": "2023-11-10T12:05:00Z",
    }
    with mock.patch("spaces.event_views.broadcast_room_presence") as broadcast:
        response = api_client.post("/api/events/", payload, format="json")
    assert response.status_code == 202
    broadcast.assert_called_once()
    message = broadcast.call_args.args[0]
    assert message["track_ids"] == ["1", "2", "3"]

