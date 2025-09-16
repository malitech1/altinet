from __future__ import annotations

import pytest


@pytest.mark.django_db
def test_openapi_schema_available(api_client, room):
    response = api_client.get("/api/schema/")
    assert response.status_code == 200
    assert "openapi" in response.data
