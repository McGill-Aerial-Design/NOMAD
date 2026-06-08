# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Boot smoke test for the real Edge Core app.

Instantiates the production app via ``create_app`` (hardware-free), confirms the
exempt endpoints respond, and verifies that a real module's routes mount through
``wire_modules``. This is the single highest-value "does it boot" test.
"""

from __future__ import annotations

import pytest

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager


@pytest.fixture
def app(monkeypatch):
    # Loopback-dev fallback: no API key, allow the TestClient (non-loopback host)
    # so module routes are reachable over HTTP.
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    application = create_app(StateManager.instance())
    # Enable only the dependency-free VIO module so configuration cannot touch
    # hardware (mavlink/payload/etc. stay disabled).
    wire_modules(application, allow_list=["vio"])
    return application


def test_health_endpoint_responds(app):
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        resp = client.get("/health")
        assert resp.status_code == 200
        body = resp.json()
        assert "status" in body
        assert "timestamp" in body


def test_docs_and_openapi_served(app):
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        assert client.get("/docs").status_code == 200
        schema = client.get("/openapi.json")
        assert schema.status_code == 200
        assert schema.json()["info"]["title"]


def test_module_route_mounts(app):
    # The wired VIO module contributes its routes to the live app.
    paths = app.openapi()["paths"]
    assert "/api/vio/status" in paths
    assert "/api/vio/update" in paths


def test_wired_module_route_serves(app):
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        resp = client.get("/api/vio/status")
        assert resp.status_code == 200
        assert resp.json()["health"] == "not_initialized"
