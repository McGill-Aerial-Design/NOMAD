# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Behavior tests for the VIO route module (edge_core.api_routes.vio)."""

from __future__ import annotations

import pytest

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager


@pytest.fixture
def client(monkeypatch):
    from fastapi.testclient import TestClient

    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    app = create_app(StateManager.instance())
    wire_modules(app, allow_list=["vio"])
    with TestClient(app) as c:
        yield c


def test_status_before_any_update(client):
    body = client.get("/api/vio/status").json()
    assert body["health"] == "not_initialized"
    assert body["position_valid"] is False


def test_update_then_status_reflects_pose(client):
    payload = {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.5, "confidence": 0.9, "source": "test"}
    assert client.post("/api/vio/update", json=payload).json()["success"] is True

    body = client.get("/api/vio/status").json()
    assert body["health"] == "ok"  # confidence > 0.5
    assert body["position_valid"] is True
    assert body["x"] == 1.0 and body["y"] == 2.0 and body["z"] == 3.0
    assert body["source"] == "test"


def test_low_confidence_is_degraded(client):
    client.post("/api/vio/update", json={"x": 0, "y": 0, "z": 0, "confidence": 0.1})
    assert client.get("/api/vio/status").json()["health"] == "degraded"


def test_trajectory_accumulates_and_clears(client):
    for i in range(3):
        client.post("/api/vio/update", json={"x": float(i), "y": 0.0, "z": 0.0, "confidence": 0.8})

    traj = client.get("/api/vio/trajectory").json()
    assert traj["count"] == 3
    assert traj["points"][-1]["x"] == 2.0

    assert client.delete("/api/vio/trajectory").json()["success"] is True
    assert client.get("/api/vio/trajectory").json()["count"] == 0


def test_malformed_update_is_handled(client):
    resp = client.post("/api/vio/update", content=b"not json")
    assert resp.status_code == 422
    assert "detail" in resp.json()
