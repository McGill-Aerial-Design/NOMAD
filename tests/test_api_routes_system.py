# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Route coverage for ``edge_core.api_routes.system``.

Exercises the System/Network HTTP surface (root, health, detailed health,
status, network status, ping, and the ``/ws/state`` WebSocket) against a live
``create_app`` instance via ``TestClient``. Subprocess calls (ping) are mocked
so no real network egress happens; ``app.state`` is populated with fakes to
drive both the present- and absent-collaborator branches.
"""

from __future__ import annotations

import subprocess
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from edge_core.api import create_app
from edge_core.services.state import StateManager


def _build_app(monkeypatch, *, api_key: str | None = None, insecure: bool = True):
    if api_key is None:
        monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    else:
        monkeypatch.setenv("NOMAD_API_KEY", api_key)
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true" if insecure else "false")
    return create_app(StateManager.instance())


class _FakeHealth:
    def __init__(self, **overrides):
        data = dict(
            cpu_temp_c=42.0,
            cpu_load_pct=11.0,
            gpu_temp_c=38.0,
            gpu_load_pct=6.0,
            memory_used_pct=33.0,
            disk_free_gb=120.0,
            power_draw_w=7.5,
            throttled=False,
            thermal_zone="normal",
            tailscale_connected=True,
            tailscale_ip="100.64.0.1",
        )
        data.update(overrides)
        self.__dict__.update(data)

    def to_dict(self) -> dict:
        return dict(self.__dict__)


def _monitor(**overrides):
    return SimpleNamespace(health=_FakeHealth(**overrides))


# ==================== root / status ====================


def test_root_returns_service_info(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        body = client.get("/").json()
    assert body["service"] == "NOMAD Edge Core"
    assert body["endpoints"]["health"] == "/health"


def test_status_returns_encoded_state(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        resp = client.get("/status")
    assert resp.status_code == 200
    assert isinstance(resp.json(), dict)


# ==================== /health ====================


def test_health_without_monitor_is_degraded(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = None
    with TestClient(app) as client:
        body = client.get("/health").json()
    assert body["status"] == "degraded"  # StateManager starts disconnected
    assert "cpu_temp" not in body  # no monitor -> no Jetson metrics


def test_health_with_monitor_includes_metrics(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = _monitor()
    with TestClient(app) as client:
        body = client.get("/health").json()
    assert body["cpu_temp"] == 42.0
    assert body["tailscale_ip"] == "100.64.0.1"


def test_health_critical_thermal_zone(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = _monitor(thermal_zone="critical")
    with TestClient(app) as client:
        body = client.get("/health").json()
    assert body["status"] == "critical"


def test_health_warning_when_throttled(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = _monitor(thermal_zone="normal", throttled=True)
    with TestClient(app) as client:
        body = client.get("/health").json()
    assert body["status"] == "warning"


def test_detailed_health_without_monitor(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = None
    with TestClient(app) as client:
        body = client.get("/health/detailed").json()
    assert body == {"error": "Health monitor not initialized"}


def test_detailed_health_with_monitor(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = _monitor()
    with TestClient(app) as client:
        body = client.get("/health/detailed").json()
    assert body["thermal_zone"] == "normal"


def test_detailed_health_includes_external_vio(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.health_monitor = _monitor()
    app.state.external_vio_state = {
        "source": "gazebo",
        "confidence": 0.9,
        "timestamp": 100.0,
        "x": 1.0,
        "y": 2.0,
        "z": -0.2,
    }
    app.state.vio_trajectory = [
        {"timestamp": 100.0},
        {"timestamp": 100.5},
        {"timestamp": 101.0},
    ]
    monkeypatch.setattr("edge_core.api_routes.system.time.time", lambda: 101.0)
    with TestClient(app) as client:
        body = client.get("/health/detailed").json()
    assert body["vio"]["health"] == "healthy"
    assert body["vio"]["source"] == "gazebo"
    assert body["vio"]["message_rate_hz"] == 2.0


# ==================== /network/status ====================


def test_network_status_no_managers(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        body = client.get("/network/status").json()
    assert body["tailscale"] is None
    assert body["modem"] is None
    assert body["internet_reachable"] is False
    assert body["gcs_reachable"] is False


def test_network_status_with_managers_and_modem(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.tailscale_manager = SimpleNamespace(
        info=SimpleNamespace(
            status=SimpleNamespace(value="connected"),
            ip_address="100.64.0.2",
            hostname="nomad-edge",
            peer_count=3,
            latency_ms=14.2,
        )
    )
    app.state.network_monitor = SimpleNamespace(
        status=SimpleNamespace(
            internet_reachable=True,
            tailscale_reachable=True,
            modem=SimpleNamespace(to_dict=lambda: {"interface": "wwan0", "ip_address": "10.0.0.5"}),
        )
    )
    with TestClient(app) as client:
        body = client.get("/network/status").json()
    assert body["tailscale"]["status"] == "connected"
    assert body["tailscale"]["ip"] == "100.64.0.2"
    assert body["internet_reachable"] is True
    assert body["gcs_reachable"] is True
    assert body["modem"]["interface"] == "wwan0"


# ==================== /network/ping/{host} ====================


def test_ping_rejects_invalid_host(monkeypatch):
    app = _build_app(monkeypatch)
    with TestClient(app) as client:
        resp = client.get("/network/ping/-flag")
    assert resp.status_code == 400


def test_ping_success_parses_counts_and_latency(monkeypatch):
    app = _build_app(monkeypatch)
    out = (
        "PING host (1.2.3.4): 56 data bytes\n"
        "3 packets transmitted, 3 received, 0% packet loss\n"
        "rtt min/avg/max/mdev = 1.000/2.500/4.000/0.500 ms\n"
    )
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, out, ""))
    with TestClient(app) as client:
        body = client.get("/network/ping/example.com").json()
    assert body["packets_sent"] == 3
    assert body["packets_received"] == 3
    assert body["latency_ms"] == 2.5


def test_ping_all_lost_returns_502(monkeypatch):
    app = _build_app(monkeypatch)
    out = "3 packets transmitted, 0 received, 100% packet loss\n"
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 1, out, ""))
    with TestClient(app) as client:
        resp = client.get("/network/ping/example.com")
    assert resp.status_code == 502


def test_ping_timeout_returns_504(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd="ping", timeout=5)

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.get("/network/ping/example.com")
    assert resp.status_code == 504


def test_ping_missing_utility_returns_503(monkeypatch):
    app = _build_app(monkeypatch)

    def _boom(*a, **k):
        raise FileNotFoundError("ping")

    monkeypatch.setattr(subprocess, "run", _boom)
    with TestClient(app) as client:
        resp = client.get("/network/ping/example.com")
    assert resp.status_code == 503


# ==================== /ws/state ====================


def test_ws_state_streams_state_insecure(monkeypatch):
    app = _build_app(monkeypatch)  # no key + insecure -> token check passes
    app.state.health_monitor = _monitor()
    app.state.obstacle_distance_last = {
        "timestamp": 0.0,
        "increment_deg": 5,
        "min_distance_cm": 100,
        "max_distance_cm": 800,
        "nearest_sector": 2,
        "nearest_distance_cm": 120,
        "nearest_bearing_deg": 30,
        "distances": [1, 2, 3],
    }
    with TestClient(app) as client:
        with client.websocket_connect("/ws/state") as ws:
            data = ws.receive_json()
    assert "jetson_health" in data
    assert data["vio_status"]["health"] == "unknown"
    assert data["obstacle_distance"]["nearest_sector"] == 2


def test_ws_state_includes_external_vio(monkeypatch):
    app = _build_app(monkeypatch)
    app.state.external_vio_state = {"source": "zed", "confidence": 88}
    with TestClient(app) as client:
        with client.websocket_connect("/ws/state") as ws:
            data = ws.receive_json()
    assert data["external_vio_state"]["source"] == "zed"


def test_ws_state_valid_token_accepted(monkeypatch):
    app = _build_app(monkeypatch, api_key="secret-key", insecure=False)
    with TestClient(app) as client:
        with client.websocket_connect("/ws/state?token=secret-key") as ws:
            data = ws.receive_json()
    assert "vio_status" in data


def test_ws_state_missing_token_rejected(monkeypatch):
    app = _build_app(monkeypatch, api_key="secret-key", insecure=False)
    with TestClient(app) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/ws/state") as ws:
                ws.receive_json()
