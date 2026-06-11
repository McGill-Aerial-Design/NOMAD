# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""NetworkModule wiring: /network/status returns real (or cleanly-degraded)
data instead of the permanent nulls of the unwired baseline."""

from __future__ import annotations

import pytest

fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from edge_core.core import AppContext  # noqa: E402
from edge_core.services.network_module import NetworkModule  # noqa: E402
from infra.tailscale.src.tailscale_manager import TailscaleStatus, parse_status_json  # noqa: E402


@pytest.fixture()
def app(monkeypatch):
    monkeypatch.setenv("NOMAD_ALLOW_INSECURE_REMOTE", "true")
    from edge_core.api import create_app
    from edge_core.services.state import StateManager

    app = create_app(StateManager.instance())
    module = NetworkModule()
    module.configure(AppContext(app=app, config={"GCS_IP": "100.64.0.2"}))
    return app


def test_module_registers_services(app):
    assert app.state.tailscale_manager is not None
    assert app.state.network_monitor is not None


def test_network_status_returns_wired_data(app):
    client = TestClient(app)
    resp = client.get("/network/status")
    assert resp.status_code == 200
    body = resp.json()
    # Wired but not yet polled: a real status string, not the permanent nulls.
    assert body["tailscale"] is not None
    assert body["tailscale"]["status"] == "disconnected"
    assert body["internet_reachable"] is False
    assert body["gcs_reachable"] is False
    assert body["modem"] is None


def test_parse_status_json_extracts_consumed_fields():
    info = parse_status_json(
        {
            "BackendState": "Running",
            "Self": {"TailscaleIPs": ["100.64.0.1", "fd7a::1"], "HostName": "nomad-jetson"},
            "Peer": {"key1": {}, "key2": {}},
        }
    )
    assert info.status is TailscaleStatus.CONNECTED
    assert info.ip_address == "100.64.0.1"
    assert info.hostname == "nomad-jetson"
    assert info.peer_count == 2


def test_monitor_threads_start_and_stop(monkeypatch):
    # Keep the first poll from shelling out during the test.
    import infra.tailscale.src.network_monitor as nm
    import infra.tailscale.src.tailscale_manager as tm

    monkeypatch.setattr(nm, "_run", lambda cmd, timeout=10.0: (1, ""))
    monkeypatch.setattr(tm, "_run", lambda cmd, timeout=10.0: (127, ""))

    module = NetworkModule()
    module.configure(AppContext(app=fastapi.FastAPI(), config={}))
    module.start()
    assert module._tailscale._thread.is_alive()
    assert module._monitor._thread.is_alive()
    module.stop()
    assert not module._tailscale._thread.is_alive()
    assert not module._monitor._thread.is_alive()
