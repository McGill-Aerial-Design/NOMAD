# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""VideoStreamModule wiring: routes respond gracefully without Docker present."""

from __future__ import annotations

import pytest

fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from edge_core.api_routes.video_slam import VideoSlamModule  # noqa: E402
from edge_core.core import AppContext  # noqa: E402
from edge_core.services.video_module import VideoStreamModule  # noqa: E402


@pytest.fixture()
def app():
    app = fastapi.FastAPI()
    ctx = AppContext(app=app, services={"state_manager": object()}, config={})
    video = VideoStreamModule()
    video.configure(ctx)
    routes = VideoSlamModule()
    routes.configure(ctx)
    routes.register_routes(app)
    return app


def test_module_registers_manager_service(app):
    assert app.state.video_stream_manager is not None


def test_bridges_status_degrades_without_docker(app):
    client = TestClient(app)
    resp = client.get("/api/video/bridges")
    assert resp.status_code == 200
    body = resp.json()
    assert body["success"] is True
    # Single stream exposed under the "stream" key (no primary/secondary).
    assert set(body["bridges"]) == {"stream"}
    assert body["bridges"]["stream"]["streaming"] is False


def test_bridges_start_fails_cleanly_without_container(app):
    client = TestClient(app)
    resp = client.post("/api/video/bridges/start")
    assert resp.status_code == 503
    assert "detail" in resp.json()


def test_source_switch_503_when_bridge_down(app):
    client = TestClient(app)
    resp = client.post("/api/video/source", params={"topic": "/zed/zed_node/rgb/color/rect/image"})
    assert resp.status_code == 503


def test_overlay_503_when_bridge_down(app):
    client = TestClient(app)
    resp = client.post("/api/video/overlay/enable")
    assert resp.status_code == 503


def test_watchdog_lifecycle_starts_and_stops():
    module = VideoStreamModule()
    ctx = AppContext(app=fastapi.FastAPI(), config={})
    module.configure(ctx)
    module.start()
    assert module._manager._watchdog_thread.is_alive()
    module.stop()
    assert not module._manager._watchdog_thread.is_alive()
