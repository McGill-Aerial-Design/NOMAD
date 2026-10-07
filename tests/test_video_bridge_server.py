# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the video bridge HTTP API routes and responses."""

from __future__ import annotations

import io
import json
from types import SimpleNamespace

import pytest
from video_bridge_fakes import make_bridge

from python.tools import video_bridge_server as server

BridgeHTTPHandler = server.BridgeHTTPHandler
BridgeHTTPServer = server.BridgeHTTPServer


def make_handler(bridge) -> server.BridgeHTTPHandler:
    """Return a handler whose JSON responses are captured instead of written."""
    handler = BridgeHTTPHandler.__new__(BridgeHTTPHandler)
    handler.server = SimpleNamespace(bridge=bridge)
    handler.captured: list[tuple[int, dict]] = []
    handler._json_response = lambda data, status=200: handler.captured.append((status, data))
    return handler


def bare_handler() -> server.BridgeHTTPHandler:
    """Return a handler with no bridge and no stub, for methods that need neither."""
    return BridgeHTTPHandler.__new__(BridgeHTTPHandler)


@pytest.mark.parametrize(
    ("path", "key"),
    [
        ("/health", "pipeline_playing"),
        ("/status", "streaming"),
        ("/overlay/status", "enabled"),
    ],
)
def test_do_get_routes_to_bridge(path, key):
    handler = make_handler(make_bridge())
    handler.path = path

    handler.do_GET()

    status, data = handler.captured[0]
    assert status == 200
    assert key in data


def test_do_get_topics_uses_topic_listing():
    handler = make_handler(make_bridge())
    handler._list_topics = lambda: {"topics": ["/zed/img"]}
    handler.path = "/topics"

    handler.do_GET()

    assert handler.captured[0][1] == {"topics": ["/zed/img"]}


def test_do_get_unknown_path_returns_404():
    handler = make_handler(make_bridge())
    handler.path = "/nope"

    handler.do_GET()

    assert handler.captured[0][0] == 404


def test_do_post_switch_reports_topic():
    handler = make_handler(SimpleNamespace(switch_topic=lambda topic: True))
    handler.path = "/switch?topic=/zed/depth"

    handler.do_POST()

    status, data = handler.captured[0]
    assert status == 200
    assert data["success"] is True
    assert data["topic"] == "/zed/depth"


def test_do_post_switch_without_topic_returns_400():
    handler = make_handler(SimpleNamespace(switch_topic=lambda topic: True))
    handler.path = "/switch"

    handler.do_POST()

    status, data = handler.captured[0]
    assert status == 400
    assert data["success"] is False


def test_do_post_restart_reports_result():
    handler = make_handler(SimpleNamespace(restart=lambda: True))
    handler.path = "/restart"

    handler.do_POST()

    assert handler.captured[0][1] == {"success": True}


def test_do_post_overlay_enable_and_disable():
    bridge = make_bridge()
    handler = make_handler(bridge)

    handler.path = "/overlay/enable"
    handler.do_POST()
    assert bridge._overlay_enabled is True

    handler.path = "/overlay/disable"
    handler.do_POST()
    assert bridge._overlay_enabled is False


def test_do_post_unknown_path_returns_404():
    handler = make_handler(make_bridge())
    handler.path = "/nope"

    handler.do_POST()

    assert handler.captured[0][0] == 404


def test_json_response_writes_status_and_body():
    handler = bare_handler()
    sent = {"headers": []}
    handler.send_response = lambda status: sent.__setitem__("status", status)
    handler.send_header = lambda key, value: sent["headers"].append((key, value))
    handler.end_headers = lambda: sent.__setitem__("ended", True)
    handler.wfile = io.BytesIO()

    handler._json_response({"ok": True}, status=201)

    assert sent["status"] == 201
    assert sent["ended"] is True
    assert ("Content-Type", "application/json") in sent["headers"]
    assert json.loads(handler.wfile.getvalue()) == {"ok": True}


def test_list_topics_filters_image_topics(monkeypatch):
    stdout = "/zed/img sensor_msgs/msg/Image\n/chatter std_msgs/msg/String\n"
    monkeypatch.setattr(server.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout=stdout))

    assert bare_handler()._list_topics() == {"topics": ["/zed/img"]}


def test_list_topics_returns_empty_on_failure(monkeypatch):
    def fail(*_args, **_kwargs):
        raise OSError("ros2 missing")

    monkeypatch.setattr(server.subprocess, "run", fail)

    assert bare_handler()._list_topics() == {"topics": []}


def test_log_message_is_quiet():
    bare_handler().log_message("%s", "GET /health")


def test_http_server_binds_and_exposes_bridge():
    bridge = make_bridge()
    http_server = BridgeHTTPServer("127.0.0.1", 0, bridge)
    try:
        assert http_server.bridge is bridge
    finally:
        http_server.server_close()
