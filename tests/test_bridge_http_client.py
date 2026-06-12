# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the ROS-HTTP bridge's Edge Core HTTP client."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from edge_core.ros_http_bridge.http_client import EdgeCoreHttpClient


class _RecordingHandler(BaseHTTPRequestHandler):
    """Records the last request and answers with a canned JSON body."""

    last_request: dict = {}

    def _record(self) -> None:
        length = int(self.headers.get("Content-Length") or 0)
        type(self).last_request = {
            "method": self.command,
            "path": self.path,
            "headers": dict(self.headers),
            "body": self.rfile.read(length) if length else b"",
        }

    def do_POST(self):  # noqa: N802 - BaseHTTPRequestHandler API
        self._record()
        self.send_response(200)
        self.end_headers()

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        self._record()
        body = json.dumps({"ok": True}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):  # silence test output
        pass


@pytest.fixture()
def http_server():
    server = HTTPServer(("127.0.0.1", 0), _RecordingHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    thread.join(timeout=2)


def _make_client(server: HTTPServer) -> EdgeCoreHttpClient:
    return EdgeCoreHttpClient("127.0.0.1", server.server_address[1])


def test_post_returns_true_on_200(http_server, monkeypatch):
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.delenv("NOMAD_INTERNAL_TOKEN", raising=False)
    client = _make_client(http_server)

    assert client.post("/api/vio/update", b'{"x": 1}') is True
    req = _RecordingHandler.last_request
    assert req["method"] == "POST"
    assert req["path"] == "/api/vio/update"
    assert req["body"] == b'{"x": 1}'
    assert req["headers"]["Content-Type"] == "application/json"
    client.close()


def test_post_sends_credential_headers(http_server, monkeypatch):
    monkeypatch.setenv("NOMAD_API_KEY", "test-key")
    monkeypatch.setenv("NOMAD_INTERNAL_TOKEN", "internal-token")
    client = _make_client(http_server)

    assert client.post("/api/vio/update", b"{}") is True
    headers = _RecordingHandler.last_request["headers"]
    assert headers["X-API-Key"] == "test-key"
    assert headers["X-NOMAD-Internal-Token"] == "internal-token"
    client.close()


def test_get_json_parses_body(http_server, monkeypatch):
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.delenv("NOMAD_INTERNAL_TOKEN", raising=False)
    client = _make_client(http_server)

    assert client.get_json("/api/servo/camera/tilt") == {"ok": True}
    client.close()


def test_post_returns_false_when_server_unreachable(monkeypatch):
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.delenv("NOMAD_INTERNAL_TOKEN", raising=False)
    # Port 9 (discard) on localhost: nothing listening.
    client = EdgeCoreHttpClient("127.0.0.1", 9)

    assert client.post("/api/vio/update", b"{}") is False
    assert client.get_json("/anything") is None
    client.close()
