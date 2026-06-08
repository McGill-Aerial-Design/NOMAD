# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the API-key authentication middleware in edge_core.api.

Covers the security-sensitive paths: exempt routes, the loopback-only dev
fallback, the NOMAD_ALLOW_INSECURE_REMOTE opt-in, X-API-Key enforcement, and the
internal-bridge token (including its minimum-length disable path).
"""

from __future__ import annotations

from edge_core.api import create_app
from edge_core.core import wire_modules
from edge_core.services.state import StateManager


def _make_app(monkeypatch, env: dict[str, str], *, wire_vio: bool = True):
    for key in (
        "NOMAD_API_KEY",
        "NOMAD_ALLOW_INSECURE_REMOTE",
        "NOMAD_INTERNAL_TOKEN",
    ):
        monkeypatch.delenv(key, raising=False)
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    app = create_app(StateManager.instance())
    if wire_vio:
        wire_modules(app, allow_list=["vio"])
    return app


def _client(app, *, client_host: str = "testclient"):
    """TestClient whose simulated peer address can be overridden."""
    from fastapi.testclient import TestClient

    return TestClient(app, client=(client_host, 50000))


# --------------------------------------------------------------------------- #
# exempt paths
# --------------------------------------------------------------------------- #
def test_exempt_paths_open_without_key(monkeypatch):
    app = _make_app(monkeypatch, {"NOMAD_API_KEY": "k" * 40})
    with _client(app) as c:
        assert c.get("/health").status_code == 200
        assert c.get("/openapi.json").status_code == 200


# --------------------------------------------------------------------------- #
# no-key loopback dev fallback
# --------------------------------------------------------------------------- #
def test_no_key_blocks_remote(monkeypatch):
    app = _make_app(monkeypatch, {})
    with _client(app, client_host="10.0.0.5") as c:
        assert c.get("/api/vio/status").status_code == 401


def test_no_key_allows_loopback(monkeypatch):
    app = _make_app(monkeypatch, {})
    with _client(app, client_host="127.0.0.1") as c:
        assert c.get("/api/vio/status").status_code == 200


def test_insecure_remote_opt_in_allows_remote(monkeypatch):
    app = _make_app(monkeypatch, {"NOMAD_ALLOW_INSECURE_REMOTE": "true"})
    with _client(app, client_host="10.0.0.5") as c:
        assert c.get("/api/vio/status").status_code == 200


# --------------------------------------------------------------------------- #
# X-API-Key enforcement
# --------------------------------------------------------------------------- #
def test_api_key_required_when_configured(monkeypatch):
    app = _make_app(monkeypatch, {"NOMAD_API_KEY": "s" * 40})
    with _client(app, client_host="127.0.0.1") as c:
        # Loopback does NOT bypass once a key is configured.
        assert c.get("/api/vio/status").status_code == 401
        assert c.get("/api/vio/status", headers={"X-API-Key": "wrong"}).status_code == 401
        assert c.get("/api/vio/status", headers={"X-API-Key": "s" * 40}).status_code == 200


# --------------------------------------------------------------------------- #
# internal bridge token
# --------------------------------------------------------------------------- #
def test_internal_token_allows_loopback_bridge_route(monkeypatch):
    token = "t" * 40
    app = _make_app(monkeypatch, {"NOMAD_API_KEY": "s" * 40, "NOMAD_INTERNAL_TOKEN": token})
    with _client(app, client_host="127.0.0.1") as c:
        # Bridge route reachable with the internal token, no API key.
        resp = c.post("/api/vio/update", json={"x": 0}, headers={"X-NOMAD-Internal-Token": token})
        assert resp.status_code == 200
        # ... but not from a remote client even with the token.
    with _client(app, client_host="10.0.0.5") as c:
        resp = c.post("/api/vio/update", json={"x": 0}, headers={"X-NOMAD-Internal-Token": token})
        assert resp.status_code == 401


def test_short_internal_token_is_disabled(monkeypatch):
    short = "t" * 10  # below the 32-char minimum -> bypass disabled
    app = _make_app(monkeypatch, {"NOMAD_API_KEY": "s" * 40, "NOMAD_INTERNAL_TOKEN": short})
    with _client(app, client_host="127.0.0.1") as c:
        resp = c.post("/api/vio/update", json={"x": 0}, headers={"X-NOMAD-Internal-Token": short})
        assert resp.status_code == 401
