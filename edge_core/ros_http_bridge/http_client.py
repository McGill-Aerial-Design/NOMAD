# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Keep-alive HTTP client for the bridge's Edge Core API calls.

Wraps a single persistent ``HTTPConnection`` behind a lock so the ROS
callbacks and timers can share it. Failures are throttled in the log and
trigger a reconnect so a restarted Edge Core picks back up automatically.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from http.client import HTTPConnection

logger = logging.getLogger("ros_http_bridge.http_client")

_INTERNAL_TOKEN_HEADER = "X-NOMAD-Internal-Token"


class EdgeCoreHttpClient:
    """Thread-safe keep-alive client for Edge Core's loopback API."""

    def __init__(self, host: str, port: int, default_timeout_s: float = 0.5):
        self._host = host
        self._port = port
        self._default_timeout_s = default_timeout_s
        self._api_key = (os.environ.get("NOMAD_API_KEY") or "").strip() or None
        self._internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip() or None
        self._conn = HTTPConnection(host, port, timeout=default_timeout_s)
        self._lock = threading.Lock()
        self._last_error_log: dict[str, float] = {}

    def post(self, path: str, data: bytes, timeout: float = 0.5, content_type: str = "application/json") -> bool:
        with self._lock:
            try:
                headers = self._build_headers(content_type, keep_alive=True)
                self._conn.timeout = timeout
                self._conn.request("POST", path, body=data, headers=headers)
                resp = self._conn.getresponse()
                resp.read()
                return resp.status == 200
            except Exception as e:
                now = time.monotonic()
                if now - self._last_error_log.get(path, 0.0) >= 2.0:
                    logger.warning(f"HTTP POST {path} failed: {e}")
                    self._last_error_log[path] = now
                self._reconnect()
                return False

    def get_json(self, path: str, timeout: float = 0.2) -> dict | None:
        with self._lock:
            try:
                headers = self._build_headers(keep_alive=True)
                self._conn.timeout = timeout
                self._conn.request("GET", path, headers=headers)
                resp = self._conn.getresponse()
                body = resp.read()
                if resp.status == 200:
                    return json.loads(body.decode("utf-8"))
            except Exception:
                self._reconnect()
            return None

    def close(self) -> None:
        try:
            self._conn.close()
        except Exception:
            pass

    def _reconnect(self) -> None:
        self.close()
        try:
            self._conn = HTTPConnection(self._host, self._port, timeout=self._default_timeout_s)
        except Exception:
            pass

    def _build_headers(self, content_type: str | None = None, keep_alive: bool = False) -> dict[str, str]:
        headers = {}
        if content_type:
            headers["Content-Type"] = content_type
        if keep_alive:
            headers["Connection"] = "keep-alive"
        if self._internal_token:
            headers[_INTERNAL_TOKEN_HEADER] = self._internal_token
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        return headers
