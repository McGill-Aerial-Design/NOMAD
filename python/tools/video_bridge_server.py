# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""HTTP API server and request handler for NOMAD Simple Video Bridge."""

from __future__ import annotations

import http.server
import json
import logging
import socket
import subprocess
from typing import cast
from urllib.parse import parse_qs, urlparse

logger = logging.getLogger("nomad.video_bridge.server")


class BridgeHTTPHandler(http.server.BaseHTTPRequestHandler):
    """HTTP API for the video bridge."""

    def _bridge(self):
        return cast("BridgeHTTPServer", self.server).bridge

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/health":
            self._json_response(self._bridge().get_health())
        elif path == "/status":
            self._json_response(self._bridge().get_status())
        elif path == "/topics":
            self._json_response(self._list_topics())
        elif path == "/overlay/status":
            self._json_response(self._bridge().get_overlay_status())
        else:
            self._json_response({"error": "not found"}, status=404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        if path == "/switch":
            self._handle_switch(params)
        elif path == "/restart":
            ok = self._bridge().restart()
            self._json_response({"success": ok})
        elif path == "/overlay/enable":
            self._bridge().set_overlay(True)
            self._json_response({"success": True, "message": "Overlay enabled"})
        elif path == "/overlay/disable":
            self._bridge().set_overlay(False)
            self._json_response({"success": True, "message": "Overlay disabled"})
        else:
            self._json_response({"error": "not found"}, status=404)

    def _handle_switch(self, params: dict):
        topic = params.get("topic", [None])[0]
        if not topic:
            self._json_response({"success": False, "message": "missing topic param"}, 400)
            return
        ok = self._bridge().switch_topic(topic)
        self._json_response(
            {
                "success": ok,
                "message": f"Switched to {topic}" if ok else "Failed to switch",
                "topic": topic,
            }
        )

    def _json_response(self, data: dict, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _list_topics(self) -> dict:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list", "-t"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2 and "sensor_msgs/msg/Image" in parts[1]:
                    topics.append(parts[0])
            return {"topics": topics}
        except Exception:
            return {"topics": []}

    def log_message(self, format, *args):
        logger.debug(f"HTTP {args[0]}")


class BridgeHTTPServer(http.server.HTTPServer):
    def __init__(self, host: str, port: int, bridge):
        self.bridge = bridge
        if ":" in host:
            self.address_family = socket.AF_INET6
        super().__init__((host, port), BridgeHTTPHandler)
