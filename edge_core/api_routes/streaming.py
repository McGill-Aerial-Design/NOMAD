# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import os

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None


def register_streaming_routes(app, ctx) -> None:
    # ==================== Streaming Endpoints ====================

    @app.get("/api/stream/info", tags=["Streaming"])
    async def stream_info():
        """Get RTSP stream information (single Jetson stream)."""
        rtsp_base = os.environ.get("MEDIA_SERVER_URL", "rtsp://localhost:8554")

        return {
            "stream": f"{rtsp_base}/stream",
            "format": "H.264/RTSP",
            "recommended_player": "VLC or FFplay",
        }
