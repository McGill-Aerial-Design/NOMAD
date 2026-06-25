# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""VideoStreamModule — wires VideoStreamManager into the module lifecycle.

The manager controls the simple video bridge inside the Isaac ROS container;
this module owns its construction (from ctx config) and the crash-recovery
watchdog thread. Routes live in ``api_routes/video_slam.py`` and reach the
manager through ``app.state.video_stream_manager``.
"""

from __future__ import annotations

import logging

from edge_core.core import AppContext, BaseModule, ModuleMetadata
from edge_core.services.video_stream_manager import DEFAULT_RTSP_URL, VideoStreamManager

logger = logging.getLogger("edge_core.services.video")


class VideoStreamModule(BaseModule):
    """Owns the VideoStreamManager instance and its watchdog lifecycle."""

    metadata = ModuleMetadata(
        name="video",
        version="1.0.0",
        description="RTSP video bridge control (Isaac ROS container)",
        enable_flag="NOMAD_ENABLE_VIDEO",
        enabled_by_default=True,
    )

    def __init__(self) -> None:
        self._manager: VideoStreamManager | None = None

    def configure(self, ctx: AppContext) -> None:
        def _int(key: str, default: int) -> int:
            try:
                return int(ctx.get_config(key) or default)
            except ValueError:
                return default

        def _bool(key: str, default: bool = False) -> bool:
            value = ctx.get_config(key)
            if value is None:
                return default
            return value.strip().lower() in {"1", "true", "yes", "on"}

        self._manager = VideoStreamManager(
            container_name=ctx.get_config("ISAAC_CONTAINER_NAME") or "nomad_isaac_ros",
            relay_http_host=ctx.get_config("VIDEO_RELAY_HTTP_HOST") or "localhost",
            relay_http_port=_int("VIDEO_RELAY_HTTP_PORT", 9200),
            rtsp_url=ctx.get_config("NOMAD_RTSP_URL") or DEFAULT_RTSP_URL,
            default_topic=ctx.get_config("NOMAD_DEFAULT_VIDEO_TOPIC") or "/zed/zed_node/rgb/color/rect/image",
            width=_int("VIDEO_BRIDGE_WIDTH", 640),
            height=_int("VIDEO_BRIDGE_HEIGHT", 360),
            fps=_int("VIDEO_BRIDGE_FPS", 15),
            bitrate=_int("VIDEO_BRIDGE_BITRATE", 800),
            external_bridge=_bool("NOMAD_VIDEO_EXTERNAL_BRIDGE"),
        )
        ctx.register_service("video_stream_manager", self._manager)
        ctx.app.state.video_stream_manager = self._manager
        logger.info("Video stream module configured")

    def start(self) -> None:
        if self._manager:
            self._manager.start_watchdog()

    def stop(self) -> None:
        if self._manager:
            self._manager.stop_watchdog()
