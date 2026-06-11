# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""RTSP Video Bridge and nvblox SLAM Mesh streaming API routes."""

from __future__ import annotations

import logging
import time
from typing import Any

from fastapi import APIRouter, HTTPException, Request

from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.video_slam")

try:
    import msgpack

    MSGPACK_AVAILABLE = True
except ImportError:
    msgpack = None
    MSGPACK_AVAILABLE = False


class VideoSlamModule(BaseModule):
    """Pluggable router for video source and SLAM mesh updates."""

    metadata = ModuleMetadata(
        name="video_slam",
        version="1.0.0",
        description="Manages RTSP topic switching, overlays, and nvblox mesh sync",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        # Global mesh cache populated by the ROS bridge
        self.mesh_cache: dict[str, Any] = {
            "voxels": [],
            "voxel_size": 0.05,
            "mode": "voxel",
            "timestamp": 0.0,
            "clear": False,
        }

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["Video & SLAM"])

        def _manager():
            return getattr(app.state, "video_stream_manager", None)

        @router.get("/api/video/bridges")
        async def get_video_bridges_status():
            """Get status of running RTSP streams."""
            mgr = _manager()
            if not mgr:
                raise HTTPException(status_code=503, detail="VideoStreamManager not initialized")
            return {"success": True, "bridges": {"primary": mgr.get_status().to_dict()}}

        @router.post("/api/video/bridges/start")
        async def start_video_bridges():
            """Launch the persistent video bridge inside the container."""
            mgr = _manager()
            if not mgr:
                raise HTTPException(status_code=503, detail="VideoStreamManager not initialized")
            ok, msg = mgr.start_with_reason()
            if not ok:
                raise HTTPException(status_code=503, detail=msg)
            return {"success": True, "message": "Video bridges started successfully"}

        @router.post("/api/video/source")
        async def switch_video_source(topic: str):
            """Switch the active camera ROS2 topic subscription."""
            mgr = _manager()
            if not mgr or not mgr.is_relay_running():
                raise HTTPException(status_code=503, detail="Video bridge not running")
            if mgr.switch_topic(topic):
                return {"success": True, "message": f"Switched to {topic}"}
            raise HTTPException(status_code=502, detail="Failed to switch topic")

        @router.post("/api/video/overlay/{action}")
        async def set_video_overlay(action: str):
            """Enable or disable the ZED detection overlay on the RTSP feed."""
            mgr = _manager()
            if not mgr or not mgr.is_relay_running():
                raise HTTPException(status_code=503, detail="Video bridge not running")
            enabled = action.lower() == "enable"
            if mgr.set_overlay(enabled):
                return {"success": True, "message": f"Overlay {action}d"}
            raise HTTPException(status_code=502, detail="Failed to toggle overlay")

        @router.post("/api/slam/mesh/update")
        async def update_slam_mesh(request: Request):
            """Incoming mesh update from the ROS-HTTP Bridge (msgpack or JSON)."""
            ctype = request.headers.get("Content-Type") or ""
            try:
                body = await request.body()
                if "msgpack" in ctype and MSGPACK_AVAILABLE:
                    data = msgpack.unpackb(body, raw=False)
                else:
                    import json

                    data = json.loads(body.decode("utf-8"))

                # Cache the mesh update
                self.mesh_cache.update(data)
                self.mesh_cache["received_at"] = time.time()
                return {"success": True}
            except Exception as e:
                raise HTTPException(status_code=400, detail=f"Failed to parse mesh: {e}")

        @router.get("/api/slam/mesh")
        async def get_slam_mesh(format: str = "json"):
            """Retrieve the cached SLAM mesh for the 3D viewer."""
            if not self.mesh_cache["timestamp"]:
                return {"available": False, "message": "No mesh data available"}

            if format == "summary":
                return {
                    "available": True,
                    "mode": self.mesh_cache.get("mode"),
                    "block_count": len(self.mesh_cache.get("voxels", [])),
                    "total_blocks": self.mesh_cache.get("total_voxels", 0),
                    "timestamp": self.mesh_cache.get("timestamp"),
                }

            return {
                "available": True,
                "timestamp": self.mesh_cache["timestamp"],
                "mesh": self.mesh_cache,
                "drone_position": self.mesh_cache.get("drone_position"),
                "drone_attitude": self.mesh_cache.get("drone_attitude"),
            }

        app.include_router(router)
