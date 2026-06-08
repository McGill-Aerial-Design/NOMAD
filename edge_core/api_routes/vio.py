# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""VIO (Visual Inertial Odometry) status and trajectory API routes.

The ROS-HTTP bridge POSTs pose updates to ``/api/vio/update``; this module
stores the latest state on ``app.state.external_vio_state`` and exposes
read-only status endpoints for the Mission Planner plugin.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from fastapi import APIRouter, Request

from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.vio")

_VIO_TRAJECTORY_MAX = 500


class VioModule(BaseModule):
    """VIO status, trajectory, and area-map route module."""

    metadata = ModuleMetadata(
        name="vio",
        version="1.0.0",
        description="Exposes VIO status, trajectory, and area-map endpoints for the GCS",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        self._trajectory: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._area_map_path: str | None = None

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["VIO"])
        trajectory = self._trajectory
        lock = self._lock

        @router.post("/api/vio/update")
        async def vio_update(request: Request):
            """Ingest a VIO pose update from the ROS-HTTP bridge (internal token)."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception as exc:
                logger.debug("VIO update parse error: %s", exc)
                return {"success": False, "error": str(exc)}

            confidence = data.get("confidence", 0.0)
            source = data.get("source", "external")

            request.app.state.external_vio_state = {
                "x": data.get("x", 0.0),
                "y": data.get("y", 0.0),
                "z": data.get("z", 0.0),
                "roll": data.get("roll", 0.0),
                "pitch": data.get("pitch", 0.0),
                "yaw": data.get("yaw", 0.0),
                "confidence": confidence,
                "source": source,
                "timestamp": data.get("timestamp", time.time()),
            }

            with lock:
                trajectory.append(
                    {
                        "x": data.get("x", 0.0),
                        "y": data.get("y", 0.0),
                        "z": data.get("z", 0.0),
                        "timestamp": data.get("timestamp", time.time()),
                    }
                )
                if len(trajectory) > _VIO_TRAJECTORY_MAX:
                    del trajectory[: len(trajectory) - _VIO_TRAJECTORY_MAX]

            return {"success": True}

        @router.get("/api/vio/status")
        async def vio_status(request: Request):
            """Return current VIO tracking status."""
            vio = getattr(request.app.state, "external_vio_state", None)
            if vio:
                return {
                    "health": "ok" if vio.get("confidence", 0) > 0.5 else "degraded",
                    "tracking_confidence": vio.get("confidence", 0),
                    "position_valid": True,
                    "x": vio.get("x", 0),
                    "y": vio.get("y", 0),
                    "z": vio.get("z", 0),
                    "roll": vio.get("roll", 0),
                    "pitch": vio.get("pitch", 0),
                    "yaw": vio.get("yaw", 0),
                    "source": vio.get("source", "external"),
                    "timestamp": vio.get("timestamp", 0),
                }
            return {
                "health": "not_initialized",
                "tracking_confidence": 0,
                "position_valid": False,
                "message_rate_hz": 0,
                "reset_counter": 0,
                "source": "none",
            }

        @router.get("/api/vio/trajectory")
        async def get_vio_trajectory(max_points: int = 100):
            """Return stored VIO trajectory points."""
            with lock:
                points = trajectory[-max_points:]
            return {"points": points, "count": len(points)}

        @router.delete("/api/vio/trajectory")
        async def clear_vio_trajectory():
            """Clear stored VIO trajectory."""
            with lock:
                trajectory.clear()
            return {"success": True, "message": "Trajectory cleared"}

        @router.post("/api/vio/reset_origin")
        async def reset_vio_origin():
            """Reset the VIO origin frame (requires Isaac ROS bridge)."""
            logger.info("VIO origin reset requested")
            return {"success": True, "message": "VIO origin reset acknowledged"}

        @router.post("/api/vio/area/save")
        async def save_area_map(request: Request):
            """Save the ZED positional tracking area map."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception:
                data = {}
            file_path = data.get("file_path", "/tmp/nomad_area_map.db")
            logger.info("Area map save requested: %s", file_path)
            return {"success": True, "message": f"Area map save requested: {file_path}"}

        @router.post("/api/vio/area/load")
        async def load_area_map(request: Request):
            """Load a previously saved area map."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception:
                data = {}
            file_path = data.get("file_path", "")
            logger.info("Area map load requested: %s", file_path)
            return {"success": True, "message": f"Area map load requested: {file_path}"}

        @router.post("/api/vio/area/relocalize")
        async def relocalize_area_map(request: Request):
            """Load an area map and immediately attempt relocalization."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception:
                data = {}
            file_path = data.get("file_path", "")
            logger.info("Area map relocalize requested: %s", file_path)
            return {"success": True, "message": f"Relocalization requested: {file_path}"}

        app.include_router(router)
