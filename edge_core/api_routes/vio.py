# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""VIO (Visual Inertial Odometry) status and trajectory API routes.

The ROS-HTTP bridge POSTs pose updates to ``/api/vio/update``; this module
stores the latest state on ``app.state.external_vio_state`` and exposes
read-only status endpoints for the Mission Planner plugin.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from fastapi import APIRouter, Request
from pydantic import BaseModel

from edge_core.core import AppContext, BaseModule, ModuleMetadata

_VIO_TRAJECTORY_MAX = 500


class VioUpdateRequest(BaseModel):
    """Pose update from the ROS-HTTP bridge."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    confidence: float = 0.0
    source: str = "external"
    timestamp: float | None = None


class VioModule(BaseModule):
    """VIO status and trajectory route module."""

    metadata = ModuleMetadata(
        name="vio",
        version="1.0.0",
        description="Exposes VIO status and trajectory endpoints for the GCS",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")
        self._trajectory: list[dict[str, Any]] = []
        self._lock = threading.Lock()

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["VIO"])
        trajectory = self._trajectory
        lock = self._lock

        @router.post("/api/vio/update")
        async def vio_update(request: Request, update: VioUpdateRequest):
            """Ingest a VIO pose update from the ROS-HTTP bridge (internal token)."""
            timestamp = update.timestamp if update.timestamp is not None else time.time()

            request.app.state.external_vio_state = {
                "x": update.x,
                "y": update.y,
                "z": update.z,
                "roll": update.roll,
                "pitch": update.pitch,
                "yaw": update.yaw,
                "confidence": update.confidence,
                "source": update.source,
                "timestamp": timestamp,
            }

            with lock:
                trajectory.append(
                    {
                        "x": update.x,
                        "y": update.y,
                        "z": update.z,
                        "timestamp": timestamp,
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

        app.include_router(router)
