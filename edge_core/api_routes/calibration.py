# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ZED calibration API routes (device-side camera tools only).

Vehicle actuation routes were removed during the C++ cutover (deletion-gate
step 3, 2026-09-05): servo/relay/payload commands belong to the C++ core
boundary (``src/vehicle/output.cpp``, the ``servo``/``relay``/``payload-demo``
CLI verbs, and ``NomadCoreClient`` in the plugin), not to a Python REST layer.
Payloads themselves are modular client profiles over generic ArduPilot outputs
— the core knows servo channels and relays, never a specific payload.
"""

from __future__ import annotations

import asyncio
import logging
import subprocess
from typing import Any

from fastapi import APIRouter, HTTPException

from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.calibration")


class CalibrationModule(BaseModule):
    """Pluggable router for ZED camera sensor diagnostics."""

    metadata = ModuleMetadata(
        name="calibration",
        version="1.0.0",
        description="Handles ZED IMU/Sensor diagnostics",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["Calibration"])

        @router.post("/api/calibration/imu/reset_biases")
        async def reset_imu_biases():
            """Reset the ZED camera's internal IMU EEPROM bias parameters."""
            try:
                # Requires exclusive camera access; check that noVNC/Isaac isn't using it
                await asyncio.to_thread(
                    subprocess.run, ["/usr/local/zed/tools/ZED_Diagnostic", "--cimu"], check=True, timeout=15
                )
                return {"success": True, "message": "IMU biases cleared and recalibrated"}
            except Exception as e:
                raise HTTPException(status_code=502, detail=f"IMU calibration failed: {e}")

        @router.post("/api/calibration/zed/sensor-viewer/start")
        async def start_sensor_viewer():
            """Launch the official ZED Sensor Viewer on the remote display."""
            try:
                await asyncio.to_thread(
                    subprocess.Popen, ["/usr/local/zed/tools/ZED_Sensor_Viewer"], start_new_session=True
                )
                return {"success": True, "message": "ZED Sensor Viewer launched"}
            except Exception as e:
                raise HTTPException(status_code=502, detail=f"Failed to launch tool: {e}")

        app.include_router(router)
