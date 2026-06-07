# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ZED Calibration and hardware diagnostic API routes."""

from __future__ import annotations

import asyncio
import logging
import subprocess
from typing import Any

from fastapi import APIRouter, HTTPException

from edge_core.core import AppContext, BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.api.calibration")


class CalibrationModule(BaseModule):
    """Pluggable router for camera sensor and actuator calibrations."""

    metadata = ModuleMetadata(
        name="calibration",
        version="1.0.0",
        description="Handles direct servo PWM testing and ZED IMU/Sensor diagnostics",
    )

    def configure(self, ctx: AppContext) -> None:
        self.state_mgr = ctx.require_service("state_manager")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["Actuators & Calibration"])

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

        @router.post("/api/servo/channel/{channel}/pwm")
        async def set_direct_pwm(channel: int, pwm: int):
            """Send a direct raw PWM microsecond output override to a Cube channel."""
            if channel < 1 or channel > 16:
                raise HTTPException(status_code=400, detail="Channel must be between 1 and 16")
            if pwm < 500 or pwm > 2500:
                raise HTTPException(status_code=400, detail="PWM must be between 500 and 2500 us")

            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")

            if servo_ctrl.set_channel_pwm(channel, pwm):
                return {"success": True}
            raise HTTPException(status_code=502, detail="MAVLink PWM command rejected")

        app.include_router(router)
