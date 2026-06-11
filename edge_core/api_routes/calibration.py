# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ZED Calibration, servo config, and spray calibration API routes."""

from __future__ import annotations

import asyncio
import logging
import subprocess
from typing import Any

from fastapi import APIRouter, HTTPException, Query, Request
from pydantic import BaseModel, ConfigDict, Field

from edge_core.core import AppContext, BaseModule, ModuleMetadata
from edge_core.safety import validate_servo_command

logger = logging.getLogger("edge_core.api.calibration")


class SprayCalibrationRequest(BaseModel):
    """The consumed subset of the GCS spray settings; extras are rejected."""

    model_config = ConfigDict(extra="forbid")

    water_pump_relay_number: int = Field(ge=0, le=15)


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
            # Rejection text comes from the SC core only (SR-PAY-01).
            decision = validate_servo_command(channel, pwm)
            if not decision.allowed:
                raise HTTPException(status_code=400, detail=decision.message)

            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")

            if servo_ctrl.set_channel_pwm(channel, pwm):
                return {"success": True}
            raise HTTPException(status_code=502, detail="MAVLink PWM command rejected")

        app.include_router(router)

        servo_router = APIRouter(tags=["Servo & Spray"])

        @servo_router.post("/api/servo/camera/tilt")
        async def set_camera_tilt(angle: float = Query(90.0, ge=0.0, le=180.0)):
            """Set camera tilt angle (0-180 degrees) via MAVLink servo."""
            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")
            if servo_ctrl.set_camera_tilt(angle):
                return {"success": True, "angle": angle}
            raise HTTPException(status_code=502, detail="MAVLink camera tilt command rejected")

        @servo_router.get("/api/servo/camera/tilt")
        async def get_camera_tilt():
            """Current commanded camera tilt angle in degrees."""
            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")
            angle = servo_ctrl.get_camera_tilt()
            if angle is None:
                raise HTTPException(status_code=503, detail="Camera tilt channel not configured")
            return {"angle": angle}

        @servo_router.post("/api/servo/camera/config")
        async def set_camera_config(request: Request):
            """Push servo channel configuration from Mission Planner."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception as exc:
                raise HTTPException(status_code=400, detail=f"Invalid JSON: {exc}")

            channel = int(data.get("channel", 14))
            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")

            servo_ctrl.configure_camera_tilt_mavlink(channel)
            logger.info("Camera servo config updated: channel=%d", channel)
            return {"success": True, "channel": channel}

        @servo_router.post("/api/spray/calibration")
        async def set_spray_calibration(req: SprayCalibrationRequest):
            """Configure the water pump relay number from Mission Planner.

            This is the only spray field the server consumes; everything else
            (gains, aim point, speeds) is GCS-local. Unknown fields are
            rejected so client and server cannot drift silently.
            """
            servo_ctrl = app.state.servo_controller
            if not servo_ctrl:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")
            if not servo_ctrl.configure_water_pump_relay(req.water_pump_relay_number):
                raise HTTPException(status_code=400, detail="Invalid relay number")
            return {"success": True, "relay_number": req.water_pump_relay_number}

        app.include_router(servo_router)
