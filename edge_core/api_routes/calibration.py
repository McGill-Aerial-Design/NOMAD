# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ZED Calibration, servo config, and spray calibration API routes."""

from __future__ import annotations

import asyncio
import logging
import subprocess
from typing import Any

from fastapi import APIRouter, HTTPException, Query, Request

from edge_core.core import AppContext, BaseModule, ModuleMetadata
from edge_core.safety import MAX_PWM_US, MAX_SERVO_CHANNEL, MIN_PWM_US, MIN_SERVO_CHANNEL

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
            if channel < MIN_SERVO_CHANNEL or channel > MAX_SERVO_CHANNEL:
                raise HTTPException(
                    status_code=400, detail=f"Channel must be between {MIN_SERVO_CHANNEL} and {MAX_SERVO_CHANNEL}"
                )
            if pwm < MIN_PWM_US or pwm > MAX_PWM_US:
                raise HTTPException(status_code=400, detail=f"PWM must be between {MIN_PWM_US} and {MAX_PWM_US} us")

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
        async def set_spray_calibration(request: Request):
            """Push spray calibration parameters from Mission Planner."""
            try:
                import json

                body = await request.body()
                data = json.loads(body.decode("utf-8"))
            except Exception as exc:
                raise HTTPException(status_code=400, detail=f"Invalid JSON: {exc}")

            servo_ctrl = app.state.servo_controller
            relay_num = int(data.get("water_pump_relay_number", 0))
            if servo_ctrl and relay_num > 0:
                servo_ctrl.configure_water_pump_relay(relay_num)

            logger.info("Spray calibration updated: %s", list(data.keys()))
            return {"success": True, "persisted": data.get("persist", False)}

        app.include_router(servo_router)
