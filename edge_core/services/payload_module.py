# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pluggable NomadModule for Servo/Relay actuations and Spray control."""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import APIRouter, HTTPException

from edge_core.core import AppContext, BaseModule, ModuleMetadata
from edge_core.modules.payload.servo import get_servo_controller, init_servo_controller

logger = logging.getLogger("edge_core.services.payload")


class PayloadModule(BaseModule):
    """Modular controller managing physical nozzle servos, pump relays, and trigger schedules."""

    metadata = ModuleMetadata(
        name="payload",
        version="1.0.0",
        description="Controls camera tilt, water shooting relays, and nozzle angles over MAVLink",
        enable_flag="NOMAD_ENABLE_SERVOS",
        enabled_by_default=True,
    )

    def configure(self, ctx: AppContext) -> None:
        mavlink_service = ctx.require_service("mavlink_service")
        tilt_channel = int(ctx.get_config("CameraTiltChannel", "14") or "14")

        # Initialize the global controller instance
        init_servo_controller(mavlink_service=mavlink_service, camera_tilt_channel=tilt_channel)
        controller = get_servo_controller()
        ctx.register_service("servo_controller", controller)
        ctx.app.state.servo_controller = controller
        logger.info("Payload actuation module configured")

    def register_routes(self, app: Any) -> None:
        router = APIRouter(tags=["Servo & Spray"])

        @router.post("/api/servo/shooter/arm")
        async def arm_shooter():
            """Arm the water-shooter release interlock (SR-PAY-03).

            Release requires this explicit arm within the returned window; the
            arm is consumed by the next trigger attempt, successful or not.
            """
            controller = get_servo_controller()
            if not controller:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")
            window_s = controller.arm_release()
            return {"success": True, "armed_for_s": window_s}

        @router.post("/api/servo/shooter/trigger")
        async def trigger_shooter(duration_ms: int = 200, relay_number: int | None = None):
            """Fire the water shooter for a clamped duration; requires a prior arm."""
            controller = get_servo_controller()
            if not controller:
                raise HTTPException(status_code=503, detail="Servo controller not initialized")
            if relay_number is not None and not controller.configure_water_pump_relay(relay_number):
                raise HTTPException(status_code=400, detail="Invalid relay number")
            # The pump-on window blocks for its duration; keep the event loop free.
            if await asyncio.to_thread(controller.trigger_water_shooter, duration_ms):
                return {"success": True}
            raise HTTPException(
                status_code=409,
                detail="Release rejected: arm via /api/servo/shooter/arm first (or MAVLink unavailable)",
            )

        app.include_router(router)

    def stop(self) -> None:
        controller = get_servo_controller()
        if controller:
            controller.shutdown()
            logger.info("Payload actuation module stopped")
