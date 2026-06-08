# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pluggable NomadModule for Servo/Relay actuations and Spray control."""

from __future__ import annotations

import logging

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

    def stop(self) -> None:
        controller = get_servo_controller()
        if controller:
            controller.shutdown()
            logger.info("Payload actuation module stopped")
