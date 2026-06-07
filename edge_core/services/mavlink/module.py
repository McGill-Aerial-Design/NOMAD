# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""NomadModule wrapping for MavlinkService."""

from __future__ import annotations

import logging
import os

from edge_core.core import AppContext, BaseModule, ModuleMetadata

from . import MavlinkService

logger = logging.getLogger("edge_core.mavlink.module")


class MavlinkModule(BaseModule):
    """Bridges MavlinkService connections into the modular SDK framework."""

    metadata = ModuleMetadata(
        name="mavlink",
        version="1.0.0",
        description="Handles flight controller serial/UDP MAVLink streams and telemetry extraction",
        enable_flag="NOMAD_AUTOSTART_MAVLINK_ROUTER",
        enabled_by_default=True,
    )

    def __init__(self) -> None:
        self._service: MavlinkService | None = None

    def configure(self, ctx: AppContext) -> None:
        state_mgr = ctx.require_service("state_manager")
        endpoint = ctx.get_config("NOMAD_MAVLINK_ENDPOINT", "127.0.0.1:14550")
        self._service = MavlinkService(state_mgr, endpoint=endpoint)
        ctx.register_service("mavlink", self._service)
        ctx.register_service("mavlink_service", self._service)

    def start(self) -> None:
        if self._service:
            self._service.start()
            interval = float(os.environ.get("NOMAD_HEALTH_BROADCAST_INTERVAL_S", "2.0"))
            self._service.start_health_broadcast(interval=interval)
            logger.info("MAVLink routing module initialized")

    def stop(self) -> None:
        if self._service:
            self._service.stop_health_broadcast()
            self._service.stop()
            logger.info("MAVLink routing module stopped")
