# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""NetworkModule — wires TailscaleManager + NetworkMonitor into the lifecycle.

Both run polling threads owned by this module. The ``/network/status`` route
(in ``api_routes/system.py``) reads them via ``app.state.tailscale_manager``
and ``app.state.network_monitor``.
"""

from __future__ import annotations

import logging

from edge_core.core import AppContext, BaseModule, ModuleMetadata
from infra.tailscale import NetworkMonitor, TailscaleManager

logger = logging.getLogger("edge_core.services.network")


class NetworkModule(BaseModule):
    """Owns the Tailscale status poller and the LTE/connectivity monitor."""

    metadata = ModuleMetadata(
        name="network",
        version="1.0.0",
        description="Tailscale VPN status and 4G/LTE connectivity monitoring",
        enable_flag="NOMAD_ENABLE_NETWORK_MONITOR",
        enabled_by_default=True,
    )

    def __init__(self) -> None:
        self._tailscale: TailscaleManager | None = None
        self._monitor: NetworkMonitor | None = None

    def configure(self, ctx: AppContext) -> None:
        self._tailscale = TailscaleManager()
        self._monitor = NetworkMonitor(gcs_tailscale_ip=ctx.get_config("GCS_IP") or None)
        ctx.register_service("tailscale_manager", self._tailscale)
        ctx.register_service("network_monitor", self._monitor)
        ctx.app.state.tailscale_manager = self._tailscale
        ctx.app.state.network_monitor = self._monitor
        logger.info("Network module configured")

    def start(self) -> None:
        if self._tailscale:
            self._tailscale.start()
        if self._monitor:
            self._monitor.start()

    def stop(self) -> None:
        if self._monitor:
            self._monitor.stop()
        if self._tailscale:
            self._tailscale.stop()
