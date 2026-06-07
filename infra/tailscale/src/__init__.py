# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Tailscale Integration Package

Provides Tailscale VPN management and network monitoring for secure
4G/LTE communication between Jetson and Ground Station.
"""

from .network_monitor import (
    ConnectionType,
    ModemStatus,
    NetworkMonitor,
    NetworkStatus,
    SignalQuality,
    get_network_monitor,
    init_network_monitor,
)
from .tailscale_manager import (
    TailscaleInfo,
    TailscaleManager,
    TailscalePeer,
    TailscaleStatus,
    get_tailscale_manager,
    init_tailscale_manager,
)

__all__ = [
    # Tailscale Manager
    "TailscaleManager",
    "TailscaleStatus",
    "TailscaleInfo",
    "TailscalePeer",
    "get_tailscale_manager",
    "init_tailscale_manager",
    # Network Monitor
    "NetworkMonitor",
    "NetworkStatus",
    "ModemStatus",
    "ConnectionType",
    "SignalQuality",
    "get_network_monitor",
    "init_network_monitor",
]
