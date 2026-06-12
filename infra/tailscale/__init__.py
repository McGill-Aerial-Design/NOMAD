# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Tailscale Integration Package

Provides Tailscale VPN management and network monitoring for secure
4G/LTE communication between Jetson and Ground Station.
"""

from .network_monitor import (
    ModemStatus,
    NetworkMonitor,
    NetworkStatus,
    SignalQuality,
)
from .tailscale_manager import (
    TailscaleInfo,
    TailscaleManager,
    TailscaleStatus,
)

__all__ = [
    "TailscaleManager",
    "TailscaleStatus",
    "TailscaleInfo",
    "NetworkMonitor",
    "NetworkStatus",
    "ModemStatus",
    "SignalQuality",
]
