# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Edge Core - Domain B for NOMAD.

Drone-side processing including state management, geospatial calculations,
MAVLink interface, and core services.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from .services.geospatial import (
    GPSCoordinate,
    NEDOffset,
    calculate_gps_offset_meters,
)
from .services.models import SystemState
from .services.state import StateManager
from .services.time_manager import (
    TimeSyncService,
    TimeSyncSource,
    TimeSyncStatus,
    get_time_sync_service,
    init_time_sync_service,
)

__all__ = [
    # Models
    "SystemState",
    "StateManager",
    # Geospatial
    "GPSCoordinate",
    "NEDOffset",
    "calculate_gps_offset_meters",
    # Time Synchronization
    "TimeSyncService",
    "TimeSyncStatus",
    "TimeSyncSource",
    "get_time_sync_service",
    "init_time_sync_service",
]
