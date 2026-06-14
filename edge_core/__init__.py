# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Edge Core - Domain B for NOMAD.

Drone-side processing including state management, geospatial calculations,
MAVLink interface, and core services.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from .services.geospatial import (
    DroneState,
    GPSCoordinate,
    NEDOffset,
    bearing_between_points,
    calculate_gps_offset_meters,
    haversine_distance,
    offset_gps_by_meters,
    raycast_target_gps,
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
    "DroneState",
    "calculate_gps_offset_meters",
    "raycast_target_gps",
    "haversine_distance",
    "offset_gps_by_meters",
    "bearing_between_points",
    # Time Synchronization
    "TimeSyncService",
    "TimeSyncStatus",
    "TimeSyncSource",
    "get_time_sync_service",
    "init_time_sync_service",
]
