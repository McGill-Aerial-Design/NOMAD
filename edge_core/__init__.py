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
from .services.ipc import (
    DEFAULT_VISION_DATA_ENDPOINT,
    DEFAULT_VISION_HEARTBEAT_ENDPOINT,
    HeartbeatMonitor,
    IPCMessage,
    ZMQPublisher,
    ZMQSubscriber,
)
from .services.logging_service import (
    list_mission_logs,
    log_mission_event,
    read_mission_log,
)
from .services.state import StateManager
from .services.time_manager import (
    TimeSyncService,
    TimeSyncSource,
    TimeSyncStatus,
    get_time_sync_service,
    init_time_sync_service,
)

# Optional imports (may not be available on all platforms)
try:
    from .modules.slam.isaac import (
        IsaacROSBridge,
        VIOState,
        get_isaac_bridge,
        init_isaac_bridge,
    )

    _ISAAC_AVAILABLE = True
except ImportError:
    _ISAAC_AVAILABLE = False

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
    # Logging
    "log_mission_event",
    "list_mission_logs",
    "read_mission_log",
    # IPC
    "IPCMessage",
    "ZMQPublisher",
    "ZMQSubscriber",
    "HeartbeatMonitor",
    "DEFAULT_VISION_HEARTBEAT_ENDPOINT",
    "DEFAULT_VISION_DATA_ENDPOINT",
    # Time Synchronization
    "TimeSyncService",
    "TimeSyncStatus",
    "TimeSyncSource",
    "get_time_sync_service",
    "init_time_sync_service",
]

# Only export Isaac symbols when the import succeeded
if _ISAAC_AVAILABLE:
    __all__ += [
        "IsaacROSBridge",
        "VIOState",
        "get_isaac_bridge",
        "init_isaac_bridge",
    ]
