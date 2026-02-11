"""
Edge Core - Domain B for NOMAD.

Drone-side processing including state management, geospatial calculations,
MAVLink interface, and core services.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from .models import (
    SystemState,
    DetectionInfo,
)
from .state import StateManager
from .geospatial import (
    GPSCoordinate,
    NEDOffset,
    DroneState,
    calculate_gps_offset_meters,
    raycast_target_gps,
    haversine_distance,
    offset_gps_by_meters,
    bearing_between_points,
)
from .logging_service import (
    log_mission_event,
    list_mission_logs,
    read_mission_log,
)
from .ipc import (
    IPCMessage,
    ZMQPublisher,
    ZMQSubscriber,
    HeartbeatMonitor,
    DEFAULT_VISION_HEARTBEAT_ENDPOINT,
    DEFAULT_VISION_DATA_ENDPOINT,
)
from .time_manager import (
    TimeSyncService,
    TimeSyncStatus,
    TimeSyncSource,
    get_time_sync_service,
    init_time_sync_service,
)
from .nav_controller import (
    NavController,
    NavMode,
    NavHealth,
    NavStatus,
    VelocityCommand,
)

# Optional imports (may not be available on all platforms)
try:
    from .isaac_ros_bridge import (
        IsaacROSBridge,
        VIOState,
        DetectedTarget,
        ExclusionEntry,
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
    "DetectionInfo",
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
    # Navigation Controller
    "NavController",
    "NavMode",
    "NavHealth",
    "NavStatus",
    "VelocityCommand",
]

# Only export Isaac symbols when the import succeeded
if _ISAAC_AVAILABLE:
    __all__ += [
        "IsaacROSBridge",
        "VIOState",
        "DetectedTarget",
        "ExclusionEntry",
        "get_isaac_bridge",
        "init_isaac_bridge",
    ]
