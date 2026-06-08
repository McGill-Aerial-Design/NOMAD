# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
from datetime import datetime, timezone

from pydantic import BaseModel, ConfigDict


class DetectionInfo(BaseModel):
    """
    Information about a detected object from vision.

    Used for UI overlay and mission logic.
    """

    model_config = ConfigDict(frozen=True)

    # Detection metadata
    class_id: int
    class_name: str
    confidence: float

    # Bounding box (normalized 0-1)
    bbox_x1: float
    bbox_y1: float
    bbox_x2: float
    bbox_y2: float

    # Timing
    timestamp: float

    @property
    def center_x(self) -> float:
        """Center X coordinate (0-1)."""
        return (self.bbox_x1 + self.bbox_x2) / 2

    @property
    def center_y(self) -> float:
        """Center Y coordinate (0-1)."""
        return (self.bbox_y1 + self.bbox_y2) / 2

    @classmethod
    def from_dict(cls, data: dict) -> "DetectionInfo":
        """Create from detection message dict."""
        bbox = data.get("bbox", {})
        return cls(
            class_id=data.get("class_id", 0),
            class_name=data.get("class_name", "unknown"),
            confidence=data.get("confidence", 0.0),
            bbox_x1=bbox.get("x1", 0.0),
            bbox_y1=bbox.get("y1", 0.0),
            bbox_x2=bbox.get("x2", 0.0),
            bbox_y2=bbox.get("y2", 0.0),
            timestamp=data.get("timestamp", 0.0),
        )


class SystemState(BaseModel):
    """
    Drone system state model.

    Contains all telemetry and sensor data needed for mission operations.
    Immutable (frozen) to ensure thread-safety.
    """

    model_config = ConfigDict(frozen=True)

    # Timestamps and status
    timestamp: datetime
    flight_mode: str
    connected: bool
    armed: bool = False  # Vehicle armed state

    # Battery
    battery_voltage: float

    # GPS (WGS84)
    gps_fix: bool
    gps_lat: float | None = None  # Latitude in degrees
    gps_lon: float | None = None  # Longitude in degrees
    gps_alt: float | None = None  # Altitude MSL in meters
    alt_agl_m: float | None = None  # Altitude AGL from FC (GLOBAL_POSITION_INT.relative_alt)
    home_lat: float | None = None  # ArduPilot home latitude in degrees
    home_lon: float | None = None  # ArduPilot home longitude in degrees
    home_alt: float | None = None  # ArduPilot home altitude MSL in meters

    # Attitude and heading
    heading_deg: float | None = None  # Magnetic heading 0-360 (0=North)
    pitch_deg: float | None = None  # Pitch angle in degrees
    roll_deg: float | None = None  # Roll angle in degrees

    # Gimbal
    gimbal_pitch_deg: float | None = None  # Gimbal pitch (-90 to 0)
    gimbal_yaw_deg: float | None = None  # Gimbal yaw relative to drone

    # Sensors
    lidar_distance_m: float | None = None  # LiDAR distance to target

    # Vision / Detection
    target_visible: bool = False  # Is a target currently detected?
    current_detection: DetectionInfo | None = None  # Latest detection

    # VIO pose in local NED/body-navigation frame for approach/positioning logic
    vio_x: float | None = None
    vio_y: float | None = None
    vio_z: float | None = None
    vio_yaw: float | None = None
    vio_confidence: float | None = None

    # Time Synchronization
    time_synced: bool = False  # Is system time synchronized (NTP or GPS)?

    # Hardware Health (Jetson/System)
    cpu_temp_c: float | None = None  # CPU temperature in Celsius
    gpu_temp_c: float | None = None  # GPU temperature in Celsius
    gpu_load_pct: float | None = None  # GPU utilization percentage
    power_draw_w: float | None = None  # Power draw in Watts
    disk_free_gb: float | None = None  # Free disk space on evidence storage
    memory_used_pct: float | None = None  # RAM usage percentage
    throttled: bool = False  # Is system in throttled mode?

    @classmethod
    def default(cls) -> "SystemState":
        return cls(
            timestamp=datetime.now(timezone.utc),
            flight_mode="UNKNOWN",
            battery_voltage=0.0,
            gps_fix=False,
            connected=False,
            time_synced=False,
            throttled=False,
        )

    def has_valid_gps(self) -> bool:
        """Check if GPS coordinates are available."""
        return self.gps_fix and self.gps_lat is not None and self.gps_lon is not None
