"""
Geospatial calculations for NOMAD Task 1 (Recon Mission).

Pure functions for GPS coordinate transformations, raycasting,
and relative position calculations.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple


# WGS84 Earth constants
EARTH_RADIUS_M = 6371000.0  # Mean Earth radius in meters


class GPSCoordinate(NamedTuple):
    """GPS coordinate in decimal degrees (WGS84)."""

    lat: float  # Latitude in degrees (-90 to 90)
    lon: float  # Longitude in degrees (-180 to 180)
    alt: float = 0.0  # Altitude in meters MSL


class NEDOffset(NamedTuple):
    """North-East-Down offset in meters from a reference point."""

    north: float  # Positive = North, Negative = South
    east: float  # Positive = East, Negative = West
    down: float = 0.0  # Positive = Down, Negative = Up


@dataclass(frozen=True, slots=True)
class DroneState:
    """Drone state for geospatial calculations."""

    gps: GPSCoordinate
    heading_deg: float  # Magnetic heading 0-360 (0 = North, 90 = East)
    gimbal_pitch_deg: float  # Gimbal pitch angle (-90 to 0, -90 = straight down)
    lidar_distance_m: float  # LiDAR measured distance to target in meters


def deg_to_rad(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def rad_to_deg(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


def normalize_heading(heading: float) -> float:
    """Normalize heading to 0-360 range."""
    return heading % 360.0


def calculate_gps_offset_meters(
    origin: GPSCoordinate,
    target: GPSCoordinate,
) -> NEDOffset:
    """
    Calculate the North/East offset (in meters) between two GPS coordinates.

    Uses Haversine-derived approximation for small distances.
    Accurate for distances < 10km.

    Args:
        origin: Reference GPS coordinate
        target: Target GPS coordinate

    Returns:
        NEDOffset with north/east/down offsets in meters

    Example:
        >>> origin = GPSCoordinate(lat=45.5, lon=-73.5)
        >>> target = GPSCoordinate(lat=45.5001, lon=-73.5001)
        >>> offset = calculate_gps_offset_meters(origin, target)
        >>> print(f"North: {offset.north:.2f}m, East: {offset.east:.2f}m")
    """
    # Latitude difference to meters (approximately 111,320 m per degree at equator)
    lat_diff_rad = deg_to_rad(target.lat - origin.lat)
    lon_diff_rad = deg_to_rad(target.lon - origin.lon)

    # Average latitude for longitude scaling
    avg_lat_rad = deg_to_rad((origin.lat + target.lat) / 2.0)

    # North offset: latitude difference * Earth radius
    north_m = lat_diff_rad * EARTH_RADIUS_M

    # East offset: longitude difference * Earth radius * cos(latitude)
    # The cos(lat) factor accounts for meridian convergence
    east_m = lon_diff_rad * EARTH_RADIUS_M * math.cos(avg_lat_rad)

    # Down offset: altitude difference (positive down)
    down_m = origin.alt - target.alt

    return NEDOffset(north=north_m, east=east_m, down=down_m)


def calculate_horizontal_distance(offset: NEDOffset) -> float:
    """Calculate horizontal distance from NED offset."""
    return math.sqrt(offset.north**2 + offset.east**2)


def calculate_3d_distance(offset: NEDOffset) -> float:
    """Calculate 3D distance from NED offset."""
    return math.sqrt(offset.north**2 + offset.east**2 + offset.down**2)


def raycast_target_gps(drone_state: DroneState) -> GPSCoordinate:
    """
    Calculate the GPS location of a target using raycasting.

    Given the drone's GPS position, heading, gimbal pitch, and LiDAR distance,
    calculates where the target is located on the ground.

    Args:
        drone_state: Current drone state with GPS, heading, gimbal pitch, and LiDAR

    Returns:
        GPSCoordinate of the target location

    Algorithm:
        1. Calculate horizontal distance from LiDAR slant range and gimbal pitch
        2. Use heading to determine North/East components
        3. Convert meter offsets back to GPS coordinates

    Example:
        >>> state = DroneState(
        ...     gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        ...     heading_deg=45.0,  # Northeast
        ...     gimbal_pitch_deg=-45.0,  # 45 degrees down
        ...     lidar_distance_m=70.71  # sqrt(50^2 + 50^2)
        ... )
        >>> target = raycast_target_gps(state)
    """
    # Convert angles to radians
    pitch_rad = deg_to_rad(drone_state.gimbal_pitch_deg)
    heading_rad = deg_to_rad(drone_state.heading_deg)

    # Calculate horizontal distance from slant range
    # pitch is negative (pointing down), so we use -pitch for calculation
    # horizontal_distance = lidar_distance * cos(pitch)
    # Since pitch is negative for downward, cos(-pitch) = cos(|pitch|)
    horizontal_distance_m = drone_state.lidar_distance_m * math.cos(pitch_rad)

    # Calculate vertical drop (for altitude calculation)
    # vertical_drop = lidar_distance * sin(|pitch|) = -lidar_distance * sin(pitch)
    vertical_drop_m = -drone_state.lidar_distance_m * math.sin(pitch_rad)

    # Calculate North and East offsets using heading
    # Heading: 0 = North, 90 = East, 180 = South, 270 = West
    north_offset_m = horizontal_distance_m * math.cos(heading_rad)
    east_offset_m = horizontal_distance_m * math.sin(heading_rad)

    # Convert meter offsets back to GPS coordinates
    target_gps = offset_gps_by_meters(
        drone_state.gps,
        NEDOffset(north=north_offset_m, east=east_offset_m, down=vertical_drop_m),
    )

    return target_gps


def offset_gps_by_meters(
    origin: GPSCoordinate,
    offset: NEDOffset,
) -> GPSCoordinate:
    """
    Calculate a new GPS coordinate given an origin and NED offset in meters.

    Args:
        origin: Starting GPS coordinate
        offset: North/East/Down offset in meters

    Returns:
        New GPSCoordinate at the offset location

    Example:
        >>> origin = GPSCoordinate(lat=45.5, lon=-73.5, alt=100.0)
        >>> offset = NEDOffset(north=100.0, east=50.0, down=50.0)
        >>> target = offset_gps_by_meters(origin, offset)
    """
    # Latitude offset (North/South)
    lat_offset_deg = rad_to_deg(offset.north / EARTH_RADIUS_M)

    # Longitude offset (East/West) - adjusted for latitude
    lat_rad = deg_to_rad(origin.lat)
    lon_offset_deg = rad_to_deg(offset.east / (EARTH_RADIUS_M * math.cos(lat_rad)))

    # Altitude offset (Down is positive, so subtract)
    new_alt = origin.alt - offset.down

    return GPSCoordinate(
        lat=origin.lat + lat_offset_deg,
        lon=origin.lon + lon_offset_deg,
        alt=new_alt,
    )


def bearing_between_points(
    origin: GPSCoordinate,
    target: GPSCoordinate,
) -> float:
    """
    Calculate the initial bearing from origin to target.

    Args:
        origin: Starting GPS coordinate
        target: Destination GPS coordinate

    Returns:
        Bearing in degrees (0-360, 0 = North, 90 = East)
    """
    lat1_rad = deg_to_rad(origin.lat)
    lat2_rad = deg_to_rad(target.lat)
    lon_diff_rad = deg_to_rad(target.lon - origin.lon)

    x = math.sin(lon_diff_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(lon_diff_rad)

    bearing_rad = math.atan2(x, y)
    bearing_deg = rad_to_deg(bearing_rad)

    return normalize_heading(bearing_deg)


def haversine_distance(
    coord1: GPSCoordinate,
    coord2: GPSCoordinate,
) -> float:
    """
    Calculate the great-circle distance between two GPS coordinates.

    Uses the Haversine formula for accurate distance calculation.

    Args:
        coord1: First GPS coordinate
        coord2: Second GPS coordinate

    Returns:
        Distance in meters

    Example:
        >>> a = GPSCoordinate(lat=45.5, lon=-73.5)
        >>> b = GPSCoordinate(lat=45.5001, lon=-73.5001)
        >>> dist = haversine_distance(a, b)
        >>> print(f"Distance: {dist:.2f}m")
    """
    lat1_rad = deg_to_rad(coord1.lat)
    lat2_rad = deg_to_rad(coord2.lat)
    delta_lat_rad = deg_to_rad(coord2.lat - coord1.lat)
    delta_lon_rad = deg_to_rad(coord2.lon - coord1.lon)

    a = (
        math.sin(delta_lat_rad / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_M * c
