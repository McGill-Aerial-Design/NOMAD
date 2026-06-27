# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Geospatial calculations for NOMAD.

Pure functions for GPS coordinate transformations and relative position
calculations.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import math
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
    lat_diff_rad = math.radians(target.lat - origin.lat)
    lon_diff_rad = math.radians(target.lon - origin.lon)

    # Average latitude scales the longitude difference for meridian convergence.
    avg_lat_rad = math.radians((origin.lat + target.lat) / 2.0)

    north_m = lat_diff_rad * EARTH_RADIUS_M
    east_m = lon_diff_rad * EARTH_RADIUS_M * math.cos(avg_lat_rad)
    down_m = origin.alt - target.alt  # positive down

    return NEDOffset(north=north_m, east=east_m, down=down_m)
