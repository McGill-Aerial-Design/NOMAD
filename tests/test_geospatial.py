# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.geospatial pure functions."""

from __future__ import annotations

import math

import pytest

from edge_core.services.geospatial import (
    DroneState,
    GPSCoordinate,
    NEDOffset,
    bearing_between_points,
    calculate_3d_distance,
    calculate_gps_offset_meters,
    calculate_horizontal_distance,
    calculate_wall_lengths_from_corners,
    haversine_distance,
    offset_gps_by_meters,
    raycast_target_gps,
)

MONTREAL = GPSCoordinate(lat=45.5, lon=-73.5, alt=0.0)


def test_offset_roundtrip():
    offset = NEDOffset(north=100.0, east=50.0, down=20.0)
    moved = offset_gps_by_meters(MONTREAL, offset)
    back = calculate_gps_offset_meters(MONTREAL, moved)
    assert back.north == pytest.approx(100.0, abs=0.5)
    assert back.east == pytest.approx(50.0, abs=0.5)


def test_distance_helpers():
    offset = NEDOffset(north=3.0, east=4.0, down=12.0)
    assert calculate_horizontal_distance(offset) == pytest.approx(5.0)
    assert calculate_3d_distance(offset) == pytest.approx(13.0)


def test_haversine_zero_distance():
    assert haversine_distance(MONTREAL, MONTREAL) == pytest.approx(0.0)


def test_haversine_matches_offset_magnitude():
    target = offset_gps_by_meters(MONTREAL, NEDOffset(north=300.0, east=400.0))
    assert haversine_distance(MONTREAL, target) == pytest.approx(500.0, rel=0.01)


def test_bearing_cardinal_directions():
    north = offset_gps_by_meters(MONTREAL, NEDOffset(north=100.0, east=0.0))
    east = offset_gps_by_meters(MONTREAL, NEDOffset(north=0.0, east=100.0))
    assert bearing_between_points(MONTREAL, north) == pytest.approx(0.0, abs=0.5)
    assert bearing_between_points(MONTREAL, east) == pytest.approx(90.0, abs=0.5)


def test_raycast_straight_down():
    # Camera straight down from 50 m: target is directly below the drone.
    state = DroneState(
        gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        heading_deg=0.0,
        gimbal_pitch_deg=-90.0,
        lidar_distance_m=50.0,
    )
    target = raycast_target_gps(state)
    assert target.lat == pytest.approx(45.5, abs=1e-6)
    assert target.lon == pytest.approx(-73.5, abs=1e-6)
    assert target.alt == pytest.approx(0.0, abs=0.5)


def test_raycast_45deg_northeast():
    state = DroneState(
        gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        heading_deg=45.0,
        gimbal_pitch_deg=-45.0,
        lidar_distance_m=math.sqrt(50.0**2 + 50.0**2),
    )
    target = raycast_target_gps(state)
    offset = calculate_gps_offset_meters(state.gps, target)
    # Horizontal range 50 m at 45deg heading -> ~35.36 m north and east.
    assert offset.north == pytest.approx(35.36, abs=1.0)
    assert offset.east == pytest.approx(35.36, abs=1.0)


def test_wall_lengths_square():
    corners = [
        {"name": "NW", "lat": 45.5010, "lon": -73.5000},
        {"name": "NE", "lat": 45.5010, "lon": -73.4990},
        {"name": "SE", "lat": 45.5000, "lon": -73.4990},
        {"name": "SW", "lat": 45.5000, "lon": -73.5000},
    ]
    walls = calculate_wall_lengths_from_corners(corners, 45.5005, -73.4995)
    assert len(walls) == 4
    assert walls[0]["name"] == "NW-NE"
    assert all(w["length_m"] > 0 for w in walls)


def test_wall_lengths_needs_three_corners():
    assert calculate_wall_lengths_from_corners([{"name": "A", "lat": 0, "lon": 0}], 0, 0) == []
