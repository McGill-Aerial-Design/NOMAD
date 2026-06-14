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
    calculate_gps_offset_meters,
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
