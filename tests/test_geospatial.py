# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.geospatial pure functions."""

from __future__ import annotations

import pytest

from edge_core.services.geospatial import (
    GPSCoordinate,
    NEDOffset,
    bearing_between_points,
    calculate_gps_offset_meters,
    haversine_distance,
    offset_gps_by_meters,
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
