# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.geospatial pure functions."""

from __future__ import annotations

import pytest

from edge_core.services.geospatial import GPSCoordinate, calculate_gps_offset_meters

MONTREAL = GPSCoordinate(lat=45.5, lon=-73.5, alt=0.0)


def test_offset_north_east_down():
    # 0.001 deg north ≈ 111.19 m; east is scaled by cos(latitude); down = alt drop.
    target = GPSCoordinate(lat=45.501, lon=-73.499, alt=-20.0)
    offset = calculate_gps_offset_meters(MONTREAL, target)
    assert offset.north == pytest.approx(111.19, abs=1.0)
    assert offset.east == pytest.approx(111.19 * 0.7009, abs=1.0)
    assert offset.down == pytest.approx(20.0, abs=0.01)


def test_offset_zero_for_same_point():
    offset = calculate_gps_offset_meters(MONTREAL, MONTREAL)
    assert offset.north == pytest.approx(0.0)
    assert offset.east == pytest.approx(0.0)
