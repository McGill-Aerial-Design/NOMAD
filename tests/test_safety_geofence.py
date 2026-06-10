# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.geofence containment primitive (SR-FEN-02).

NOTE: this primitive is not yet wired into the command path; these tests prove
the geometry so the wiring step (a later, SITL-verified change) can rely on it.
"""

from __future__ import annotations

import math

from edge_core.safety.geofence import (
    Point,
    distance_to_boundary,
    is_contained,
    point_in_polygon,
)

# A 10 x 10 square centred on the origin.
SQUARE = [Point(-5, -5), Point(5, -5), Point(5, 5), Point(-5, 5)]


def test_degenerate_polygon_is_never_inside():
    assert point_in_polygon(Point(0, 0), [Point(0, 0), Point(1, 1)]) is False


def test_point_clearly_inside():
    assert point_in_polygon(Point(0, 0), SQUARE) is True
    assert point_in_polygon(Point(4.9, -4.9), SQUARE) is True


def test_point_clearly_outside():
    assert point_in_polygon(Point(10, 0), SQUARE) is False
    assert point_in_polygon(Point(0, 100), SQUARE) is False
    assert point_in_polygon(Point(-6, -6), SQUARE) is False


def test_concave_polygon_notch():
    # An L / arrow shape: the notch region is outside.
    poly = [Point(0, 0), Point(4, 0), Point(4, 4), Point(2, 1), Point(0, 4)]
    assert point_in_polygon(Point(2, 0.5), poly) is True
    assert point_in_polygon(Point(2, 3.5), poly) is False  # in the notch


def test_distance_to_boundary_from_centre():
    # Centre of the 10x10 square is 5 from the nearest edge.
    assert math.isclose(distance_to_boundary(Point(0, 0), SQUARE), 5.0)


def test_distance_to_boundary_near_edge():
    assert math.isclose(distance_to_boundary(Point(4, 0), SQUARE), 1.0)


def test_is_contained_default_margin():
    assert is_contained(Point(0, 0), SQUARE) is True
    assert is_contained(Point(10, 0), SQUARE) is False


def test_is_contained_with_keep_in_margin():
    # 4.0 from centre leaves 1.0 of clearance; require 2.0 -> rejected.
    assert is_contained(Point(4, 0), SQUARE, margin=0.5) is True
    assert is_contained(Point(4, 0), SQUARE, margin=2.0) is False


def test_margin_rejects_point_outside_regardless():
    assert is_contained(Point(100, 100), SQUARE, margin=1.0) is False


def test_distance_to_boundary_too_few_vertices_is_infinite():
    assert distance_to_boundary(Point(0, 0), [Point(0, 0)]) == math.inf


def test_distance_to_boundary_handles_degenerate_edge():
    # A polygon containing a repeated vertex (zero-length edge) must not divide
    # by zero; distance falls back to the point-to-vertex distance.
    # All edges (and the degenerate one) put the nearest feature at (0, 0).
    poly = [Point(0, 0), Point(0, 0), Point(3, 0)]
    assert math.isclose(distance_to_boundary(Point(0, 4), poly), 4.0)
