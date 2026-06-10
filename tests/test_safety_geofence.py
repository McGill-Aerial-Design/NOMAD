# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.geofence (SR-FEN-02).

Covers the containment geometry (point-in-polygon, distance, keep-in margin)
and the pure :func:`evaluate_position` decision the MAVLink command adapter
asks before sending any position target, including its fault-injection
branches (non-finite target, unconfigured fence, degenerate boundary).
"""

from __future__ import annotations

import math

from edge_core.safety.geofence import (
    FencePolicy,
    Point,
    distance_to_boundary,
    evaluate_position,
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


def test_evaluate_position_unconfigured_allows():
    # No boundary configured: pass through, the FC fence is the enforcement.
    decision = evaluate_position(FencePolicy(), Point(1e9, -1e9))
    assert decision.allowed is True
    assert decision.reason is None


def test_evaluate_position_inside_allows():
    decision = evaluate_position(FencePolicy(boundary=tuple(SQUARE)), Point(0, 0))
    assert decision.allowed is True


def test_evaluate_position_rejects_outside_boundary():
    decision = evaluate_position(FencePolicy(boundary=tuple(SQUARE)), Point(10, 0))
    assert decision.allowed is False
    assert decision.reason == "fence"
    assert decision.setpoint is None


def test_evaluate_position_rejects_inside_keep_in_margin():
    policy = FencePolicy(boundary=tuple(SQUARE), margin=2.0)
    # 1.0 from the edge with a 2.0 margin: rejected before the hard boundary.
    assert evaluate_position(policy, Point(4, 0)).allowed is False
    assert evaluate_position(policy, Point(0, 0)).allowed is True


def test_evaluate_position_rejects_nonfinite_target():
    policy = FencePolicy(boundary=tuple(SQUARE))
    for bad in (Point(math.nan, 0.0), Point(0.0, math.inf), Point(-math.inf, math.nan)):
        decision = evaluate_position(policy, bad)
        assert decision.allowed is False
        assert decision.reason == "nonfinite"
    # A non-finite target is rejected even with no fence configured.
    assert evaluate_position(FencePolicy(), Point(math.nan, 0.0)).allowed is False


def test_evaluate_position_degenerate_boundary_rejects_everything():
    # A configured-but-broken boundary (< 3 vertices) must fail closed.
    decision = evaluate_position(FencePolicy(boundary=()), Point(0, 0))
    assert decision.allowed is False
    assert decision.reason == "fence"
