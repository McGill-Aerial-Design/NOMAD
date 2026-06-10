# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Boundary containment for the safety-critical core (requirement SR-FEN-02).

Tier SC. Pure planar geometry — point-in-polygon and distance-to-boundary —
plus :func:`evaluate_position`, the single "is this position target inside the
fence?" decision the MAVLink command adapter
(``services/mavlink/commands.py``) asks before transmitting any position
target. The FC fence (uploaded from the C# ground station) remains the
independent backstop; this check is NOMAD's own containment layer. See
``docs/safety/hazards.md`` (H-05).

Inputs are 2D points in a consistent planar frame (e.g. local NED north/east in
metres, projected by the caller via ``services.geospatial``). The math is
frame-agnostic: it only requires that both the point and the polygon share units.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import NamedTuple

from .envelope import Decision


class Point(NamedTuple):
    """A 2D point in a planar frame (units must match the polygon)."""

    x: float
    y: float


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """True if ``point`` lies inside ``polygon`` (ray-casting, even-odd rule).

    ``polygon`` is an ordered ring of >= 3 vertices; the closing edge from the
    last vertex back to the first is implicit. Points exactly on an edge are
    treated conservatively as *outside* (the caller should use a positive
    ``margin`` via :func:`is_contained` rather than rely on boundary points).
    """
    n = len(polygon)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        # Does the horizontal ray from `point` cross edge (j -> i)?
        if (yi > point.y) != (yj > point.y):
            x_cross = (xj - xi) * (point.y - yi) / (yj - yi) + xi
            if point.x < x_cross:
                inside = not inside
        j = i
    return inside


def _distance_point_to_segment(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from ``p`` to segment ``a``-``b``."""
    abx, aby = b.x - a.x, b.y - a.y
    seg_len_sq = abx * abx + aby * aby
    if seg_len_sq == 0.0:
        return math.hypot(p.x - a.x, p.y - a.y)
    t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x, proj_y = a.x + t * abx, a.y + t * aby
    return math.hypot(p.x - proj_x, p.y - proj_y)


def distance_to_boundary(point: Point, polygon: Sequence[Point]) -> float:
    """Shortest distance from ``point`` to the polygon boundary (always >= 0).

    The sign of containment is *not* encoded here; combine with
    :func:`point_in_polygon` (or use :func:`is_contained`) to know inside/outside.
    """
    n = len(polygon)
    if n < 2:
        return math.inf
    best = math.inf
    j = n - 1
    for i in range(n):
        best = min(best, _distance_point_to_segment(point, polygon[j], polygon[i]))
        j = i
    return best


def is_contained(point: Point, polygon: Sequence[Point], margin: float = 0.0) -> bool:
    """True if ``point`` is inside ``polygon`` by at least ``margin`` units.

    With the default ``margin=0`` this is plain containment. A positive
    ``margin`` defines a keep-in buffer: the point must be inside *and* at least
    ``margin`` away from the nearest edge, so a command is rejected before the
    aircraft reaches the hard boundary.
    """
    if not point_in_polygon(point, polygon):
        return False
    if margin <= 0.0:
        return True
    return distance_to_boundary(point, polygon) >= margin


@dataclass(frozen=True)
class FencePolicy:
    """Optional keep-in boundary for position targets.

    ``boundary is None`` means no NOMAD-side fence is configured: position
    targets pass through and the FC fence is the only enforcement. A configured
    boundary with fewer than 3 vertices is a broken configuration and rejects
    every target — a bad fence must fail closed, never open.
    """

    boundary: tuple[Point, ...] | None = None
    margin: float = 0.0


def evaluate_position(policy: FencePolicy, point: Point) -> Decision:
    """Decide whether a position target at ``point`` stays inside the fence.

    Pure and frame-agnostic: the caller projects the target and the boundary
    into the same planar frame before asking. The first failing check
    short-circuits with the safe default (reject, send nothing); only a finite
    target inside the boundary by at least ``margin`` yields ``allowed=True``.
    """
    if not (math.isfinite(point.x) and math.isfinite(point.y)):
        return Decision(False, "nonfinite", "Non-finite position target - dropping command")

    if policy.boundary is None:
        return Decision(True)

    if not is_contained(point, policy.boundary, policy.margin):
        return Decision(False, "fence", "Position target outside geofence boundary - dropping command")

    return Decision(True)
