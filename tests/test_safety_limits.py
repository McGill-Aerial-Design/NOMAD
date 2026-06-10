# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for edge_core.safety.limits (SR-VEL-01..03)."""

from __future__ import annotations

import math

import pytest

from edge_core.safety.limits import (
    DEFAULT_MAX_VELOCITY_XY,
    DEFAULT_MAX_VELOCITY_Z,
    DEFAULT_MAX_YAW_RATE,
    VelocityLimits,
    clamp,
)


def test_clamp_within_and_outside_bounds():
    assert clamp(0.5, -1.0, 1.0) == 0.5
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0


def test_clamp_rejects_nonfinite():
    for bad in (math.inf, -math.inf, math.nan):
        with pytest.raises(ValueError):
            clamp(bad, -1.0, 1.0)


def test_default_limits_match_historical_constants():
    limits = VelocityLimits()
    assert limits.max_velocity_xy == DEFAULT_MAX_VELOCITY_XY == 2.0
    assert limits.max_velocity_z == DEFAULT_MAX_VELOCITY_Z == 1.0
    assert limits.max_yaw_rate == DEFAULT_MAX_YAW_RATE == 1.0


def test_clamp_command_clamps_each_axis_independently():
    limits = VelocityLimits()
    # XY uses 2.0, Z uses 1.0, yaw uses 1.0.
    assert limits.clamp_command(99.0, -99.0, 99.0, -99.0) == (2.0, -2.0, 1.0, -1.0)
    # In-range values pass through unchanged.
    assert limits.clamp_command(1.0, -0.5, 0.25, 0.9) == (1.0, -0.5, 0.25, 0.9)


def test_clamp_command_rejects_any_nonfinite_axis():
    limits = VelocityLimits()
    for bad_cmd in (
        (math.nan, 0.0, 0.0, 0.0),
        (0.0, math.inf, 0.0, 0.0),
        (0.0, 0.0, -math.inf, 0.0),
        (0.0, 0.0, 0.0, math.nan),
    ):
        with pytest.raises(ValueError):
            limits.clamp_command(*bad_cmd)


def test_custom_limits_are_respected():
    limits = VelocityLimits(max_velocity_xy=0.5, max_velocity_z=0.2, max_yaw_rate=0.1)
    assert limits.clamp_command(10.0, 10.0, 10.0, 10.0) == (0.5, 0.5, 0.2, 0.1)
