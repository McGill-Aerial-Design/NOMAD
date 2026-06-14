# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.ros_http_bridge.coordinate_math."""

from __future__ import annotations

import math

import pytest

from edge_core.ros_http_bridge.coordinate_math import (
    quat_to_euler,
    wrap_angle_rad,
)


def test_identity_quaternion_is_zero_attitude():
    roll, pitch, yaw = quat_to_euler(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_yaw_90_degrees():
    # Rotation of +90deg about z: q = (0,0,sin45,cos45)
    s = math.sqrt(0.5)
    _, _, yaw = quat_to_euler(0.0, 0.0, s, s)
    assert yaw == pytest.approx(math.pi / 2)


def test_gimbal_lock_pitch_clamps():
    # sinp >= 1 -> pitch saturates at +/- pi/2 rather than raising.
    _, pitch, _ = quat_to_euler(0.0, s := math.sqrt(0.5), 0.0, s)
    assert abs(pitch) == pytest.approx(math.pi / 2)


def test_wrap_angle_to_pi_range():
    # 2*pi + pi/4 wraps back to pi/4.
    assert wrap_angle_rad(2 * math.pi + math.pi / 4) == pytest.approx(math.pi / 4)
    assert wrap_angle_rad(-math.pi / 2) == pytest.approx(-math.pi / 2)
    for raw in (10.0, -10.0, 100.0, 0.0):
        assert -math.pi <= wrap_angle_rad(raw) <= math.pi
