# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.ros_http_bridge.vio_math.

These cover the VIO-derivation math extracted from the ROS callbacks in
``node.py`` (which cannot be imported without rclpy). The sign conventions
are flight-relevant, so each frame conversion is pinned explicitly.
"""

from __future__ import annotations

import math

import pytest

from edge_core.ros_http_bridge.vio_math import (
    camera_to_body_pose,
    flu_to_ned_attitude,
    flu_to_ned_vec3,
    position_variance,
    tilt_compensated_heading,
    vio_confidence,
)

# --------------------------------------------------------------------------- #
# position_variance
# --------------------------------------------------------------------------- #


def _covariance(xx: float, yy: float, zz: float) -> list[float]:
    """A 6x6 row-major pose covariance with the given x/y/z position variances."""
    cov = [0.0] * 36
    cov[0] = xx  # (x, x)
    cov[7] = yy  # (y, y)
    cov[14] = zz  # (z, z)
    return cov


def test_position_variance_picks_worst_axis():
    assert position_variance(_covariance(0.01, 0.04, 0.02)) == pytest.approx(0.04)


def test_position_variance_ignores_off_diagonal():
    cov = _covariance(0.01, 0.01, 0.01)
    cov[1] = 99.0  # off-diagonal cross-term must not be selected
    cov[21] = 99.0  # a roll variance (index 21) is not a position term
    assert position_variance(cov) == pytest.approx(0.01)


# --------------------------------------------------------------------------- #
# vio_confidence
# --------------------------------------------------------------------------- #


def test_vio_confidence_zero_variance_is_full():
    assert vio_confidence(0.0) == pytest.approx(1.0)


def test_vio_confidence_linear_midpoint():
    assert vio_confidence(0.05) == pytest.approx(0.5)


def test_vio_confidence_clamps_high_variance_to_zero():
    assert vio_confidence(0.1) == pytest.approx(0.0)
    assert vio_confidence(5.0) == 0.0


def test_vio_confidence_clamps_negative_variance_to_one():
    # A degenerate negative variance must not produce confidence > 1.
    assert vio_confidence(-1.0) == 1.0


# --------------------------------------------------------------------------- #
# REP-103 -> NED conversions
# --------------------------------------------------------------------------- #


def test_flu_to_ned_vec3_flips_y_and_z():
    assert flu_to_ned_vec3(1.0, 2.0, 3.0) == (1.0, -2.0, -3.0)


def test_flu_to_ned_attitude_negates_pitch_and_yaw():
    assert flu_to_ned_attitude(0.1, 0.2, 0.3) == (0.1, -0.2, -0.3)


# --------------------------------------------------------------------------- #
# tilt_compensated_heading
# --------------------------------------------------------------------------- #


def test_heading_none_when_horizontal_field_negligible():
    assert tilt_compensated_heading(0.0, 0.0, 5.0, 0.0, 0.0, has_imu=False) is None


def test_heading_no_imu_is_plain_atan2():
    # mx=1, my=0 -> heading 0; mx=0, my=1 -> atan2(-1, 0) = -pi/2.
    assert tilt_compensated_heading(1.0, 0.0, 0.0, 0.0, 0.0, has_imu=False) == pytest.approx(0.0)
    assert tilt_compensated_heading(0.0, 1.0, 0.0, 0.0, 0.0, has_imu=False) == pytest.approx(-math.pi / 2)


def test_heading_level_imu_matches_plain_atan2():
    # With zero roll/pitch the tilt compensation is the identity projection,
    # so the IMU path must agree with the no-IMU path.
    no_imu = tilt_compensated_heading(0.6, -0.4, 0.9, 0.0, 0.0, has_imu=False)
    level_imu = tilt_compensated_heading(0.6, -0.4, 0.9, 0.0, 0.0, has_imu=True)
    assert level_imu == pytest.approx(no_imu)


def test_heading_none_when_tilt_projection_collapses():
    # roll=0, pitch=pi/2 makes xh=mz and yh=my; with my=mz=0 the projected
    # horizontal field vanishes even though raw (mx, my) passed the first gate.
    assert tilt_compensated_heading(1.0, 0.0, 0.0, 0.0, math.pi / 2, has_imu=True) is None


def test_heading_tilt_changes_result():
    # A non-zero pitch must actually alter the heading (regression guard that
    # the IMU branch is wired, not a no-op).
    flat = tilt_compensated_heading(0.5, 0.5, 0.3, 0.0, 0.0, has_imu=True)
    pitched = tilt_compensated_heading(0.5, 0.5, 0.3, 0.0, 0.4, has_imu=True)
    assert flat != pytest.approx(pitched)
    assert math.isfinite(pitched)


# --------------------------------------------------------------------------- #
# camera_to_body_pose
# --------------------------------------------------------------------------- #


def test_body_pose_identity_subtracts_mount_offset():
    # Identity orientation + zero gimbal pitch: the body quaternion is identity,
    # so the mount offset subtracts directly and attitude is zero.
    pose = camera_to_body_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0, (0.10, 0.0, -0.05))
    assert pose["position"] == {"x": 0.9, "y": 2.0, "z": 3.05}
    assert pose["attitude"] == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def test_body_pose_rounds_to_four_decimals():
    pose = camera_to_body_pose(1.0 / 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, (0.0, 0.0, 0.0))
    # 0.33333... rounds to 0.3333; structure must always carry rounded floats.
    assert pose["position"]["x"] == 0.3333


def test_body_pose_recovers_camera_yaw_with_zero_gimbal_pitch():
    # Yaw-only camera quaternion, zero gimbal pitch -> body yaw equals camera yaw.
    s = math.sqrt(0.5)  # +90deg about z
    pose = camera_to_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, s, s, 0.0, (0.0, 0.0, 0.0))
    assert pose["attitude"]["yaw"] == pytest.approx(math.pi / 2, abs=1e-4)
    assert pose["attitude"]["roll"] == pytest.approx(0.0, abs=1e-4)
    assert pose["attitude"]["pitch"] == pytest.approx(0.0, abs=1e-4)


def test_body_pose_gimbal_pitch_undone_in_body_attitude():
    # Identity camera quaternion with the gimbal pitched by +30deg: the body
    # frame is the camera with the gimbal rotation removed, q_body = R_y(-pitch),
    # so the recovered body pitch is -30deg. Regression guard on the quaternion
    # composition sign.
    pose = camera_to_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, math.radians(30.0), (0.10, 0.0, -0.05))
    assert pose["attitude"]["pitch"] == pytest.approx(math.radians(-30.0), abs=1e-3)
    for key in ("x", "y", "z"):
        assert math.isfinite(pose["position"][key])
