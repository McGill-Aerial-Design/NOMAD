# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Coordinate frame and attitude math utilities for the ROS-HTTP Bridge."""

from __future__ import annotations

import math


def quat_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_angle_rad(angle: float) -> float:
    """Normalize an angle in radians to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_ned_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert ROS optical-frame quaternion attitude to NED roll, pitch, and yaw."""
    # Quaternion -> rotation matrix in ROS optical basis
    r = [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]

    # Basis transform: optical (x-right, y-down, z-forward) -> NED (x-forward, y-right, z-down)
    b = (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )

    # Change basis for rotation matrix: R_ned = B * R_optical * B^T
    br = [[sum(b[i][k] * r[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    r_ned = [[sum(br[i][k] * b[j][k] for k in range(3)) for j in range(3)] for i in range(3)]

    roll = math.atan2(r_ned[2][1], r_ned[2][2])
    sinp = max(-1.0, min(1.0, -r_ned[2][0]))
    pitch = math.asin(sinp)
    yaw = math.atan2(r_ned[1][0], r_ned[0][0])
    return roll, pitch, yaw
