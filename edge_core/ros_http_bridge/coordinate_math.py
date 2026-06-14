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
