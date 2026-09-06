# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Synthetic scene math for the hardware-free ROS2 sim publisher.

Holds the trajectory, voxel-grid, odometry-covariance, and quaternion helpers
used by ``zed_sim_publisher`` so the publisher file stays small and the math
is independently readable.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# ---------------------------------------------------------------------------
# Covariance design note:
#   position_variance() = max(cov[0], cov[7], cov[14])  (6×6 row-major diag)
#   vio_confidence()    = max(0.0, min(1.0, 1.0 - pos_var * 10.0))
#
#   Setting diagonal elements at indices 0, 7, 14 to 0.001 gives:
#     pos_var   = 0.001
#     confidence = 1.0 - 0.001 * 10.0 = 0.99   (> 0.5 ✓)
#     pos_var <= 0.1                             (VIO healthy ✓)
# ---------------------------------------------------------------------------
_ODOM_COV_DIAG_VALUE: float = 0.001  # m² — satisfies confidence > 0.5

ODOM_COVARIANCE: list[float] = [0.0] * 36
for _idx in (0, 7, 14):
    ODOM_COVARIANCE[_idx] = _ODOM_COV_DIAG_VALUE

# Small but non-zero orientation variance (indices 21, 28, 35).
for _idx in (21, 28, 35):
    ODOM_COVARIANCE[_idx] = 0.01

# Publish a 4×5 grid of voxels (20 cubes) for the nvblox CUBE_LIST marker.
VOXEL_GRID_ROWS: int = 4
VOXEL_GRID_COLS: int = 5
VOXEL_SIZE: float = 0.05  # metres — matches scale.x/y/z requirement

# Slow circular trajectory parameters.
TRAJ_RADIUS: float = 1.0  # metres
TRAJ_OMEGA: float = 0.2  # rad/s — full circle in ~31 s
TRAJ_ALT: float = 1.5  # metres altitude (z in ROS/FLU frame = up)

# Servo sine wave parameters (0–180 °, period ~20 s).
SERVO_OMEGA: float = math.pi / 10.0  # rad/s → 20-s period
SERVO_MID: float = 90.0
SERVO_AMP: float = 80.0

# A plausible horizontal magnetic-field magnitude (µT).
MAG_FIELD_X: float = 20.0e-6  # T  (pointing forward)
MAG_FIELD_Y: float = -5.0e-6  # T
MAG_FIELD_Z: float = -45.0e-6  # T  (downward component, northern hemisphere)


def build_voxel_grid() -> tuple[list[Point], list[ColorRGBA]]:
    """Build a static 4×5 grid of voxel positions and per-voxel colors."""
    points: list[Point] = []
    colors: list[ColorRGBA] = []

    for row in range(VOXEL_GRID_ROWS):
        for col in range(VOXEL_GRID_COLS):
            point = Point()
            point.x = float(col) * VOXEL_SIZE * 2.0
            point.y = float(row) * VOXEL_SIZE * 2.0
            point.z = 0.0
            points.append(point)

            color = ColorRGBA()
            color.r = float(col) / max(1, VOXEL_GRID_COLS - 1)
            color.g = float(row) / max(1, VOXEL_GRID_ROWS - 1)
            color.b = 0.5
            color.a = 1.0
            colors.append(color)

    return points, colors


def euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (rad) to a unit quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * cp * cy

    return qx, qy, qz, qw
