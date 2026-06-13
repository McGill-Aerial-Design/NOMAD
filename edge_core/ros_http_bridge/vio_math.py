# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pure VIO-derivation math for the ROS-HTTP Bridge.

These helpers were extracted verbatim from the ROS callbacks in
:mod:`edge_core.ros_http_bridge.node` so the frame conversions and pose
arithmetic can be unit-tested without a ROS2 runtime (``node`` imports
``rclpy`` at module load and so is unreachable in the hardware-free env).
The node is now a thin adapter: it unpacks ROS messages and calls these
functions. Keep them dependency-free (``math`` + :mod:`coordinate_math`
only) so the test suite covers them on every platform.

Sign conventions are load-bearing — a flipped axis here steers the vehicle
the wrong way — so each conversion is named and individually tested.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from .coordinate_math import quat_to_euler


def position_variance(covariance: Sequence[float]) -> float:
    """Largest of the x/y/z position variances on a ROS pose covariance.

    ROS ``geometry_msgs/PoseWithCovariance`` carries a row-major 6x6 matrix
    over (x, y, z, roll, pitch, yaw); the position variances sit on the
    diagonal at indices 0, 7 and 14. Taking the max is a conservative
    "worst axis" estimate of positional uncertainty.
    """
    return max(covariance[0], covariance[7], covariance[14])


def vio_confidence(pos_var: float) -> float:
    """Map a position variance to a tracking confidence in ``[0, 1]``.

    ``confidence = 1 - 10 * pos_var``, clamped — so variance >= 0.1 m^2
    collapses confidence to zero. Mirrors the safety core's
    ``min_vio_confidence`` gate input (SR-VIO-01).
    """
    return max(0.0, min(1.0, 1.0 - pos_var * 10.0))


def flu_to_ned_vec3(fwd: float, left: float, up: float) -> tuple[float, float, float]:
    """REP-103 body vector (x-forward, y-left, z-up) -> NED (x-fwd, y-right, z-down).

    Used for both position and linear velocity: flip the y and z axes.
    """
    return (fwd, -left, -up)


def flu_to_ned_attitude(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """REP-103 roll/pitch/yaw -> NED roll/pitch/yaw (negate pitch and yaw)."""
    return (roll, -pitch, -yaw)


def tilt_compensated_heading(
    mag_x: float,
    mag_y: float,
    mag_z: float,
    imu_roll: float,
    imu_pitch: float,
    has_imu: bool,
) -> float | None:
    """Magnetometer heading (rad), tilt-compensated by IMU roll/pitch when available.

    Frame: REP-103 body (x-forward, y-left, z-up). Returns ``None`` when the
    reading is degenerate (negligible horizontal field, or a tilt projection
    that collapses to zero) so the caller skips the update rather than
    publishing a meaningless heading.
    """
    if math.sqrt(mag_x * mag_x + mag_y * mag_y) < 1e-9:
        return None

    if has_imu:
        cr, sr = math.cos(imu_roll), math.sin(imu_roll)
        cp, sp = math.cos(imu_pitch), math.sin(imu_pitch)
        xh = mag_x * cp + mag_z * sp
        yh = mag_x * sr * sp + mag_y * cr - mag_z * sr * cp
        if math.sqrt(xh * xh + yh * yh) < 1e-12:
            return None
        return math.atan2(-yh, xh)

    return math.atan2(-mag_y, mag_x)


def camera_to_body_pose(
    cam_x: float,
    cam_y: float,
    cam_z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    gimbal_pitch_rad: float,
    mount_offset: tuple[float, float, float],
) -> dict:
    """Drone-body pose from the gimbal-camera SLAM pose.

    The camera rides a pitch gimbal at ``mount_offset`` from the body origin.
    Compose the camera orientation with the inverse gimbal-pitch rotation to
    recover body attitude, then subtract the mount offset rotated into the
    world frame to recover the body position. Position and attitude are
    rounded to 4 decimals to match the wire payload the SLAM view consumes.
    """
    # Undo the gimbal pitch: q_body = q_cam * R_y(-pitch).
    sq_y = math.sin(-gimbal_pitch_rad / 2.0)
    sq_w = math.cos(-gimbal_pitch_rad / 2.0)

    bqx = qw * 0.0 + qx * sq_w + qy * 0.0 - qz * sq_y
    bqy = qw * sq_y - qx * 0.0 + qy * sq_w + qz * 0.0
    bqz = qw * 0.0 + qx * sq_y + qy * 0.0 + qz * sq_w
    bqw = qw * sq_w - qx * 0.0 - qy * sq_y - qz * 0.0

    n = math.sqrt(bqx**2 + bqy**2 + bqz**2 + bqw**2)
    if n > 1e-9:
        bqx /= n
        bqy /= n
        bqz /= n
        bqw /= n

    # Rotate the mount offset by the body quaternion (v' = q * v * q^-1).
    mx, my, mz = mount_offset
    tx = 2.0 * (bqy * mz - bqz * my)
    ty = 2.0 * (bqz * mx - bqx * mz)
    tz = 2.0 * (bqx * my - bqy * mx)
    ox = mx + bqw * tx + (bqy * tz - bqz * ty)
    oy = my + bqw * ty + (bqz * tx - bqx * tz)
    oz = mz + bqw * tz + (bqx * ty - bqy * tx)

    body_roll, body_pitch, body_yaw = quat_to_euler(bqx, bqy, bqz, bqw)

    return {
        "position": {"x": round(cam_x - ox, 4), "y": round(cam_y - oy, 4), "z": round(cam_z - oz, 4)},
        "attitude": {"roll": round(body_roll, 4), "pitch": round(body_pitch, 4), "yaw": round(body_yaw, 4)},
    }
