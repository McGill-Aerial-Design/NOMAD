#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Record ZED odometry pose and validate roll/pitch drift for stationary checks.

Usage:
  python3 scripts/dev/zed_pose_stability_check.py --duration 20

Notes:
- Subscribes to /zed/zed_node/odom by default.
- Writes CSV with raw pose samples.
- Computes mean roll/pitch at beginning and end windows, then compares drift.
"""

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_euler_deg(x: float, y: float, z: float, w: float):
    # Standard aerospace roll-pitch-yaw extraction.
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


@dataclass
class PoseSample:
    t_wall: float
    t_msg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    x: float
    y: float
    z: float


class PoseRecorder(Node):
    def __init__(self, topic: str):
        super().__init__("zed_pose_stability_check")
        self.samples: list[PoseSample] = []
        self._sub = self.create_subscription(Odometry, topic, self._cb, 20)

    def _cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp
        t_msg = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        p = msg.pose.pose.position
        self.samples.append(
            PoseSample(
                t_wall=time.time(),
                t_msg=t_msg,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                x=float(p.x),
                y=float(p.y),
                z=float(p.z),
            )
        )


def mean(values: list[float]) -> float:
    return sum(values) / max(1, len(values))


def write_csv(path: Path, samples: list[PoseSample]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_wall",
                "t_msg",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "x",
                "y",
                "z",
            ]
        )
        for s in samples:
            writer.writerow(
                [
                    f"{s.t_wall:.6f}",
                    f"{s.t_msg:.6f}",
                    f"{s.roll_deg:.6f}",
                    f"{s.pitch_deg:.6f}",
                    f"{s.yaw_deg:.6f}",
                    f"{s.x:.6f}",
                    f"{s.y:.6f}",
                    f"{s.z:.6f}",
                ]
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Check ZED pose drift stability")
    parser.add_argument("--topic", default="/zed/zed_node/odom")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--window-seconds", type=float, default=3.0)
    parser.add_argument("--threshold-deg", type=float, default=0.5)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.output) if args.output else Path(f"/tmp/zed_pose_stability_{ts}.csv")

    rclpy.init(args=None)
    node = PoseRecorder(args.topic)

    t_end = time.time() + max(1.0, args.duration)
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        print(f"ERROR: no samples on topic {args.topic}")
        return 3

    write_csv(out_path, samples)

    # Time-based windows for robust start/end means.
    t0 = samples[0].t_wall
    t1 = samples[-1].t_wall
    win = max(0.5, float(args.window_seconds))
    start_samples = [s for s in samples if s.t_wall <= t0 + win]
    end_samples = [s for s in samples if s.t_wall >= t1 - win]

    start_roll = mean([s.roll_deg for s in start_samples])
    start_pitch = mean([s.pitch_deg for s in start_samples])
    end_roll = mean([s.roll_deg for s in end_samples])
    end_pitch = mean([s.pitch_deg for s in end_samples])

    roll_delta = end_roll - start_roll
    pitch_delta = end_pitch - start_pitch

    threshold = float(args.threshold_deg)
    roll_ok = abs(roll_delta) <= threshold
    pitch_ok = abs(pitch_delta) <= threshold

    print("=== ZED Pose Stability Check ===")
    print(f"Topic: {args.topic}")
    print(f"Duration: {args.duration:.1f}s, Samples: {len(samples)}")
    print(f"CSV: {out_path}")
    print(f"Start roll/pitch: {start_roll:.3f} / {start_pitch:.3f} deg")
    print(f"End   roll/pitch: {end_roll:.3f} / {end_pitch:.3f} deg")
    print(f"Delta roll/pitch: {roll_delta:+.3f} / {pitch_delta:+.3f} deg")
    print(f"Threshold: +/-{threshold:.3f} deg")
    print(f"Result: roll_ok={roll_ok}, pitch_ok={pitch_ok}")

    return 0 if (roll_ok and pitch_ok) else 2


if __name__ == "__main__":
    sys.exit(main())
