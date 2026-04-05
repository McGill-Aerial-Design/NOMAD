#!/usr/bin/env python3
"""
Compare raw ZED odom orientation against Edge Core /api/vio/pose orientation.

Runs both streams in the same window and reports roll/pitch/yaw deltas.
"""

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass
from typing import Optional

import requests
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_euler_deg(x: float, y: float, z: float, w: float):
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


def wrap_deg(delta: float) -> float:
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


@dataclass
class OdomSample:
    t_wall: float
    roll: float
    pitch: float
    yaw: float


class OdomTap(Node):
    def __init__(self, topic: str):
        super().__init__("odom_pose_compare")
        self.latest: Optional[OdomSample] = None
        self.create_subscription(Odometry, topic, self._cb, 20)

    def _cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        r, p, y = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        self.latest = OdomSample(time.time(), r, p, y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--api", default="http://100.85.121.98:8000")
    ap.add_argument("--api-key", required=True)
    ap.add_argument("--topic", default="/zed/zed_node/odom")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--poll-hz", type=float, default=15.0)
    args = ap.parse_args()

    headers = {"X-API-Key": args.api_key}
    period = 1.0 / max(1.0, float(args.poll_hz))

    rclpy.init(args=None)
    tap = OdomTap(args.topic)

    roll_d = []
    pitch_d = []
    yaw_d = []
    samples = 0

    t_end = time.time() + max(2.0, args.duration)
    while time.time() < t_end:
        t0 = time.time()
        rclpy.spin_once(tap, timeout_sec=0.05)

        od = tap.latest
        if od is not None:
            try:
                resp = requests.get(
                    f"{args.api.rstrip('/')}/api/vio/pose",
                    headers=headers,
                    timeout=1.0,
                )
                if resp.status_code == 200:
                    data = resp.json()
                    if isinstance(data, dict) and "roll" in data and "pitch" in data and "yaw" in data:
                        api_roll = float(data.get("roll", 0.0)) * 180.0 / math.pi
                        api_pitch = float(data.get("pitch", 0.0)) * 180.0 / math.pi
                        api_yaw = float(data.get("yaw", 0.0)) * 180.0 / math.pi

                        roll_d.append(wrap_deg(api_roll - od.roll))
                        pitch_d.append(wrap_deg(api_pitch - od.pitch))
                        yaw_d.append(wrap_deg(api_yaw - od.yaw))
                        samples += 1
            except Exception:
                pass

        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

    tap.destroy_node()
    rclpy.shutdown()

    if samples == 0:
        print("ERROR: No comparison samples captured")
        return 3

    def stats(vals):
        abs_vals = [abs(v) for v in vals]
        return {
            "mean": statistics.mean(vals),
            "mean_abs": statistics.mean(abs_vals),
            "max_abs": max(abs_vals),
            "p95_abs": statistics.quantiles(abs_vals, n=20)[18] if len(abs_vals) >= 20 else max(abs_vals),
        }

    out = {
        "samples": samples,
        "roll_delta_deg": stats(roll_d),
        "pitch_delta_deg": stats(pitch_d),
        "yaw_delta_deg": stats(yaw_d),
    }
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
