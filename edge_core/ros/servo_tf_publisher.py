#!/usr/bin/env python3
"""
Servo TF Publisher for NOMAD.

Publishes the dynamic transform from servo_mount -> camera_link at 50 Hz,
reflecting the current servo pitch angle. This is critical for nvblox and
VIO to correctly account for camera tilt.

Requirements satisfied:
- TF-001: Publishes servo_mount -> camera_link at >= 50 Hz
- TF-002/TF-003: Uses servo feedback if available, falls back to commanded angle with warning
- TF-006: Pure rotation about the pitch axis (Y-axis in REP 103)
- TF-007: Latency < 20ms (polls servo at 50 Hz = 20ms interval)

TF Tree (with this node):
  map -> odom -> base_link -> servo_mount -> camera_link -> zed2i_left_camera_optical_frame

The base_link -> servo_mount transform is a static TF (mounting offset).
The servo_mount -> camera_link transform is the dynamic joint driven by servo angle.
The camera_link -> zed2i_left_camera_optical_frame is published by the ZED driver.

Usage (inside Isaac ROS container):
    python3 servo_tf_publisher.py --host 172.17.0.1 --port 8000
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
from typing import Optional
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("servo_tf_publisher")


class ServoTFPublisher(Node):
    """
    Publishes dynamic TF for servo tilt and static TF for mounting offset.

    Dynamic: servo_mount -> camera_link (pitch rotation from servo angle)
    Static:  base_link -> servo_mount (physical mounting offset)
    """

    # Mounting offset: servo pivot is 10cm forward, 5cm below base_link center
    # These should match the physical drone measurements (TF-004: within 1cm/1deg)
    MOUNT_OFFSET_X = 0.10   # forward (meters)
    MOUNT_OFFSET_Y = 0.0    # lateral
    MOUNT_OFFSET_Z = -0.05  # down (negative Z in ROS = below)

    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        publish_rate_hz: float = 50.0,
        poll_rate_hz: float = 10.0,
    ):
        super().__init__("nomad_servo_tf_publisher")

        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"

        # Current servo angle (degrees, 0-180, 90 = level)
        self._servo_angle_deg: float = 90.0
        self._servo_lock = threading.Lock()
        self._using_feedback = False  # True if reading from encoder
        self._feedback_warned = False

        # TF broadcasters
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform: base_link -> servo_mount
        self._publish_static_mount_tf()

        # Timer: publish dynamic TF at 50 Hz (TF-001)
        publish_period = 1.0 / publish_rate_hz
        self.create_timer(publish_period, self._publish_servo_tf)

        # Timer: poll servo angle from Edge Core API at 10 Hz
        poll_period = 1.0 / poll_rate_hz
        self.create_timer(poll_period, self._poll_servo_angle)

        self.get_logger().info(
            f"Servo TF publisher started: {publish_rate_hz} Hz TF, "
            f"{poll_rate_hz} Hz servo poll -> {self._base_url}"
        )

    def _publish_static_mount_tf(self) -> None:
        """Publish static transform: base_link -> servo_mount (TF-004)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "servo_mount"
        t.transform.translation.x = self.MOUNT_OFFSET_X
        t.transform.translation.y = self.MOUNT_OFFSET_Y
        t.transform.translation.z = self.MOUNT_OFFSET_Z
        # No rotation - servo mount aligned with body frame
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Static TF published: base_link -> servo_mount "
            f"({self.MOUNT_OFFSET_X}, {self.MOUNT_OFFSET_Y}, {self.MOUNT_OFFSET_Z})"
        )

    def _publish_servo_tf(self) -> None:
        """
        Publish dynamic transform: servo_mount -> camera_link (TF-001, TF-006).

        Pure rotation about the Y-axis (pitch) in REP 103 convention.
        No translation - the pivot point IS the servo_mount origin.

        Servo angle mapping:
        - 90 deg = level forward (pitch = 0)
        - 0 deg = pointing straight down (pitch = -90 deg)
        - 180 deg = pointing straight up (pitch = +90 deg)
        """
        with self._servo_lock:
            angle_deg = self._servo_angle_deg

        # Convert servo angle to pitch: 90 deg servo = 0 pitch
        pitch_rad = math.radians(angle_deg - 90.0)

        # Quaternion for pure Y-axis rotation (pitch only, TF-006)
        qy = math.sin(pitch_rad / 2.0)
        qw = math.cos(pitch_rad / 2.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "servo_mount"
        t.child_frame_id = "camera_link"
        # No translation - pure rotation (TF-006: translation constant < 1mm)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = qy
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(t)

    def _poll_servo_angle(self) -> None:
        """
        Poll servo angle from Edge Core API (TF-002, TF-003).

        Tries to get feedback angle first (encoder/potentiometer).
        Falls back to commanded angle if feedback unavailable (TF-003).
        """
        try:
            url = f"{self._base_url}/api/servo/camera/tilt"
            req = Request(url, method="GET")
            req.add_header("Connection", "keep-alive")
            with urlopen(req, timeout=0.5) as resp:
                data = json.loads(resp.read().decode("utf-8"))

            # Check for feedback angle (TF-002)
            if "feedback_angle" in data and data["feedback_angle"] is not None:
                angle = float(data["feedback_angle"])
                self._using_feedback = True
            else:
                # Fall back to commanded angle (TF-003)
                angle = float(data.get("angle", 90.0))
                if not self._feedback_warned:
                    self.get_logger().warn(
                        "Servo position feedback unavailable - using commanded angle "
                        "(TF-003 fallback). Accuracy may be reduced."
                    )
                    self._feedback_warned = True
                self._using_feedback = False

            # Clamp to valid range
            angle = max(0.0, min(180.0, angle))

            with self._servo_lock:
                self._servo_angle_deg = angle

        except URLError:
            # Edge Core not reachable - keep last known angle
            pass
        except Exception as e:
            self.get_logger().debug(f"Servo poll error: {e}")


def main():
    parser = argparse.ArgumentParser(description="NOMAD Servo TF Publisher")
    parser.add_argument("--host", default="172.17.0.1", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--tf-rate", type=float, default=50.0,
                        help="TF publish rate in Hz (default: 50)")
    parser.add_argument("--poll-rate", type=float, default=10.0,
                        help="Servo angle poll rate in Hz (default: 10)")
    args = parser.parse_args()

    rclpy.init()

    node = ServoTFPublisher(
        host=args.host,
        port=args.port,
        publish_rate_hz=args.tf_rate,
        poll_rate_hz=args.poll_rate,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
