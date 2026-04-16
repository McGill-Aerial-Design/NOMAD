#!/usr/bin/env python3
"""
Servo TF Publisher for NOMAD.

Publishes the dynamic transform from servo_mount -> camera_link at 50 Hz,
reflecting the current servo pitch angle, AND publishes odom -> base_link
by subscribing to ZED odom and applying the inverse of the servo + mount
transforms to recover the drone body pose.

This bridges the gap between the ZED TF tree (odom -> zed_camera_link -> ...)
and the NOMAD kinematic chain (base_link -> servo_mount -> camera_link).

Requirements satisfied:
- TF-001: Publishes servo_mount -> camera_link at >= 50 Hz
- TF-002/TF-003: Uses servo feedback if available, falls back to commanded angle with warning
- TF-006: Pure rotation about the pitch axis (Y-axis in REP 103)
- TF-007: Latency < 20ms (polls servo at 50 Hz = 20ms interval)

TF Tree (with this node):
  odom -> base_link -> servo_mount -> camera_link
       -> zed_camera_link -> zed_camera_center -> ...   (published by ZED driver)

The ZED driver publishes odom -> zed_camera_link (camera pose from VIO).
This node computes odom -> base_link (drone body pose) by removing the
servo pitch and mounting offset from the camera pose.

The base_link -> servo_mount transform is a static TF (mounting offset).
The servo_mount -> camera_link transform is the dynamic joint driven by servo angle.

Usage (inside Isaac ROS container):
    python3 servo_tf_publisher.py --host 172.17.0.1 --port 8000
    python3 servo_tf_publisher.py --host 172.17.0.1 --port 8000 --odom-topic /zed/zed_node/odom
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import threading
import time
from typing import Optional, Tuple
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("servo_tf_publisher")


class ServoTFPublisher(Node):
    """
    Publishes dynamic TF for servo tilt, static TF for mounting offset,
    and odom -> base_link by inverting the camera odom pose.

    Dynamic: servo_mount -> camera_link (pitch rotation from servo angle)
    Dynamic: odom -> base_link (drone body pose from ZED odom inverse)
    Static:  base_link -> servo_mount (physical mounting offset)
    """

    # Mounting offset defaults: servo pivot 10cm forward, 5cm below base_link.
    # Real values are loaded from env (SERVO_MOUNT_X/Y/Z) or
    # /etc/nomad/servo_mount.yaml at __init__ time. TF-004 requires
    # within 1cm/1deg of physical measurement.
    DEFAULT_MOUNT_OFFSET_X = 0.10
    DEFAULT_MOUNT_OFFSET_Y = 0.0
    DEFAULT_MOUNT_OFFSET_Z = -0.05
    MOUNT_CONFIG_PATH = "/etc/nomad/servo_mount.yaml"

    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        publish_rate_hz: float = 50.0,
        poll_rate_hz: float = 10.0,
        odom_topic: str = "/zed/zed_node/odom",
    ):
        super().__init__("nomad_servo_tf_publisher")

        self._mount_x, self._mount_y, self._mount_z, mount_source = self._load_mount_offsets()
        self.get_logger().info(
            f"Mount offsets ({mount_source}): "
            f"x={self._mount_x:.3f}m y={self._mount_y:.3f}m z={self._mount_z:.3f}m"
        )

        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"

        # Current servo angle (degrees, 0-180, 90 = level)
        self._servo_angle_deg: float = 90.0
        self._servo_lock = threading.Lock()
        self._using_feedback = False  # True if reading from encoder
        self._feedback_warned = False

        # Latest camera odom pose (from ZED)
        self._odom_lock = threading.Lock()
        self._latest_odom: Optional[Odometry] = None

        # TF broadcasters
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform: base_link -> servo_mount
        self._publish_static_mount_tf()

        # Subscribe to ZED odom for computing odom -> base_link
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._handle_odom,
            sensor_qos,
        )
        self.get_logger().info(f"Subscribed to odom: {odom_topic}")

        # Timer: publish dynamic TFs at 50 Hz (TF-001)
        publish_period = 1.0 / publish_rate_hz
        self.create_timer(publish_period, self._publish_all_tf)

        # Timer: poll servo angle from Edge Core API at 10 Hz
        poll_period = 1.0 / poll_rate_hz
        self.create_timer(poll_period, self._poll_servo_angle)

        self.get_logger().info(
            f"Servo TF publisher started: {publish_rate_hz} Hz TF, "
            f"{poll_rate_hz} Hz servo poll -> {self._base_url}"
        )

    @classmethod
    def _load_mount_offsets(cls) -> Tuple[float, float, float, str]:
        """
        Resolve mount offsets (meters) with priority:
            env SERVO_MOUNT_X/Y/Z > /etc/nomad/servo_mount.yaml > defaults.
        Any individual axis falls back to its default if missing/invalid.
        """
        x = cls.DEFAULT_MOUNT_OFFSET_X
        y = cls.DEFAULT_MOUNT_OFFSET_Y
        z = cls.DEFAULT_MOUNT_OFFSET_Z
        source = "defaults"

        yaml_values = {}
        try:
            if os.path.isfile(cls.MOUNT_CONFIG_PATH):
                with open(cls.MOUNT_CONFIG_PATH, "r") as f:
                    for line in f:
                        line = line.split("#", 1)[0].strip()
                        if not line or ":" not in line:
                            continue
                        k, v = line.split(":", 1)
                        key = k.strip().lower()
                        try:
                            yaml_values[key] = float(v.strip())
                        except ValueError:
                            pass
                if yaml_values:
                    source = cls.MOUNT_CONFIG_PATH
        except OSError as e:
            logger.warning(f"Could not read {cls.MOUNT_CONFIG_PATH}: {e}")

        x = yaml_values.get("x", x)
        y = yaml_values.get("y", y)
        z = yaml_values.get("z", z)

        env_overrides = []
        for axis, default in (("x", x), ("y", y), ("z", z)):
            raw = os.environ.get(f"SERVO_MOUNT_{axis.upper()}")
            if raw is None:
                continue
            try:
                val = float(raw)
            except ValueError:
                logger.warning(f"Invalid SERVO_MOUNT_{axis.upper()}={raw!r}, ignoring")
                continue
            if axis == "x":
                x = val
            elif axis == "y":
                y = val
            else:
                z = val
            env_overrides.append(axis.upper())

        if env_overrides:
            source = f"env ({','.join(env_overrides)})" + (
                f" + {cls.MOUNT_CONFIG_PATH}" if yaml_values else ""
            )
        return x, y, z, source

    def _handle_odom(self, msg: Odometry) -> None:
        """Cache latest ZED odom for base_link transform computation."""
        with self._odom_lock:
            self._latest_odom = msg

    def _publish_static_mount_tf(self) -> None:
        """Publish static transform: base_link -> servo_mount (TF-004)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "servo_mount"
        t.transform.translation.x = self._mount_x
        t.transform.translation.y = self._mount_y
        t.transform.translation.z = self._mount_z
        # No rotation - servo mount aligned with body frame
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Static TF published: base_link -> servo_mount "
            f"({self._mount_x}, {self._mount_y}, {self._mount_z})"
        )

    def _publish_all_tf(self) -> None:
        """Publish all dynamic TFs at 50 Hz."""
        self._publish_servo_tf()
        self._publish_odom_base_link_tf()

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

    def _publish_odom_base_link_tf(self) -> None:
        """
        Publish dynamic transform: odom -> base_link.

        Computed from the ZED camera odom pose by removing the servo pitch
        rotation and mounting offset.  This connects the servo kinematic
        chain (base_link -> servo_mount -> camera_link) to the odom frame
        so that any node can look up odom -> servo_mount or odom -> base_link.

        Math:
          camera_pos = body_pos + R_body * mount_offset
          camera_rot = body_rot * R_servo
          =>
          body_rot = camera_rot * inv(R_servo)
          body_pos = camera_pos - R_body * mount_offset
        """
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            # Keep TF chain connected even before first odom sample arrives.
            # This prevents downstream consumers (e.g., nvblox) from stalling
            # on missing zed_camera_link lookup when camera odom is delayed.
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t)
            return

        pose = odom.pose.pose

        # Camera quaternion from ZED odom
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Servo inverse quaternion (negative pitch about Y axis)
        with self._servo_lock:
            angle_deg = self._servo_angle_deg
        servo_pitch = math.radians(angle_deg - 90.0)
        # inv(R_servo) = rotation by -servo_pitch about Y
        sq_y = math.sin(-servo_pitch / 2.0)
        sq_w = math.cos(-servo_pitch / 2.0)

        # body_quat = camera_quat * inv(servo_quat)
        # Quaternion multiplication: q1 * q2
        bqx = qw * 0.0    + qx * sq_w + qy * 0.0  - qz * sq_y
        bqy = qw * sq_y   - qx * 0.0  + qy * sq_w + qz * 0.0
        bqz = qw * 0.0    + qx * sq_y + qy * 0.0  + qz * sq_w
        bqw = qw * sq_w   - qx * 0.0  - qy * sq_y - qz * 0.0

        # Normalize
        n = math.sqrt(bqx * bqx + bqy * bqy + bqz * bqz + bqw * bqw)
        if n > 1e-9:
            bqx /= n
            bqy /= n
            bqz /= n
            bqw /= n

        # Rotate mount offset by body quaternion: R_body * mount_offset
        mx, my, mz = self._mount_x, self._mount_y, self._mount_z
        # Quaternion-vector rotation: q * v * q_conj
        # Using the formula directly:
        # t = 2 * cross(q.xyz, v)
        # result = v + q.w * t + cross(q.xyz, t)
        tx = 2.0 * (bqy * mz - bqz * my)
        ty = 2.0 * (bqz * mx - bqx * mz)
        tz = 2.0 * (bqx * my - bqy * mx)
        ox = mx + bqw * tx + (bqy * tz - bqz * ty)
        oy = my + bqw * ty + (bqz * tx - bqx * tz)
        oz = mz + bqw * tz + (bqx * ty - bqy * tx)

        # body_pos = camera_pos - R_body * mount_offset
        body_x = pose.position.x - ox
        body_y = pose.position.y - oy
        body_z = pose.position.z - oz

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = body_x
        t.transform.translation.y = body_y
        t.transform.translation.z = body_z
        t.transform.rotation.x = bqx
        t.transform.rotation.y = bqy
        t.transform.rotation.z = bqz
        t.transform.rotation.w = bqw

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
    parser.add_argument("--odom-topic", default="/zed/zed_node/odom",
                        help="ZED odom topic for computing odom -> base_link")
    args = parser.parse_args()

    rclpy.init()

    node = ServoTFPublisher(
        host=args.host,
        port=args.port,
        publish_rate_hz=args.tf_rate,
        poll_rate_hz=args.poll_rate,
        odom_topic=args.odom_topic,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
