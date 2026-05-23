#!/usr/bin/env python3
"""
Drone State Publisher for NOMAD.

Runs INSIDE the Isaac ROS Docker container. Polls the NOMAD Edge Core
HTTP API for drone telemetry (GPS, heading, altitude, servo angle) and
publishes ROS 2 topics that the target_localizer node expects.

This bridges the gap between Edge Core's MAVLink interface (which runs
natively on the host) and the ROS 2 nodes inside the container that
expect MAVROS-style topics.

Published topics:
  - /mavros/global_position/global   (sensor_msgs/NavSatFix)
  - /mavros/global_position/compass_hdg (std_msgs/Float64)
  - /mavros/local_position/pose      (geometry_msgs/PoseStamped)
  - /servo/angle                     (std_msgs/Float64)

Usage (inside Isaac ROS container):
    python3 drone_state_publisher.py --host localhost --port 8000
    python3 drone_state_publisher.py --host localhost --port 8000 --rate 10.0
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import threading
from http.client import HTTPConnection, HTTPException

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("drone_state_publisher")


class DroneStatePublisher(Node):
    """Polls Edge Core HTTP and publishes drone state as ROS 2 topics."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8000,
        poll_rate_hz: float = 10.0,
    ):
        super().__init__("drone_state_publisher")

        self._base_url = f"http://{host}:{port}"
        # Persistent keep-alive connection: urllib.request.urlopen does NOT
        # pool sockets, so polling at 10 Hz across two endpoints would burn
        # 20 TCP handshakes/sec and leave thousands of sockets in TIME_WAIT.
        # http.client.HTTPConnection reuses the same TCP socket.
        self._http_host = host
        self._http_port = port
        self._http_conn = HTTPConnection(host, port, timeout=0.5)
        self._http_lock = threading.Lock()
        # Edge Core requires X-API-Key when NOMAD_API_KEY is set and
        # NOMAD_ALLOW_INSECURE_REMOTE is false. Inside the Isaac container
        # this hits localhost, but the auth check still applies.
        self._api_key = os.environ.get("NOMAD_API_KEY", "").strip()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._gps_pub = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", sensor_qos
        )
        self._hdg_pub = self.create_publisher(
            Float64, "/mavros/global_position/compass_hdg", sensor_qos
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", sensor_qos
        )
        self._servo_pub = self.create_publisher(
            Float64, "/servo/angle", sensor_qos
        )

        poll_period = 1.0 / max(poll_rate_hz, 0.1)
        self.create_timer(poll_period, self._poll_and_publish)

        self._connected = False
        self.get_logger().info(
            f"Drone state publisher started: {poll_rate_hz} Hz -> {self._base_url}"
        )

    def _http_get_json(self, path: str, timeout: float = 0.5):
        """GET JSON from Edge Core over a pooled keep-alive connection.

        Returns dict or None on failure. On any error we close and lazily
        reopen the connection on the next call so a stuck socket cannot
        permanently wedge the publisher.
        """
        with self._http_lock:
            try:
                self._http_conn.timeout = timeout
                headers = {"Connection": "keep-alive"}
                if self._api_key:
                    headers["X-API-Key"] = self._api_key
                self._http_conn.request("GET", path, headers=headers)
                resp = self._http_conn.getresponse()
                body = resp.read()
                if resp.status != 200:
                    return None
                return json.loads(body.decode("utf-8"))
            except (HTTPException, OSError, ValueError) as e:
                self.get_logger().debug(f"HTTP error ({path}): {e}")
                try:
                    self._http_conn.close()
                except Exception:
                    pass
                self._http_conn = HTTPConnection(
                    self._http_host, self._http_port, timeout=timeout
                )
                return None

    def _poll_and_publish(self) -> None:
        """Poll Edge Core and publish all drone state topics."""
        now = self.get_clock().now().to_msg()

        # Poll system state (GPS, heading, attitude)
        state = self._http_get_json("/status")
        if state:
            if not self._connected:
                self.get_logger().info("Connected to Edge Core")
                self._connected = True

            # Publish GPS
            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = "gps"
            lat = state.get("gps_lat")
            lon = state.get("gps_lon")
            alt = state.get("gps_alt")
            if lat is not None and lon is not None:
                gps_msg.latitude = float(lat)
                gps_msg.longitude = float(lon)
                gps_msg.altitude = float(alt) if alt is not None else 0.0
                gps_msg.status.status = NavSatStatus.STATUS_FIX if state.get("gps_fix") else NavSatStatus.STATUS_NO_FIX
                gps_msg.status.service = NavSatStatus.SERVICE_GPS
                self._gps_pub.publish(gps_msg)

            # Publish heading
            heading = state.get("heading_deg")
            if heading is not None:
                hdg_msg = Float64()
                hdg_msg.data = float(heading)
                self._hdg_pub.publish(hdg_msg)

            # Publish local pose (using altitude as Z for AGL estimate).
            # Prefer alt_agl_m (rangefinder/baro AGL from MAVLink) over gps_alt
            # (WGS84 MSL, useless without a GPS fix and unsuitable as AGL).
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = "map"
            agl = state.get("alt_agl_m")
            if agl is None:
                agl = alt  # fallback to whatever altitude the status reports
            pose_msg.pose.position.z = float(agl) if agl is not None else 0.0
            # Convert heading to quaternion (yaw only, in ENU: Z-up)
            if heading is not None:
                yaw_rad = math.radians(float(heading))
                pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
                pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
            else:
                pose_msg.pose.orientation.w = 1.0
            self._pose_pub.publish(pose_msg)

        elif self._connected:
            self.get_logger().warn("Lost connection to Edge Core")
            self._connected = False

        # Poll servo angle
        servo_data = self._http_get_json("/api/servo/camera/tilt")
        if servo_data:
            servo_msg = Float64()
            # Get feedback angle if available, else commanded angle
            if servo_data.get("feedback_angle") is not None:
                angle = float(servo_data["feedback_angle"])
            else:
                angle = float(servo_data.get("angle", 90.0))
            # Convert servo angle (0-180, 90=level) to pitch degrees
            # target_localizer expects: positive = tilt up, negative = tilt down
            servo_msg.data = angle - 90.0
            self._servo_pub.publish(servo_msg)


def main():
    parser = argparse.ArgumentParser(description="NOMAD Drone State Publisher")
    parser.add_argument("--host", default="localhost", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Poll rate in Hz (default: 10)")
    args = parser.parse_args()

    rclpy.init()
    node = DroneStatePublisher(
        host=args.host,
        port=args.port,
        poll_rate_hz=args.rate,
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
