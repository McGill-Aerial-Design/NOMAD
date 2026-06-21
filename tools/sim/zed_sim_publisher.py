# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Synthetic ZED 2i + nvblox + nav2 topic publisher for hardware-free ROS2 testing.

This module simulates the exact set of ROS2 topics that the NOMAD ROS-HTTP
bridge (``edge_core.ros_http_bridge.node.ROSHTTPBridge``) subscribes to, so
the bridge can be exercised in CI without a Jetson, ZED camera, or GPU.

Topics published (all configurable via constructor kwargs):

  ``nav_msgs/Odometry``              → /zed/zed_node/odom
  ``sensor_msgs/Imu``                → /zed/zed_node/imu/data
  ``sensor_msgs/MagneticField``      → /zed/zed_node/imu/mag
  ``geometry_msgs/Twist``            → /cmd_vel
  ``visualization_msgs/Marker``      → /nvblox_node/color_layer_marker
  ``std_msgs/Float32``               → /nomad/servo/nozzle_angle
  ``sensor_msgs/Image``              → /zed/zed_node/rgb/color/rect/image
                                       (only when enable_image=True)

The QoS profile on all publishers matches the bridge's subscriber:
``BEST_EFFORT`` reliability, ``KEEP_LAST`` history, depth 1.

The class is directly instantiable by pytest (no argparse side-effects in
``__init__``), as required by the Wave-2 integration test in ``tests/ros/``.
"""

from __future__ import annotations

import argparse
import math

import rclpy
import rclpy.node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker

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

# Build a 36-element all-zero covariance, then set x/y/z diagonal positions.
_ODOM_COVARIANCE: list[float] = [0.0] * 36
for _idx in (0, 7, 14):
    _ODOM_COVARIANCE[_idx] = _ODOM_COV_DIAG_VALUE

# Small but non-zero orientation variance (indices 21, 28, 35).
for _idx in (21, 28, 35):
    _ODOM_COVARIANCE[_idx] = 0.01

# Publish a 4×5 grid of voxels (20 cubes) for the nvblox CUBE_LIST marker.
_VOXEL_GRID_ROWS: int = 4
_VOXEL_GRID_COLS: int = 5
_VOXEL_SIZE: float = 0.05  # metres — matches scale.x/y/z requirement

# Slow circular trajectory parameters.
_TRAJ_RADIUS: float = 1.0  # metres
_TRAJ_OMEGA: float = 0.2  # rad/s — full circle in ~31 s
_TRAJ_ALT: float = 1.5  # metres altitude (z in ROS/FLU frame = up)

# Servo sine wave parameters (0–180 °, period ~20 s).
_SERVO_OMEGA: float = math.pi / 10.0  # rad/s → 20-s period
_SERVO_MID: float = 90.0
_SERVO_AMP: float = 80.0

# A plausible horizontal magnetic-field magnitude (µT).
_MAG_FIELD_X: float = 20.0e-6  # T  (pointing forward)
_MAG_FIELD_Y: float = -5.0e-6  # T
_MAG_FIELD_Z: float = -45.0e-6  # T  (downward component, northern hemisphere)

# QoS matching the bridge's subscriber.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ZedSimPublisher(rclpy.node.Node):
    """Synthetic publisher node simulating ZED 2i + nvblox + nav2 outputs.

    Instantiate directly in tests — ``__init__`` does NOT call ``rclpy.init()``
    and has no argparse side-effects.

    Parameters
    ----------
    rate_hz:
        Timer rate for all publish loops.
    vio_topic:
        ``nav_msgs/Odometry`` topic name.
    imu_topic:
        ``sensor_msgs/Imu`` topic name.
    mag_topic:
        ``sensor_msgs/MagneticField`` topic name.
    cmd_vel_topic:
        ``geometry_msgs/Twist`` topic name.
    mesh_topic:
        ``visualization_msgs/Marker`` (CUBE_LIST) topic name.
    servo_topic:
        ``std_msgs/Float32`` nozzle-angle topic name.
    image_topic:
        ``sensor_msgs/Image`` RGB topic name (only published if
        ``enable_image=True``).
    enable_cmd_vel:
        Publish cmd_vel (nav2 stand-in). Default ``True``.
    enable_mesh:
        Publish nvblox CUBE_LIST marker. Default ``True``.
    enable_servo:
        Publish nozzle-angle Float32. Default ``True``.
    enable_image:
        Publish a synthetic RGB Image frame. Default ``False``
        (numpy is imported lazily only when this is ``True``).
    """

    def __init__(
        self,
        *,
        rate_hz: float = 30.0,
        vio_topic: str = "/zed/zed_node/odom",
        imu_topic: str = "/zed/zed_node/imu/data",
        mag_topic: str = "/zed/zed_node/imu/mag",
        cmd_vel_topic: str = "/cmd_vel",
        mesh_topic: str = "/nvblox_node/color_layer_marker",
        servo_topic: str = "/nomad/servo/nozzle_angle",
        image_topic: str = "/zed/zed_node/rgb/color/rect/image",
        enable_cmd_vel: bool = True,
        enable_mesh: bool = True,
        enable_servo: bool = True,
        enable_image: bool = False,
    ) -> None:
        super().__init__("zed_sim_publisher")

        self._rate_hz = rate_hz
        self._enable_cmd_vel = enable_cmd_vel
        self._enable_mesh = enable_mesh
        self._enable_servo = enable_servo
        self._enable_image = enable_image

        # Trajectory phase accumulator (seconds, advanced by timer period).
        self._t: float = 0.0
        self._dt: float = 1.0 / rate_hz

        # Always-on publishers -------------------------------------------
        self._pub_odom = self.create_publisher(Odometry, vio_topic, _SENSOR_QOS)
        self._pub_imu = self.create_publisher(Imu, imu_topic, _SENSOR_QOS)
        self._pub_mag = self.create_publisher(MagneticField, mag_topic, _SENSOR_QOS)

        # Optional publishers --------------------------------------------
        self._pub_cmd_vel = self.create_publisher(Twist, cmd_vel_topic, _SENSOR_QOS) if enable_cmd_vel else None
        self._pub_marker = self.create_publisher(Marker, mesh_topic, _SENSOR_QOS) if enable_mesh else None
        self._pub_servo = self.create_publisher(Float32, servo_topic, _SENSOR_QOS) if enable_servo else None

        if enable_image:
            # Lazy import — only load numpy when image publishing is active.
            import numpy as _np  # noqa: F401 (used in _publish_image)

            self._np = _np
            from sensor_msgs.msg import Image as _Image

            self._pub_image = self.create_publisher(_Image, image_topic, _SENSOR_QOS)
            self._Image = _Image
        else:
            self._np = None
            self._pub_image = None
            self._Image = None

        # Pre-build the voxel grid (static layout, positions don't move).
        self._marker_points, self._marker_colors = self._build_voxel_grid()

        # Single timer drives all publishing.
        timer_period_s = 1.0 / rate_hz
        self.create_timer(timer_period_s, self._publish_all)

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _publish_all(self) -> None:
        """Advance the trajectory and publish all enabled topics."""
        t = self._t
        self._t += self._dt

        now = self.get_clock().now().to_msg()

        # ---- circular trajectory in the XY plane (REP-103: X-fwd, Y-left) ----
        theta = _TRAJ_OMEGA * t  # yaw angle of the trajectory
        x = _TRAJ_RADIUS * math.cos(theta)
        y = _TRAJ_RADIUS * math.sin(theta)
        z = _TRAJ_ALT

        # Velocity tangent to the circle.
        vx = -_TRAJ_RADIUS * _TRAJ_OMEGA * math.sin(theta)
        vy = _TRAJ_RADIUS * _TRAJ_OMEGA * math.cos(theta)

        # Heading = direction of travel = theta + pi/2 (tangent to circle).
        yaw = theta + math.pi / 2.0
        # Small level platform — no roll or pitch.
        roll = 0.0
        pitch = 0.0

        # Quaternion from roll/pitch/yaw (ZYX convention: yaw first).
        qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)

        self._publish_odom(now, x, y, z, qx, qy, qz, qw, vx, vy)
        self._publish_imu(now, qx, qy, qz, qw, vx, vy)
        self._publish_mag(now, yaw)

        if self._pub_cmd_vel is not None:
            self._publish_cmd_vel(now, vx, vy)

        if self._pub_marker is not None:
            self._publish_marker(now)

        if self._pub_servo is not None:
            self._publish_servo(now, t)

        if self._pub_image is not None:
            self._publish_image(now)

    # ------------------------------------------------------------------
    # Individual topic publishers
    # ------------------------------------------------------------------

    def _publish_odom(
        self,
        stamp,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        vx: float,
        vy: float,
    ) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "zed_camera_link"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariance: pos_var=0.001 → confidence=0.99, healthy (<=0.1).
        msg.pose.covariance = list(_ODOM_COVARIANCE)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0

        self._pub_odom.publish(msg)

    def _publish_imu(
        self,
        stamp,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        vx: float,
        vy: float,
    ) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_link"

        # Orientation consistent with odometry yaw.
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Small angular velocity from the circular motion.
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = _TRAJ_OMEGA  # yaw rate (rad/s)

        # Plausible linear acceleration: gravity + centripetal.
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # gravity (m/s²)

        self._pub_imu.publish(msg)

    def _publish_mag(self, stamp, yaw: float) -> None:
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_link"

        # Rotate a fixed NED field into the body frame using the current yaw.
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # NED field (Bx = north, By = east → 0, Bz = down).
        bn = _MAG_FIELD_X
        be = _MAG_FIELD_Y

        # Body (FLU): Bx_body = Bn*cos(yaw) + Be*sin(yaw).
        msg.magnetic_field.x = bn * cos_y + be * sin_y
        msg.magnetic_field.y = -bn * sin_y + be * cos_y
        msg.magnetic_field.z = _MAG_FIELD_Z

        self._pub_mag.publish(msg)

    def _publish_cmd_vel(self, stamp, vx: float, vy: float) -> None:
        msg = Twist()
        # Small commanded velocity — a fraction of the simulated speed.
        msg.linear.x = vx * 0.5
        msg.linear.y = vy * 0.5
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = _TRAJ_OMEGA * 0.5
        self._pub_cmd_vel.publish(msg)

    def _publish_marker(self, stamp) -> None:
        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"

        msg.ns = "nvblox_sim"
        msg.id = 0
        msg.type = 6  # CUBE_LIST — required by the bridge handler
        msg.action = 0  # ADD

        msg.scale.x = _VOXEL_SIZE
        msg.scale.y = _VOXEL_SIZE
        msg.scale.z = _VOXEL_SIZE

        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.5
        msg.color.a = 1.0

        msg.points = self._marker_points
        msg.colors = self._marker_colors

        self._pub_marker.publish(msg)

    def _publish_servo(self, stamp, t: float) -> None:
        angle = _SERVO_MID + _SERVO_AMP * math.sin(_SERVO_OMEGA * t)
        msg = Float32()
        msg.data = float(angle)
        self._pub_servo.publish(msg)

    def _publish_image(self, stamp) -> None:
        """Publish a synthetic 64×64 RGB8 gradient image (numpy required)."""
        np = self._np
        Image = self._Image

        height, width = 64, 64
        # Simple gradient: R increases with x, G increases with y, B constant.
        row = np.arange(width, dtype=np.uint8)
        col = np.arange(height, dtype=np.uint8).reshape(height, 1)
        r = np.broadcast_to(row, (height, width)).copy()
        g = np.broadcast_to(col, (height, width)).copy()
        b = np.full((height, width), 128, dtype=np.uint8)
        frame = np.stack([r, g, b], axis=2)  # (H, W, 3)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_link"
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = frame.tobytes()

        self._pub_image.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _build_voxel_grid() -> tuple[list[Point], list[ColorRGBA]]:
        """Build a static 4×5 grid of voxel positions and per-voxel colors."""
        points: list[Point] = []
        colors: list[ColorRGBA] = []

        for row in range(_VOXEL_GRID_ROWS):
            for col in range(_VOXEL_GRID_COLS):
                p = Point()
                p.x = float(col) * _VOXEL_SIZE * 2.0
                p.y = float(row) * _VOXEL_SIZE * 2.0
                p.z = 0.0
                points.append(p)

                c = ColorRGBA()
                c.r = float(col) / max(1, _VOXEL_GRID_COLS - 1)
                c.g = float(row) / max(1, _VOXEL_GRID_ROWS - 1)
                c.b = 0.5
                c.a = 1.0
                colors.append(c)

        return points, colors


# ---------------------------------------------------------------------------
# Euler → quaternion helper (ZYX convention: applied as Rz * Ry * Rx).
# ---------------------------------------------------------------------------


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
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
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """Run the ZedSimPublisher as a standalone ROS2 node."""
    parser = argparse.ArgumentParser(
        description="Synthetic ZED 2i + nvblox + nav2 topic publisher for NOMAD ROS integration testing."
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate in Hz (default: 30)")
    parser.add_argument("--no-cmd-vel", action="store_true", help="Disable /cmd_vel publishing")
    parser.add_argument("--no-mesh", action="store_true", help="Disable nvblox Marker publishing")
    parser.add_argument("--no-servo", action="store_true", help="Disable nozzle-angle Float32 publishing")
    parser.add_argument(
        "--enable-image", action="store_true", help="Enable synthetic RGB Image publishing (requires numpy)"
    )

    # Topic overrides matching the bridge defaults.
    parser.add_argument("--vio-topic", default="/zed/zed_node/odom")
    parser.add_argument("--imu-topic", default="/zed/zed_node/imu/data")
    parser.add_argument("--mag-topic", default="/zed/zed_node/imu/mag")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--mesh-topic", default="/nvblox_node/color_layer_marker")
    parser.add_argument("--servo-topic", default="/nomad/servo/nozzle_angle")
    parser.add_argument("--image-topic", default="/zed/zed_node/rgb/color/rect/image")

    args = parser.parse_args()

    rclpy.init()
    node = ZedSimPublisher(
        rate_hz=args.rate,
        vio_topic=args.vio_topic,
        imu_topic=args.imu_topic,
        mag_topic=args.mag_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        mesh_topic=args.mesh_topic,
        servo_topic=args.servo_topic,
        image_topic=args.image_topic,
        enable_cmd_vel=not args.no_cmd_vel,
        enable_mesh=not args.no_mesh,
        enable_servo=not args.no_servo,
        enable_image=args.enable_image,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
