# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Synthetic ZED 2i + nvblox + nav2 topic publisher for hardware-free ROS2 testing.

Simulates the ROS2 topics the C++ adapter node (``nomad_vehicle_node``) and the
retired ROS-HTTP bridge consumed, so the ROS2 pipeline can be exercised in CI
without a Jetson, ZED camera, or GPU.

Topics published (all configurable via constructor kwargs):

  ``nav_msgs/Odometry``              → /zed/zed_node/odom
  ``sensor_msgs/Imu``                → /zed/zed_node/imu/data
  ``sensor_msgs/MagneticField``      → /zed/zed_node/imu/mag
  ``geometry_msgs/Twist``            → /cmd_vel
  ``visualization_msgs/Marker``      → /nvblox_node/color_layer_marker
  ``std_msgs/Float32``               → /nomad/servo/nozzle_angle
  ``sensor_msgs/Image``              → /zed/zed_node/rgb/color/rect/image
                                       (only when enable_image=True)
  ``geometry_msgs/TwistStamped``     → /nomad/cmd_vel
  ``std_msgs/Bool``                  → /nomad/vio_health
  ``std_msgs/Float32``               → /nomad/vio_confidence
                                       (only when enable_node_topics=True)

The QoS profile on all publishers is ``BEST_EFFORT`` reliability,
``KEEP_LAST`` history, depth 1.

The class is directly instantiable by pytest (no argparse side-effects in
``__init__``), as required by the integration test in ``tests/ros/``.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

from tools.sim.sim_scene import (
    MAG_FIELD_X,
    MAG_FIELD_Y,
    MAG_FIELD_Z,
    ODOM_COVARIANCE,
    SERVO_AMP,
    SERVO_MID,
    SERVO_OMEGA,
    TRAJ_ALT,
    TRAJ_OMEGA,
    TRAJ_RADIUS,
    VOXEL_SIZE,
    build_voxel_grid,
    euler_to_quat,
)

# QoS matching the adapter stack's subscribers.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass(frozen=True)
class SceneTopics:
    """Topic names for every simulated publisher."""

    vio: str = "/zed/zed_node/odom"
    imu: str = "/zed/zed_node/imu/data"
    mag: str = "/zed/zed_node/imu/mag"
    cmd_vel: str = "/cmd_vel"
    mesh: str = "/nvblox_node/color_layer_marker"
    servo: str = "/nomad/servo/nozzle_angle"
    image: str = "/zed/zed_node/rgb/color/rect/image"
    node_cmd_vel: str = "/nomad/cmd_vel"
    vio_health: str = "/nomad/vio_health"
    vio_confidence: str = "/nomad/vio_confidence"


@dataclass(frozen=True)
class SceneSample:
    """Position, velocity, and attitude of the simulated vehicle at time t."""

    x: float
    y: float
    z: float
    vx: float
    vy: float
    yaw: float
    qx: float
    qy: float
    qz: float
    qw: float


def _scene_at(t: float) -> SceneSample:
    """Circular trajectory in the XY plane (REP-103: X-fwd, Y-left)."""
    theta = TRAJ_OMEGA * t  # yaw angle of the trajectory
    x = TRAJ_RADIUS * math.cos(theta)
    y = TRAJ_RADIUS * math.sin(theta)
    z = TRAJ_ALT

    # Velocity tangent to the circle.
    vx = -TRAJ_RADIUS * TRAJ_OMEGA * math.sin(theta)
    vy = TRAJ_RADIUS * TRAJ_OMEGA * math.cos(theta)

    # Heading = direction of travel = theta + pi/2 (tangent to circle); the
    # platform stays level, so roll and pitch are zero.
    yaw = theta + math.pi / 2.0
    qx, qy, qz, qw = euler_to_quat(0.0, 0.0, yaw)
    return SceneSample(x, y, z, vx, vy, yaw, qx, qy, qz, qw)


class ZedSimPublisher(rclpy.node.Node):
    """Synthetic publisher node simulating ZED 2i + nvblox + nav2 outputs.

    Instantiate directly in tests — ``__init__`` does NOT call ``rclpy.init()``
    and has no argparse side-effects.

    Parameters
    ----------
    rate_hz:
        Timer rate for all publish loops.
    topics:
        ``SceneTopics`` naming every simulated topic.
    enable_cmd_vel:
        Publish cmd_vel (nav2 stand-in). Default ``True``.
    enable_mesh:
        Publish nvblox CUBE_LIST marker. Default ``True``.
    enable_servo:
        Publish nozzle-angle Float32. Default ``True``.
    enable_image:
        Publish a synthetic RGB Image frame. Default ``False``
        (numpy is imported lazily only when this is ``True``).
    enable_node_topics:
        Publish the adapter node's cmd_vel/VIO topics. Default ``False``
        (the bridge-era topics remain the default surface).
    """

    def __init__(
        self,
        *,
        rate_hz: float = 30.0,
        topics: SceneTopics = SceneTopics(),
        enable_cmd_vel: bool = True,
        enable_mesh: bool = True,
        enable_servo: bool = True,
        enable_image: bool = False,
        enable_node_topics: bool = False,
    ) -> None:
        super().__init__("zed_sim_publisher")

        # Trajectory phase accumulator (seconds, advanced by timer period).
        self._t: float = 0.0
        self._dt: float = 1.0 / rate_hz

        # Always-on publishers -------------------------------------------
        self._pub_odom = self.create_publisher(Odometry, topics.vio, _SENSOR_QOS)
        self._pub_imu = self.create_publisher(Imu, topics.imu, _SENSOR_QOS)
        self._pub_mag = self.create_publisher(MagneticField, topics.mag, _SENSOR_QOS)

        self._create_optional_publishers(
            topics,
            enable_cmd_vel=enable_cmd_vel,
            enable_mesh=enable_mesh,
            enable_servo=enable_servo,
            enable_node_topics=enable_node_topics,
        )
        self._create_image_publisher(topics.image, enable_image)

        # Pre-build the voxel grid (static layout, positions don't move).
        self._marker_points, self._marker_colors = build_voxel_grid()

        # Single timer drives all publishing.
        timer_period_s = 1.0 / rate_hz
        self.create_timer(timer_period_s, self._publish_all)

    def _create_optional_publishers(
        self,
        topics: SceneTopics,
        *,
        enable_cmd_vel: bool,
        enable_mesh: bool,
        enable_servo: bool,
        enable_node_topics: bool,
    ) -> None:
        """Create the optional publishers (nav2 stand-in, mesh, servo, node topics)."""
        self._pub_cmd_vel = self.create_publisher(Twist, topics.cmd_vel, _SENSOR_QOS) if enable_cmd_vel else None
        self._pub_marker = self.create_publisher(Marker, topics.mesh, _SENSOR_QOS) if enable_mesh else None
        self._pub_servo = self.create_publisher(Float32, topics.servo, _SENSOR_QOS) if enable_servo else None

        # Adapter node topics (nomad_vehicle_node): TwistStamped cmd_vel +
        # VIO health/confidence, published at the same timer rate.
        if enable_node_topics:
            self._pub_node_cmd_vel = self.create_publisher(TwistStamped, topics.node_cmd_vel, _SENSOR_QOS)
            self._pub_vio_health = self.create_publisher(Bool, topics.vio_health, _SENSOR_QOS)
            self._pub_vio_confidence = self.create_publisher(Float32, topics.vio_confidence, _SENSOR_QOS)
        else:
            self._pub_node_cmd_vel = None
            self._pub_vio_health = None
            self._pub_vio_confidence = None

    def _create_image_publisher(self, image_topic: str, enable_image: bool) -> None:
        """Create the synthetic image publisher, loading numpy only when needed."""
        if not enable_image:
            self._np = None
            self._pub_image = None
            self._Image = None
            return
        import numpy as _np  # noqa: F401 (used in _publish_image)
        from sensor_msgs.msg import Image as _Image

        self._np = _np
        self._pub_image = self.create_publisher(_Image, image_topic, _SENSOR_QOS)
        self._Image = _Image

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _publish_all(self) -> None:
        """Advance the trajectory and publish all enabled topics."""
        t = self._t
        self._t += self._dt

        now = self.get_clock().now().to_msg()
        sample = _scene_at(t)

        self._publish_odom(now, sample)
        self._publish_imu(now, sample)
        self._publish_mag(now, sample.yaw)

        if self._pub_cmd_vel is not None:
            self._publish_cmd_vel(now, sample.vx, sample.vy)

        if self._pub_marker is not None:
            self._publish_marker(now)

        if self._pub_servo is not None:
            self._publish_servo(now, t)

        if self._pub_node_cmd_vel is not None:
            self._publish_node_cmd_vel(now, sample.vx, sample.vy)
            self._publish_vio()

        if self._pub_image is not None:
            self._publish_image(now)

    # ------------------------------------------------------------------
    # Individual topic publishers
    # ------------------------------------------------------------------

    def _publish_odom(self, stamp, sample: SceneSample) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "zed_camera_link"

        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        msg.pose.pose.position.z = sample.z
        msg.pose.pose.orientation.x = sample.qx
        msg.pose.pose.orientation.y = sample.qy
        msg.pose.pose.orientation.z = sample.qz
        msg.pose.pose.orientation.w = sample.qw

        # Covariance: pos_var=0.001 → confidence=0.99, healthy (<=0.1).
        msg.pose.covariance = list(ODOM_COVARIANCE)

        msg.twist.twist.linear.x = sample.vx
        msg.twist.twist.linear.y = sample.vy
        msg.twist.twist.linear.z = 0.0

        self._pub_odom.publish(msg)

    def _publish_node_cmd_vel(self, stamp, vx: float, vy: float) -> None:
        """Publish the adapter node's TwistStamped velocity command topic."""
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        self._pub_node_cmd_vel.publish(msg)

    def _publish_vio(self) -> None:
        """Publish healthy, high-confidence VIO state for the adapter node."""
        health = Bool()
        health.data = True
        self._pub_vio_health.publish(health)

        confidence = Float32()
        confidence.data = 0.99  # matches the odometry covariance design note
        self._pub_vio_confidence.publish(confidence)

    def _publish_imu(self, stamp, sample: SceneSample) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_link"

        # Orientation consistent with odometry yaw.
        msg.orientation.x = sample.qx
        msg.orientation.y = sample.qy
        msg.orientation.z = sample.qz
        msg.orientation.w = sample.qw

        # Small angular velocity from the circular motion.
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = TRAJ_OMEGA  # yaw rate (rad/s)

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
        bn = MAG_FIELD_X
        be = MAG_FIELD_Y

        # Body (FLU): Bx_body = Bn*cos(yaw) + Be*sin(yaw).
        msg.magnetic_field.x = bn * cos_y + be * sin_y
        msg.magnetic_field.y = -bn * sin_y + be * cos_y
        msg.magnetic_field.z = MAG_FIELD_Z

        self._pub_mag.publish(msg)

    def _publish_cmd_vel(self, stamp, vx: float, vy: float) -> None:
        msg = Twist()
        # Small commanded velocity — a fraction of the simulated speed.
        msg.linear.x = vx * 0.5
        msg.linear.y = vy * 0.5
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = TRAJ_OMEGA * 0.5
        self._pub_cmd_vel.publish(msg)

    def _publish_marker(self, stamp) -> None:
        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"

        msg.ns = "nvblox_sim"
        msg.id = 0
        msg.type = 6  # CUBE_LIST — required by the mesh consumer
        msg.action = 0  # ADD

        msg.scale.x = VOXEL_SIZE
        msg.scale.y = VOXEL_SIZE
        msg.scale.z = VOXEL_SIZE

        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.5
        msg.color.a = 1.0

        msg.points = self._marker_points
        msg.colors = self._marker_colors

        self._pub_marker.publish(msg)

    def _publish_servo(self, stamp, t: float) -> None:
        angle = SERVO_MID + SERVO_AMP * math.sin(SERVO_OMEGA * t)
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


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser for the synthetic publisher."""
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
    parser.add_argument(
        "--node-topics", action="store_true", help="Enable the adapter node's /nomad/cmd_vel + VIO topics"
    )

    # Topic overrides matching the bridge-era defaults.
    parser.add_argument("--vio-topic", default="/zed/zed_node/odom")
    parser.add_argument("--imu-topic", default="/zed/zed_node/imu/data")
    parser.add_argument("--mag-topic", default="/zed/zed_node/imu/mag")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--mesh-topic", default="/nvblox_node/color_layer_marker")
    parser.add_argument("--servo-topic", default="/nomad/servo/nozzle_angle")
    parser.add_argument("--image-topic", default="/zed/zed_node/rgb/color/rect/image")
    # Topic overrides for the C++ adapter node (used with --node-topics).
    parser.add_argument("--node-cmd-vel-topic", default="/nomad/cmd_vel")
    parser.add_argument("--vio-health-topic", default="/nomad/vio_health")
    parser.add_argument("--vio-confidence-topic", default="/nomad/vio_confidence")
    return parser


def _topics_from_args(args: argparse.Namespace) -> SceneTopics:
    """Build the scene topic set from CLI overrides."""
    return SceneTopics(
        vio=args.vio_topic,
        imu=args.imu_topic,
        mag=args.mag_topic,
        cmd_vel=args.cmd_vel_topic,
        mesh=args.mesh_topic,
        servo=args.servo_topic,
        image=args.image_topic,
        node_cmd_vel=args.node_cmd_vel_topic,
        vio_health=args.vio_health_topic,
        vio_confidence=args.vio_confidence_topic,
    )


def main() -> None:
    """Run the ZedSimPublisher as a standalone ROS2 node."""
    args = _build_parser().parse_args()

    rclpy.init()
    node = ZedSimPublisher(
        rate_hz=args.rate,
        topics=_topics_from_args(args),
        enable_cmd_vel=not args.no_cmd_vel,
        enable_mesh=not args.no_mesh,
        enable_servo=not args.no_servo,
        enable_image=args.enable_image,
        enable_node_topics=args.node_topics,
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
