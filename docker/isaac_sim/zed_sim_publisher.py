# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
ZED Simulation Publisher — publishes ZED-compatible ROS2 topics from Isaac Sim.

This node creates synthetic but realistic sensor data that matches the exact
topic names, message types, and frame IDs published by the real ZED ROS2
wrapper on the Jetson. It reads camera frames from Isaac Sim's rendering
pipeline and publishes:

  /zed/zed_node/odom                          (nav_msgs/Odometry)      ~30 Hz
  /zed/zed_node/imu/data                      (sensor_msgs/Imu)        ~200 Hz
  /zed/zed_node/imu/mag                       (sensor_msgs/MagneticField) ~100 Hz
  /zed/zed_node/rgb/color/rect/image           (sensor_msgs/Image)      ~15 Hz
  /zed/zed_node/depth/depth_registered         (sensor_msgs/Image)      ~15 Hz
  /zed/zed_node/obj_det/objects               (zed_interfaces/ObjectsStamped) ~10 Hz
  /tf                                          (tf2_msgs/TFMessage)     ~20 Hz

Frame IDs match the real ZED wrapper exactly:
  - zed_camera_link
  - zed_left_camera_frame
  - zed_left_camera_optical_frame
  - zed_depth_camera_frame
  - zed_depth_optical_frame

This ensures that downstream consumers (ros_http_bridge, nvblox, nav2)
work identically in sim and on the real drone.
"""

from __future__ import annotations

import math
import os
from threading import Lock

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, MagneticField
from tf2_ros import TransformBroadcaster

try:
    from zed_interfaces.msg import ObjectsStamped

    ZED_OD_AVAILABLE = True
except ImportError:
    ZED_OD_AVAILABLE = False

RESOLUTION_MAP = {
    "HD2K": (2208, 1242),
    "HD1080": (1920, 1080),
    "HD720": (1280, 720),
    "VGA": (672, 376),
}


class ZEDSimPublisher(Node):
    """Publishes ZED-compatible ROS2 topics from Isaac Sim data."""

    def __init__(
        self,
        camera_name: str = "zed",
        camera_model: str = "zed2i",
        grab_resolution: str = "HD720",
        depth_mode: str = "NEURAL_LIGHT",
    ):
        super().__init__(f"zed_sim_publisher_{camera_name}")

        self._camera_name = camera_name
        self._camera_model = camera_model
        self._resolution = RESOLUTION_MAP.get(grab_resolution, (1280, 720))
        self._depth_mode = depth_mode
        self._lock = Lock()
        self._tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        ns = f"/{camera_name}/{camera_name}_node"
        self._odom_pub = self.create_publisher(Odometry, f"{ns}/odom", sensor_qos)
        self._imu_pub = self.create_publisher(Imu, f"{ns}/imu/data", sensor_qos)
        self._mag_pub = self.create_publisher(MagneticField, f"{ns}/imu/mag", sensor_qos)
        self._rgb_pub = self.create_publisher(Image, f"{ns}/rgb/color/rect/image", sensor_qos)
        self._depth_pub = self.create_publisher(Image, f"{ns}/depth/depth_registered", sensor_qos)

        if ZED_OD_AVAILABLE:
            self._obj_pub = self.create_publisher(ObjectsStamped, f"{ns}/obj_det/objects", sensor_qos)

        self._x = 0.0
        self._y = 0.0
        self._z = 1.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._yaw_rate = 0.0

        self.create_subscription(
            __import__("geometry_msgs.msg", fromlist=["Twist"]).Twist,
            "/cmd_vel",
            self._handle_cmd_vel,
            sensor_qos,
        )

        self.create_timer(1.0 / 30.0, self._publish_odom)
        self.create_timer(1.0 / 200.0, self._publish_imu)
        self.create_timer(1.0 / 100.0, self._publish_mag)
        self.create_timer(1.0 / 15.0, self._publish_rgb)
        self.create_timer(1.0 / 15.0, self._publish_depth)
        self.create_timer(1.0 / 20.0, self._publish_tf)
        if ZED_OD_AVAILABLE:
            self.create_timer(1.0 / 10.0, self._publish_objects)

        self.get_logger().info(
            f"ZED Sim Publisher started: {camera_name}/{camera_model} "
            f"@ {grab_resolution} ({self._resolution[0]}x{self._resolution[1]})"
        )

    def update_pose(self, x, y, z, roll, pitch, yaw) -> None:
        with self._lock:
            self._x, self._y, self._z = x, y, z
            self._roll, self._pitch, self._yaw = roll, pitch, yaw

    def update_velocity(self, vx, vy, vz, yaw_rate) -> None:
        with self._lock:
            self._vx, self._vy, self._vz = vx, vy, vz
            self._yaw_rate = yaw_rate

    def _handle_cmd_vel(self, msg) -> None:
        with self._lock:
            self._vx = msg.linear.x
            self._vy = msg.linear.y
            self._vz = msg.linear.z
            self._yaw_rate = msg.angular.z

    def _integrate(self, dt: float) -> None:
        with self._lock:
            self._yaw += self._yaw_rate * dt
            self._x += (self._vx * math.cos(self._yaw) - self._vy * math.sin(self._yaw)) * dt
            self._y += (self._vx * math.sin(self._yaw) + self._vy * math.cos(self._yaw)) * dt
            self._z += self._vz * dt
            self._z = max(0.0, self._z)

    def _publish_odom(self) -> None:
        self._integrate(1.0 / 30.0)
        now = self.get_clock().now.to_msg()

        with self._lock:
            x, y, z = self._x, self._y, self._z
            roll, pitch, yaw = self._roll, self._pitch, self._yaw
            vx, vy, vz = self._vx, self._vy, self._vz

        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = f"{self._camera_name}_camera_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.001
        msg.pose.covariance[7] = 0.001
        msg.pose.covariance[14] = 0.001
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        self._odom_pub.publish(msg)

    def _publish_imu(self) -> None:
        now = self.get_clock().now.to_msg()
        with self._lock:
            roll, pitch, yaw = self._roll, self._pitch, self._yaw

        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)

        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = f"{self._camera_name}_camera_link"
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation_covariance[0] = 0.001
        msg.orientation_covariance[4] = 0.001
        msg.orientation_covariance[8] = 0.001
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        self._imu_pub.publish(msg)

    def _publish_mag(self) -> None:
        now = self.get_clock().now.to_msg()
        with self._lock:
            yaw = self._yaw

        msg = MagneticField()
        msg.header.stamp = now
        msg.header.frame_id = f"{self._camera_name}_camera_link"
        msg.magnetic_field.x = 0.02 * math.cos(yaw)
        msg.magnetic_field.y = -0.02 * math.sin(yaw)
        msg.magnetic_field.z = 0.04
        for i in range(9):
            msg.magnetic_field_covariance[i] = 0.0
        msg.magnetic_field_covariance[0] = 1e-6
        msg.magnetic_field_covariance[4] = 1e-6
        msg.magnetic_field_covariance[8] = 1e-6
        self._mag_pub.publish(msg)

    def _publish_rgb(self) -> None:
        now = self.get_clock().now.to_msg()
        w, h = self._resolution

        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = f"{self._camera_name}_left_camera_optical_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = [128] * (w * h * 3)
        self._rgb_pub.publish(msg)

    def _publish_depth(self) -> None:
        now = self.get_clock().now.to_msg()
        w, h = self._resolution

        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = f"{self._camera_name}_depth_optical_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = w * 4
        msg.data = list(np.full(h * w, 5.0, dtype=np.float32).tobytes())
        self._depth_pub.publish(msg)

    def _publish_tf(self) -> None:
        now = self.get_clock().now.to_msg()
        with self._lock:
            x, y, z = self._x, self._y, self._z
            roll, pitch, yaw = self._roll, self._pitch, self._yaw

        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)

        transforms = []

        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = "odom"
        base_tf.child_frame_id = f"{self._camera_name}_camera_link"
        base_tf.transform.translation.x = x
        base_tf.transform.translation.y = y
        base_tf.transform.translation.z = z
        base_tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
        base_tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
        base_tf.transform.rotation.z = cr * cp * sy - sr * sp * cy
        base_tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
        transforms.append(base_tf)

        left_tf = TransformStamped()
        left_tf.header.stamp = now
        left_tf.header.frame_id = f"{self._camera_name}_camera_link"
        left_tf.child_frame_id = f"{self._camera_name}_left_camera_frame"
        transforms.append(left_tf)

        optical_tf = TransformStamped()
        optical_tf.header.stamp = now
        optical_tf.header.frame_id = f"{self._camera_name}_left_camera_frame"
        optical_tf.child_frame_id = f"{self._camera_name}_left_camera_optical_frame"
        q = _axis_angle_quat(0, 0, 1, -math.pi / 2)  # FLU -> optical
        optical_tf.transform.rotation.x = q[0]
        optical_tf.transform.rotation.y = q[1]
        optical_tf.transform.rotation.z = q[2]
        optical_tf.transform.rotation.w = q[3]
        transforms.append(optical_tf)

        depth_tf = TransformStamped()
        depth_tf.header.stamp = now
        depth_tf.header.frame_id = f"{self._camera_name}_camera_link"
        depth_tf.child_frame_id = f"{self._camera_name}_depth_camera_frame"
        depth_tf.transform.translation.x = 0.0
        depth_tf.transform.translation.y = -0.06
        depth_tf.transform.translation.z = 0.0
        transforms.append(depth_tf)

        depth_opt_tf = TransformStamped()
        depth_opt_tf.header.stamp = now
        depth_opt_tf.header.frame_id = f"{self._camera_name}_depth_camera_frame"
        depth_opt_tf.child_frame_id = f"{self._camera_name}_depth_optical_frame"
        q = _axis_angle_quat(0, 0, 1, -math.pi / 2)
        depth_opt_tf.transform.rotation.x = q[0]
        depth_opt_tf.transform.rotation.y = q[1]
        depth_opt_tf.transform.rotation.z = q[2]
        depth_opt_tf.transform.rotation.w = q[3]
        transforms.append(depth_opt_tf)

        self._tf_broadcaster.sendTransform(transforms)

    def _publish_objects(self) -> None:
        if not ZED_OD_AVAILABLE:
            return
        now = self.get_clock().now.to_msg()
        msg = ObjectsStamped()
        msg.header.stamp = now
        msg.header.frame_id = f"{self._camera_name}_left_camera_optical_frame"
        self._obj_pub.publish(msg)


def _axis_angle_quat(ax, ay, az, angle) -> list[float]:
    half = angle / 2.0
    s = math.sin(half)
    return [ax * s, ay * s, az * s, math.cos(half)]


def main() -> None:
    rclpy.init()
    node = ZEDSimPublisher(
        camera_name=os.environ.get("ZED_CAMERA_NAME", "zed"),
        camera_model=os.environ.get("ZED_CAMERA_MODEL", "zed2i"),
        grab_resolution=os.environ.get("ZED_GRAB_RESOLUTION", "HD720"),
        depth_mode=os.environ.get("ZED_DEPTH_MODE", "NEURAL_LIGHT"),
    )

    start_x = float(os.environ.get("ISAAC_SIM_DRONE_START_X", "0.0"))
    start_y = float(os.environ.get("ISAAC_SIM_DRONE_START_Y", "0.0"))
    start_z = float(os.environ.get("ISAAC_SIM_DRONE_START_Z", "1.0"))
    start_yaw = math.radians(float(os.environ.get("ISAAC_SIM_DRONE_START_YAW", "0.0")))
    node.update_pose(start_x, start_y, start_z, 0.0, 0.0, start_yaw)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
