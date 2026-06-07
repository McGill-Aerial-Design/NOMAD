# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Main ROS2 node definition for the NOMAD ROS-HTTP Bridge."""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import asdict, dataclass
from http.client import HTTPConnection

import rclpy
import rclpy.time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .coordinate_math import quat_to_euler, wrap_angle_rad
from .mavlink_velocity import MavlinkVelocityController
from .mesh_packer import VoxelMeshPacker

try:
    from sensor_msgs.msg import Imu, MagneticField
    from std_msgs.msg import Float32

    SENSOR_MSGS_AVAILABLE = True
except ImportError:
    SENSOR_MSGS_AVAILABLE = False

try:
    from visualization_msgs.msg import Marker

    MARKER_AVAILABLE = True
except ImportError:
    MARKER_AVAILABLE = False

try:
    from tf2_ros import Buffer, TransformListener

    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

logger = logging.getLogger("ros_http_bridge.node")


@dataclass
class VIOData:
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    confidence: float = 1.0
    source: str = "isaac_ros"
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0
    body_roll: float = 0.0
    body_pitch: float = 0.0
    body_yaw: float = 0.0
    ros_qx: float = 0.0
    ros_qy: float = 0.0
    ros_qz: float = 0.0
    ros_qw: float = 1.0
    frame_id: str = "ros_odom"


@dataclass
class VelocityCommand:
    timestamp: float
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    source: str = "nav2"


class ROSHTTPBridge(Node):
    """ROS2 node bridging VIO telemetry, servo, and visual mesh data to Edge Core."""

    def __init__(
        self,
        host: str,
        port: int,
        vio_topic: str,
        imu_topic: str,
        mag_topic: str,
        cmd_vel_topic: str,
        mesh_topic: str,
        servo_topic: str,
        send_rate_hz: float,
        enable_nav_control: bool,
        enable_mesh: bool,
        enable_servo: bool,
        use_imu_attitude: bool,
        use_mag_heading: bool,
    ):
        super().__init__("nomad_ros_http_bridge")

        self._host = host
        self._port = port
        self._api_key = (os.environ.get("NOMAD_API_KEY") or "").strip() or None
        self._internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip() or None
        self._internal_token_header = "X-NOMAD-Internal-Token"
        self._send_interval = 1.0 / send_rate_hz

        self._enable_nav_control = enable_nav_control
        self._mavlink_vel = None
        if self._enable_nav_control:
            self._mavlink_vel = MavlinkVelocityController(logger_adapter=self.get_logger())

        self._enable_mesh = enable_mesh and MARKER_AVAILABLE
        self._mesh_packer = VoxelMeshPacker(self) if self._enable_mesh else None

        self._enable_servo = enable_servo and SENSOR_MSGS_AVAILABLE
        self._use_imu_attitude = use_imu_attitude and SENSOR_MSGS_AVAILABLE
        self._use_mag_heading = use_mag_heading and SENSOR_MSGS_AVAILABLE

        self._http_timeout_default_s = 0.5
        self._http_conn = HTTPConnection(host, port, timeout=self._http_timeout_default_s)
        self._http_lock = threading.Lock()
        self._last_http_error_log: dict[str, float] = {}

        self._latest_vio: VIOData | None = None
        self._latest_cmd_vel: VelocityCommand | None = None
        self._lock = threading.Lock()

        self._imu_roll = 0.0
        self._imu_pitch = 0.0
        self._imu_yaw = 0.0
        self._imu_recv_count = 0
        self._mag_heading = 0.0
        self._mag_recv_count = 0

        self._vio_recv_count = 0
        self._vio_send_count = 0
        self._cmd_vel_recv_count = 0
        self._cmd_vel_send_count = 0
        self._mesh_recv_count = 0
        self._mesh_send_count = 0
        self._mesh_coalesced_count = 0
        self._voxel_empty_count = 0
        self._servo_recv_count = 0
        self._servo_send_count = 0
        self._send_errors = 0
        self._send_errors_lock = threading.Lock()

        self._last_vio_http_send_time = 0.0
        self._last_cmd_vel_send_time = 0.0
        self._last_mesh_send_time = 0.0
        self._last_empty_mesh_send_time = 0.0
        self._empty_mesh_send_interval_s = 2.0
        self._last_servo_send_time = 0.0
        self._last_servo_angle = -1.0

        self._gimbal_pitch_rad = 0.0
        self._gimbal_angle_deg = 90.0
        self._gimbal_mount_offset = (0.10, 0.0, -0.05)

        self._tf_buffer = None
        self._tf_listener = None
        if TF2_AVAILABLE:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._camera_frame = "zed_camera_link"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, vio_topic, self._handle_vio, sensor_qos)

        if self._use_imu_attitude:
            self.create_subscription(Imu, imu_topic, self._handle_imu, sensor_qos)
        if self._use_mag_heading:
            self.create_subscription(MagneticField, mag_topic, self._handle_mag, sensor_qos)
        if self._enable_nav_control:
            self.create_subscription(Twist, cmd_vel_topic, self._handle_cmd_vel, sensor_qos)
        if self._enable_mesh:
            self.create_subscription(Marker, mesh_topic, self._handle_voxel_marker, sensor_qos)
        if self._enable_servo:
            self.create_subscription(Float32, servo_topic, self._handle_servo_angle, sensor_qos)

        self.create_timer(self._send_interval, self._send_to_edge_core)
        self.create_timer(0.5, self._poll_gimbal_angle)

    def _handle_vio(self, msg: Odometry) -> None:
        try:
            pose = msg.pose.pose
            twist = msg.twist.twist
            cov = msg.pose.covariance
            pos_var = max(cov[0], cov[7], cov[14])

            confidence = max(0.0, min(1.0, 1.0 - pos_var * 10.0))

            if self._mavlink_vel is not None:
                self._mavlink_vel.note_vio(confidence, healthy=pos_var <= 0.1)

            ros_roll, ros_pitch, ros_yaw = quat_to_euler(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            )

            body_roll = wrap_angle_rad(ros_roll)
            body_pitch = wrap_angle_rad(ros_pitch - self._gimbal_pitch_rad)
            body_yaw = wrap_angle_rad(ros_yaw)

            # Prefer gravity-aligned IMU roll/pitch and magnetometer yaw for the
            # NED/MAVLink attitude when they are available; fall back to VIO.
            if self._use_imu_attitude and self._imu_recv_count > 0:
                attitude_roll, attitude_pitch = self._imu_roll, self._imu_pitch
            else:
                attitude_roll, attitude_pitch = ros_roll, ros_pitch
            if self._use_mag_heading and self._mag_recv_count > 0:
                attitude_yaw = self._mag_heading
            else:
                attitude_yaw = ros_yaw

            # REP-103 (X-fwd, Y-left, Z-up) -> NED (X-north, Y-east, Z-down).
            ned_x, ned_y, ned_z = pose.position.x, -pose.position.y, -pose.position.z
            ned_roll, ned_pitch, ned_yaw = attitude_roll, -attitude_pitch, -attitude_yaw
            ned_vx, ned_vy, ned_vz = twist.linear.x, -twist.linear.y, -twist.linear.z

            slam_x, slam_y, slam_z = pose.position.x, pose.position.y, pose.position.z
            slam_roll, slam_pitch, slam_yaw = ros_roll, ros_pitch, ros_yaw
            slam_qx, slam_qy, slam_qz, slam_qw = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            if self._tf_buffer is not None:
                try:
                    _tf_t = self._tf_buffer.lookup_transform("map", self._camera_frame, rclpy.time.Time())
                    slam_x = _tf_t.transform.translation.x
                    slam_y = _tf_t.transform.translation.y
                    slam_z = _tf_t.transform.translation.z
                    slam_qx = _tf_t.transform.rotation.x
                    slam_qy = _tf_t.transform.rotation.y
                    slam_qz = _tf_t.transform.rotation.z
                    slam_qw = _tf_t.transform.rotation.w
                    slam_roll, slam_pitch, slam_yaw = quat_to_euler(slam_qx, slam_qy, slam_qz, slam_qw)
                except Exception:
                    pass

            vio = VIOData(
                timestamp=time.time(),
                x=ned_x,
                y=ned_y,
                z=ned_z,
                roll=ned_roll,
                pitch=ned_pitch,
                yaw=ned_yaw,
                vx=ned_vx,
                vy=ned_vy,
                vz=ned_vz,
                confidence=confidence,
                ros_x=slam_x,
                ros_y=slam_y,
                ros_z=slam_z,
                ros_roll=slam_roll,
                ros_pitch=slam_pitch,
                ros_yaw=slam_yaw,
                body_roll=body_roll,
                body_pitch=body_pitch,
                body_yaw=body_yaw,
                ros_qx=slam_qx,
                ros_qy=slam_qy,
                ros_qz=slam_qz,
                ros_qw=slam_qw,
            )

            with self._lock:
                self._latest_vio = vio
                self._vio_recv_count += 1
        except Exception as e:
            logger.error(f"VIO processing failed: {e}")

    def _handle_imu(self, msg: Imu) -> None:
        try:
            q = msg.orientation
            imu_roll, imu_pitch, imu_yaw = quat_to_euler(q.x, q.y, q.z, q.w)
            with self._lock:
                self._imu_roll = imu_roll
                self._imu_pitch = imu_pitch
                self._imu_yaw = imu_yaw
                self._imu_recv_count += 1
        except Exception as e:
            logger.error(f"IMU handler failed: {e}")

    def _handle_mag(self, msg: MagneticField) -> None:
        try:
            mag_x = msg.magnetic_field.x
            mag_y = msg.magnetic_field.y
            mag_z = msg.magnetic_field.z
            if math.sqrt(mag_x * mag_x + mag_y * mag_y) < 1e-9:
                return

            with self._lock:
                imu_roll = self._imu_roll
                imu_pitch = self._imu_pitch
                has_imu = self._imu_recv_count > 0

            # Tilt-compensate using IMU roll/pitch so yaw stays stable while the
            # camera pitches/rolls. Frame: X-fwd, Y-left, Z-up (REP-103 body).
            if has_imu:
                cr, sr = math.cos(imu_roll), math.sin(imu_roll)
                cp, sp = math.cos(imu_pitch), math.sin(imu_pitch)
                xh = mag_x * cp + mag_z * sp
                yh = mag_x * sr * sp + mag_y * cr - mag_z * sr * cp
                if math.sqrt(xh * xh + yh * yh) < 1e-12:
                    return
                heading = math.atan2(-yh, xh)
            else:
                heading = math.atan2(-mag_y, mag_x)

            with self._lock:
                self._mag_heading = heading
                self._mag_recv_count += 1
        except Exception as e:
            logger.error(f"Mag handler failed: {e}")

    def _handle_cmd_vel(self, msg: Twist) -> None:
        try:
            cmd = VelocityCommand(
                timestamp=time.time(),
                vx=msg.linear.x,
                vy=msg.linear.y,
                vz=msg.linear.z,
                yaw_rate=msg.angular.z,
            )
            with self._lock:
                self._latest_cmd_vel = cmd
                self._cmd_vel_recv_count += 1

            if self._mavlink_vel is not None and self._mavlink_vel.submit(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate):
                self._cmd_vel_send_count += 1
                self._last_cmd_vel_send_time = time.monotonic()
        except Exception as e:
            logger.error(f"cmd_vel handler failed: {e}")

    def _handle_voxel_marker(self, msg: Marker) -> None:
        if not self._enable_mesh or self._mesh_packer is None:
            return
        try:
            now_mono = time.monotonic()
            timestamp = time.time()

            if msg.type != 6:  # CUBE_LIST
                return

            n_pts = len(msg.points)
            if n_pts == 0:
                self._voxel_empty_count += 1
                self._send_empty_mesh_heartbeat(timestamp, now_mono)
                return

            self._voxel_empty_count = 0
            if now_mono - self._last_mesh_send_time < self._send_interval:
                return

            voxel_size = msg.scale.x if msg.scale.x > 0.0 else 0.05
            has_colors = len(msg.colors) == n_pts

            voxels = []
            for i in range(n_pts):
                p = msg.points[i]
                v_dict = {"p": [round(p.x, 4), round(p.y, 4), round(p.z, 4)]}
                if has_colors:
                    c = msg.colors[i]
                    v_dict["c"] = [
                        int(max(0.0, min(1.0, c.r)) * 255.0),
                        int(max(0.0, min(1.0, c.g)) * 255.0),
                        int(max(0.0, min(1.0, c.b)) * 255.0),
                    ]
                voxels.append(v_dict)

            drone_pose = self._get_drone_body_pose()
            mesh_data = {
                "voxels": voxels,
                "voxel_size": round(voxel_size, 4),
                "mode": "voxel",
                "total_voxels": n_pts,
                "timestamp": timestamp,
                "frame_id": "map",
                "clear": False,
            }

            if drone_pose:
                mesh_data["drone_position"] = drone_pose["position"]
                mesh_data["drone_attitude"] = drone_pose["attitude"]

            self._mesh_recv_count += 1
            self._mesh_packer.queue_mesh(mesh_data)
        except Exception as e:
            logger.error(f"Voxel processing failed: {e}")

    def _handle_servo_angle(self, msg: Float32) -> None:
        if not self._enable_servo:
            return
        try:
            angle = max(0.0, min(180.0, float(msg.data)))
            self._servo_recv_count += 1
            if abs(angle - self._last_servo_angle) < 0.5:
                return
            self._last_servo_angle = angle
            self._send_servo_to_edge_core(angle)
        except Exception as e:
            logger.error(f"Servo angle failed: {e}")

    def _poll_gimbal_angle(self) -> None:
        data = self._http_get_json("/api/servo/camera/tilt", timeout=0.1)
        if data is None:
            return
        try:
            fb_angle = data.get("feedback_angle") or data.get("angle", 90.0)
            self._gimbal_angle_deg = max(0.0, min(180.0, float(fb_angle)))
            self._gimbal_pitch_rad = math.radians(self._gimbal_angle_deg - 90.0)
        except (TypeError, ValueError):
            pass

    def _get_drone_body_pose(self) -> dict | None:
        with self._lock:
            vio = self._latest_vio
        if vio is None:
            return None

        cx, cy, cz = vio.ros_x, vio.ros_y, vio.ros_z
        qx, qy, qz, qw = vio.ros_qx, vio.ros_qy, vio.ros_qz, vio.ros_qw

        servo_pitch = self._gimbal_pitch_rad
        sq_y = math.sin(-servo_pitch / 2.0)
        sq_w = math.cos(-servo_pitch / 2.0)

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

        mx, my, mz = self._gimbal_mount_offset
        tx = 2.0 * (bqy * mz - bqz * my)
        ty = 2.0 * (bqz * mx - bqx * mz)
        tz = 2.0 * (bqx * my - bqy * mx)
        ox = mx + bqw * tx + (bqy * tz - bqz * ty)
        oy = my + bqw * ty + (bqz * tx - bqx * tz)
        oz = mz + bqw * tz + (bqx * ty - bqy * tx)

        body_roll, body_pitch, body_yaw = quat_to_euler(bqx, bqy, bqz, bqw)

        return {
            "position": {"x": round(cx - ox, 4), "y": round(cy - oy, 4), "z": round(cz - oz, 4)},
            "attitude": {"roll": round(body_roll, 4), "pitch": round(body_pitch, 4), "yaw": round(body_yaw, 4)},
        }

    def _send_to_edge_core(self) -> None:
        with self._lock:
            vio = self._latest_vio
        if vio is None:
            return

        now = time.monotonic()
        if now - self._last_vio_http_send_time < self._send_interval:
            return

        try:
            data = json.dumps(asdict(vio)).encode("utf-8")
            if self._http_post("/api/vio/update", data):
                self._vio_send_count += 1
                self._last_vio_http_send_time = now
            else:
                self._bump_send_errors()
        except Exception as e:
            self._bump_send_errors()
            logger.error(f"VIO HTTP send failed: {e}")

    def _send_servo_to_edge_core(self, angle: float) -> None:
        now = time.monotonic()
        if now - self._last_servo_send_time < 0.1:
            return
        self._last_servo_send_time = now

        try:
            path = f"/api/servo/camera/tilt?angle={angle:.1f}"
            if self._http_post(path, b"", timeout=0.2):
                self._servo_send_count += 1
            else:
                self._bump_send_errors()
        except Exception as e:
            self._bump_send_errors()
            logger.error(f"Servo API send failed: {e}")

    def _send_empty_mesh_heartbeat(self, timestamp: float, now_mono: float) -> None:
        if now_mono - self._last_empty_mesh_send_time < self._empty_mesh_send_interval_s:
            return
        self._last_empty_mesh_send_time = now_mono

        mesh_data = {
            "mode": "voxel",
            "timestamp": timestamp,
            "frame_id": "map",
            "clear": False,
            "voxels": [],
            "voxel_size": 0.0,
            "total_voxels": 0,
        }
        drone_pose = self._get_drone_body_pose()
        if drone_pose:
            mesh_data["drone_position"] = drone_pose["position"]
            mesh_data["drone_attitude"] = drone_pose["attitude"]

        if self._mesh_packer:
            self._mesh_packer.queue_mesh(mesh_data)

    def _http_post(self, path: str, data: bytes, timeout: float = 0.5, content_type: str = "application/json") -> bool:
        with self._http_lock:
            try:
                headers = self._build_internal_headers(content_type, keep_alive=True)
                self._http_conn.timeout = timeout
                self._http_conn.request("POST", path, body=data, headers=headers)
                resp = self._http_conn.getresponse()
                resp.read()
                return resp.status == 200
            except Exception as e:
                now = time.monotonic()
                if now - self._last_http_error_log.get(path, 0.0) >= 2.0:
                    logger.warning(f"HTTP POST {path} failed: {e}")
                    self._last_http_error_log[path] = now
                self._reconnect_http()
                return False

    def _http_get_json(self, path: str, timeout: float = 0.2) -> dict | None:
        with self._http_lock:
            try:
                headers = self._build_internal_headers(keep_alive=True)
                self._http_conn.timeout = timeout
                self._http_conn.request("GET", path, headers=headers)
                resp = self._http_conn.getresponse()
                body = resp.read()
                if resp.status == 200:
                    return json.loads(body.decode("utf-8"))
            except Exception:
                self._reconnect_http()
            return None

    def _reconnect_http(self) -> None:
        try:
            self._http_conn.close()
        except Exception:
            pass
        try:
            self._http_conn = HTTPConnection(self._host, self._port, timeout=self._http_timeout_default_s)
        except Exception:
            pass

    def _build_internal_headers(self, content_type: str | None = None, keep_alive: bool = False) -> dict[str, str]:
        headers = {}
        if content_type:
            headers["Content-Type"] = content_type
        if keep_alive:
            headers["Connection"] = "keep-alive"
        if self._internal_token:
            headers[self._internal_token_header] = self._internal_token
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        return headers

    def _bump_send_errors(self) -> None:
        with self._send_errors_lock:
            self._send_errors += 1

    def destroy_node(self) -> bool:
        if self._mavlink_vel is not None:
            self._mavlink_vel.stop()
        if self._mesh_packer:
            self._mesh_packer.stop()
        try:
            self._http_conn.close()
        except Exception:
            pass
        return super().destroy_node()

    def get_stats(self) -> dict:
        return {
            "vio_received": self._vio_recv_count,
            "vio_sent": self._vio_send_count,
            "cmd_vel_received": self._cmd_vel_recv_count,
            "cmd_vel_sent": self._cmd_vel_send_count,
            "mesh_received": self._mesh_recv_count,
            "mesh_sent": self._mesh_send_count,
            "mesh_coalesced": self._mesh_coalesced_count,
            "servo_received": self._servo_recv_count,
            "servo_sent": self._servo_send_count,
            "send_errors": self._send_errors,
        }
