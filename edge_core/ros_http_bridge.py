#!/usr/bin/env python3
"""
ROS2-HTTP Bridge for NOMAD.

This script runs INSIDE the Isaac ROS Docker container and bridges
ROS2 topics to the NOMAD Edge Core HTTP API running on the host.

It subscribes to:
- /visual_slam/tracking/odometry (VIO pose from Isaac ROS VSLAM)
- /cmd_vel (Twist velocity commands from nav2/nvblox for autonomous navigation)
- /nvblox_node/mesh (3D mesh from Nvblox) - optional
- /nvblox_node/map_slice (2D occupancy slice) - for visualization
- /nomad/servo/nozzle_angle (Float32 servo angle for nozzle control)

And sends data to NOMAD Edge Core via HTTP POST requests.

Architecture (Jetson-Centric Navigation):
    Isaac ROS (nav2/nvblox) generates /cmd_vel
    -> ros_http_bridge subscribes and forwards to Edge Core
    -> Edge Core NavController sends MAVLink velocity commands
    -> ArduPilot executes in GUIDED mode (flight controller only)

Usage (inside Isaac ROS container):
    python3 ros_http_bridge.py --host 172.17.0.1 --port 8000
    
Note: 172.17.0.1 is the default Docker host IP from inside a container.
      For Jetson with network_mode=host, use localhost.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
from dataclasses import dataclass, asdict
from http.client import HTTPConnection
from typing import Optional
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Float32MultiArray

# sensor_msgs for HSV color verification (TD-005) and HSV circle detection
try:
    from sensor_msgs.msg import Image
    IMAGE_AVAILABLE = True
except ImportError:
    IMAGE_AVAILABLE = False

# OpenCV + numpy for standalone HSV circle detection
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger = logging.getLogger("ros_http_bridge")
    # logger not yet created, will warn later

# TF2 for camera pose lookup
try:
    from tf2_ros import Buffer, TransformListener, TransformException
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("ros_http_bridge")

# Try to import nvblox_msgs for mesh data
try:
    from nvblox_msgs.msg import Mesh, MeshBlock
    NVBLOX_AVAILABLE = True
except ImportError:
    NVBLOX_AVAILABLE = False
    logger.warning("nvblox_msgs not available - mesh bridge disabled")

# visualization_msgs for per-voxel colored marker data
try:
    from visualization_msgs.msg import Marker
    MARKER_AVAILABLE = True
except ImportError:
    MARKER_AVAILABLE = False
    logger.warning("visualization_msgs not available - per-voxel mesh disabled")

# ZED object detection messages (custom circle detection via HSV)
try:
    from zed_interfaces.msg import ObjectsStamped
    ZED_OD_AVAILABLE = True
except ImportError:
    ZED_OD_AVAILABLE = False
    logger.warning("zed_interfaces not available - object detection bridge disabled")


@dataclass
class VIOData:
    """VIO pose data to send to edge_core."""
    timestamp: float
    x: float      # NED: forward (north)
    y: float      # NED: right (east)
    z: float      # NED: down
    roll: float
    pitch: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    confidence: float = 1.0
    source: str = "isaac_ros"
    # Raw ROS-frame pose (odom/map) for SLAM 3D visualization
    # Always in "ros_optical" frame (ZED camera frame: X-right, Y-down, Z-forward)
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    # Raw ROS-frame orientation (from odom topic, same frame as mesh)
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0
    frame_id: str = "ros_optical"  # Explicit frame identifier for all ros_* fields


@dataclass
class VelocityCommand:
    """Velocity command from nav2/nvblox to send to edge_core."""
    timestamp: float
    vx: float       # Forward velocity (m/s)
    vy: float       # Lateral velocity (m/s)
    vz: float       # Vertical velocity (m/s)
    yaw_rate: float # Yaw rate (rad/s)
    source: str = "nav2"


@dataclass
class DetectedObject:
    """Detected object from HSV circle detection."""
    timestamp: float
    label: str           # Class label (e.g. 'red_circle')
    label_id: int        # Class ID
    confidence: float    # 0-1
    # 3D position in camera/map frame (meters)
    x: float
    y: float
    z: float
    # 3D bounding box dimensions
    width: float = 0.0
    height: float = 0.0
    depth: float = 0.0
    # 2D bounding box in pixel coordinates (from ZED image plane)
    bbox_x: float = 0.0   # top-left x
    bbox_y: float = 0.0   # top-left y
    bbox_w: float = 0.0   # width in pixels
    bbox_h: float = 0.0   # height in pixels
    # Tracking state: 0=OFF, 1=OK, 2=SEARCHING, 3=TERMINATE
    tracking_state: int = 0
    # HSV color verification
    hsv_color: str = ""          # HSV-derived color label (e.g. 'red', 'blue')
    color_match: bool = True     # True if color is verified
    needs_review: bool = False   # True if color verification failed


def _hsv_color_to_id(color: str) -> int:
    """Map HSV color name to a class ID."""
    return {"black": 0, "blue": 1, "green": 2, "red": 3, "white": 4, "yellow": 5}.get(color, -1)


class ROSHTTPBridge(Node):
    """
    ROS2 node that bridges topics to NOMAD Edge Core HTTP API.
    
    Bridges:
    - VIO odometry (position feedback)
    - Velocity commands from nav2/nvblox (navigation control)
    - 3D mesh from nvblox (for Mission Planner 3D visualization)
    """
    
    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        vio_topic: str = "/zed/zed_node/odom",  # Default to ZED odom
        cmd_vel_topic: str = "/cmd_vel",         # Nav2 velocity commands
        mesh_topic: str = "/nvblox_node/mesh",   # Nvblox 3D mesh
        servo_topic: str = "/nomad/servo/nozzle_angle",  # Nozzle servo angle
        detection_topic: str = "/zed/zed_node/obj_det/objects",  # ZED custom OD
        send_rate_hz: float = 30.0,
        enable_nav_control: bool = True,         # Enable velocity command forwarding
        enable_mesh: bool = True,                # Enable mesh forwarding
        enable_servo: bool = True,               # Enable servo control forwarding
        enable_detections: bool = True,          # Enable object detection forwarding
    ):
        super().__init__("nomad_ros_http_bridge")
        
        # Validate send_rate_hz to prevent divide-by-zero
        if send_rate_hz <= 0:
            raise ValueError(f"send_rate_hz must be positive, got {send_rate_hz}")
        
        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._send_interval = 1.0 / send_rate_hz
        self._enable_nav_control = enable_nav_control
        self._enable_mesh = enable_mesh and (NVBLOX_AVAILABLE or MARKER_AVAILABLE)
        self._enable_servo = enable_servo
        self._enable_detections = enable_detections and ZED_OD_AVAILABLE
        
        # Persistent HTTP connection (keep-alive) for efficiency.
        # Keep a short default timeout and override per-request in _http_post.
        self._http_timeout_default_s = 0.5
        self._http_conn = HTTPConnection(host, port, timeout=self._http_timeout_default_s)
        self._http_lock = threading.Lock()
        # Throttle repeated HTTP error logs per endpoint to reduce log spam under backpressure.
        self._http_warn_interval_s = 2.0
        self._last_http_error_log: dict[str, float] = {}
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # QoS for mesh data (match nvblox publisher QoS)
        mesh_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # Latest data
        self._latest_vio: Optional[VIOData] = None
        self._latest_cmd_vel: Optional[VelocityCommand] = None
        self._lock = threading.Lock()
        
        # Stats
        self._vio_recv_count = 0
        self._vio_send_count = 0
        self._cmd_vel_recv_count = 0
        self._cmd_vel_send_count = 0
        self._mesh_recv_count = 0
        self._mesh_send_count = 0
        self._voxel_empty_count = 0  # consecutive empty voxel markers; fall back to block mode after threshold
        self._servo_recv_count = 0
        self._servo_send_count = 0
        self._detection_recv_count = 0
        self._detection_send_count = 0
        self._last_detection_send_time = 0.0
        self._latest_detections: list[DetectedObject] = []
        self._send_errors = 0
        self._last_send_time = 0.0
        # Back off VIO POSTs briefly on repeated failures to avoid overwhelming Edge Core.
        self._vio_send_backoff_s = 0.0
        self._vio_backoff_until = 0.0
        self._vio_backoff_max_s = 1.0
        self._last_cmd_vel_send_time = 0.0
        self._last_mesh_send_time = 0.0
        self._last_empty_mesh_send_time = 0.0
        self._empty_mesh_send_interval_s = 2.0
        # Keep mesh forwarding capped by the configured bridge rate (default 30 Hz).
        # A fixed 10 Hz cap causes visible lag in world-view updates.
        self._mesh_send_interval_s = self._send_interval
        self._last_servo_send_time = 0.0
        self._last_servo_angle = -1.0
        
        # Subscribe to VIO odometry
        self.create_subscription(
            Odometry,
            vio_topic,
            self._handle_vio,
            sensor_qos,
        )
        self.get_logger().info(f"Subscribed to VIO: {vio_topic}")
        
        # Subscribe to cmd_vel for navigation control
        if enable_nav_control:
            self.create_subscription(
                Twist,
                cmd_vel_topic,
                self._handle_cmd_vel,
                sensor_qos,
            )
            self.get_logger().info(f"Subscribed to cmd_vel: {cmd_vel_topic}")
        
        # Subscribe to mesh for 3D visualization
        # Prefer triangle mesh from /nvblox_node/mesh (smooth surfaces) over
        # color_layer_marker (cube voxels). The voxel marker is kept as fallback
        # if the triangle mesh topic stops producing data.
        self._use_voxel_marker = False
        self._triangle_recv_count = 0  # track if triangle mesh is producing data
        if self._enable_mesh and NVBLOX_AVAILABLE:
            self.create_subscription(
                Mesh,
                mesh_topic,
                self._handle_mesh,
                mesh_qos,
            )
            self.get_logger().info(f"Subscribed to triangle mesh (primary): {mesh_topic}")
        if self._enable_mesh and MARKER_AVAILABLE:
            voxel_topic = "/nvblox_node/color_layer_marker"
            self.create_subscription(
                Marker,
                voxel_topic,
                self._handle_voxel_marker,
                mesh_qos,
            )
            self.get_logger().info(f"Subscribed to per-voxel marker (fallback): {voxel_topic}")
        elif enable_mesh and not NVBLOX_AVAILABLE and not MARKER_AVAILABLE:
            self.get_logger().warning("Mesh requested but nvblox_msgs not available")
        
        # Subscribe to servo angle for nozzle control
        if self._enable_servo:
            self.create_subscription(
                Float32,
                servo_topic,
                self._handle_servo_angle,
                sensor_qos,
            )
            self.get_logger().info(f"Subscribed to servo angle: {servo_topic}")
        
        # Subscribe to ZED custom object detections (HSV circle detection)
        if self._enable_detections:
            self.create_subscription(
                ObjectsStamped,
                detection_topic,
                self._handle_detections,
                sensor_qos,
            )
            self.get_logger().info(f"Subscribed to detections: {detection_topic}")
        elif enable_detections and not ZED_OD_AVAILABLE:
            self.get_logger().warning("Detections requested but zed_interfaces not available")
        
        # Subscribe to camera image for HSV color verification and
        # standalone HSV circle detection
        self._latest_image = None  # Raw image bytes (RGB8)
        self._image_width = 0
        self._image_height = 0
        self._image_lock = threading.Lock()

        # HSV circle detection state
        self._enable_hsv_circles = CV2_AVAILABLE and IMAGE_AVAILABLE
        self._hsv_circle_interval = 0.5  # Run HSV circle detection at 2 Hz
        self._last_hsv_circle_time = 0.0
        self._hsv_circle_send_count = 0

        # Subscribe to camera image for both HSV verification and circle detection
        if IMAGE_AVAILABLE:
            image_topic = "/zed/zed_node/rgb/image_rect_color"
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Image,
                image_topic,
                self._handle_image,
                image_qos,
            )
            self.get_logger().info(f"Subscribed to camera image: {image_topic}")
            if self._enable_hsv_circles:
                self.get_logger().info("HSV circle detection ENABLED (standalone)")
            else:
                self.get_logger().warning("HSV circle detection disabled (cv2 not available)")

        # Subscribe to depth image for 3D position of HSV-detected circles
        self._latest_depth = None
        self._depth_lock = threading.Lock()
        if IMAGE_AVAILABLE and self._enable_hsv_circles:
            depth_topic = "/zed/zed_node/depth/depth_registered"
            self.create_subscription(
                Image,
                depth_topic,
                self._handle_depth,
                image_qos,
            )
            self.get_logger().info(f"Subscribed to depth image: {depth_topic}")

        # ZED camera intrinsics (will be populated from camera_info if available)
        # Default ZED 2i HD720 intrinsics as fallback
        self._camera_fx = 528.0
        self._camera_fy = 528.0
        self._camera_cx = 640.0
        self._camera_cy = 360.0

        # TF2 buffer for camera pose lookup
        self._tf_buffer = None
        self._tf_listener = None
        if TF2_AVAILABLE:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.get_logger().info("TF2 listener initialized for camera pose tracking")
        else:
            self.get_logger().warning("TF2 not available - camera pose tracking disabled")
        
        # Camera frame to track (zed2_base_link matches camera:=zed2 launch param)
        self._camera_frame = "zed_camera_link"
        self._reference_frame = "odom"  # Or "map" depending on your setup
        
        # Thermal-aware detection rate throttling (RM-005)
        self._gpu_temp_c = 0.0
        self._last_thermal_check = 0.0
        self._thermal_check_interval = 2.0   # check every 2s
        self._thermal_throttle_temp = 85.0   # °C threshold
        self._thermal_recover_temp = 75.0    # °C to resume normal rate
        self._detection_throttled = False

        # VIO tracking loss detection (VO-005)
        self._vio_healthy_time = 0.0        # last time VIO was healthy
        self._vio_loss_servo_leveled = False # True if we already sent level command
        self._vio_loss_timeout_s = 3.0      # seconds of bad VIO before leveling servo

        # Scan-stop-scan state (VO-004)
        self._drone_velocity_mps = 0.0      # current drone velocity magnitude
        self._scan_stop_enabled = True      # enable scan-stop-scan protocol
        self._scan_stop_vel_threshold = 0.1 # m/s threshold for "stopped"

        # VO-006: Per-tilt-cycle drift tracking
        self._tilt_active = False           # True while servo is tilted (not level)
        self._tilt_start_pos = None         # (x, y, z) NED at tilt start
        self._tilt_cycle_count = 0
        self._tilt_total_drift_m = 0.0
        self._tilt_max_drift_m = 0.0
        self._tilt_drift_warning_threshold = 0.05  # 5cm per cycle (VO-006)

        # Timer to send data to edge_core
        self.create_timer(self._send_interval, self._send_to_edge_core)
        
        self.get_logger().info(f"ROS-HTTP Bridge started -> {self._base_url}")
        if enable_nav_control:
            self.get_logger().info("Navigation control ENABLED - forwarding cmd_vel to Edge Core")
        if self._enable_mesh:
            self.get_logger().info("Mesh forwarding ENABLED - forwarding nvblox mesh to Edge Core")
    
    def _handle_vio(self, msg: Odometry) -> None:
        """Handle VIO odometry from ZED ROS2 driver."""
        try:
            pose = msg.pose.pose
            twist = msg.twist.twist

            # Check pose covariance — high values indicate tracking degradation
            # (e.g. obstructed camera). Diagonal elements [0,7,14] = x,y,z variance.
            cov = msg.pose.covariance
            pos_var = max(cov[0], cov[7], cov[14])  # largest positional variance
            if pos_var > 0.1:
                self.get_logger().warn(
                    f"VIO degraded: high covariance ({pos_var:.4f})", throttle_duration_sec=5.0)

                # VO-005: Auto-level servo on sustained tracking loss
                now = time.time()
                if (self._vio_healthy_time > 0 and
                        now - self._vio_healthy_time > self._vio_loss_timeout_s and
                        not self._vio_loss_servo_leveled):
                    self.get_logger().warn(
                        "VO-005: VIO tracking lost for >3s - leveling servo to 90° "
                        "to re-acquire tracking"
                    )
                    self._send_servo_to_edge_core(90.0)
                    self._vio_loss_servo_leveled = True
                # Keep forwarding pose for SLAM visualization continuity even when
                # covariance is degraded; consumers can down-weight via confidence.

            # VIO is healthy - reset tracking loss state
            self._vio_healthy_time = time.time()
            self._vio_loss_servo_leveled = False

            # Derive confidence from covariance (lower variance = higher confidence)
            confidence = max(0.0, min(1.0, 1.0 - pos_var * 10.0))

            # Track drone velocity for scan-stop-scan (VO-004)
            vel_mag = math.sqrt(
                twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2
            )
            self._drone_velocity_mps = vel_mag

            # Quaternion to Euler
            roll, pitch, yaw = self._quat_to_euler(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            # Convert from ZED camera frame to NED frame
            # ZED odom: X-right, Y-down, Z-forward (camera/OpenCV convention)
            # NED: X-north(forward), Y-east(right), Z-down
            vio = VIOData(
                timestamp=time.time(),
                x=pose.position.z,   # Forward -> North
                y=pose.position.x,   # Right -> East
                z=pose.position.y,   # Down -> Down (ZED Y-down = NED Z-down)
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                vx=twist.linear.z,
                vy=twist.linear.x,
                vz=twist.linear.y,
                confidence=confidence,
                source="isaac_ros",
                # Also store the raw odom pose for SLAM 3D (same frame as mesh)
                ros_x=pose.position.x,
                ros_y=pose.position.y,
                ros_z=pose.position.z,
                ros_roll=roll,
                ros_pitch=pitch,
                ros_yaw=yaw,
            )

            with self._lock:
                self._latest_vio = vio
                self._vio_recv_count += 1

        except Exception as e:
            self.get_logger().error(f"VIO processing error: {e}")
    
    def _handle_cmd_vel(self, msg: Twist) -> None:
        """
        Handle cmd_vel from nav2/nvblox for autonomous navigation.
        
        This is the core of Jetson-centric navigation. Velocity commands
        from the ROS2 navigation stack are forwarded to Edge Core, which
        sends them to ArduPilot via MAVLink.
        
        Twist message convention (ROS REP 103):
        - linear.x: Forward velocity (m/s)
        - linear.y: Left velocity (m/s)
        - linear.z: Up velocity (m/s)
        - angular.z: Yaw rate (rad/s, CCW positive)
        """
        try:
            cmd = VelocityCommand(
                timestamp=time.time(),
                vx=msg.linear.x,    # Forward
                vy=msg.linear.y,    # Left
                vz=msg.linear.z,    # Up
                yaw_rate=msg.angular.z,  # Yaw CCW
                source="nav2",
            )
            
            with self._lock:
                self._latest_cmd_vel = cmd
                self._cmd_vel_recv_count += 1
            
            # Send immediately for low latency control
            self._send_cmd_vel_to_edge_core(cmd)
            
        except Exception as e:
            self.get_logger().error(f"cmd_vel processing error: {e}")
    
    def _http_post(self, path: str, data: bytes, timeout: float = 0.5) -> bool:
        """Send HTTP POST using persistent connection with keep-alive."""
        with self._http_lock:
            effective_timeout = max(0.05, timeout)
            try:
                self._http_conn.timeout = effective_timeout
                self._http_conn.request(
                    "POST", path, body=data,
                    headers={"Content-Type": "application/json", "Connection": "keep-alive"}
                )
                resp = self._http_conn.getresponse()
                resp.read()  # Drain response to allow connection reuse
                if resp.status == 200:
                    return True

                now = time.time()
                last_warn = self._last_http_error_log.get(path, 0.0)
                if now - last_warn >= self._http_warn_interval_s:
                    self.get_logger().warning(
                        f"HTTP POST to {path} returned status {resp.status}"
                    )
                    self._last_http_error_log[path] = now
                return False
            except Exception as e:
                now = time.time()
                last_warn = self._last_http_error_log.get(path, 0.0)
                if now - last_warn >= self._http_warn_interval_s:
                    self.get_logger().warning(
                        f"HTTP POST to {path} failed (timeout={effective_timeout:.2f}s): {e}"
                    )
                    self._last_http_error_log[path] = now

                # Recreate connection on failure (more reliable than reconnecting same object).
                try:
                    self._http_conn.close()
                except Exception:
                    pass
                try:
                    self._http_conn = HTTPConnection(
                        self._host, self._port, timeout=self._http_timeout_default_s
                    )
                except Exception:
                    pass
                return False
            finally:
                try:
                    self._http_conn.timeout = self._http_timeout_default_s
                except Exception:
                    pass

    def _send_to_edge_core(self) -> None:
        """Send latest VIO data to edge_core via HTTP."""
        now = time.time()
        if now < self._vio_backoff_until:
            return

        with self._lock:
            vio = self._latest_vio
        
        if vio is None:
            return
        
        try:
            data = json.dumps(asdict(vio)).encode("utf-8")
            if self._http_post("/api/vio/update", data, timeout=0.15):
                self._vio_send_count += 1
                self._vio_send_backoff_s = 0.0
            else:
                self._send_errors += 1
                self._vio_send_backoff_s = (
                    0.05 if self._vio_send_backoff_s <= 0.0
                    else min(self._vio_backoff_max_s, self._vio_send_backoff_s * 2.0)
                )
                self._vio_backoff_until = now + self._vio_send_backoff_s
                    
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 100 == 1:
                self.get_logger().warning(f"Failed to send VIO: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Send error: {e}")
    
    def _send_cmd_vel_to_edge_core(self, cmd: VelocityCommand) -> None:
        """
        Send velocity command to edge_core for navigation control.
        
        This enables Jetson-centric navigation where Isaac ROS nav2/nvblox
        generates velocity commands that are executed by ArduPilot in GUIDED mode.
        """
        if not self._enable_nav_control:
            return
        
        try:
            data = json.dumps(asdict(cmd)).encode("utf-8")
            if self._http_post("/api/nav/velocity", data, timeout=0.1):
                self._cmd_vel_send_count += 1
                self._last_cmd_vel_send_time = time.time()
            else:
                self._send_errors += 1
                    
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 100 == 1:
                self.get_logger().warning(f"Failed to send cmd_vel: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"cmd_vel send error: {e}")
    
    def _handle_servo_angle(self, msg: Float32) -> None:
        """
        Handle servo angle from a ROS node for autonomous nozzle control.

        The nozzle servo is controlled via the Edge Core HTTP API.
        A ROS node (e.g. a fire detection pipeline) publishes a Float32
        angle to /nomad/servo/nozzle_angle, and this bridge forwards it
        to Edge Core which drives the physical servo on GPIO Pin 15.

        Float32 value: angle in degrees (0-180, where 90 is center).

        VO-004: Scan-stop-scan protocol - only allow tilt when drone
        velocity is below threshold (hovering). During translational
        motion, servo is held level (90°) to prevent VIO drift.
        """
        if not self._enable_servo:
            return

        try:
            angle = float(msg.data)
            self._servo_recv_count += 1

            # Clamp to valid range
            angle = max(0.0, min(180.0, angle))

            # VO-006: Track tilt cycle start/end for drift measurement
            is_tilted = abs(angle - 90.0) > 2.0
            if is_tilted and not self._tilt_active:
                # Tilt cycle starting - record position
                self._tilt_active = True
                if self._latest_vio is not None:
                    self._tilt_start_pos = (
                        self._latest_vio.x,
                        self._latest_vio.y,
                        self._latest_vio.z,
                    )
            elif not is_tilted and self._tilt_active:
                # Tilt cycle ending - measure drift
                self._tilt_active = False
                if self._tilt_start_pos is not None and self._latest_vio is not None:
                    dx = self._latest_vio.x - self._tilt_start_pos[0]
                    dy = self._latest_vio.y - self._tilt_start_pos[1]
                    dz = self._latest_vio.z - self._tilt_start_pos[2]
                    drift = math.sqrt(dx * dx + dy * dy + dz * dz)
                    self._tilt_cycle_count += 1
                    self._tilt_total_drift_m += drift
                    self._tilt_max_drift_m = max(self._tilt_max_drift_m, drift)
                    if drift > self._tilt_drift_warning_threshold:
                        self.get_logger().warn(
                            f"VO-006: Tilt cycle #{self._tilt_cycle_count} drift "
                            f"{drift:.3f}m > {self._tilt_drift_warning_threshold}m threshold"
                        )
                self._tilt_start_pos = None

            # VO-004: Scan-stop-scan - block tilt during translational motion
            if self._scan_stop_enabled and abs(angle - 90.0) > 2.0:
                if self._drone_velocity_mps > self._scan_stop_vel_threshold:
                    # Drone is moving - force level
                    self.get_logger().info(
                        f"VO-004: Servo tilt blocked (vel={self._drone_velocity_mps:.2f} m/s > "
                        f"{self._scan_stop_vel_threshold} m/s) - holding level",
                        throttle_duration_sec=5.0,
                    )
                    angle = 90.0

            # Skip if angle hasn't changed significantly (avoid flooding)
            if abs(angle - self._last_servo_angle) < 0.5:
                return

            self._last_servo_angle = angle
            self._send_servo_to_edge_core(angle)

        except Exception as e:
            self.get_logger().error(f"Servo angle processing error: {e}")
    
    def _send_servo_to_edge_core(self, angle: float) -> None:
        """
        Send servo angle to Edge Core via HTTP POST.
        
        Uses the /api/servo/camera/tilt endpoint which controls the
        nozzle servo on Jetson GPIO Pin 15 via bit-bang PWM.
        
        Rate limited to 10 Hz to avoid overwhelming the servo controller.
        """
        # Rate limit servo commands (max 10 Hz)
        now = time.time()
        if now - self._last_servo_send_time < 0.1:
            return
        self._last_servo_send_time = now
        
        try:
            path = f"/api/servo/camera/tilt?angle={angle:.1f}"
            if self._http_post(path, b"", timeout=0.2):
                self._servo_send_count += 1
            else:
                self._send_errors += 1
                    
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 100 == 1:
                self.get_logger().warning(f"Failed to send servo angle: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Servo send error: {e}")
    
    def _handle_image(self, msg: 'Image') -> None:
        """Store latest camera image for HSV color verification and circle detection."""
        try:
            with self._image_lock:
                self._latest_image = bytes(msg.data)
                self._image_width = msg.width
                self._image_height = msg.height

            # Run standalone HSV circle detection at configured rate
            if self._enable_hsv_circles:
                now = time.time()
                if now - self._last_hsv_circle_time >= self._hsv_circle_interval:
                    self._last_hsv_circle_time = now
                    self._run_hsv_circle_detection(msg)
        except Exception:
            pass

    def _handle_depth(self, msg: 'Image') -> None:
        """Store latest depth image for 3D position of HSV-detected circles."""
        try:
            with self._depth_lock:
                self._latest_depth = msg
        except Exception:
            pass

    def _run_hsv_circle_detection(self, img_msg: 'Image') -> None:
        """
        Run standalone HSV circle detection on the camera image.

        Uses OpenCV HoughCircles to find circles, then classifies their
        color using HSV analysis. Results are sent to Edge Core as
        DetectedObject entries.
        """
        if not CV2_AVAILABLE:
            return

        try:
            # Convert ROS Image to numpy BGR
            w = img_msg.width
            h = img_msg.height
            encoding = img_msg.encoding if hasattr(img_msg, 'encoding') else 'rgb8'

            img_data = np.frombuffer(bytes(img_msg.data), dtype=np.uint8)

            if 'bgra' in encoding.lower():
                img_data = img_data.reshape((h, w, 4))
                image_bgr = cv2.cvtColor(img_data, cv2.COLOR_BGRA2BGR)
            elif 'bgr' in encoding.lower():
                img_data = img_data.reshape((h, w, 3))
                image_bgr = img_data
            elif 'rgba' in encoding.lower():
                img_data = img_data.reshape((h, w, 4))
                image_bgr = cv2.cvtColor(img_data, cv2.COLOR_RGBA2BGR)
            else:
                # Assume RGB8
                img_data = img_data.reshape((h, w, 3))
                image_bgr = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

            # Get depth image for 3D projection
            depth_np = None
            with self._depth_lock:
                depth_msg = self._latest_depth
            if depth_msg is not None:
                try:
                    depth_data = np.frombuffer(bytes(depth_msg.data), dtype=np.float32)
                    depth_np = depth_data.reshape((depth_msg.height, depth_msg.width))
                except Exception:
                    depth_np = None

            # Build camera matrix
            cam_matrix = np.array([
                [self._camera_fx, 0, self._camera_cx],
                [0, self._camera_fy, self._camera_cy],
                [0, 0, 1],
            ], dtype=np.float32)

            # Import and run the HSV circle detector
            # Use try-except to handle both installed package and standalone script scenarios
            try:
                from edge_core.hsv_circle_detector import detect_circles_hsv
            except ImportError:
                # Fallback: try direct import when running in container without edge_core package
                import sys
                import os
                script_dir = os.path.dirname(os.path.abspath(__file__))
                if script_dir not in sys.path:
                    sys.path.insert(0, script_dir)
                from hsv_circle_detector import detect_circles_hsv

            circles = detect_circles_hsv(
                image_bgr,
                depth_image=depth_np,
                camera_matrix=cam_matrix,
                min_radius=8,
                max_radius=min(w, h) // 3,
                min_color_confidence=0.20,
            )

            if not circles:
                return

            # Convert to DetectedObject format for Edge Core compatibility
            capture_time = time.time()
            detections = []
            for circ in circles:
                label = f"{circ.color}_circle"
                det = DetectedObject(
                    timestamp=capture_time,
                    label=label,
                    label_id=_hsv_color_to_id(circ.color),
                    confidence=circ.confidence,
                    x=circ.x if circ.x is not None else 0.0,
                    y=circ.y if circ.y is not None else 0.0,
                    z=circ.z if circ.z is not None else 0.0,
                    bbox_x=float(circ.bbox_x),
                    bbox_y=float(circ.bbox_y),
                    bbox_w=float(circ.bbox_w),
                    bbox_h=float(circ.bbox_h),
                    tracking_state=1,  # OK
                    hsv_color=circ.color,
                    color_match=True,
                    needs_review=False,
                )
                detections.append(det)

            with self._lock:
                self._latest_detections = detections
                self._detection_recv_count += 1

            self._send_detections_to_edge_core(detections)
            self._hsv_circle_send_count += 1

            if self._hsv_circle_send_count % 20 == 1:
                colors = [c.color for c in circles]
                self.get_logger().info(
                    f"HSV circle detection: {len(circles)} targets ({', '.join(colors)})"
                )

        except Exception as e:
            self.get_logger().error(f"HSV circle detection error: {e}")

    # HSV color ranges for circle detection classes (TD-005)
    # Each entry maps a color name to (H_low, H_high, S_min, V_min) in OpenCV HSV
    # H: 0-179, S: 0-255, V: 0-255
    _HSV_RANGES = {
        "red":    [(0, 10, 80, 50), (170, 179, 80, 50)],   # red wraps around 0/180
        "blue":   [(100, 130, 80, 50)],
        "green":  [(35, 85, 80, 50)],
        "yellow": [(20, 35, 80, 50)],
        "white":  [(0, 179, 0, 200)],   # low saturation, high value
        "black":  [(0, 179, 0, 0)],     # special: V < 50
    }

    def _verify_hsv_color(self, bbox_x: float, bbox_y: float,
                          bbox_w: float, bbox_h: float) -> str:
        """
        Analyze HSV color distribution within a bounding box (TD-005).

        Returns the dominant color name from HSV analysis, or "" if
        image data is unavailable.
        """
        with self._image_lock:
            image = self._latest_image
            img_w = self._image_width
            img_h = self._image_height

        if image is None or img_w == 0 or img_h == 0:
            return ""

        # Convert bbox to integer pixel coordinates (clamped to image bounds)
        x1 = max(0, int(bbox_x))
        y1 = max(0, int(bbox_y))
        x2 = min(img_w, int(bbox_x + bbox_w))
        y2 = min(img_h, int(bbox_y + bbox_h))
        if x2 <= x1 or y2 <= y1:
            return ""

        # Use center 50% of bbox to avoid edge noise
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        hw = max(1, (x2 - x1) // 4)
        hh = max(1, (y2 - y1) // 4)
        x1 = max(0, cx - hw)
        y1 = max(0, cy - hh)
        x2 = min(img_w, cx + hw)
        y2 = min(img_h, cy + hh)

        # Extract RGB pixels from the flat byte array (row-major, 3 channels)
        # and convert to HSV for color classification
        total_pixels = 0
        color_votes = {}
        step = 3  # RGB8 encoding
        row_stride = img_w * step

        for row in range(y1, y2, 2):  # sample every other pixel for speed
            for col in range(x1, x2, 2):
                idx = row * row_stride + col * step
                if idx + 2 >= len(image):
                    continue
                r, g, b = image[idx], image[idx + 1], image[idx + 2]

                # RGB to HSV (OpenCV convention: H 0-179, S 0-255, V 0-255)
                r_f, g_f, b_f = r / 255.0, g / 255.0, b / 255.0
                c_max = max(r_f, g_f, b_f)
                c_min = min(r_f, g_f, b_f)
                delta = c_max - c_min

                # Value (0-255)
                v = int(c_max * 255)

                # Saturation (0-255)
                s = int((delta / c_max) * 255) if c_max > 0 else 0

                # Hue (0-179)
                if delta == 0:
                    h = 0
                elif c_max == r_f:
                    h = int(30.0 * (((g_f - b_f) / delta) % 6))
                elif c_max == g_f:
                    h = int(30.0 * (((b_f - r_f) / delta) + 2))
                else:
                    h = int(30.0 * (((r_f - g_f) / delta) + 4))
                h = h % 180

                total_pixels += 1

                # Check black first (low value)
                if v < 50:
                    color_votes["black"] = color_votes.get("black", 0) + 1
                    continue

                # Check white (low saturation, high value)
                if s < 40 and v > 200:
                    color_votes["white"] = color_votes.get("white", 0) + 1
                    continue

                # Check chromatic colors by hue range
                for color_name, ranges in self._HSV_RANGES.items():
                    if color_name in ("white", "black"):
                        continue
                    for rng in ranges:
                        h_lo, h_hi, s_min, v_min = rng
                        if h_lo <= h <= h_hi and s >= s_min and v >= v_min:
                            color_votes[color_name] = color_votes.get(color_name, 0) + 1
                            break

        if not color_votes or total_pixels == 0:
            return ""

        # Return the color with the most votes
        return max(color_votes, key=lambda k: color_votes[k])

    def _handle_detections(self, msg) -> None:
        """
        Handle ZED custom object detections (HSV circle detection).

        Converts the zed_interfaces/ObjectsStamped message into DetectedObject
        dataclasses and forwards them to Edge Core.

        Uses image capture timestamp from message header, not inference
        completion time, to avoid TF lookup errors due to inference latency.
        """
        if not self._enable_detections:
            return

        try:
            # TD-003: Use image capture timestamp from message header
            capture_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if capture_timestamp <= 0:
                capture_timestamp = time.time()  # fallback if header is empty

            detections = []
            for obj in msg.objects:
                # Filter by tracking state -- only keep active detections
                if obj.tracking_state == 3:  # TERMINATE
                    continue

                # Validate position is finite (ZED can return NaN for failed depth)
                pos_x = obj.position[0]
                pos_y = obj.position[1]
                pos_z = obj.position[2]
                if not (math.isfinite(pos_x) and math.isfinite(pos_y) and math.isfinite(pos_z)):
                    continue

                # Extract 2D bounding box from ZED corners
                # bounding_box_2d has 4 KeyPoint2Df: TL, TR, BR, BL
                bbox_x = 0.0
                bbox_y = 0.0
                bbox_w = 0.0
                bbox_h = 0.0
                if hasattr(obj, 'bounding_box_2d') and len(obj.bounding_box_2d) >= 4:
                    corners = obj.bounding_box_2d
                    # Use min/max over all 4 corners for robustness
                    xs = [c.kp[0] for c in corners if hasattr(c, 'kp')]
                    ys = [c.kp[1] for c in corners if hasattr(c, 'kp')]
                    if xs and ys:
                        bbox_x = min(xs)
                        bbox_y = min(ys)
                        bbox_w = max(xs) - bbox_x
                        bbox_h = max(ys) - bbox_y

                # HSV color verification (TD-005)
                hsv_color = ""
                color_match = True
                needs_review = False
                if bbox_w > 0 and bbox_h > 0:
                    hsv_color = self._verify_hsv_color(bbox_x, bbox_y, bbox_w, bbox_h)
                    if hsv_color:
                        # Extract base color from YOLO label (e.g. 'red_circle' -> 'red')
                        yolo_color = obj.label.replace("_circle", "").lower()
                        color_match = (hsv_color == yolo_color)
                        needs_review = not color_match
                        if needs_review:
                            self.get_logger().info(
                                f"TD-005: Color mismatch - YOLO={yolo_color} HSV={hsv_color} "
                                f"conf={obj.confidence}% - flagged for review",
                                throttle_duration_sec=2.0,
                            )

                det = DetectedObject(
                    timestamp=capture_timestamp,
                    label=obj.label,
                    label_id=obj.label_id,
                    confidence=obj.confidence / 100.0,  # ZED uses 1-99 scale
                    x=pos_x,
                    y=pos_y,
                    z=pos_z,
                    width=obj.dimensions_3d[0] if len(obj.dimensions_3d) >= 3 else 0.0,
                    height=obj.dimensions_3d[1] if len(obj.dimensions_3d) >= 3 else 0.0,
                    depth=obj.dimensions_3d[2] if len(obj.dimensions_3d) >= 3 else 0.0,
                    bbox_x=bbox_x,
                    bbox_y=bbox_y,
                    bbox_w=bbox_w,
                    bbox_h=bbox_h,
                    tracking_state=obj.tracking_state,
                    hsv_color=hsv_color,
                    color_match=color_match,
                    needs_review=needs_review,
                )
                detections.append(det)
            
            with self._lock:
                self._latest_detections = detections
                self._detection_recv_count += 1
            
            if detections:
                self._send_detections_to_edge_core(detections)
                
        except Exception as e:
            self.get_logger().error(f"Detection processing error: {e}")
    
    def _read_gpu_temp(self) -> float:
        """Read GPU temperature from sysfs (RM-005). Returns 0 on failure."""
        try:
            with open("/sys/devices/virtual/thermal/thermal_zone1/temp", "r") as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def _send_detections_to_edge_core(self, detections: list) -> None:
        """
        Send object detections to Edge Core via HTTP POST.

        Rate limited to 5 Hz normally, throttled to 3 Hz when GPU temp
        exceeds 85°C (RM-005). Resumes normal rate when temp drops below 75°C.
        """
        now = time.time()

        # RM-005: Check GPU temperature periodically
        if now - self._last_thermal_check > self._thermal_check_interval:
            self._last_thermal_check = now
            self._gpu_temp_c = self._read_gpu_temp()
            if self._gpu_temp_c >= self._thermal_throttle_temp and not self._detection_throttled:
                self._detection_throttled = True
                self.get_logger().warn(
                    f"RM-005: GPU temp {self._gpu_temp_c:.0f}°C >= {self._thermal_throttle_temp}°C "
                    "- throttling detection rate to 3 Hz"
                )
            elif self._gpu_temp_c <= self._thermal_recover_temp and self._detection_throttled:
                self._detection_throttled = False
                self.get_logger().info(
                    f"RM-005: GPU temp {self._gpu_temp_c:.0f}°C <= {self._thermal_recover_temp}°C "
                    "- resuming normal 5 Hz detection rate"
                )

        # Apply rate limit: 5 Hz normal, 3 Hz when throttled (RM-005)
        min_interval = 0.333 if self._detection_throttled else 0.2
        if now - self._last_detection_send_time < min_interval:
            return
        self._last_detection_send_time = now
        
        try:
            payload = {
                "detections": [asdict(d) for d in detections],
                "count": len(detections),
            }
            data = json.dumps(payload).encode("utf-8")
            if self._http_post("/api/detections/update", data, timeout=0.25):
                self._detection_send_count += 1
            else:
                self._send_errors += 1
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 100 == 1:
                self.get_logger().warning(f"Failed to send detections: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Detection send error: {e}")
    
    def _handle_mesh(self, msg) -> None:
        """
        Handle mesh data from nvblox for 3D visualization.

        Primary mode: extract actual triangle vertices, indices, and per-vertex
        colors from the nvblox Mesh message for smooth surface rendering.
        Falls back to block-averaged colors if triangle extraction fails.
        """
        if not self._enable_mesh:
            return

        try:
            now = time.time()
            if now - self._last_mesh_send_time < self._mesh_send_interval_s:
                return

            block_size = msg.block_size_m if hasattr(msg, 'block_size_m') else 0.2

            # Triangle mesh mode: extract actual vertices, triangles, and colors
            # from nvblox MeshBlock data for smooth surface rendering.
            all_vertices = []
            all_indices = []
            all_colors = []
            vertex_offset = 0
            blocks_processed = 0

            for i, ros_block in enumerate(msg.blocks[:500]):  # Cap blocks per message
                if not hasattr(ros_block, 'vertices') or not ros_block.vertices:
                    continue
                if not hasattr(ros_block, 'triangles') or not ros_block.triangles:
                    continue

                # Extract vertices (Point32: x, y, z)
                block_verts = []
                for v in ros_block.vertices:
                    if hasattr(v, 'x'):
                        block_verts.append([round(float(v.x), 4),
                                            round(float(v.y), 4),
                                            round(float(v.z), 4)])
                    else:
                        block_verts.append([round(float(v[0]), 4),
                                            round(float(v[1]), 4),
                                            round(float(v[2]), 4)])

                if not block_verts:
                    continue

                # Extract triangle indices (flat array: [i0, i1, i2, ...])
                tri_indices = list(ros_block.triangles)
                if len(tri_indices) < 3:
                    continue

                # Extract per-vertex colors (ColorRGBA: r, g, b, a in 0-1)
                block_colors = []
                has_colors = hasattr(ros_block, 'colors') and ros_block.colors
                if has_colors and len(ros_block.colors) == len(block_verts):
                    for c in ros_block.colors:
                        if hasattr(c, 'r'):
                            block_colors.append([int(c.r * 255),
                                                 int(c.g * 255),
                                                 int(c.b * 255)])
                        else:
                            block_colors.append([int(c[0]), int(c[1]), int(c[2])])

                # Offset triangle indices for the global vertex array
                for idx in tri_indices:
                    all_indices.append(int(idx) + vertex_offset)

                all_vertices.extend(block_verts)
                all_colors.extend(block_colors)
                vertex_offset += len(block_verts)
                blocks_processed += 1

            if not all_vertices or not all_indices:
                self._send_empty_mesh_heartbeat(mode="triangle", timestamp=now)
                return

            # Subsample if too large (cap at ~20k vertices to keep payload manageable)
            max_vertices = 20000
            if len(all_vertices) > max_vertices:
                # Keep every Nth vertex and remap indices
                stride = len(all_vertices) / float(max_vertices)
                keep_set = set(int(i * stride) for i in range(max_vertices))
                old_to_new = {}
                new_verts = []
                new_colors = []
                new_idx = 0
                for old_idx in sorted(keep_set):
                    old_to_new[old_idx] = new_idx
                    new_verts.append(all_vertices[old_idx])
                    if old_idx < len(all_colors):
                        new_colors.append(all_colors[old_idx])
                    new_idx += 1
                # Rebuild triangles using only kept vertices
                new_indices = []
                for ti in range(0, len(all_indices), 3):
                    if ti + 2 < len(all_indices):
                        i0, i1, i2 = all_indices[ti], all_indices[ti+1], all_indices[ti+2]
                        if i0 in old_to_new and i1 in old_to_new and i2 in old_to_new:
                            new_indices.extend([old_to_new[i0], old_to_new[i1], old_to_new[i2]])
                all_vertices = new_verts
                all_colors = new_colors
                all_indices = new_indices

            camera_pose = self._get_camera_pose()

            mesh_data = {
                "mode": "triangle",
                "vertices": all_vertices,       # [[x,y,z], ...]
                "indices": all_indices,          # [i0, i1, i2, ...] (flat, every 3 = triangle)
                "colors": all_colors if all_colors else None,  # [[r,g,b], ...] per vertex
                "total_vertices": len(all_vertices),
                "total_triangles": len(all_indices) // 3,
                "block_size": block_size,
                "blocks_processed": blocks_processed,
                "timestamp": now,
                "frame_id": "ros_optical",
                "clear": msg.clear if hasattr(msg, 'clear') else False,
            }

            if camera_pose:
                mesh_data["drone_position"] = camera_pose["position"]
                mesh_data["drone_attitude"] = camera_pose["attitude"]

            self._mesh_recv_count += 1
            self._triangle_recv_count += 1
            self._send_mesh_to_edge_core(mesh_data)

        except Exception as e:
            self.get_logger().error(f"Mesh processing error: {e}")
    
    def _handle_voxel_marker(self, msg: 'Marker') -> None:
        """
        Handle per-voxel colored data from nvblox color_layer_marker topic.

        This is a fallback path: only used when triangle mesh from
        /nvblox_node/mesh is not producing data (e.g. nvblox_msgs unavailable).
        """
        if not self._enable_mesh:
            return
        # Skip voxel mode if triangle mesh is actively producing data
        if self._triangle_recv_count > 0 and not self._use_voxel_marker:
            return
        try:
            now = time.time()

            if msg.type != 6:  # CUBE_LIST
                return

            n_pts = len(msg.points)

            # Track empty markers for fallback logic (DO THIS BEFORE rate limiting)
            # so we count ALL empty messages, not just the ones that pass the rate limit
            if n_pts == 0:
                self._voxel_empty_count += 1
                if self._voxel_empty_count == 20:
                    self._use_voxel_marker = False
                    self.get_logger().warning(
                        "color_layer_marker has been empty for 20 consecutive messages -- "
                        "falling back to /nvblox_node/mesh (triangle mode)"
                    )
                self._send_empty_mesh_heartbeat(mode="voxel", timestamp=now)
                # Return AFTER tracking, but before rate limit check
                # (so we don't skip block mesh if voxels are temporarily empty)
                return

            # Got real points -- reset empty counter and re-enable voxel mode
            self._voxel_empty_count = 0
            if not self._use_voxel_marker:
                self._use_voxel_marker = True
                self.get_logger().info("color_layer_marker has data -- switching back to voxel mode")

            # Now apply rate limit only for sending (not for tracking empty)
            if now - self._last_mesh_send_time < self._mesh_send_interval_s:
                return

            voxel_size = msg.scale.x  # All 3 scales should be equal
            has_colors = len(msg.colors) == n_pts

            # Cap payload size so large voxel messages do not starve pose/world cadence.
            # 8k voxels is a good balance between detail and real-time responsiveness.
            limit = min(n_pts, 8000)

            voxels = []
            if limit == n_pts:
                sample_indices = range(limit)
            else:
                # Evenly subsample across the marker to preserve spatial coverage.
                stride = n_pts / float(limit)
                sample_indices = (int(i * stride) for i in range(limit))

            for i in sample_indices:
                p = msg.points[i]
                entry = {"p": [round(p.x, 4), round(p.y, 4), round(p.z, 4)]}
                if has_colors:
                    c = msg.colors[i]
                    entry["c"] = [
                        int(c.r * 255),
                        int(c.g * 255),
                        int(c.b * 255),
                    ]
                voxels.append(entry)

            camera_pose = self._get_camera_pose()

            mesh_data = {
                "voxels": voxels,
                "voxel_size": round(voxel_size, 4),
                "mode": "voxel",
                "total_voxels": n_pts,
                "sent_voxels": limit,
                "timestamp": now,
                "frame_id": "ros_optical",  # Same frame as mesh vertices and drone_position/attitude
                "clear": False,
            }

            if camera_pose:
                mesh_data["drone_position"] = camera_pose["position"]
                mesh_data["drone_attitude"] = camera_pose["attitude"]

            self._mesh_recv_count += 1
            self._send_mesh_to_edge_core(mesh_data)

        except Exception as e:
            self.get_logger().error(f"Voxel marker processing error: {e}")

    def _get_camera_pose(self) -> Optional[dict]:
        """
        Get camera pose from latest VIO odometry data.
        Returns position (x, y, z) and attitude (roll, pitch, yaw) in the odom frame.
        Uses the raw odom pose directly (same coordinate frame as the mesh).
        """
        with self._lock:
            vio = self._latest_vio

        if vio is None:
            return None

        return {
            "position": {"x": vio.ros_x, "y": vio.ros_y, "z": vio.ros_z},
            "attitude": {"roll": vio.ros_roll, "pitch": vio.ros_pitch, "yaw": vio.ros_yaw}
        }
    
    def _send_mesh_to_edge_core(self, mesh_data: dict) -> None:
        """Send mesh data to edge_core via HTTP."""
        if not self._enable_mesh:
            return
        
        try:
            data = json.dumps(mesh_data).encode("utf-8")
            if self._http_post("/api/task/2/slam/mesh/update", data, timeout=2.0):
                self._mesh_send_count += 1
                self._last_mesh_send_time = time.time()
                if self._mesh_send_count % 10 == 1:
                    mode = mesh_data.get('mode', 'block')
                    if mode == 'triangle':
                        count = mesh_data.get('total_vertices', 0)
                        tri_count = mesh_data.get('total_triangles', 0)
                        self.get_logger().info(
                            f"Mesh sent: {count} vertices, {tri_count} triangles "
                            f"(mode=triangle, blocks={mesh_data.get('blocks_processed', 0)})"
                        )
                    else:
                        count = mesh_data.get('total_voxels', mesh_data.get('total_blocks', 0))
                        unit = "voxels" if mode == 'voxel' else "blocks"
                        self.get_logger().info(
                            f"Mesh sent: {count} {unit} (mode={mode})"
                        )
            else:
                self._send_errors += 1
                    
        except URLError as e:
            self._send_errors += 1
            if self._send_errors % 50 == 1:
                self.get_logger().warning(f"Failed to send mesh: {e}")
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Mesh send error: {e}")

    def _send_empty_mesh_heartbeat(self, mode: str, timestamp: float) -> None:
        """Send sparse heartbeat updates so SLAM status does not remain stuck at 'no data'."""
        if not self._enable_mesh:
            return

        if timestamp - self._last_empty_mesh_send_time < self._empty_mesh_send_interval_s:
            return

        mesh_data = {
            "mode": mode,
            "timestamp": timestamp,
            "frame_id": "ros_optical",
            "clear": False,
        }
        if mode == "triangle":
            mesh_data.update({"vertices": [], "indices": [], "colors": None,
                              "total_vertices": 0, "total_triangles": 0})
        elif mode == "voxel":
            mesh_data.update({"voxels": [], "voxel_size": 0.0, "total_voxels": 0, "sent_voxels": 0})
        else:
            mesh_data.update({"blocks": [], "block_size": 0.0, "total_blocks": 0})

        camera_pose = self._get_camera_pose()
        if camera_pose:
            mesh_data["drone_position"] = camera_pose["position"]
            mesh_data["drone_attitude"] = camera_pose["attitude"]

        self._send_mesh_to_edge_core(mesh_data)
        self._last_empty_mesh_send_time = timestamp
    
    def _quat_to_euler(
        self, x: float, y: float, z: float, w: float
    ) -> tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        return {
            "vio_received": self._vio_recv_count,
            "vio_sent": self._vio_send_count,
            "cmd_vel_received": self._cmd_vel_recv_count,
            "cmd_vel_sent": self._cmd_vel_send_count,
            "mesh_received": self._mesh_recv_count,
            "mesh_sent": self._mesh_send_count,
            "servo_received": self._servo_recv_count,
            "servo_sent": self._servo_send_count,
            "detection_received": self._detection_recv_count,
            "detection_sent": self._detection_send_count,
            "send_errors": self._send_errors,
            "nav_control_enabled": self._enable_nav_control,
            "mesh_enabled": self._enable_mesh,
            "servo_enabled": self._enable_servo,
            "detections_enabled": self._enable_detections,
            "hsv_circles_enabled": self._enable_hsv_circles,
            "hsv_circles_sent": self._hsv_circle_send_count,
            # VO-006: tilt cycle drift stats
            "tilt_drift": {
                "cycles": self._tilt_cycle_count,
                "total_drift_m": round(self._tilt_total_drift_m, 4),
                "max_drift_m": round(self._tilt_max_drift_m, 4),
                "avg_drift_m": round(
                    self._tilt_total_drift_m / self._tilt_cycle_count, 4
                ) if self._tilt_cycle_count > 0 else 0.0,
                "tilt_active": self._tilt_active,
            },
        }


def main():
    parser = argparse.ArgumentParser(description="ROS2-HTTP Bridge for NOMAD")
    parser.add_argument("--host", default="172.17.0.1", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--vio-topic", default="/visual_slam/tracking/odometry",
                        help="VIO odometry topic")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel",
                        help="Navigation velocity command topic")
    parser.add_argument("--mesh-topic", default="/nvblox_node/mesh",
                        help="Nvblox mesh topic")
    parser.add_argument("--rate", type=float, default=30.0, help="Send rate Hz")
    parser.add_argument("--disable-nav", action="store_true",
                        help="Disable navigation control (cmd_vel forwarding)")
    parser.add_argument("--disable-mesh", action="store_true",
                        help="Disable mesh forwarding for 3D visualization")
    parser.add_argument("--servo-topic", default="/nomad/servo/nozzle_angle",
                        help="Servo nozzle angle topic (Float32, 0-180 degrees)")
    parser.add_argument("--disable-servo", action="store_true",
                        help="Disable servo angle forwarding")
    parser.add_argument("--detection-topic", default="/zed/zed_node/obj_det/objects",
                        help="ZED custom object detection topic (ObjectsStamped)")
    parser.add_argument("--disable-detections", action="store_true",
                        help="Disable object detection forwarding")
    
    # Parse args and validate send_rate_hz early
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error(f"--rate must be positive, got {args.rate}")
    
    rclpy.init()
    
    bridge = ROSHTTPBridge(
        host=args.host,
        port=args.port,
        vio_topic=args.vio_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        mesh_topic=args.mesh_topic,
        servo_topic=args.servo_topic,
        detection_topic=args.detection_topic,
        send_rate_hz=args.rate,
        enable_nav_control=not args.disable_nav,
        enable_mesh=not args.disable_mesh,
        enable_servo=not args.disable_servo,
        enable_detections=not args.disable_detections,
    )
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Log exception with traceback before swallowing
        # Launch system shutdown can raise ExternalShutdownException from rclpy.
        import traceback
        logger.error(f"ros_http_bridge crashed: {e}")
        logger.error(f"Traceback:\n{traceback.format_exc()}")
    finally:
        try:
            stats = bridge.get_stats()
            logger.info(f"Bridge stats: {stats}")
        except Exception:
            pass

        try:
            bridge.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
