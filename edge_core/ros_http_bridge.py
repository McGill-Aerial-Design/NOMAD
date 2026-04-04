#!/usr/bin/env python3
"""
ROS2-HTTP Bridge for NOMAD.

This script runs INSIDE the Isaac ROS Docker container and bridges
ROS2 topics to the NOMAD Edge Core HTTP API running on the host.

It subscribes to:
- /zed/zed_node/odom (VIO odometry pose from ZED wrapper)
- /cmd_vel (Twist velocity commands from nav2/nvblox for autonomous navigation)
- /nvblox_node/color_layer_marker (3D colored markers from nvblox) - optional
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
import os
import sys
import threading
import time
from datetime import datetime, timezone
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

# sensor_msgs for image metadata and HSV color verification
try:
    from sensor_msgs.msg import CameraInfo, Image
    IMAGE_AVAILABLE = True
except ImportError:
    IMAGE_AVAILABLE = False

# OpenCV + numpy for HSV ROI verification
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger = logging.getLogger("ros_http_bridge")
    # logger not yet created, will warn later

# msgpack for efficient mesh serialization (3-10x faster, 2-5x smaller than JSON)
try:
    import msgpack
    MSGPACK_AVAILABLE = True
except ImportError:
    MSGPACK_AVAILABLE = False

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

try:
    from ipc import (  # type: ignore
        DEFAULT_ROS_HIGH_RATE_ENDPOINT,
        HIGH_RATE_MSG_TYPE_CMD_VEL,
        HIGH_RATE_MSG_TYPE_DETECTIONS,
        HIGH_RATE_MSG_TYPE_VIO,
        IPCMessage,
        ZMQPublisher,
    )
    IPC_AVAILABLE = True
    IPC_IMPORT_ERROR = ""
except Exception as e_import:
    try:
        from edge_core.ipc import (
            DEFAULT_ROS_HIGH_RATE_ENDPOINT,
            HIGH_RATE_MSG_TYPE_CMD_VEL,
            HIGH_RATE_MSG_TYPE_DETECTIONS,
            HIGH_RATE_MSG_TYPE_VIO,
            IPCMessage,
            ZMQPublisher,
        )
        IPC_AVAILABLE = True
        IPC_IMPORT_ERROR = ""
    except Exception as e_pkg_import:
        IPC_AVAILABLE = False
        IPC_IMPORT_ERROR = f"{e_import}; {e_pkg_import}"
        DEFAULT_ROS_HIGH_RATE_ENDPOINT = "tcp://127.0.0.1:5557"
        HIGH_RATE_MSG_TYPE_VIO = "ROS_VIO_UPDATE"
        HIGH_RATE_MSG_TYPE_CMD_VEL = "ROS_CMD_VEL"
        HIGH_RATE_MSG_TYPE_DETECTIONS = "ROS_DETECTIONS"

# Try to import nvblox_msgs for mesh data
try:
    from nvblox_msgs.msg import Mesh, MeshBlock  # noqa: F401 (kept for future use)
    NVBLOX_AVAILABLE = True
except ImportError:
    NVBLOX_AVAILABLE = False
    logger.warning("nvblox_msgs not available - block mesh path disabled")

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

# Import target_localizer HSV verifier so HSV thresholds remain single-sourced.
TargetLocalizerColorVerifier = None
TARGET_LOCALIZER_IMPORT_ERROR = ""
try:
    from target_localizer.detectors import ColorVerifier as TargetLocalizerColorVerifier
except Exception as e_import:
    try:
        target_localizer_pkg_root = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "target_localizer"
        )
        if target_localizer_pkg_root not in sys.path:
            sys.path.insert(0, target_localizer_pkg_root)
        from target_localizer.detectors import ColorVerifier as TargetLocalizerColorVerifier
    except Exception as e_pkg_import:
        TARGET_LOCALIZER_IMPORT_ERROR = f"{e_import}; {e_pkg_import}"


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
    # Raw camera pose in odom frame (ZED odom topic, X-right, Y-down, Z-forward).
    # This is the camera position, NOT the drone body. _get_drone_body_pose()
    # removes the servo pitch and mounting offset to get the true body pose.
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0
    # Body attitude in the same ROS optical basis as ros_* pose fields.
    # This removes gimbal pitch so SLAM viewers can render airframe attitude
    # and apply camera tilt separately.
    body_roll: float = 0.0
    body_pitch: float = 0.0
    body_yaw: float = 0.0
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
    """Detected object forwarded from the ROS detection stream."""
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
        mesh_topic: str = "/nvblox_node/color_layer_marker",   # Nvblox marker topic
        mesh_output_mode: str = "voxel",         # Runtime-selectable output mode: voxel-only
        servo_topic: str = "/nomad/servo/nozzle_angle",  # Nozzle servo angle
        detection_topic: str = "/zed/zed_node/obj_det/objects",  # ZED custom OD
        send_rate_hz: float = 30.0,
        enable_nav_control: bool = True,         # Enable velocity command forwarding
        enable_mesh: bool = True,                # Enable mesh forwarding
        enable_servo: bool = True,               # Enable servo control forwarding
        enable_detections: bool = True,          # Enable object detection forwarding
        high_rate_transport: str = "zmq",       # High-rate stream transport: zmq/http/both (zmq preferred, HTTP fallback auto-activates)
        high_rate_zmq_endpoint: Optional[str] = None,
        high_rate_zmq_pub_mode: str = "connect",
    ):
        super().__init__("nomad_ros_http_bridge")
        
        # Validate send_rate_hz to prevent divide-by-zero
        if send_rate_hz <= 0:
            raise ValueError(f"send_rate_hz must be positive, got {send_rate_hz}")
        
        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._api_key = (os.environ.get("NOMAD_API_KEY") or "").strip() or None
        self._internal_token_header = "X-NOMAD-Internal-Token"
        self._internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip() or None
        if self._internal_token is not None and len(self._internal_token) < 32:
            logger.warning("NOMAD_INTERNAL_TOKEN is shorter than 32 chars; internal token auth disabled")
            self._internal_token = None
        self._send_interval = 1.0 / send_rate_hz
        self._enable_nav_control = enable_nav_control
        self._enable_mesh = enable_mesh and MARKER_AVAILABLE
        self._mesh_topic = (mesh_topic or "/nvblox_node/color_layer_marker").strip() or "/nvblox_node/color_layer_marker"
        self._mesh_output_mode = "voxel"
        self._enable_servo = enable_servo
        self._enable_detections = enable_detections and ZED_OD_AVAILABLE

        self._high_rate_transport = high_rate_transport.strip().lower()
        if self._high_rate_transport not in ("zmq", "http", "both"):
            raise ValueError(
                f"high_rate_transport must be one of zmq/http/both, got {high_rate_transport}"
            )
        self._use_high_rate_http = self._high_rate_transport in ("http", "both")
        self._use_high_rate_zmq = self._high_rate_transport in ("zmq", "both")
        self._high_rate_zmq_pub_mode = high_rate_zmq_pub_mode.strip().lower()
        if self._high_rate_zmq_pub_mode not in ("bind", "connect"):
            raise ValueError(
                "high_rate_zmq_pub_mode must be one of bind/connect, "
                f"got {high_rate_zmq_pub_mode}"
            )

        configured_zmq_endpoint = (high_rate_zmq_endpoint or "").strip()
        if not configured_zmq_endpoint:
            configured_zmq_endpoint = os.environ.get(
                "NOMAD_HIGH_RATE_ZMQ_ENDPOINT",
                "",
            ).strip()

        if configured_zmq_endpoint:
            self._high_rate_zmq_endpoint = configured_zmq_endpoint
        elif self._high_rate_zmq_pub_mode == "connect":
            try:
                scheme, endpoint_rest = DEFAULT_ROS_HIGH_RATE_ENDPOINT.split("://", 1)
                _, default_port = endpoint_rest.rsplit(":", 1)
                self._high_rate_zmq_endpoint = f"{scheme}://{host}:{default_port}"
            except Exception:
                self._high_rate_zmq_endpoint = f"tcp://{host}:5557"
        else:
            self._high_rate_zmq_endpoint = DEFAULT_ROS_HIGH_RATE_ENDPOINT

        self._zmq_publisher: Optional[ZMQPublisher] = None
        
        # Persistent HTTP connection (keep-alive) for efficiency.
        # Keep a short default timeout and override per-request in _http_post.
        self._http_timeout_default_s = 0.5
        self._http_conn = HTTPConnection(host, port, timeout=self._http_timeout_default_s)
        self._http_lock = threading.Lock()
        # Throttle repeated HTTP error logs per endpoint to reduce log spam under backpressure.
        self._http_warn_interval_s = 2.0
        self._last_http_error_log: dict[str, float] = {}
        self._last_zmq_error_log = 0.0
        self._zmq_restart_backoff_s = 0.0
        self._zmq_restart_backoff_min_s = 0.5
        self._zmq_restart_backoff_max_s = 8.0
        self._zmq_restart_retry_after = 0.0

        if self._use_high_rate_zmq:
            self._start_high_rate_publisher()
            if self._zmq_publisher is None:
                self._activate_high_rate_http_fallback(
                    "high-rate ZMQ publisher is unavailable at startup"
                )
        
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
        self._vio_send_zmq_count = 0
        self._vio_send_http_count = 0
        self._cmd_vel_recv_count = 0
        self._cmd_vel_send_count = 0
        self._cmd_vel_send_zmq_count = 0
        self._cmd_vel_send_http_count = 0
        self._mesh_recv_count = 0
        self._mesh_send_count = 0
        self._voxel_empty_count = 0  # consecutive empty voxel markers
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

        # Background mesh sender: decouples ROS callback from HTTP blocking.
        # Uses a threading.Event + single-slot pattern so only the latest
        # mesh payload is sent; stale data is discarded automatically.
        self._mesh_pending_data: Optional[bytes] = None
        self._mesh_pending_lock = threading.Lock()
        self._mesh_send_event = threading.Event()
        self._mesh_sender_stop = threading.Event()
        self._mesh_sender_thread = threading.Thread(
            target=self._mesh_sender_loop, daemon=True, name="mesh-sender"
        )
        self._mesh_sender_thread.start()

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
        
        # Subscribe to voxel marker for 3D visualization (cube rendering)
        self._use_voxel_marker = True
        if self._enable_mesh and MARKER_AVAILABLE:
            voxel_topic = self._mesh_topic
            self.create_subscription(
                Marker,
                voxel_topic,
                self._handle_voxel_marker,
                mesh_qos,
            )
            self.get_logger().info(f"Subscribed to per-voxel marker: {voxel_topic}")
        elif enable_mesh and not MARKER_AVAILABLE:
            self.get_logger().warning("Mesh requested but visualization_msgs not available")
        
        # Subscribe to servo angle for nozzle control
        if self._enable_servo:
            self.create_subscription(
                Float32,
                servo_topic,
                self._handle_servo_angle,
                sensor_qos,
            )
            self.get_logger().info(f"Subscribed to servo angle: {servo_topic}")
        
        # Subscribe to ZED custom object detections
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
        
        # Subscribe to camera image for HSV color verification.
        self._latest_image = None  # Raw image bytes
        self._image_width = 0
        self._image_height = 0
        self._image_encoding = "rgb8"
        self._image_step = 0
        self._image_lock = threading.Lock()
        self._hsv_verify_min_ratio = 0.20
        self._color_verifier = None

        if CV2_AVAILABLE and TargetLocalizerColorVerifier is not None:
            self._color_verifier = TargetLocalizerColorVerifier()
            self.get_logger().info(
                "HSV verifier source: target_localizer.detectors.ColorVerifier"
            )
        elif not CV2_AVAILABLE:
            self.get_logger().warning("HSV verification disabled (cv2 not available)")
        else:
            self.get_logger().warning(
                "HSV verification disabled (target_localizer verifier unavailable): "
                f"{TARGET_LOCALIZER_IMPORT_ERROR}"
            )

        if IMAGE_AVAILABLE:
            image_topic = "/zed/zed_node/rgb/color/rect/image"
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

            camera_info_topic = "/zed/zed_node/rgb/color/rect/camera_info"
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self._handle_camera_info,
                image_qos,
            )
            self.get_logger().info(f"Subscribed to camera info: {camera_info_topic}")

        # ZED camera intrinsics (will be populated from camera_info if available)
        # Default ZED 2i HD720 intrinsics as fallback
        self._camera_fx = 528.0
        self._camera_fy = 528.0
        self._camera_cx = 640.0
        self._camera_cy = 360.0
        self._camera_intrinsics_received = False

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

        # Camera gimbal (servo) angle for drone body pose computation.
        # The ZED odom gives the camera pose which includes the servo tilt.
        # To get the true drone body pose, we subtract the servo rotation
        # and mounting offset from the camera pose.
        # Same physical offsets as servo_tf_publisher.py (TF-004).
        self._gimbal_pitch_rad = 0.0         # Current servo pitch in radians (0 = level)
        self._gimbal_angle_deg = 90.0        # Raw servo angle in degrees (90 = level)
        self._gimbal_mount_offset = (0.10, 0.0, -0.05)  # (x, y, z) base_link -> servo_mount in body frame
        self._gimbal_last_poll = 0.0
        self._gimbal_poll_interval = 0.5     # Poll servo angle every 500ms
        self._mesh_mode_last_poll = 0.0
        self._mesh_mode_poll_interval = 1.0  # Poll runtime mesh output mode every 1s
        
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
        # Poll gimbal angle on its own timer to avoid blocking VIO send timer work.
        self.create_timer(self._gimbal_poll_interval, self._poll_gimbal_angle)
        # Poll runtime mesh output mode from Edge Core (voxel-only) without restart.
        self.create_timer(self._mesh_mode_poll_interval, self._poll_mesh_output_mode)

        self.get_logger().info(
            f"High-rate transport mode: {self._high_rate_transport}"
        )
        if self._use_high_rate_zmq:
            self.get_logger().info(
                f"High-rate ZMQ endpoint: {self._high_rate_zmq_endpoint}"
            )
            self.get_logger().info(
                f"High-rate ZMQ socket mode: {self._high_rate_zmq_pub_mode}"
            )
        if self._use_high_rate_http:
            self.get_logger().info("High-rate HTTP forwarding enabled")
        
        self.get_logger().info(f"ROS-HTTP Bridge started -> {self._base_url}")
        if enable_nav_control:
            self.get_logger().info("Navigation control ENABLED - forwarding cmd_vel to Edge Core")
        if self._enable_mesh:
            self.get_logger().info("Mesh forwarding ENABLED - forwarding nvblox mesh to Edge Core")
            self.get_logger().info(
                f"Mesh output mode: {self._mesh_output_mode} "
                "(runtime toggle via /api/task/2/slam/mesh/mode)"
            )
    
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
                # Edge case: if we have never seen a healthy sample yet, start the timer
                # from the first degraded sample so sustained loss still triggers leveling.
                if self._vio_healthy_time <= 0:
                    self._vio_healthy_time = now

                if (now - self._vio_healthy_time > self._vio_loss_timeout_s and
                        not self._vio_loss_servo_leveled):
                    self.get_logger().warn(
                        "VO-005: VIO tracking lost for >3s - leveling servo to 90° "
                        "to re-acquire tracking"
                    )
                    self._send_servo_to_edge_core(90.0)
                    self._vio_loss_servo_leveled = True
                # Keep forwarding pose for SLAM visualization continuity even when
                # covariance is degraded; consumers can down-weight via confidence.
            else:
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

            # Keep raw ROS optical attitude for SLAM consumers.
            ros_roll, ros_pitch, ros_yaw = self._quat_to_euler(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            # Roll follows raw ZED pose. Compensate only pitch for servo tilt;
            # keep yaw from raw ZED pose.
            body_roll = self._wrap_angle_rad(ros_roll)
            body_pitch = self._wrap_angle_rad(ros_pitch - self._gimbal_pitch_rad)
            body_yaw = self._wrap_angle_rad(ros_yaw)

            # Convert attitude to NED so primary pose fields match position frame.
            roll, pitch, yaw = self._quat_to_ned_euler(
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
                ros_roll=ros_roll,
                ros_pitch=ros_pitch,
                ros_yaw=ros_yaw,
                body_roll=body_roll,
                body_pitch=body_pitch,
                body_yaw=body_yaw,
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
        - angular.x: Roll rate (rad/s) - ignored by this bridge
        - angular.y: Pitch rate (rad/s) - ignored by this bridge
        - angular.z: Yaw rate (rad/s, CCW positive) - forwarded as yaw_rate
        """
        try:
            angular_epsilon = 1e-3
            if (
                abs(msg.angular.x) > angular_epsilon
                or abs(msg.angular.y) > angular_epsilon
            ):
                self.get_logger().warn(
                    "cmd_vel angular.x/angular.y are ignored (roll/pitch rates unsupported); forwarding angular.z as yaw_rate only",
                    throttle_duration_sec=5.0,
                )

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
    
    def _http_post(
        self,
        path: str,
        data: bytes,
        timeout: float = 0.5,
        accepted_statuses: tuple[int, ...] = (200,),
        content_type: str = "application/json",
    ) -> bool:
        """Send HTTP POST using persistent connection with keep-alive."""
        with self._http_lock:
            effective_timeout = max(0.05, timeout)
            try:
                headers = self._build_internal_headers(content_type=content_type, keep_alive=True)

                self._http_conn.timeout = effective_timeout
                self._http_conn.request(
                    "POST", path, body=data,
                    headers=headers
                )
                resp = self._http_conn.getresponse()
                resp.read()  # Drain response to allow connection reuse
                if resp.status in accepted_statuses:
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

    def _build_internal_headers(
        self,
        content_type: Optional[str] = None,
        keep_alive: bool = False,
    ) -> dict[str, str]:
        """Build consistent headers for internal Edge Core API calls."""
        headers: dict[str, str] = {}
        if content_type:
            headers["Content-Type"] = content_type
        if keep_alive:
            headers["Connection"] = "keep-alive"
        if self._internal_token:
            headers[self._internal_token_header] = self._internal_token
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        return headers

    def _schedule_high_rate_zmq_restart(self) -> float:
        """Increase restart backoff after a ZMQ failure and return delay."""
        self._zmq_restart_backoff_s = (
            self._zmq_restart_backoff_min_s
            if self._zmq_restart_backoff_s <= 0.0
            else min(
                self._zmq_restart_backoff_max_s,
                self._zmq_restart_backoff_s * 2.0,
            )
        )
        self._zmq_restart_retry_after = time.time() + self._zmq_restart_backoff_s
        return self._zmq_restart_backoff_s

    def _start_high_rate_publisher(self) -> None:
        """Start high-rate ZMQ publisher (best effort, non-fatal on failure)."""
        if not self._use_high_rate_zmq:
            return
        if self._zmq_publisher is not None:
            return
        if not IPC_AVAILABLE:
            self.get_logger().warning(
                f"High-rate ZMQ transport unavailable: IPC import failed ({IPC_IMPORT_ERROR})"
            )
            self._activate_high_rate_http_fallback(
                "high-rate ZMQ transport dependencies are unavailable"
            )
            return

        now = time.time()
        if now < self._zmq_restart_retry_after:
            return

        try:
            publisher = ZMQPublisher(
                endpoint=self._high_rate_zmq_endpoint,
                socket_mode=self._high_rate_zmq_pub_mode,
                snd_hwm=2,
                conflate=True,
                linger_ms=0,
            )
            publisher.start()
            self._zmq_publisher = publisher
            self._zmq_restart_backoff_s = 0.0
            self._zmq_restart_retry_after = 0.0
            self.get_logger().info(
                "High-rate ZMQ publisher started on "
                f"{self._high_rate_zmq_endpoint} ({self._high_rate_zmq_pub_mode})"
            )
        except Exception as e:
            self._zmq_publisher = None
            backoff_s = self._schedule_high_rate_zmq_restart()
            now = time.time()
            if now - self._last_zmq_error_log >= self._http_warn_interval_s:
                self.get_logger().warning(
                    "Failed to start high-rate ZMQ publisher "
                    f"({self._high_rate_zmq_endpoint}): {e}; retrying in {backoff_s:.2f}s"
                )
                self._last_zmq_error_log = now
            self._activate_high_rate_http_fallback(
                "high-rate ZMQ publisher failed to start"
            )

    def _activate_high_rate_http_fallback(self, reason: str) -> None:
        """Enable HTTP transport when ZMQ-only mode would drop high-rate data."""
        if self._use_high_rate_http:
            return
        self._use_high_rate_http = True
        self.get_logger().warning(f"High-rate HTTP fallback enabled: {reason}")

    def _effective_high_rate_transport(self) -> str:
        """Return the currently active high-rate transport mode."""
        if self._use_high_rate_http and self._use_high_rate_zmq:
            return "both"
        if self._use_high_rate_zmq:
            return "zmq"
        if self._use_high_rate_http:
            return "http"
        return "none"

    def _stop_high_rate_publisher(self) -> None:
        """Stop high-rate ZMQ publisher cleanly."""
        if self._zmq_publisher is None:
            return

        try:
            self._zmq_publisher.stop()
        except Exception as e:
            self.get_logger().warning(f"Failed to stop high-rate ZMQ publisher cleanly: {e}")
        finally:
            self._zmq_publisher = None

    def _send_high_rate_zmq(self, msg_type: str, payload: dict) -> bool:
        """Publish high-rate telemetry via ZMQ IPC."""
        if not self._use_high_rate_zmq:
            return False

        if self._zmq_publisher is None:
            self._start_high_rate_publisher()
            if self._zmq_publisher is None:
                self._activate_high_rate_http_fallback(
                    "high-rate ZMQ publisher is unavailable at runtime"
                )
                return False

        try:
            self._zmq_publisher.send(
                IPCMessage(
                    msg_type=msg_type,
                    timestamp=datetime.now(timezone.utc).isoformat(),
                    data=payload,
                )
            )
            return True
        except Exception as e:
            self._activate_high_rate_http_fallback(
                "high-rate ZMQ publish errors detected"
            )
            try:
                self._stop_high_rate_publisher()
            except Exception:
                pass
            backoff_s = self._schedule_high_rate_zmq_restart()
            now = time.time()
            if now - self._last_zmq_error_log >= self._http_warn_interval_s:
                self.get_logger().warning(
                    f"High-rate ZMQ publish failed ({msg_type}): {e}; "
                    f"retrying publisher start in {backoff_s:.2f}s"
                )
                self._last_zmq_error_log = now
            return False

    def _send_to_edge_core(self) -> None:
        """Send latest VIO data to edge_core via configured high-rate transport."""
        with self._lock:
            vio = self._latest_vio

        if vio is None:
            return

        payload = asdict(vio)

        if self._use_high_rate_zmq:
            if self._send_high_rate_zmq(HIGH_RATE_MSG_TYPE_VIO, payload):
                self._vio_send_zmq_count += 1
                self._vio_send_count += 1
            else:
                self._send_errors += 1

        if not self._use_high_rate_http:
            return

        now = time.time()
        if now < self._vio_backoff_until:
            return

        try:
            data = json.dumps(payload).encode("utf-8")
            if self._http_post("/api/vio/update", data, timeout=0.15):
                self._vio_send_http_count += 1
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

        payload = asdict(cmd)
        zmq_sent = False

        if self._use_high_rate_zmq:
            if self._send_high_rate_zmq(HIGH_RATE_MSG_TYPE_CMD_VEL, payload):
                self._cmd_vel_send_zmq_count += 1
                self._cmd_vel_send_count += 1
                self._last_cmd_vel_send_time = time.time()
                zmq_sent = True
            else:
                self._send_errors += 1

        # Keep ZMQ as the primary path. In explicit "both" mode, also send
        # HTTP as a secondary path with the same payload so API-side monotonic
        # dedupe can safely drop duplicates.
        if zmq_sent and self._high_rate_transport != "both":
            return

        if not self._use_high_rate_http:
            return
        
        try:
            data = json.dumps(payload).encode("utf-8")
            accepted_http_statuses = (200, 409) if (zmq_sent and self._high_rate_transport == "both") else (200,)
            if self._http_post(
                "/api/nav/velocity",
                data,
                timeout=0.1,
                accepted_statuses=accepted_http_statuses,
            ):
                self._cmd_vel_send_http_count += 1
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

    def _handle_camera_info(self, msg: 'CameraInfo') -> None:
        """Update camera intrinsics from CameraInfo K matrix when valid."""
        try:
            if len(msg.k) < 9:
                return

            fx = float(msg.k[0])
            fy = float(msg.k[4])
            cx = float(msg.k[2])
            cy = float(msg.k[5])

            if fx <= 0.0 or fy <= 0.0:
                return

            self._camera_fx = fx
            self._camera_fy = fy
            self._camera_cx = cx
            self._camera_cy = cy

            if not self._camera_intrinsics_received:
                self._camera_intrinsics_received = True
                self.get_logger().info(
                    "Camera intrinsics received from CameraInfo: "
                    f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
                )
        except Exception:
            pass
    
    def _handle_image(self, msg: 'Image') -> None:
        """Store latest camera image for HSV color verification."""
        try:
            with self._image_lock:
                self._latest_image = bytes(msg.data)
                self._image_width = msg.width
                self._image_height = msg.height
                self._image_encoding = msg.encoding if hasattr(msg, 'encoding') and msg.encoding else 'rgb8'
                self._image_step = int(msg.step) if hasattr(msg, 'step') and msg.step else 0
        except Exception:
            pass

    def _verify_hsv_color(self, bbox_x: float, bbox_y: float,
                          bbox_w: float, bbox_h: float) -> str:
        """
        Cross-check detection color using target_localizer HSV verifier.

        Returns a verified color name, or "" when verification is unavailable
        or confidence is too low.
        """
        if self._color_verifier is None or not CV2_AVAILABLE:
            return ""

        with self._image_lock:
            image = self._latest_image
            img_w = self._image_width
            img_h = self._image_height
            img_encoding = self._image_encoding
            img_step = self._image_step

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

        encoding = (img_encoding or "rgb8").lower()
        if encoding == "mono8":
            channels = 1
            is_bgr = True
        elif encoding == "bgr8":
            channels = 3
            is_bgr = True
        elif encoding == "rgba8":
            channels = 4
            is_bgr = False
        elif encoding == "bgra8":
            channels = 4
            is_bgr = True
        else:
            channels = 3
            is_bgr = False

        min_row_stride = img_w * channels
        row_stride = img_step if isinstance(img_step, int) and img_step >= min_row_stride else min_row_stride
        required_size = img_h * row_stride
        if len(image) < required_size:
            return ""

        try:
            image_rows = np.frombuffer(image[:required_size], dtype=np.uint8).reshape((img_h, row_stride))
            roi_flat = image_rows[y1:y2, x1 * channels:x2 * channels]
            if roi_flat.size == 0:
                return ""

            roi = roi_flat.reshape((y2 - y1, x2 - x1, channels))
            roi = np.ascontiguousarray(roi)

            if channels == 1:
                roi_bgr = cv2.cvtColor(roi[:, :, 0], cv2.COLOR_GRAY2BGR)
            elif is_bgr and channels == 4:
                roi_bgr = cv2.cvtColor(roi, cv2.COLOR_BGRA2BGR)
            elif is_bgr:
                roi_bgr = roi
            elif channels == 4:
                roi_bgr = cv2.cvtColor(roi, cv2.COLOR_RGBA2BGR)
            else:
                roi_bgr = cv2.cvtColor(roi, cv2.COLOR_RGB2BGR)

            color, ratio = self._color_verifier.classify_roi(
                roi_bgr,
                (0, 0, roi_bgr.shape[1], roi_bgr.shape[0]),
            )
            color_name = str(getattr(color, "value", color)).lower()
            if color_name == "unknown" or ratio < self._hsv_verify_min_ratio:
                return ""
            return color_name
        except Exception:
            return ""

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
            source_timestamp = None
            if detections:
                raw_source_timestamp = getattr(detections[0], "timestamp", None)
                if raw_source_timestamp is None and isinstance(detections[0], dict):
                    raw_source_timestamp = detections[0].get("timestamp")
                try:
                    if raw_source_timestamp is not None:
                        source_timestamp = float(raw_source_timestamp)
                        if not math.isfinite(source_timestamp):
                            source_timestamp = None
                except (TypeError, ValueError):
                    source_timestamp = None

            payload = {
                "detections": [asdict(d) for d in detections],
                "count": len(detections),
                "source_timestamp": source_timestamp,
            }

            # Try ZMQ first (non-blocking, ~50us), fall back to HTTP
            zmq_ok = False
            if self._use_high_rate_zmq:
                zmq_ok = self._send_high_rate_zmq(HIGH_RATE_MSG_TYPE_DETECTIONS, payload)
                if zmq_ok:
                    self._detection_send_count += 1

            # HTTP fallback (or dual-send in "both" mode)
            if not zmq_ok or self._high_rate_transport == "both":
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
    

    def _handle_voxel_marker(self, msg: 'Marker') -> None:
        """Handle nvblox CUBE_LIST marker data and forward voxel payloads."""
        if not self._enable_mesh:
            return
        try:
            now = time.time()

            if msg.type != 6:  # CUBE_LIST
                return

            n_pts = len(msg.points)

            # Track empty markers before rate limiting so we count all messages.
            if n_pts == 0:
                self._voxel_empty_count += 1
                if self._voxel_empty_count == 20:
                    self.get_logger().warning(
                        f"{self._mesh_topic} has been empty for 20 consecutive messages"
                    )
                self._send_empty_mesh_heartbeat(mode="voxel", timestamp=now)
                return

            # Got real points -- reset empty counter
            self._voxel_empty_count = 0

            # Now apply rate limit only for sending (not for tracking empty)
            if now - self._last_mesh_send_time < self._mesh_send_interval_s:
                return

            voxel_size = msg.scale.x if msg.scale.x > 0.0 else 0.05
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
                if has_colors:
                    c = msg.colors[i]
                    voxels.append({
                        "p": [round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                        "c": [
                            int(max(0.0, min(1.0, c.r)) * 255.0),
                            int(max(0.0, min(1.0, c.g)) * 255.0),
                            int(max(0.0, min(1.0, c.b)) * 255.0),
                        ],
                    })
                else:
                    voxels.append({"p": [round(p.x, 4), round(p.y, 4), round(p.z, 4)]})

            drone_pose = self._get_drone_body_pose()

            mesh_data = {
                "voxels": voxels,
                "voxel_size": round(voxel_size, 4),
                "mode": "voxel",
                "total_voxels": n_pts,
                "sent_voxels": len(voxels),
                "timestamp": now,
                "frame_id": "ros_optical",  # Same frame as mesh vertices and drone_position/attitude
                "clear": False,
            }

            if drone_pose:
                mesh_data["drone_position"] = drone_pose["position"]
                mesh_data["drone_attitude"] = drone_pose["attitude"]

            self._mesh_recv_count += 1
            self._send_mesh_to_edge_core(mesh_data)

        except Exception as e:
            self.get_logger().error(f"Voxel marker processing error: {e}")

    def _poll_mesh_output_mode(self) -> None:
        """Poll Edge Core for runtime mesh output mode (voxel-only)."""
        now = time.time()
        if now - self._mesh_mode_last_poll < self._mesh_mode_poll_interval:
            return
        self._mesh_mode_last_poll = now

        try:
            url = f"{self._base_url}/api/task/2/slam/mesh/mode"
            headers = self._build_internal_headers()
            req = Request(url, headers=headers)
            with urlopen(req, timeout=0.1) as resp:
                data = json.loads(resp.read().decode("utf-8"))

            self._mesh_output_mode = "voxel"
        except Exception:
            # Keep last known mode on transient API/network errors.
            pass

    def _poll_gimbal_angle(self) -> None:
        """Poll camera gimbal servo angle from Edge Core API.

        Caches the result so the fast-path (_get_drone_body_pose) never blocks
        on HTTP. Called from a dedicated polling timer, not ROS callbacks.
        """
        now = time.time()
        if now - self._gimbal_last_poll < self._gimbal_poll_interval:
            return
        self._gimbal_last_poll = now

        try:
            url = f"{self._base_url}/api/servo/camera/tilt"
            headers = self._build_internal_headers()
            req = Request(url, headers=headers)
            with urlopen(req, timeout=0.1) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            feedback_angle = data.get("feedback_angle")
            if feedback_angle is None:
                feedback_angle = data.get("angle", 90.0)
            angle = float(feedback_angle)
            angle = max(0.0, min(180.0, angle))
            self._gimbal_angle_deg = angle
            # 90 deg = level forward (pitch = 0)
            self._gimbal_pitch_rad = math.radians(angle - 90.0)
        except Exception:
            pass  # Keep last known angle

    def _get_drone_body_pose(self) -> Optional[dict]:
        """
        Compute the drone body (base_link) pose from the camera odom pose.

        The ZED odom topic reports the *camera* position/orientation, which
        includes the servo gimbal tilt and mounting offset.  To show the
        correct drone position in the SLAM 3D view, we reverse-transform:

            T_body = T_camera * inv(T_servo) * inv(T_mount)

        The mount offset and servo pitch match servo_tf_publisher.py so the
        drone marker sits at the airframe, not at the camera.

        Returns position (x, y, z) and attitude (roll, pitch, yaw) in the
        odom frame — same coordinate frame as the nvblox mesh.
        """
        with self._lock:
            vio = self._latest_vio

        if vio is None:
            return None

        # Camera pose in odom frame (from ZED odom topic)
        cx, cy, cz = vio.ros_x, vio.ros_y, vio.ros_z
        c_roll, c_pitch, c_yaw = vio.ros_roll, vio.ros_pitch, vio.ros_yaw

        # Mirror VIO attitude mapping: pitch corrected by servo tilt only.
        body_roll = c_roll
        body_pitch = c_pitch - self._gimbal_pitch_rad
        body_yaw = c_yaw

        # Compute the mount offset vector rotated into the odom frame,
        # then subtract it from the camera position to get body position.
        # Mount offset is in body frame: (forward, lateral, down).
        mx, my, mz = self._gimbal_mount_offset

        # Rotation matrix from body orientation (simplified: roll≈0 for a drone)
        cos_p = math.cos(body_pitch)
        sin_p = math.sin(body_pitch)
        cos_y = math.cos(body_yaw)
        sin_y = math.sin(body_yaw)
        cos_r = math.cos(body_roll)
        sin_r = math.sin(body_roll)

        # Full rotation of mount offset from body frame to odom frame
        # R = Rz(yaw) * Ry(pitch) * Rx(roll) applied to (mx, my, mz)
        # Then add servo pitch rotation on top for camera offset
        #
        # But we want: body_pos = camera_pos - R_body * (mount_offset)
        # because camera_pos = body_pos + R_body * mount_offset (no servo
        # contribution to translation since servo is pure rotation at pivot)
        ox = (cos_y * cos_p) * mx + (cos_y * sin_p * sin_r - sin_y * cos_r) * my + (cos_y * sin_p * cos_r + sin_y * sin_r) * mz
        oy = (sin_y * cos_p) * mx + (sin_y * sin_p * sin_r + cos_y * cos_r) * my + (sin_y * sin_p * cos_r - cos_y * sin_r) * mz
        oz = (-sin_p) * mx + (cos_p * sin_r) * my + (cos_p * cos_r) * mz

        body_x = cx - ox
        body_y = cy - oy
        body_z = cz - oz

        return {
            "position": {
                "x": round(body_x, 4),
                "y": round(body_y, 4),
                "z": round(body_z, 4),
            },
            "attitude": {
                "roll": round(body_roll, 4),
                "pitch": round(body_pitch, 4),
                "yaw": round(body_yaw, 4),
            },
        }
    
    def _send_mesh_to_edge_core(self, mesh_data: dict) -> None:
        """Queue mesh data for background send to edge_core (non-blocking).

        Serializes JSON on the caller thread (ROS callback) and hands off
        the bytes to the background mesh sender. If the sender is still
        busy with a previous payload, the old one is replaced — only the
        latest mesh matters.
        """
        if not self._enable_mesh:
            return

        try:
            # Keep mesh payload format aligned with API defaults for compatibility.
            data = json.dumps(mesh_data).encode("utf-8")
            ctype = "application/json"
        except Exception as e:
            self._send_errors += 1
            self.get_logger().error(f"Mesh serialize error: {e}")
            return

        with self._mesh_pending_lock:
            self._mesh_pending_data = data
            self._mesh_pending_ctype = ctype
            self._mesh_pending_meta = mesh_data.get('mode', 'voxel'), mesh_data.get('total_voxels', 0)
        self._mesh_send_event.set()

    def _mesh_sender_loop(self) -> None:
        """Background thread that sends queued mesh data via HTTP."""
        while not self._mesh_sender_stop.is_set():
            # Wait for data or stop signal
            self._mesh_send_event.wait(timeout=1.0)
            if self._mesh_sender_stop.is_set():
                break
            self._mesh_send_event.clear()

            # Grab the latest payload (atomic swap)
            with self._mesh_pending_lock:
                data = self._mesh_pending_data
                ctype = getattr(self, '_mesh_pending_ctype', 'application/json')
                meta = getattr(self, '_mesh_pending_meta', ('voxel', 0))
                self._mesh_pending_data = None

            if data is None:
                continue

            try:
                if self._http_post("/api/task/2/slam/mesh/update", data, timeout=2.0, content_type=ctype):
                    self._mesh_send_count += 1
                    self._last_mesh_send_time = time.time()
                    if self._mesh_send_count % 10 == 1:
                        mode, count = meta
                        self.get_logger().info(
                            f"Mesh sent: {count} voxels (mode={mode})"
                        )
                else:
                    self._send_errors += 1
            except Exception as e:
                self._send_errors += 1
                if self._send_errors % 50 == 1:
                    self.get_logger().warning(f"Failed to send mesh: {e}")

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
        mesh_data.update({"voxels": [], "voxel_size": 0.0, "total_voxels": 0, "sent_voxels": 0})

        drone_pose = self._get_drone_body_pose()
        if drone_pose:
            mesh_data["drone_position"] = drone_pose["position"]
            mesh_data["drone_attitude"] = drone_pose["attitude"]

        self._send_mesh_to_edge_core(mesh_data)
        self._last_empty_mesh_send_time = timestamp

    def destroy_node(self) -> bool:
        """Destroy ROS node and cleanup transport resources."""
        self._stop_high_rate_publisher()
        # Stop background mesh sender
        self._mesh_sender_stop.set()
        self._mesh_send_event.set()  # wake up the thread so it exits
        if self._mesh_sender_thread.is_alive():
            self._mesh_sender_thread.join(timeout=3.0)
        with self._http_lock:
            try:
                self._http_conn.close()
            except Exception:
                pass
        return super().destroy_node()
    
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

    @staticmethod
    def _wrap_angle_rad(angle: float) -> float:
        """Normalize an angle to [-pi, pi] for stable downstream interpolation."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _quat_to_ned_euler(
        self, x: float, y: float, z: float, w: float
    ) -> tuple[float, float, float]:
        """Convert ROS optical-frame quaternion attitude to NED roll/pitch/yaw."""
        # Quaternion -> rotation matrix in ROS optical basis.
        r = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]

        # Basis transform: optical (x-right, y-down, z-forward) -> NED (x-forward, y-right, z-down)
        b = (
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        )

        # Change basis for rotation matrix: R_ned = B * R_optical * B^T
        br = [
            [sum(b[i][k] * r[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        r_ned = [
            [sum(br[i][k] * b[j][k] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

        roll = math.atan2(r_ned[2][1], r_ned[2][2])
        sinp = max(-1.0, min(1.0, -r_ned[2][0]))
        pitch = math.asin(sinp)
        yaw = math.atan2(r_ned[1][0], r_ned[0][0])
        return roll, pitch, yaw
    
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        return {
            "vio_received": self._vio_recv_count,
            "vio_sent": self._vio_send_count,
            "vio_sent_zmq": self._vio_send_zmq_count,
            "vio_sent_http": self._vio_send_http_count,
            "cmd_vel_received": self._cmd_vel_recv_count,
            "cmd_vel_sent": self._cmd_vel_send_count,
            "cmd_vel_sent_zmq": self._cmd_vel_send_zmq_count,
            "cmd_vel_sent_http": self._cmd_vel_send_http_count,
            "mesh_received": self._mesh_recv_count,
            "mesh_sent": self._mesh_send_count,
            "servo_received": self._servo_recv_count,
            "servo_sent": self._servo_send_count,
            "detection_received": self._detection_recv_count,
            "detection_sent": self._detection_send_count,
            "send_errors": self._send_errors,
            "high_rate_transport_requested": self._high_rate_transport,
            "high_rate_transport_effective": self._effective_high_rate_transport(),
            "high_rate_zmq_pub_mode": self._high_rate_zmq_pub_mode,
            "nav_control_enabled": self._enable_nav_control,
            "mesh_enabled": self._enable_mesh,
            "mesh_output_mode": self._mesh_output_mode,
            "servo_enabled": self._enable_servo,
            "detections_enabled": self._enable_detections,
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
    parser.add_argument("--vio-topic", default="/zed/zed_node/odom",
                        help="VIO odometry topic")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel",
                        help="Navigation velocity command topic")
    parser.add_argument("--mesh-topic", default="/nvblox_node/color_layer_marker",
                        help="Nvblox marker topic (visualization_msgs/Marker CUBE_LIST)")
    parser.add_argument("--mesh-output-mode", default="voxel", choices=["voxel"],
                        help="Mesh payload mode sent to Edge Core")
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
    parser.add_argument(
        "--high-rate-transport",
        default="zmq",
        choices=["zmq", "http", "both"],
        help="Transport for high-rate VIO/cmd_vel/detection streams (zmq preferred, HTTP fallback auto-activates)",
    )
    parser.add_argument(
        "--high-rate-zmq-endpoint",
        default=None,
        help="Optional ZMQ endpoint override for high-rate VIO/cmd_vel IPC",
    )
    parser.add_argument(
        "--high-rate-zmq-pub-mode",
        default=os.environ.get("NOMAD_HIGH_RATE_ZMQ_PUB_MODE", "connect"),
        choices=["bind", "connect"],
        help="ZMQ PUB socket mode for high-rate IPC",
    )
    
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
        mesh_output_mode=args.mesh_output_mode,
        servo_topic=args.servo_topic,
        detection_topic=args.detection_topic,
        send_rate_hz=args.rate,
        enable_nav_control=not args.disable_nav,
        enable_mesh=not args.disable_mesh,
        enable_servo=not args.disable_servo,
        enable_detections=not args.disable_detections,
        high_rate_transport=args.high_rate_transport,
        high_rate_zmq_endpoint=args.high_rate_zmq_endpoint,
        high_rate_zmq_pub_mode=args.high_rate_zmq_pub_mode,
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
