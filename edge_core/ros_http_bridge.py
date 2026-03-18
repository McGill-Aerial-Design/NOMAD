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

# ZED object detection messages (custom circle detection via YOLO26)
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
    ros_x: float = 0.0
    ros_y: float = 0.0
    ros_z: float = 0.0
    # Raw ROS-frame orientation (from odom topic, same frame as mesh)
    ros_roll: float = 0.0
    ros_pitch: float = 0.0
    ros_yaw: float = 0.0


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
    """Detected object from ZED custom OD (YOLO26 circle detection)."""
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
        
        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._send_interval = 1.0 / send_rate_hz
        self._enable_nav_control = enable_nav_control
        self._enable_mesh = enable_mesh and NVBLOX_AVAILABLE
        self._enable_servo = enable_servo
        self._enable_detections = enable_detections and ZED_OD_AVAILABLE
        
        # Persistent HTTP connection (keep-alive) for efficiency
        self._http_conn = HTTPConnection(host, port, timeout=1.0)
        self._http_lock = threading.Lock()
        
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
        self._last_cmd_vel_send_time = 0.0
        self._last_mesh_send_time = 0.0
        self._mesh_send_interval_s = 0.1  # 10 Hz mesh update cap
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
        # Prefer color_layer_marker (per-voxel) over raw Mesh (per-block)
        self._use_voxel_marker = False
        if self._enable_mesh and MARKER_AVAILABLE:
            voxel_topic = "/nvblox_node/color_layer_marker"
            self.create_subscription(
                Marker,
                voxel_topic,
                self._handle_voxel_marker,
                mesh_qos,
            )
            self._use_voxel_marker = True
            self.get_logger().info(f"Subscribed to per-voxel marker: {voxel_topic}")
        if self._enable_mesh and NVBLOX_AVAILABLE:
            self.create_subscription(
                Mesh,
                mesh_topic,
                self._handle_mesh,
                mesh_qos,
            )
            self.get_logger().info(f"Subscribed to mesh (fallback): {mesh_topic}")
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
        
        # Subscribe to ZED custom object detections (YOLO26 circle detection)
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
                confidence=1.0,
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
            try:
                self._http_conn.request(
                    "POST", path, body=data,
                    headers={"Content-Type": "application/json", "Connection": "keep-alive"}
                )
                resp = self._http_conn.getresponse()
                resp.read()  # Drain response to allow connection reuse
                return resp.status == 200
            except Exception:
                # Reconnect on failure
                try:
                    self._http_conn.close()
                    self._http_conn.connect()
                except Exception:
                    pass
                return False

    def _send_to_edge_core(self) -> None:
        """Send latest VIO data to edge_core via HTTP."""
        with self._lock:
            vio = self._latest_vio
        
        if vio is None:
            return
        
        # Rate limit
        now = time.time()
        if now - self._last_send_time < self._send_interval:
            return
        self._last_send_time = now
        
        try:
            data = json.dumps(asdict(vio)).encode("utf-8")
            if self._http_post("/api/vio/update", data):
                self._vio_send_count += 1
            else:
                self._send_errors += 1
                    
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
        """
        if not self._enable_servo:
            return
        
        try:
            angle = float(msg.data)
            self._servo_recv_count += 1
            
            # Clamp to valid range
            angle = max(0.0, min(180.0, angle))
            
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
            if self._http_post(path, b""):
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
    
    def _handle_detections(self, msg) -> None:
        """
        Handle ZED custom object detections (YOLO26 circle detection).
        
        The ZED SDK runs the ONNX model with TensorRT, detects colored circles,
        and provides 3D positions via stereo depth. This handler converts the
        zed_interfaces/ObjectsStamped message into DetectedObject dataclasses
        and forwards them to Edge Core.
        """
        if not self._enable_detections:
            return
        
        try:
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
                
                det = DetectedObject(
                    timestamp=time.time(),
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
                )
                detections.append(det)
            
            with self._lock:
                self._latest_detections = detections
                self._detection_recv_count += 1
            
            if detections:
                self._send_detections_to_edge_core(detections)
                
        except Exception as e:
            self.get_logger().error(f"Detection processing error: {e}")
    
    def _send_detections_to_edge_core(self, detections: list) -> None:
        """
        Send object detections to Edge Core via HTTP POST.
        
        Rate limited to 5 Hz to avoid overwhelming the API while
        keeping detection data fresh for SLAM visualization.
        """
        now = time.time()
        if now - self._last_detection_send_time < 0.2:  # 5 Hz max
            return
        self._last_detection_send_time = now
        
        try:
            payload = {
                "detections": [asdict(d) for d in detections],
                "count": len(detections),
            }
            data = json.dumps(payload).encode("utf-8")
            if self._http_post("/api/detections/update", data):
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
        Handle mesh data from nvblox for 3D visualization (block-only fallback).

        Only used when color_layer_marker is not available.
        Sends block indices + average color per block.
        """
        if not self._enable_mesh:
            return
        # Skip block mode if we have per-voxel marker data
        if self._use_voxel_marker:
            return

        try:
            # Rate limit mesh updates (max 10 Hz to avoid overwhelming)
            now = time.time()
            if now - self._last_mesh_send_time < self._mesh_send_interval_s:  # 10 Hz max
                return

            block_size = msg.block_size_m if hasattr(msg, 'block_size_m') else 0.2

            # Block-only mode: extract index + average color per block
            blocks = []
            for i, idx in enumerate(msg.block_indices[:2000]):
                block_entry = {
                    "index": [int(idx.x), int(idx.y), int(idx.z)],
                }

                # Compute average color from block vertices if available
                if i < len(msg.blocks) and hasattr(msg.blocks[i], 'colors') and msg.blocks[i].colors:
                    colors = msg.blocks[i].colors
                    if colors:
                        n = len(colors)
                        avg_r = int(sum(c.r for c in colors) / n * 255)
                        avg_g = int(sum(c.g for c in colors) / n * 255)
                        avg_b = int(sum(c.b for c in colors) / n * 255)
                        block_entry["color"] = [avg_r, avg_g, avg_b]

                blocks.append(block_entry)

            if not blocks:
                return  # Skip empty meshes

            # Get camera pose from TF
            camera_pose = self._get_camera_pose()

            # Create lightweight mesh data payload
            mesh_data = {
                "blocks": blocks,
                "block_size": block_size,
                "mode": "block",
                "total_blocks": len(blocks),
                "timestamp": now,
                "frame_id": msg.header.frame_id if hasattr(msg, 'header') else "map",
                "clear": msg.clear if hasattr(msg, 'clear') else False,
            }

            # Add camera pose if available
            if camera_pose:
                mesh_data["drone_position"] = camera_pose["position"]
                mesh_data["drone_attitude"] = camera_pose["attitude"]

            self._mesh_recv_count += 1
            self._send_mesh_to_edge_core(mesh_data)

        except Exception as e:
            self.get_logger().error(f"Mesh processing error: {e}")
    
    def _handle_voxel_marker(self, msg: 'Marker') -> None:
        """
        Handle per-voxel colored data from nvblox color_layer_marker topic.
        
        This gives individual voxel positions + colors (Marker type CUBE_LIST),
        producing a much finer 3D map than the block-only approach.
        """
        if not self._enable_mesh:
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
                        "falling back to /nvblox_node/mesh (block mode)"
                    )
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

            # Cap at 8000 voxels per update to limit bandwidth (~320KB)
            limit = min(n_pts, 8000)

            voxels = []
            for i in range(limit):
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
                "frame_id": msg.header.frame_id,
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
                    count = mesh_data.get('total_voxels', mesh_data.get('total_blocks', 0))
                    unit = "voxels" if mesh_data.get('mode') == 'voxel' else "blocks"
                    self.get_logger().info(
                        f"Mesh sent: {count} {unit} "
                        f"(mode={mesh_data.get('mode', 'block')})"
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
    args = parser.parse_args()
    
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
    finally:
        stats = bridge.get_stats()
        logger.info(f"Bridge stats: {stats}")
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
