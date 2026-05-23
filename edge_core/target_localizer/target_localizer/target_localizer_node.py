"""
target_localizer_node.py

Main ROS 2 node for AEAC 2026 Task 1 target detection + GPS localization.

The ground station now owns the building model: it stores corners, runs
calibration, classifies faces/walls/heights, and generates the spatial
target descriptions. This node's job is reduced to:

  1. Subscribe to ZED RGB + depth, drone pose (MAVROS), servo angle.
  2. On button press, run HSV circle detection.
  3. Back-project each detection through depth + servo tilt + drone heading
     into an absolute (lat, lon, height_agl) tuple.
  4. Hand that off to the GCS via /api/task/1/target/list_structured;
     anything resembling "north face" or "0.4 m above ground near corner 2"
     is generated on the C# side from those raw coordinates.

Topics subscribed:
  - /zed2i/zed_node/rgb/image_rect_color (sensor_msgs/Image)
  - /zed2i/zed_node/depth/depth_registered (sensor_msgs/Image)
  - /zed2i/zed_node/left_camera_frame (camera_info for intrinsics)
  - /mavros/global_position/global (NavSatFix - drone GPS)
  - /mavros/local_position/pose (PoseStamped - drone local pose)
  - /mavros/global_position/compass_hdg (Float64 - heading)
  - /servo/angle (Float64 - current servo pitch in degrees)

Services:
  - ~/capture_target (std_srvs/Trigger) - run detection + localization
  - ~/save_targets (std_srvs/Trigger) - write .txt file
  - ~/clear_targets (std_srvs/Trigger) - drop everything from memory
  - ~/delete_target (std_srvs/Trigger) - drop one by ID (uses sidecar JSON)
  - ~/set_ground_alt (std_srvs/Trigger) - latch current AGL as ground=0

Parameters (set via YAML config):
  - team_name
  - output_dir
  - dedup_radius_m  (meters; converted to lat/lon delta internally)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from cv_bridge import CvBridge

import numpy as np
import cv2
import math
import os
import shutil
import threading
import time
import traceback
import urllib.request
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .detectors import CircleDetector, ColorVerifier, TargetColor


_EARTH_RADIUS_M = 6_371_000.0


def _offset_gps(lat: float, lon: float, east_m: float, north_m: float):
    """Equirectangular: add a local-ENU offset (in meters) onto a GPS fix."""
    out_lat = lat + math.degrees(north_m / _EARTH_RADIUS_M)
    out_lon = lon + math.degrees(east_m / (_EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return out_lat, out_lon


def _gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Small-angle equirectangular distance — fine for the < 1 km radii we deal with."""
    avg_lat = math.radians(0.5 * (lat1 + lat2))
    dx = math.radians(lon2 - lon1) * _EARTH_RADIUS_M * math.cos(avg_lat)
    dy = math.radians(lat2 - lat1) * _EARTH_RADIUS_M
    return math.hypot(dx, dy)


def target_letter_from_index(i: int) -> str:
    """Map 0->A, 25->Z, 26->AA, 27->AB, ... (Excel-column style, no zero)."""
    if i < 0:
        raise ValueError("target index must be non-negative")
    letters = ""
    n = i
    while True:
        letters = chr(ord("A") + (n % 26)) + letters
        n = n // 26 - 1
        if n < 0:
            break
    return letters


@dataclass
class TargetRecord:
    """A confirmed target with absolute GPS + AGL height.

    The Jetson does not classify which building face the target is on or
    write a natural-language description — the GCS does all of that from
    these raw coordinates. We only carry the bare-minimum identification
    fields (id, color, timestamp, confidence, image).
    """

    target_id: str
    color: TargetColor
    lat: float
    lon: float
    height_agl: float
    timestamp: float
    confidence: float
    image_path: Optional[str] = None
    approved: bool = False
    raw_data: Optional[dict] = None


class TargetLocalizerNode(Node):
    def __init__(self):
        super().__init__("target_localizer")

        # ----- Parameters ----- #
        # Building geometry parameters used to live here (center_lat/lon, height,
        # corner_lats/lons, rectangle fallback). They were removed when the GCS
        # took over face/wall classification — see Task1 module on the C# side.
        self.declare_parameter("team_name", "MAD")
        self.declare_parameter("output_dir", "/home/mad/targets")
        # Keep at most this many timestamped capture folders. Older ones are
        # swept on each new capture to keep the Jetson NVMe bounded.
        self.declare_parameter("capture_retention_keep_last", 200)
        self.declare_parameter("dedup_radius_m", 0.5)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("camera.gps_offset_forward_m", 0.35)
        self.declare_parameter("camera.gps_offset_right_m", 0.0)
        # HSV circle detector knobs. Exposed so we can tune live via
        #   ros2 param set /target_localizer circle.min_circularity 0.65
        # without rebuilding the container.
        self.declare_parameter("circle.min_radius_px", 12)
        self.declare_parameter("circle.max_radius_px", 400)
        self.declare_parameter("circle.min_circularity", 0.70)
        self.declare_parameter("circle.min_solidity", 0.80)
        self.declare_parameter("circle.blur_kernel", 7)

        # Load parameters
        self.team_name = self.get_parameter("team_name").value
        self.output_dir = self.get_parameter("output_dir").value
        self._capture_retention_keep_last = int(
            self.get_parameter("capture_retention_keep_last").value
        )
        self.dedup_radius = self.get_parameter("dedup_radius_m").value

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.camera_gps_offset_forward_m = float(
            self.get_parameter("camera.gps_offset_forward_m").value
        )
        self.camera_gps_offset_right_m = float(
            self.get_parameter("camera.gps_offset_right_m").value
        )

        os.makedirs(self.output_dir, exist_ok=True)

        # ----- Detectors ----- #
        # Defaults: 12px min radius, 0.70 circularity, 0.80 solidity.
        # Looser than the distant-blob filter but still rejects obvious noise.
        # Live-tunable via the circle.* ROS parameters declared above.
        self.circle_detector = CircleDetector(
            min_radius_px=int(self.get_parameter("circle.min_radius_px").value),
            max_radius_px=int(self.get_parameter("circle.max_radius_px").value),
            min_circularity=float(self.get_parameter("circle.min_circularity").value),
            min_solidity=float(self.get_parameter("circle.min_solidity").value),
            blur_kernel=int(self.get_parameter("circle.blur_kernel").value),
        )
        self.get_logger().info(f"Circle detector backend: {self.circle_detector.backend}")

        self.color_verifier = ColorVerifier()
        self.bridge = CvBridge()

        # ----- State ----- #
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_rgb_stamp = None
        self.drone_lat: float = 0.0
        self.drone_lon: float = 0.0
        self.has_gps_fix: bool = False
        self.has_local_pose: bool = False
        self.has_mavros_pose: bool = False  # set when /mavros/local_position/pose arrives
        self.drone_local_east: float = 0.0
        self.drone_local_north: float = 0.0
        self.drone_alt: float = 0.0  # AGL from rangefinder or baro
        self.drone_heading: float = 0.0
        self.servo_pitch_deg: float = 0.0
        self.camera_fx: float = 0.0
        self.camera_fy: float = 0.0
        self.camera_cx: float = 0.0
        self.camera_cy: float = 0.0
        self.intrinsics_received: bool = False
        self.rgb_msg_count: int = 0
        self.depth_msg_count: int = 0
        self.cam_info_msg_count: int = 0
        self._last_rgb_error_ts: float = 0.0
        self._last_depth_error_ts: float = 0.0

        self.targets: List[TargetRecord] = []
        self.next_target_index: int = 0
        self.ground_alt_offset: float = 0.0

        self._latest_detections_json: str = "{}"

        # Push detection status to edge_core API (avoids docker exec polling)
        _edge_core_host = os.environ.get("NOMAD_EDGE_CORE_HOST", "localhost")
        self._edge_core_det_url = f"http://{_edge_core_host}:8000/api/task/1/target/detection_status/update"
        self._edge_core_internal_token = (os.environ.get("NOMAD_INTERNAL_TOKEN") or "").strip() or None

        # ----- QoS ----- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        zed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Subscribers ----- #
        self.rgb_sub = self.create_subscription(
            Image,
            "/zed2i/zed_node/rgb/image_rect_color",
            self._rgb_callback,
            zed_qos,
        )
        self.rgb_sub_direct = self.create_subscription(
            Image, "/zed/zed_node/rgb/color/rect/image", self._rgb_callback, zed_qos
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/zed2i/zed_node/depth/depth_registered",
            self._depth_callback,
            zed_qos,
        )
        self.depth_sub_direct = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self._depth_callback,
            zed_qos,
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            "/zed2i/zed_node/rgb/camera_info",
            self._cam_info_callback,
            zed_qos,
        )
        self.cam_info_sub_direct = self.create_subscription(
            CameraInfo,
            "/zed/zed_node/rgb/color/rect/camera_info",
            self._cam_info_callback,
            zed_qos,
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_callback, sensor_qos
        )
        self.heading_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self._heading_callback,
            sensor_qos,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_callback, sensor_qos
        )
        self.zed_odom_sub = self.create_subscription(
            Odometry, "/zed/zed_node/odom", self._zed_odom_callback, zed_qos
        )
        self.servo_sub = self.create_subscription(
            Float64, "/servo/angle", self._servo_callback, sensor_qos
        )

        # ----- Services ----- #
        self.capture_srv = self.create_service(
            Trigger, "/target_localizer/capture_target", self._capture_callback
        )
        self.save_srv = self.create_service(
            Trigger, "/target_localizer/save_targets", self._save_callback
        )
        self.clear_srv = self.create_service(
            Trigger, "/target_localizer/clear_targets", self._clear_targets_callback
        )
        self.ground_alt_srv = self.create_service(
            Trigger, "/target_localizer/set_ground_alt", self._set_ground_alt_callback
        )
        self.delete_target_srv = self.create_service(
            Trigger, "/target_localizer/delete_target", self._delete_target_callback
        )
        # Building-model services (print_model, set_building_corners,
        # regenerate_descriptions, set_target_plane) were removed when the
        # GCS took ownership of the building geometry.

        self._detection_status_pub = self.create_publisher(
            Float64, "/target_localizer/detection_status", 10
        )
        self._detection_status_json_pub = self.create_publisher(
            String, "/target_localizer/detection_status_json", 10
        )

        # Emit concrete service registrations for runtime diagnostics.
        for name, types in self.get_service_names_and_types():
            if "target_localizer" in name:
                self.get_logger().info(f"Registered service: {name} types={types}")

        # Periodic input health watchdog so startup issues are visible in logs.
        self.create_timer(5.0, self._input_watchdog_callback)

        self.create_timer(0.5, self._detection_status_timer_callback)

        self.get_logger().info(
            "Target localizer node started. "
            "Call /target_localizer/capture_target to detect and describe targets."
        )

    # ================================================================ #
    #  Subscriber callbacks
    # ================================================================ #
    def _rgb_callback(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_rgb_stamp = msg.header.stamp
            self.rgb_msg_count += 1
        except Exception as e:
            now = time.time()
            if now - self._last_rgb_error_ts > 5.0:
                self._last_rgb_error_ts = now
                self.get_logger().warn(
                    f"RGB conversion failed (encoding={getattr(msg, 'encoding', '?')}): {e}"
                )

    def _depth_callback(self, msg: Image):
        try:
            # ZED publishes depth as 32FC1 in meters
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.depth_msg_count += 1
        except Exception as e:
            now = time.time()
            if now - self._last_depth_error_ts > 5.0:
                self._last_depth_error_ts = now
                self.get_logger().warn(
                    f"Depth conversion failed (encoding={getattr(msg, 'encoding', '?')}): {e}"
                )

    def _cam_info_callback(self, msg: CameraInfo):
        self.cam_info_msg_count += 1
        if not self.intrinsics_received:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self.intrinsics_received = True
            self.get_logger().info(
                f"Camera intrinsics received: fx={self.camera_fx:.1f}, "
                f"fy={self.camera_fy:.1f}, cx={self.camera_cx:.1f}, cy={self.camera_cy:.1f}"
            )

    def _input_watchdog_callback(self):
        if (
            self.latest_rgb is not None
            and self.latest_depth is not None
            and self.intrinsics_received
        ):
            return

        self.get_logger().warn(
            "Waiting for camera inputs: "
            f"rgb_msg_count={self.rgb_msg_count}, "
            f"depth_msg_count={self.depth_msg_count}, "
            f"cam_info_msg_count={self.cam_info_msg_count}, "
            f"intrinsics_received={self.intrinsics_received}"
        )

    def _gps_callback(self, msg: NavSatFix):
        self.drone_lat = msg.latitude
        self.drone_lon = msg.longitude
        self.has_gps_fix = (
            math.isfinite(self.drone_lat)
            and math.isfinite(self.drone_lon)
            and not (abs(self.drone_lat) < 1e-8 and abs(self.drone_lon) < 1e-8)
        )
        # Altitude from GPS is MSL; we use local pose for AGL
        # but store GPS alt as fallback
        self.drone_gps_alt = msg.altitude

    def _heading_callback(self, msg: Float64):
        self.drone_heading = msg.data

    def _pose_callback(self, msg: PoseStamped):
        # drone_state_publisher only fills z (AGL); x/y are hardcoded to 0.0
        # and would clobber the authoritative ZED VIO east/north estimate.
        self.drone_alt = msg.pose.position.z
        self.has_local_pose = True
        self.has_mavros_pose = True

    def _zed_odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        # ZED VIO is the primary east/north source (better than MAVROS without RTK).
        self.drone_local_east = pose.position.x
        self.drone_local_north = pose.position.y
        # Do NOT trust ZED z as altitude — it's the visual-odometry origin.
        # Only fall back to it if MAVROS hasn't produced any pose yet.
        if not self.has_mavros_pose:
            self.drone_alt = pose.position.z
        self.has_local_pose = True

        # Fallback heading from odometry quaternion when compass is unavailable.
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        if yaw_deg < 0.0:
            yaw_deg += 360.0
        self.drone_heading = yaw_deg

    def _servo_callback(self, msg: Float64):
        # Positive = tilting up, negative = tilting down
        self.servo_pitch_deg = msg.data

    # ================================================================ #
    #  3D back-projection
    # ================================================================ #
    def _pixel_to_3d_local(
        self, px: int, py: int, depth_image: np.ndarray,
        servo_pitch_override: Optional[float] = None,
        max_range_m: float = 35.0,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project a pixel to 3D coordinates in the drone's local ENU frame,
        accounting for servo tilt.

        Returns (east_offset, north_offset, up) relative to the camera origin,
        or None if depth is invalid.
        """
        if not self.intrinsics_received:
            return None

        def _sample_depth(half_window: int, sample_max_range_m: float) -> Optional[float]:
            y1 = max(0, py - half_window)
            y2 = min(depth_image.shape[0], py + half_window + 1)
            x1 = max(0, px - half_window)
            x2 = min(depth_image.shape[1], px + half_window + 1)
            roi = depth_image[y1:y2, x1:x2]
            valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < sample_max_range_m)]
            if len(valid) == 0:
                return None
            return float(np.median(valid))

        # Start with a tight ROI for precision, then expand to handle common
        # depth holes on low-texture/reflective target patches. The tight
        # sample uses a slightly stricter cap so close targets don't average
        # in the far wall behind them; the expanding retries honor the
        # caller-supplied max_range_m.
        tight_cap = min(20.0, max_range_m)
        depth = _sample_depth(half_window=5, sample_max_range_m=tight_cap)
        if depth is None:
            for half_window in (12, 20, 30, 60, 100):
                depth = _sample_depth(half_window=half_window, sample_max_range_m=max_range_m)
                if depth is not None:
                    break

        if depth is None:
            # No valid depth within a 100px window around the target pixel.
            # Do NOT fall back to the global frame median: a target on a near
            # wall surrounded by a deep empty room would be geolocated tens of
            # meters past the wall, which can mislead navigation and obstacle
            # avoidance. Invalidate instead.
            self.get_logger().warn(
                f"Depth at ({px},{py}) all-NaN in ±100px window; "
                f"refusing to geolocalize (no global-median fallback)"
            )
            return None

        # Pixel to camera-frame 3D (OpenCV convention: Z forward, X right, Y down)
        cam_x = (px - self.camera_cx) * depth / self.camera_fx
        cam_y = (py - self.camera_cy) * depth / self.camera_fy
        cam_z = depth

        # Apply servo pitch rotation (rotate around camera X axis)
        # Servo pitch: positive = tilt up, negative = tilt down
        # Use captured snapshot if provided, fall back to latest live value
        pitch = servo_pitch_override if servo_pitch_override is not None else self.servo_pitch_deg
        pitch_rad = math.radians(pitch)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)

        # Rotate Y and Z by pitch
        body_x = cam_x
        body_y = cos_p * cam_y + sin_p * cam_z
        body_z = -sin_p * cam_y + cos_p * cam_z

        # Convert from camera/body frame (X-right, Y-down, Z-forward)
        # to ENU frame using drone heading
        heading_rad = math.radians(self.drone_heading)

        # Forward = body_z, Right = body_x
        # In ENU: East = Forward*sin(heading) + Right*cos(heading)
        #         North = Forward*cos(heading) - Right*sin(heading)
        east_offset = body_z * math.sin(heading_rad) + body_x * math.cos(heading_rad)
        north_offset = body_z * math.cos(heading_rad) - body_x * math.sin(heading_rad)

        # Up: drone_alt minus the downward component
        # body_y is positive downward in camera frame
        target_up = self.drone_alt - body_y

        return east_offset, north_offset, target_up

    def _pixel_to_world_gps(
        self, px: int, py: int, depth_image: np.ndarray,
        servo_pitch_override: Optional[float] = None,
        max_range_m: float = 35.0,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project a pixel to absolute (lat, lon, height_agl).

        Requires a GPS fix on the drone — we no longer have a building origin
        to anchor an ENU-only fallback against, and the GCS would have nothing
        to convert the result with anyway.
        """
        local = self._pixel_to_3d_local(px, py, depth_image, servo_pitch_override, max_range_m=max_range_m)
        if local is None:
            return None

        east_off, north_off, up = local

        if not self.has_gps_fix:
            return None

        # Apply the camera's offset from the GPS antenna in the drone heading
        # frame, then walk that meter offset onto the drone GPS as a lat/lon
        # delta. The GCS turns this into building-local ENU on its end using
        # its own (calibrated) corner list.
        heading_rad = math.radians(self.drone_heading)
        forward = self.camera_gps_offset_forward_m
        right = self.camera_gps_offset_right_m
        cam_east_off = forward * math.sin(heading_rad) + right * math.cos(heading_rad)
        cam_north_off = forward * math.cos(heading_rad) - right * math.sin(heading_rad)

        lat, lon = _offset_gps(
            self.drone_lat,
            self.drone_lon,
            cam_east_off + east_off,
            cam_north_off + north_off,
        )
        return lat, lon, up

    # ================================================================ #
    #  Target capture (button press)
    # ================================================================ #
    def _sweep_old_captures(self, keep_last: int) -> None:
        """Keep the newest `keep_last` timestamped capture folders.

        Mirrors task2_spray_artifacts.sweep_old_sessions() so Task 1 disk
        growth is bounded across long deployments.
        """
        if keep_last <= 0 or not os.path.isdir(self.output_dir):
            return
        entries = []
        for name in os.listdir(self.output_dir):
            full = os.path.join(self.output_dir, name)
            if not os.path.isdir(full):
                continue
            # Skip non-timestamp folders (e.g. ad-hoc artifacts).
            if len(name) < 8 or not name[:8].isdigit():
                continue
            try:
                mtime = os.path.getmtime(full)
            except OSError:
                continue
            entries.append((mtime, full))
        if len(entries) <= keep_last:
            return
        entries.sort(reverse=True)  # newest first
        for _mtime, full in entries[keep_last:]:
            try:
                shutil.rmtree(full)
            except OSError as e:
                self.get_logger().warn(
                    f"Capture retention: could not remove {full}: {e}"
                )

    # ================================================================ #
    def _capture_callback(self, request, response):
        """Service handler for ~/capture_target."""
        self.get_logger().info("=== CAPTURE TARGET triggered ===")

        # Validate state
        if self.latest_rgb is None or self.latest_depth is None:
            response.success = False
            response.message = (
                "No RGB or depth image available. "
                f"(rgb_msgs={self.rgb_msg_count}, depth_msgs={self.depth_msg_count}, "
                f"cam_info_msgs={self.cam_info_msg_count})"
            )
            return response

        # Log frame age for diagnostics. Intentionally non-fatal: ZED can
        # publish with sim time while rclpy reads wall time, and a noisy
        # clock comparison used to reject every capture with a cryptic 502.
        # If the stream really is frozen, the rgb_msg_count will stop
        # climbing in the 5s watchdog log instead.
        try:
            if self.latest_rgb_stamp is not None:
                stamp_sec = (
                    self.latest_rgb_stamp.sec
                    + self.latest_rgb_stamp.nanosec * 1e-9
                )
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                age = now_sec - stamp_sec
                # Only flag absurdly large gaps; small negatives are fine.
                if abs(age) < 3600:
                    self.get_logger().info(
                        f"Capture: RGB frame age = {age:.2f}s "
                        f"(rgb_msgs={self.rgb_msg_count})"
                    )
        except Exception:
            pass

        if not self.intrinsics_received:
            response.success = False
            response.message = (
                "Camera intrinsics not yet received. "
                f"(cam_info_msgs={self.cam_info_msg_count})"
            )
            return response

        # Create timestamped capture folder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        capture_dir = os.path.join(self.output_dir, timestamp)
        os.makedirs(capture_dir, exist_ok=True)

        # Bound disk growth: in a long competition deployment unsupervised
        # capture folders fill the Jetson NVMe. Mirrors task2_spray_artifacts
        # .sweep_old_sessions(): keep the most recent N timestamped folders.
        try:
            self._sweep_old_captures(keep_last=self._capture_retention_keep_last)
        except Exception as e:
            self.get_logger().warn(f"Capture retention sweep failed: {e}")

        # Snapshot current data (including servo angle to avoid stale value for 3D back-projection)
        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()
        captured_servo_pitch = self.servo_pitch_deg

        # Run HSV circle detection unless the GCS requested a one-shot
        # crosshair capture. The request is passed through a sidecar because
        # std_srvs/Trigger has no payload fields.
        force_crosshair = False
        try:
            import json as _json
            for options_path in (
                "/workspaces/isaac_ros-dev/config/capture_options.json",
                os.path.join(self.output_dir, "capture_options.json"),
            ):
                if not os.path.exists(options_path):
                    continue
                with open(options_path, "r") as f:
                    options = _json.load(f)
                force_crosshair = bool(options.get("force_crosshair"))
                try:
                    os.remove(options_path)
                except OSError:
                    pass
                break
        except Exception as e:
            self.get_logger().warn(f"Capture options read failed (ignored): {e}")

        if force_crosshair:
            self.get_logger().info("Capture: forcing crosshair fallback; circle detections ignored.")
            circles = []
        else:
            circles = self.circle_detector.detect(rgb)

        new_targets: List[TargetRecord] = []

        if len(circles) == 0:
            # No circles detected, or the GCS explicitly requested the
            # crosshair backup: use the frame center. The GCS still gets a
            # (lat, lon, height) and decides which building face / corner
            # reference to attach.
            #
            # Crosshair captures use a wider depth range (100 m) than circle
            # detection (35 m). Operators sometimes line up a far building
            # corner from a high hover where the true center-pixel depth
            # exceeds 35 m; rejecting those captures (when GPS and depth are
            # otherwise fine) was surfacing as a misleading "depth or GPS
            # invalid" error.
            crosshair_max_range_m = 100.0

            center_px = self.latest_rgb.shape[1] // 2
            center_py = self.latest_rgb.shape[0] // 2
            _chw = 5
            _cy1 = max(0, center_py - _chw); _cy2 = min(depth.shape[0], center_py + _chw + 1)
            _cx1 = max(0, center_px - _chw); _cx2 = min(depth.shape[1], center_px + _chw + 1)
            _croi = depth[_cy1:_cy2, _cx1:_cx2]
            _cvalid = _croi[np.isfinite(_croi) & (_croi > 0.1) & (_croi < crosshair_max_range_m)]
            center_distance_m: Optional[float] = float(np.median(_cvalid)) if len(_cvalid) > 0 else None

            world = self._pixel_to_world_gps(
                center_px, center_py, depth, captured_servo_pitch,
                max_range_m=crosshair_max_range_m,
            )
            if world is not None:
                target_lat, target_lon, up = world
                height_agl = up - self.ground_alt_offset
                target_letter = target_letter_from_index(self.next_target_index)
                img_filename = f"target_{target_letter}.jpg"
                img_path = os.path.join(capture_dir, img_filename)
                annotated = rgb.copy()
                h, w = annotated.shape[:2]
                cv2.line(annotated, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 255), 2)
                cv2.line(annotated, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 255), 2)
                fallback_label = "forced crosshair" if force_crosshair else "center-depth fallback"
                cv2.putText(annotated, f"{target_letter}: {fallback_label}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                try:
                    ok, buf = cv2.imencode(".jpg", annotated)
                    if ok:
                        with open(img_path, "wb") as _f:
                            _f.write(buf.tobytes())
                except Exception as _imwrite_err:
                    self.get_logger().warn(f"Image save failed (non-fatal): {_imwrite_err}")
                    img_path = None
                record = TargetRecord(
                    target_id=target_letter,
                    color=TargetColor.UNKNOWN,
                    lat=target_lat,
                    lon=target_lon,
                    height_agl=height_agl,
                    timestamp=time.time(),
                    confidence=0.0,
                    image_path=img_path,
                    approved=False,
                    raw_data={
                        "lat": target_lat,
                        "lon": target_lon,
                        "height_agl": height_agl,
                        "drone_lat": self.drone_lat,
                        "drone_lon": self.drone_lon,
                        "drone_heading": self.drone_heading,
                        "servo_pitch": captured_servo_pitch,
                        "drone_alt": self.drone_alt,
                        "distance_m": center_distance_m,
                        "center_fallback": True,
                        "force_crosshair": force_crosshair,
                    },
                )
                self.targets.append(record)
                new_targets.append(record)
                self.next_target_index += 1
                dist_str = (
                    f" [center_distance={center_distance_m * 100:.0f}cm]"
                    if center_distance_m is not None else ""
                )
                self.get_logger().info(
                    f"TARGET {record.target_id} (center-fallback): "
                    f"({target_lat:.7f}, {target_lon:.7f}) h={height_agl:.2f}m{dist_str}"
                )
            else:
                # Pinpoint which precondition failed so the operator isn't
                # chasing the wrong subsystem.
                reasons = []
                if not self.has_gps_fix:
                    reasons.append("no GPS fix (MAVROS NavSatFix is zero/NaN)")
                if center_distance_m is None:
                    reasons.append(
                        f"no valid depth at frame center within {crosshair_max_range_m:.0f} m "
                        "(ZED depth hole or target out of range)"
                    )
                if not self.intrinsics_received:
                    reasons.append("camera intrinsics not yet received")
                if not reasons:
                    reasons.append("back-projection returned no usable position")
                detail = "; ".join(reasons)
                response.success = False
                response.message = f"Crosshair capture failed: {detail}."
                self.get_logger().warn(f"Crosshair fallback failed: {detail}.")
                return response

        self.get_logger().info(f"Detected {len(circles)} circle(s)")

        # Save one preview image per detection with letter names derived from
        # the running global counter so each capture produces unique filenames.
        # This avoids the old overwrite-same-name pattern (target_00.jpg every
        # capture), which caused the MissionPlanner UI to show cached stale
        # images due to browser/URL caching.
        saved_images = []
        preview_letters = [
            target_letter_from_index(self.next_target_index + offset)
            for offset in range(len(circles))
        ]
        for det, preview_letter in zip(circles, preview_letters):
            img_filename = f"target_{preview_letter}.jpg"
            img_path = os.path.join(capture_dir, img_filename)

            annotated = rgb.copy()
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{preview_letter}: {det.color.value} ({det.confidence:.2f})",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            try:
                ok, buf = cv2.imencode(".jpg", annotated)
                if ok:
                    with open(img_path, "wb") as _f:
                        _f.write(buf.tobytes())
                    saved_images.append(img_filename)
                    self.get_logger().info(f"Saved detection image: {img_filename}")
                else:
                    self.get_logger().warn(f"Image encode returned False for {img_filename}")
            except Exception as _imwrite_err:
                self.get_logger().warn(f"Image save failed (non-fatal): {_imwrite_err}")

        # GPS fix is required: we can't produce absolute target coordinates
        # without one, and the GCS would have nothing to convert.
        if not self.has_gps_fix:
            response.success = True
            response.message = (
                f"Detected {len(circles)} circle(s) and saved {len(saved_images)} image(s). "
                "No GPS fix — targets not created (images saved for review)."
            )
            self.get_logger().info(
                f"Saved {len(saved_images)} detection images without GPS fix"
            )
            return response

        duplicate_count = 0
        depth_fail_count = 0
        confidence_filtered_count = 0
        for det in circles:
            if det.confidence < self.min_confidence:
                confidence_filtered_count += 1
                continue

            # Cross-verify color with independent HSV check
            verified_color, ratio = self.color_verifier.classify_roi(rgb, det.bbox)
            if verified_color != det.color and ratio > 0.3:
                self.get_logger().warn(
                    f"Color mismatch: detector={det.color.value}, "
                    f"verifier={verified_color.value} (ratio={ratio:.2f}). "
                    f"Using verifier result."
                )
                final_color = verified_color
            else:
                final_color = det.color

            world = self._pixel_to_world_gps(det.cx, det.cy, depth, captured_servo_pitch)
            if world is None:
                self.get_logger().warn(
                    f"Could not back-project circle at ({det.cx}, {det.cy}), skipping."
                )
                depth_fail_count += 1
                continue

            target_lat, target_lon, up = world
            height_agl = up - self.ground_alt_offset

            # Raw slant range to target center (meters) for distance reporting
            _hw = 5
            _py, _px = int(det.cy), int(det.cx)
            _y1 = max(0, _py - _hw); _y2 = min(depth.shape[0], _py + _hw + 1)
            _x1 = max(0, _px - _hw); _x2 = min(depth.shape[1], _px + _hw + 1)
            _roi = depth[_y1:_y2, _x1:_x2]
            _valid = _roi[np.isfinite(_roi) & (_roi > 0.1) & (_roi < 35.0)]
            det_distance_m: Optional[float] = float(np.median(_valid)) if len(_valid) > 0 else None

            if self._is_duplicate(target_lat, target_lon, height_agl):
                self.get_logger().info(
                    f"Duplicate target at ({target_lat:.7f}, {target_lon:.7f}, "
                    f"h={height_agl:.1f}m), skipping."
                )
                duplicate_count += 1
                continue

            target_letter = target_letter_from_index(self.next_target_index)
            img_filename = f"target_{target_letter}.jpg"
            img_path = os.path.join(capture_dir, img_filename)
            annotated = rgb.copy()
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            dist_label = (
                f"{det_distance_m * 100:.0f}cm" if det_distance_m is not None else "?cm"
            )
            cv2.putText(
                annotated,
                f"{final_color.value} ({det.confidence:.2f}) {dist_label}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            try:
                ok, buf = cv2.imencode(".jpg", annotated)
                if ok:
                    with open(img_path, "wb") as _f:
                        _f.write(buf.tobytes())
                else:
                    self.get_logger().warn(f"Image encode returned False for {img_filename}")
                    img_path = None
            except Exception as _imwrite_err:
                self.get_logger().warn(f"Image save failed (non-fatal): {_imwrite_err}")
                img_path = None

            record = TargetRecord(
                target_id=target_letter,
                color=final_color,
                lat=target_lat,
                lon=target_lon,
                height_agl=height_agl,
                timestamp=time.time(),
                confidence=det.confidence,
                image_path=img_path,
                approved=False,
                raw_data={
                    "lat": target_lat,
                    "lon": target_lon,
                    "height_agl": height_agl,
                    "drone_lat": self.drone_lat,
                    "drone_lon": self.drone_lon,
                    "drone_heading": self.drone_heading,
                    "servo_pitch": captured_servo_pitch,
                    "drone_alt": self.drone_alt,
                    "det_cx": int(det.cx), "det_cy": int(det.cy),
                    "det_radius": int(det.radius),
                    "det_color": det.color.value,
                    "det_confidence": float(det.confidence),
                    "distance_m": det_distance_m,
                    "center_fallback": False,
                },
            )
            self.targets.append(record)
            new_targets.append(record)
            self.next_target_index += 1

            dist_str = (
                f" distance={det_distance_m * 100:.0f}cm"
                if det_distance_m is not None else ""
            )
            self.get_logger().info(
                f"TARGET {record.target_id}: {description} "
                f"(plane={plane_hit.label}, slant_range={plane_hit.distance:.2f}m{dist_str})"
            )

        if new_targets:
            response.success = True
            # Description is the spatial body only -- no color, no distance.
            # Mission Planner prepends "<Color> target " from the table column;
            # raw slant-range stays in t.raw_data["distance_m"] for diagnostics
            # but is intentionally NOT in user-visible text (the operator wants
            # description = building-relative geometry only).
            lines = [f"Target {t.target_id}: {t.description}" for t in new_targets]
            response.message = "\n".join(lines)
        else:
            # A filter-only result means HSV detection is working, but no NEW
            # targets were produced for this capture request.
            response.success = True
            response.message = (
                f"Detected {len(circles)} circle(s), but no new targets added "
                f"(duplicates={duplicate_count}, depth_filtered={depth_fail_count}, "
                f"face_filtered={face_fail_count}, "
                f"confidence_filtered={confidence_filtered_count})."
            )

        self.get_logger().info(f"Total targets so far: {len(self.targets)}")
        return response

    def _is_duplicate(self, lat: float, lon: float, height_agl: float) -> bool:
        """Check if a detection is within dedup_radius of an existing target.

        Horizontal distance comes from the equirectangular GPS helper; vertical
        is the AGL difference. Both contribute to a 3D distance so a target
        directly above another (e.g. wall vs. roof) is not dropped as a dupe.
        """
        for t in self.targets:
            horiz = _gps_distance_m(lat, lon, t.lat, t.lon)
            vert = height_agl - t.height_agl
            if math.sqrt(horiz * horiz + vert * vert) < self.dedup_radius:
                return True
        return False

    # ================================================================ #
    #  Save targets to file
    # ================================================================ #
    def _save_callback(self, request, response):
        """Service handler for ~/save_targets.

        Writes a compact debug log of localized targets to disk. The
        operator-visible submission .txt is generated by Mission Planner
        from the GCS-side description, so this file is purely diagnostic.
        """
        if not self.targets:
            response.success = False
            response.message = "No targets to save."
            return response

        debug_filename = f"Task_1_{self.team_name}_targets_debug.txt"
        debug_filepath = os.path.join(self.output_dir, debug_filename)
        sorted_targets = sorted(self.targets, key=lambda t: t.target_id)
        debug_lines = []
        for t in sorted_targets:
            debug_lines.append(
                f"Target {t.target_id}:\n"
                f"  Color: {t.color.value}\n"
                f"  GPS: ({t.lat:.7f}, {t.lon:.7f})\n"
                f"  Height AGL: {t.height_agl:.2f}m\n"
                f"  Confidence: {t.confidence:.2f}\n"
                f"  Image: {t.image_path}\n"
                f"  Timestamp: {datetime.fromtimestamp(t.timestamp).isoformat()}"
            )
        with open(debug_filepath, "w") as f:
            f.write("\n\n".join(debug_lines) + "\n")

        self.get_logger().info(
            f"Saved {len(self.targets)} target debug records to {debug_filepath}"
        )
        response.success = True
        response.message = f"Saved {len(self.targets)} target debug records to {debug_filepath}"
        return response

    def _clear_targets_callback(self, request, response):
        """Service handler for ~/clear_targets. Clears all captured targets."""
        count = len(self.targets)
        self.targets.clear()
        self.next_target_index = 0
        self.get_logger().info(f"Cleared {count} target(s)")
        response.success = True
        response.message = f"Cleared {count} target(s)."
        return response

    def _delete_target_callback(self, request, response):
        """Remove a single target by ID, re-letter remaining targets, and delete its capture folder.

        The API writes delete_target.json with {"target_id": "B"} before calling this service.
        After deletion the remaining targets are re-lettered A, B, C... and next_target_index reset.
        """
        import json as _json
        import shutil as _shutil
        _config_override = "/workspaces/isaac_ros-dev/config/delete_target.json"
        delete_path = _config_override if os.path.exists(_config_override) else os.path.join(self.output_dir, "delete_target.json")
        if not os.path.exists(delete_path):
            response.success = False
            response.message = f"Delete request file not found: {delete_path}"
            return response
        try:
            with open(delete_path, "r") as f:
                data = _json.load(f)
        except Exception as e:
            response.success = False
            response.message = f"Failed to read delete request: {e}"
            return response

        target_id = str(data.get("target_id", "")).strip().upper()
        target = next((t for t in self.targets if t.target_id == target_id), None)
        if target is None:
            response.success = False
            response.message = f"Target {target_id!r} not found"
            return response

        # Delete capture folder from disk (best-effort)
        deleted_folder = None
        if target.image_path:
            capture_dir = os.path.dirname(target.image_path)
            if os.path.isdir(capture_dir):
                try:
                    _shutil.rmtree(capture_dir)
                    deleted_folder = capture_dir
                    self.get_logger().info(f"Deleted capture folder: {capture_dir}")
                except Exception as e:
                    self.get_logger().warn(f"Could not delete capture folder {capture_dir}: {e}")

        # Remove from list and re-letter remaining targets
        self.targets = [t for t in self.targets if t.target_id != target_id]
        for i, t in enumerate(self.targets):
            t.target_id = chr(ord("A") + i)
        self.next_target_index = len(self.targets)

        self.get_logger().info(
            f"Deleted target {target_id}; {len(self.targets)} target(s) remain, re-lettered."
        )
        response.success = True
        response.message = (
            f"Deleted target {target_id}. "
            + (f"Removed folder: {deleted_folder}. " if deleted_folder else "")
            + f"{len(self.targets)} target(s) remain."
        )
        return response

    def _set_ground_alt_callback(self, request, response):
        """Set the ground altitude offset from the drone's current AGL.

        The pilot lands the drone on the ground, then triggers this service.
        The drone's current AGL reading becomes the 0m ground reference.
        All subsequent height_agl values are adjusted by this offset.
        """
        self.ground_alt_offset = self.drone_alt
        self.get_logger().info(
            f"Ground altitude set: drone_alt={self.drone_alt:.2f}m, "
            f"offset={self.ground_alt_offset:.2f}m"
        )
        response.success = True
        response.message = (
            f"Ground altitude set to {self.ground_alt_offset:.2f}m. "
            f"All heights will be relative to this level."
        )
        return response

    def _detection_status_timer_callback(self):
        """Run HSV detection on the latest frame and publish status."""
        if self.latest_rgb is None:
            return
        try:
            circles = self.circle_detector.detect(self.latest_rgb)
            import json as _json

            depth_img = self.latest_depth
            depth_h, depth_w = (depth_img.shape if depth_img is not None else (0, 0))

            def _depth_at(px: int, py: int, radius_px: int = 0) -> tuple[Optional[float], float]:
                """Median registered depth near the circle center.

                For Task 2 standoff we prefer the circle interior rather than a
                single center pixel. ZED stereo can have small holes on glossy,
                low-texture paper, so the median of valid pixels is more stable
                and rejects isolated bad depth samples.
                """
                if depth_img is None or depth_h == 0 or depth_w == 0:
                    return None, 0.0
                hw = max(5, min(80, int(radius_px * 0.75))) if radius_px > 0 else 5
                y1 = max(0, py - hw); y2 = min(depth_h, py + hw + 1)
                x1 = max(0, px - hw); x2 = min(depth_w, px + hw + 1)
                roi = depth_img[y1:y2, x1:x2]
                if roi.size == 0:
                    return None, 0.0

                sample_mask = np.ones_like(roi, dtype=bool)
                valid_mask = np.isfinite(roi) & (roi > 0.1) & (roi < 35.0)
                if radius_px > 0:
                    yy, xx = np.ogrid[y1:y2, x1:x2]
                    sample_mask = (xx - px) ** 2 + (yy - py) ** 2 <= (hw * hw)
                    valid_mask &= sample_mask

                valid = roi[valid_mask]
                sampled = int(np.count_nonzero(sample_mask))
                valid_ratio = float(len(valid)) / float(sampled) if sampled > 0 else 0.0
                if len(valid) == 0:
                    return None, 0.0
                return float(np.median(valid)), valid_ratio

            circle_payloads = []
            top_distance: Optional[float] = None
            for d in circles[:5]:
                dist, depth_valid_ratio = _depth_at(int(d.cx), int(d.cy), int(d.radius))
                circle_payloads.append({
                    "cx": int(d.cx),
                    "cy": int(d.cy),
                    "radius": int(d.radius),
                    "color": d.color.value,
                    "confidence": round(float(d.confidence), 3),
                    "distance_m": round(dist, 3) if dist is not None else None,
                    "depth_valid_ratio": round(depth_valid_ratio, 3),
                })
                if top_distance is None and dist is not None:
                    top_distance = round(dist, 3)

            # Center-pixel depth as a fallback distance when no circle is detected
            center_distance: Optional[float] = None
            if depth_img is not None:
                center_distance, _ = _depth_at(depth_w // 2, depth_h // 2)
                if center_distance is not None:
                    center_distance = round(center_distance, 3)

            status = {
                "circle_count": len(circles),
                "has_depth": depth_img is not None,
                "has_gps": self.has_gps_fix,
                "has_local_pose": self.has_local_pose,
                "ground_alt_offset": self.ground_alt_offset,
                "target_count": len(self.targets),
                "top_distance_m": top_distance,
                "center_distance_m": center_distance,
                "circles": circle_payloads,
            }
            self._latest_detections_json = _json.dumps(status)
            self._detection_status_pub.publish(
                Float64(data=float(len(circles)))
            )
            self._detection_status_json_pub.publish(
                String(data=self._latest_detections_json)
            )

            # Push to edge_core in a daemon thread so the ROS timer callback returns immediately.
            _payload = self._latest_detections_json.encode()
            _url = self._edge_core_det_url
            _token = self._edge_core_internal_token

            def _push():
                try:
                    hdrs = {"Content-Type": "application/json"}
                    if _token:
                        hdrs["X-NOMAD-Internal-Token"] = _token
                    urllib.request.urlopen(
                        urllib.request.Request(_url, data=_payload, headers=hdrs, method="POST"),
                        timeout=0.4,
                    )
                except Exception:
                    pass

            threading.Thread(target=_push, daemon=True).start()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled target_localizer exception: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

