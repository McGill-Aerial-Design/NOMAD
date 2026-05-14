"""
target_localizer_node.py

Main ROS 2 node for AEAC 2026 Task 1 automated target description.

Workflow:
  1. Subscribe to ZED RGB + depth, drone pose (MAVROS), servo angle
  2. Continuously run YOLO landmark detection in background to
     auto-populate the building model with doors, windows, etc.
  3. On button press (service call or joystick trigger):
     a. Capture current ZED frame
     b. Run HSV circle detection
     c. Back-project detections to 3D using depth + TF
      d. Classify nearest analytical plane (4 walls + ground + roof)
          and project onto nearest wall face for references
     e. Find nearest landmark/corner
     f. Generate natural-language description from template
     g. Append to target list (with deduplication)
  4. On "save" command, write target list to .txt file

Topics subscribed:
  - /zed2i/zed_node/rgb/image_rect_color (sensor_msgs/Image)
  - /zed2i/zed_node/depth/depth_registered (sensor_msgs/Image)
  - /zed2i/zed_node/left_camera_frame (camera_info for intrinsics)
  - /mavros/global_position/global (NavSatFix - drone GPS)
  - /mavros/local_position/pose (PoseStamped - drone local pose)
  - /mavros/global_position/compass_hdg (Float64 - heading)
  - /servo/angle (Float64 - current servo pitch in degrees)

Services:
  - ~/capture_target (std_srvs/Trigger) - run detection + description
  - ~/save_targets (std_srvs/Trigger) - write .txt file
  - ~/print_model (std_srvs/Trigger) - print building model summary
   - ~/set_building_corners (std_srvs/Trigger) - rebuild building model from corners JSON file

Parameters (set via YAML config):
  - building.center_lat, building.center_lon, building.height
  - building.corner_names + building.corner_lats + building.corner_lons
    (parallel arrays, one entry per polygon corner)
  - building.rectangle.{length,width,orientation_deg}
    (only used when corner_names is empty -- rectangle convenience fallback)
  - team_name
  - output_dir
  - yolo_model_path  (Task 1: leave empty; CONOPS provides no landmark info)
  - dedup_radius_m
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
import threading
import time
import traceback
import urllib.request
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .building_model import BuildingModel, Face, gps_to_local
from .detectors import CircleDetector, LandmarkDetector, ColorVerifier, TargetColor


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
    """A confirmed target with 3D position and description."""

    target_id: str
    color: TargetColor
    face_label: str
    height_agl: float
    horiz_from_left: float
    east: float
    north: float
    up: float
    description: str
    timestamp: float
    confidence: float
    image_path: Optional[str] = None
    plane_kind: str = "wall"
    face_name: str = ""
    approved: bool = False
    raw_data: Optional[dict] = None


class TargetLocalizerNode(Node):
    def __init__(self):
        super().__init__("target_localizer")

        # ----- Parameters ----- #
        self.declare_parameter("building.center_lat", 0.0)
        self.declare_parameter("building.center_lon", 0.0)
        self.declare_parameter("building.height", 5.0)
        # N-corner polygon (preferred path). Three parallel arrays so ROS 2
        # parameters can carry the schema (no list-of-dict support natively).
        self.declare_parameter("building.corner_names", [""])
        self.declare_parameter("building.corner_lats", [0.0])
        self.declare_parameter("building.corner_lons", [0.0])
        # Rectangle convenience fallback, only used when corner_names is empty.
        self.declare_parameter("building.rectangle.length", 10.0)
        self.declare_parameter("building.rectangle.width", 6.0)
        self.declare_parameter("building.rectangle.orientation_deg", 0.0)
        self.declare_parameter("team_name", "MAD")
        self.declare_parameter("output_dir", "/home/mad/targets")
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("dedup_radius_m", 0.5)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("landmark_detect_rate_hz", 2.0)
        self.declare_parameter("auto_landmark_detection", False)
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
        self.dedup_radius = self.get_parameter("dedup_radius_m").value

        self.min_confidence = float(self.get_parameter("min_confidence").value)

        os.makedirs(self.output_dir, exist_ok=True)

        # ----- Building model ----- #
        center_lat = float(self.get_parameter("building.center_lat").value)
        center_lon = float(self.get_parameter("building.center_lon").value)
        height = float(self.get_parameter("building.height").value)

        corner_names = [
            str(s) for s in self.get_parameter("building.corner_names").value or []
            if str(s).strip() != ""
        ]
        corner_lats = list(self.get_parameter("building.corner_lats").value or [])
        corner_lons = list(self.get_parameter("building.corner_lons").value or [])

        corners_local = None
        if len(corner_names) >= 3:
            if not (len(corner_names) == len(corner_lats) == len(corner_lons)):
                raise RuntimeError(
                    "building.corner_names / corner_lats / corner_lons must "
                    f"have equal length, got {len(corner_names)} / "
                    f"{len(corner_lats)} / {len(corner_lons)}"
                )
            corners_local = [
                (name,) + gps_to_local(float(lat), float(lon), center_lat, center_lon)
                for name, lat, lon in zip(corner_names, corner_lats, corner_lons)
            ]
            self.get_logger().info(
                f"Loaded {len(corners_local)} polygon corners from CONOPS data."
            )
        else:
            self.get_logger().warn(
                "No polygon corners configured; falling back to rectangle "
                "convenience path. Update building.corner_* params for the real "
                "building shape."
            )

        self.building = BuildingModel(
            center_lat=center_lat,
            center_lon=center_lon,
            height=height,
            corners_local=corners_local,
            length=float(self.get_parameter("building.rectangle.length").value),
            width=float(self.get_parameter("building.rectangle.width").value),
            orientation_deg=float(
                self.get_parameter("building.rectangle.orientation_deg").value
            ),
        )
        self.get_logger().info(
            f"Building model initialized:\n{self.building.get_summary()}"
        )

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

        yolo_path = self.get_parameter("yolo_model_path").value
        self.landmark_detector = (
            LandmarkDetector(
                model_path=yolo_path,
                conf_threshold=0.35,
            )
            if yolo_path
            else None
        )

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

        # Landmark detection timer
        self.last_landmark_time = 0.0
        lm_rate = self.get_parameter("landmark_detect_rate_hz").value
        self.landmark_interval = 1.0 / max(lm_rate, 0.1)

        # ----- QoS ----- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Subscribers ----- #
        self.rgb_sub = self.create_subscription(
            Image,
            "/zed2i/zed_node/rgb/image_rect_color",
            self._rgb_callback,
            sensor_qos,
        )
        self.rgb_sub_direct = self.create_subscription(
            Image, "/zed/zed_node/rgb/color/rect/image", self._rgb_callback, sensor_qos
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/zed2i/zed_node/depth/depth_registered",
            self._depth_callback,
            sensor_qos,
        )
        self.depth_sub_direct = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self._depth_callback,
            sensor_qos,
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            "/zed2i/zed_node/rgb/camera_info",
            self._cam_info_callback,
            sensor_qos,
        )
        self.cam_info_sub_direct = self.create_subscription(
            CameraInfo,
            "/zed/zed_node/rgb/color/rect/camera_info",
            self._cam_info_callback,
            sensor_qos,
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
            Odometry, "/zed/zed_node/odom", self._zed_odom_callback, sensor_qos
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
        self.model_srv = self.create_service(
            Trigger, "/target_localizer/print_model", self._print_model_callback
        )
        self.corners_srv = self.create_service(
            Trigger, "/target_localizer/set_building_corners", self._set_corners_callback
        )
        self.clear_srv = self.create_service(
            Trigger, "/target_localizer/clear_targets", self._clear_targets_callback
        )
        self.ground_alt_srv = self.create_service(
            Trigger, "/target_localizer/set_ground_alt", self._set_ground_alt_callback
        )
        self.regen_srv = self.create_service(
            Trigger, "/target_localizer/regenerate_descriptions", self._regenerate_descriptions_callback
        )
        self.plane_override_srv = self.create_service(
            Trigger, "/target_localizer/set_target_plane", self._set_target_plane_callback
        )
        self.delete_target_srv = self.create_service(
            Trigger, "/target_localizer/delete_target", self._delete_target_callback
        )

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

        # ----- Background landmark detection timer ----- #
        if (
            self.get_parameter("auto_landmark_detection").value
            and self.landmark_detector
        ):
            self.create_timer(self.landmark_interval, self._landmark_timer_callback)

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
        self.drone_local_east = msg.pose.position.x
        self.drone_local_north = msg.pose.position.y
        # MAVROS local_position/pose Z is the authoritative AGL (published by
        # drone_state_publisher from MAVLink alt_agl_m). ZED odom's z is the
        # camera-frame origin (~0) and must NOT be used as altitude.
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
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project a pixel to 3D coordinates in the drone's local ENU frame,
        accounting for servo tilt.

        Returns (east_offset, north_offset, up) relative to the drone,
        or None if depth is invalid.
        """
        if not self.intrinsics_received:
            return None

        def _sample_depth(half_window: int, max_range_m: float) -> Optional[float]:
            y1 = max(0, py - half_window)
            y2 = min(depth_image.shape[0], py + half_window + 1)
            x1 = max(0, px - half_window)
            x2 = min(depth_image.shape[1], px + half_window + 1)
            roi = depth_image[y1:y2, x1:x2]
            valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < max_range_m)]
            if len(valid) == 0:
                return None
            return float(np.median(valid))

        # Start with a tight ROI for precision, then expand to handle common
        # depth holes on low-texture/reflective target patches.
        depth = _sample_depth(half_window=5, max_range_m=20.0)
        if depth is None:
            for half_window in (12, 20, 30, 60, 100):
                depth = _sample_depth(half_window=half_window, max_range_m=35.0)
                if depth is not None:
                    break

        if depth is None:
            # Last resort: use global frame median. If the ZED depth is mostly NaN
            # (e.g. textureless wall, strong reflections), use any valid pixel to
            # at least produce a rough geolocalization rather than hard failing.
            global_valid = depth_image[
                np.isfinite(depth_image) & (depth_image > 0.1) & (depth_image < 35.0)
            ]
            if len(global_valid) >= 10:
                depth = float(np.median(global_valid))
                self.get_logger().warn(
                    f"Depth at ({px},{py}) all-NaN in ±100px window; "
                    f"using global frame median {depth:.2f}m "
                    f"({len(global_valid)} valid pixels in frame)"
                )
            else:
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

    def _pixel_to_world_enu(
        self, px: int, py: int, depth_image: np.ndarray,
        servo_pitch_override: Optional[float] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project pixel to world ENU coordinates (relative to building center).
        """
        local = self._pixel_to_3d_local(px, py, depth_image, servo_pitch_override)
        if local is None:
            return None

        east_off, north_off, up = local

        # Drone position in building-relative ENU.
        # Prefer GPS geodetic conversion when available, otherwise fall back
        # to local pose (e.g., ZED odom / MAVROS local_position) so Task 1
        # capture remains usable indoors or before GPS lock.
        if self.has_gps_fix:
            drone_east, drone_north = gps_to_local(
                self.drone_lat,
                self.drone_lon,
                self.building.center_lat,
                self.building.center_lon,
            )
        else:
            drone_east, drone_north = self.drone_local_east, self.drone_local_north

        return (drone_east + east_off, drone_north + north_off, up)

    # ================================================================ #
    #  Description generation
    # ================================================================ #
    def _generate_description(
        self,
        color: TargetColor,
        face: Face,
        horiz_from_left: float,
        height_agl: float,
        plane_kind: str = "wall",
    ) -> str:
        """
        Generate a natural-language target description.

        Output is the COLOR-FREE spatial body, e.g.
          "on the <face> face of the building, <height>m above ground,
           <horizontal reference>."

        Color is intentionally NOT included: the Mission Planner submission
        table is the single source of truth for target color. The C# side
        prefixes "<Color> target " in front of this body at upload time.

        plane_kind is 'wall', 'ground', or 'roof'.
        """
        height_rounded = round(height_agl, 1)
        ref_name, ref_dist, ref_phrase = self.building.find_nearest_reference(
            face, horiz_from_left, height_agl
        )
        face_name = face.name

        if plane_kind == "ground" or height_agl < 0.3:
            return (
                f"on the ground near the {face_name} face of the building, "
                f"{height_rounded}m above ground, {ref_phrase}."
            )
        if plane_kind == "roof":
            return (
                f"on the roof near the {face_name} face of the building, "
                f"{height_rounded}m above ground, {ref_phrase}."
            )
        return (
            f"on the {face_name} face of the building, "
            f"{height_rounded}m above ground, {ref_phrase}."
        )

    def _resolve_observed_face(self) -> Tuple[Optional["Face"], str]:
        """Resolve active building face using GPS first, then local-pose fallback."""
        if self.has_gps_fix:
            face = self.building.get_face_from_drone_pose(
                self.drone_lat,
                self.drone_lon,
                self.drone_heading,
            )
            if face is not None:
                return face, "gps"

        if self.has_local_pose:
            face = self.building.get_face_from_local_pose(
                self.drone_local_east,
                self.drone_local_north,
                self.drone_heading,
            )
            if face is not None:
                return face, "local_pose"

        return None, "none"

    # ================================================================ #
    #  Target capture (button press)
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

        # Snapshot current data (including servo angle to avoid stale value for 3D back-projection)
        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()
        captured_servo_pitch = self.servo_pitch_deg

        # Run HSV circle detection
        circles = self.circle_detector.detect(rgb)

        new_targets: List[TargetRecord] = []

        if len(circles) == 0:
            # No circles detected — automatically use the frame center (crosshair)
            # for localization. Always attempted; _pixel_to_world_enu falls back to
            # local_pose when GPS is unavailable.
            center_px = self.latest_rgb.shape[1] // 2
            center_py = self.latest_rgb.shape[0] // 2
            # Raw depth at center pixel for distance reporting
            _chw = 5
            _cy_c, _cx_c = center_py, center_px
            _cy1 = max(0, _cy_c - _chw); _cy2 = min(depth.shape[0], _cy_c + _chw + 1)
            _cx1 = max(0, _cx_c - _chw); _cx2 = min(depth.shape[1], _cx_c + _chw + 1)
            _croi = depth[_cy1:_cy2, _cx1:_cx2]
            _cvalid = _croi[np.isfinite(_croi) & (_croi > 0.1) & (_croi < 35.0)]
            center_distance_m: Optional[float] = float(np.median(_cvalid)) if len(_cvalid) > 0 else None

            world = self._pixel_to_world_enu(center_px, center_py, depth, captured_servo_pitch)
            if world is not None:
                east, north, up = world
                observed_face, _ = self._resolve_observed_face()
                plane_hit = self.building.classify_nearest_plane(east, north, up)
                if observed_face is not None:
                    face = observed_face
                elif plane_hit.face is not None:
                    face = plane_hit.face
                else:
                    face, _ = self.building.get_nearest_wall_face(east, north)
                # Subtract ground_alt_offset BEFORE projecting: the projection
                # clamps height_agl to [0, building.height+0.05] in the building
                # frame, so a post-projection subtraction would either clip the
                # offset away or push a ground target to a large negative value.
                horiz_from_left, height_agl = self.building.project_point_onto_face(
                    east, north, up - self.ground_alt_offset, face
                )
                target_letter = target_letter_from_index(self.next_target_index)
                description = self._generate_description(
                    TargetColor.UNKNOWN, face, horiz_from_left, height_agl, plane_hit.kind
                )
                img_filename = f"target_{target_letter}.jpg"
                img_path = os.path.join(capture_dir, img_filename)
                annotated = rgb.copy()
                h, w = annotated.shape[:2]
                cv2.line(annotated, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 255), 2)
                cv2.line(annotated, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 255), 2)
                cv2.putText(annotated, f"{target_letter}: center-depth fallback", (10, 30),
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
                    face_label=plane_hit.label,
                    height_agl=height_agl,
                    horiz_from_left=horiz_from_left,
                    east=east,
                    north=north,
                    up=up,
                    description=description,
                    timestamp=time.time(),
                    confidence=0.0,
                    image_path=img_path,
                    plane_kind=plane_hit.kind,
                    face_name=face.name,
                    approved=False,
                    raw_data={
                        "east": east, "north": north, "up": up,
                        "plane_kind": plane_hit.kind,
                        "drone_lat": self.drone_lat,
                        "drone_lon": self.drone_lon,
                        "drone_heading": self.drone_heading,
                        "servo_pitch": captured_servo_pitch,
                        "drone_alt": self.drone_alt,
                        "distance_m": center_distance_m,
                        "center_fallback": True,
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
                    f"TARGET {record.target_id} (center-fallback): {description}{dist_str}"
                )
            else:
                response.success = False
                response.message = (
                    "No circles detected and depth is invalid at frame center "
                    "(all NaN or out of range). Check ZED depth stream."
                )
                self.get_logger().warn("Crosshair fallback failed: center-pixel depth is invalid.")
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

        # Check if we can create targets (requires GPS/local pose)
        can_create_targets = self.has_gps_fix or self.has_local_pose
        if not can_create_targets:
            response.success = True
            response.message = (
                f"Detected {len(circles)} circle(s) and saved {len(saved_images)} image(s). "
                "GPS/local pose unavailable - targets not created but images saved for review."
            )
            self.get_logger().info(
                f"Saved {len(saved_images)} detection images without GPS"
            )
            return response

        duplicate_count = 0
        depth_fail_count = 0
        face_fail_count = 0
        confidence_filtered_count = 0
        for det in circles:
            # Confidence threshold filter: reject detections below minimum
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

            # Back-project to 3D world coordinates using captured servo angle
            world = self._pixel_to_world_enu(det.cx, det.cy, depth, captured_servo_pitch)
            if world is None:
                self.get_logger().warn(
                    f"Could not get depth for circle at ({det.cx}, {det.cy}), skipping."
                )
                depth_fail_count += 1
                continue

            east, north, up = world

            # Raw slant range to target center (meters) for distance reporting
            _hw = 5
            _py, _px = int(det.cy), int(det.cx)
            _y1 = max(0, _py - _hw); _y2 = min(depth.shape[0], _py + _hw + 1)
            _x1 = max(0, _px - _hw); _x2 = min(depth.shape[1], _px + _hw + 1)
            _roi = depth[_y1:_y2, _x1:_x2]
            _valid = _roi[np.isfinite(_roi) & (_roi > 0.1) & (_roi < 35.0)]
            det_distance_m: Optional[float] = float(np.median(_valid)) if len(_valid) > 0 else None

            # Prefer the face the drone is actively looking at; fall back to
            # nearest-wall classification if view direction is unknown. Either
            # way `face` is guaranteed non-None for the description below.
            observed_face, _ = self._resolve_observed_face()
            plane_hit = self.building.classify_nearest_plane(east, north, up)

            if observed_face is not None:
                face: Face = observed_face
            elif plane_hit.face is not None:
                face = plane_hit.face
            else:
                face, _ = self.building.get_nearest_wall_face(east, north)

            # Project onto face. Pre-subtract ground_alt_offset so the
            # projection's [0, building.height] clamp operates in
            # ground-relative coordinates.
            horiz_from_left, height_agl = self.building.project_point_onto_face(
                east, north, up - self.ground_alt_offset, face
            )

            # Deduplication check
            if self._is_duplicate(east, north, up):
                self.get_logger().info(
                    f"Duplicate target at ({east:.1f}, {north:.1f}, {up:.1f}), skipping."
                )
                duplicate_count += 1
                continue

            # Generate description
            description = self._generate_description(
                final_color,
                face,
                horiz_from_left,
                height_agl,
                plane_kind=plane_hit.kind,
            )

            # Save target image (already saved detection images above, this is for confirmed targets)
            target_letter = target_letter_from_index(self.next_target_index)
            img_filename = f"target_{target_letter}.jpg"
            img_path = os.path.join(capture_dir, img_filename)
            # Draw bounding box on image copy
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

            # Create record
            record = TargetRecord(
                target_id=target_letter,
                color=final_color,
                face_label=plane_hit.label,
                height_agl=height_agl,
                horiz_from_left=horiz_from_left,
                east=east,
                north=north,
                up=up,
                description=description,
                timestamp=time.time(),
                confidence=det.confidence,
                image_path=img_path,
                plane_kind=plane_hit.kind,
                face_name=face.name,
                approved=False,
                raw_data={
                    "east": east, "north": north, "up": up,
                    "plane_kind": plane_hit.kind,
                    "face_name_override": None,
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

    def _is_duplicate(self, east: float, north: float, up: float) -> bool:
        """Check if a detection is within dedup_radius of an existing target."""
        for t in self.targets:
            dist = math.sqrt(
                (east - t.east) ** 2 + (north - t.north) ** 2 + (up - t.up) ** 2
            )
            if dist < self.dedup_radius:
                return True
        return False

    # ================================================================ #
    #  Background landmark detection
    # ================================================================ #
    def _landmark_timer_callback(self):
        """Periodically run YOLO landmark detection to build up the model."""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if not self.intrinsics_received:
            return
        if not self.has_gps_fix and not self.has_local_pose:
            return
        if self.landmark_detector is None:
            return

        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()

        landmarks = self.landmark_detector.detect(rgb)

        for lm in landmarks:
            world = self._pixel_to_world_enu(lm.cx, lm.cy, depth)
            if world is None:
                continue

            east, north, up = world
            self.building.add_landmark_from_3d(
                east,
                north,
                up,
                kind=lm.kind,
                confidence=lm.confidence,
                drone_lat=self.drone_lat,
                drone_lon=self.drone_lon,
                drone_heading_deg=self.drone_heading,
                drone_local_east=(
                    self.drone_local_east
                    if (self.has_local_pose and not self.has_gps_fix)
                    else None
                ),
                drone_local_north=(
                    self.drone_local_north
                    if (self.has_local_pose and not self.has_gps_fix)
                    else None
                ),
            )

        if landmarks:
            self.get_logger().debug(
                f"Registered {len(landmarks)} landmark detection(s) into building model."
            )

    # ================================================================ #
    #  Save targets to file
    # ================================================================ #
    def _save_callback(self, request, response):
        """Service handler for ~/save_targets. Writes the .txt file."""
        if not self.targets:
            response.success = False
            response.message = "No targets to save."
            return response

        filename = f"Task_1_{self.team_name}_targets.txt"
        filepath = os.path.join(self.output_dir, filename)

        sorted_targets = sorted(self.targets, key=lambda t: t.target_id)
        lines = []
        for t in sorted_targets:
            lines.append(f"Target {t.target_id}: {t.description}")

        content = "\n\n".join(lines) + "\n"

        with open(filepath, "w") as f:
            f.write(content)

        self.get_logger().info(f"Saved {len(self.targets)} targets to {filepath}")
        self.get_logger().info(f"Content:\n{content}")

        # Also save a detailed log with raw data for debugging
        debug_filename = f"Task_1_{self.team_name}_targets_debug.txt"
        debug_filepath = os.path.join(self.output_dir, debug_filename)
        debug_lines = []
        for t in sorted_targets:
            debug_lines.append(
                f"Target {t.target_id}:\n"
                f"  Description: {t.description}\n"
                f"  Color: {t.color.value}\n"
                f"  Face: {t.face_label}\n"
                f"  Height AGL: {t.height_agl:.2f}m\n"
                f"  Horiz from left: {t.horiz_from_left:.2f}m\n"
                f"  World ENU: ({t.east:.2f}, {t.north:.2f}, {t.up:.2f})\n"
                f"  Confidence: {t.confidence:.2f}\n"
                f"  Image: {t.image_path}\n"
                f"  Timestamp: {datetime.fromtimestamp(t.timestamp).isoformat()}"
            )
        with open(debug_filepath, "w") as f:
            f.write("\n\n".join(debug_lines) + "\n")

        response.success = True
        response.message = f"Saved to {filepath} ({len(self.targets)} targets)"
        return response


    def _set_corners_callback(self, request, response):
        """Rebuild the building model from a corners JSON file.

        The API writes a JSON file to the output_dir containing corner
        GPS coordinates. This service reads it and rebuilds the
        BuildingModel at runtime, so operators can calibrate corners
        by flying to each one and capturing the drone GPS.

        Expected JSON format (written to <output_dir>/building_corners.json):
        {
            "center_lat": 45.322,
            "center_lon": -75.760,
            "height": 5.0,
            "corners": [
                {"name": "NW", "lat": 45.32205, "lon": -75.7601},
                {"name": "NE", "lat": 45.32205, "lon": -75.7595},
                ...
            ]
        }
        """
        import json as _json
        # API writes to the config bind-mount (host /home/mad/NOMAD/config -> container /workspaces/isaac_ros-dev/config).
        # Fall back to output_dir for manual placement.
        _config_corners = "/workspaces/isaac_ros-dev/config/building_corners.json"
        corners_path = _config_corners if os.path.exists(_config_corners) else os.path.join(self.output_dir, "building_corners.json")
        if not os.path.exists(corners_path):
            response.success = False
            response.message = f"Corners file not found: {corners_path}. Write building_corners.json first."
            return response

        try:
            with open(corners_path, "r") as f:
                data = _json.load(f)
        except Exception as e:
            response.success = False
            response.message = f"Failed to read corners file: {e}"
            return response

        try:
            center_lat = float(data.get("center_lat", self.building.center_lat))
            center_lon = float(data.get("center_lon", self.building.center_lon))
            height = float(data.get("height", self.building.height))
            corners_data = data.get("corners", [])

            if len(corners_data) < 3:
                response.success = False
                response.message = f"Need at least 3 corners, got {len(corners_data)}"
                return response

            # Convert corner GPS to local ENU
            corners_local = [
                (c["name"],) + gps_to_local(float(c["lat"]), float(c["lon"]), center_lat, center_lon)
                for c in corners_data
            ]

            # Rebuild the building model
            self.building = BuildingModel(
                center_lat=center_lat,
                center_lon=center_lon,
                height=height,
                corners_local=corners_local,
            )

            summary = self.building.get_summary()
            self.get_logger().info(
                f"Building model rebuilt from corner calibration:\n{summary}"
            )
            response.success = True
            response.message = f"Building model rebuilt with {len(corners_data)} corners.\n{summary}"
        except Exception as e:
            self.get_logger().error(f"Failed to rebuild building model: {e}")
            response.success = False
            response.message = f"Error rebuilding building model: {e}"

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

    def _regenerate_one_target(self, t) -> bool:
        """Recompute face/height/description for a single target from its raw_data.

        Returns True if the target was regenerated, False if it had no raw_data.
        """
        if t.raw_data is None:
            return False
        rd = t.raw_data
        plane_kind = rd.get("plane_kind", t.plane_kind)
        face_name_override = rd.get("face_name_override")

        east = rd.get("east", t.east)
        north = rd.get("north", t.north)
        up = rd.get("up", t.up)

        if face_name_override:
            face = self.building.faces_by_name.get(face_name_override)
            if face is None:
                face, _ = self.building.get_nearest_wall_face(east, north)
        else:
            observed_face, _ = self._resolve_observed_face()
            plane_hit = self.building.classify_nearest_plane(east, north, up)
            if observed_face is not None:
                face = observed_face
            elif plane_hit.face is not None:
                face = plane_hit.face
            else:
                face, _ = self.building.get_nearest_wall_face(east, north)

        # Pre-subtract offset (see _capture_callback for rationale).
        horiz_from_left, height_agl = self.building.project_point_onto_face(
            east, north, up - self.ground_alt_offset, face
        )

        if plane_kind == "ground":
            height_agl = max(0.0, height_agl)

        t.face_label = face.name if face else plane_kind
        t.face_name = face.name if face else ""
        t.plane_kind = plane_kind
        t.height_agl = height_agl
        t.horiz_from_left = horiz_from_left
        t.description = self._generate_description(
            t.color, face, horiz_from_left, height_agl, plane_kind
        )
        return True

    def _regenerate_descriptions_callback(self, request, response):
        """Regenerate all target descriptions from stored raw data.

        Called after the pilot changes building model, ground altitude,
        or plane overrides so that all descriptions stay consistent.
        """
        regenerated = sum(1 for t in self.targets if self._regenerate_one_target(t))
        msg = f"Regenerated {regenerated}/{len(self.targets)} target description(s)."
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def _set_target_plane_callback(self, request, response):
        """Override the plane classification of a single target.

        The API writes a ``plane_override.json`` file to the output directory
        containing ``{"target_id": "A", "plane_kind": "wall|ground|roof",
        "face_name": "N"}`` (face_name optional). This service reads it,
        updates that target's raw_data, and regenerates its description.
        """
        import json as _json
        _config_override = "/workspaces/isaac_ros-dev/config/plane_override.json"
        override_path = _config_override if os.path.exists(_config_override) else os.path.join(self.output_dir, "plane_override.json")
        if not os.path.exists(override_path):
            response.success = False
            response.message = f"Override file not found: {override_path}"
            return response
        try:
            with open(override_path, "r") as f:
                data = _json.load(f)
        except Exception as e:
            response.success = False
            response.message = f"Failed to read override file: {e}"
            return response

        target_id = str(data.get("target_id", "")).strip()
        plane_kind = str(data.get("plane_kind", "")).strip().lower()
        face_name = data.get("face_name")
        if plane_kind not in {"wall", "ground", "roof"}:
            response.success = False
            response.message = f"Invalid plane_kind: {plane_kind!r}"
            return response

        target = next((t for t in self.targets if t.target_id == target_id), None)
        if target is None:
            response.success = False
            response.message = f"Target {target_id!r} not found"
            return response

        if target.raw_data is None:
            target.raw_data = {}
        target.raw_data["plane_kind"] = plane_kind
        if face_name:
            target.raw_data["face_name_override"] = str(face_name)
        elif "face_name_override" in target.raw_data:
            del target.raw_data["face_name_override"]

        self._regenerate_one_target(target)
        msg = (
            f"Target {target_id} plane set to {plane_kind}"
            + (f" (face={face_name})" if face_name else "")
            + f"; description: {target.description}"
        )
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
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

    def _print_model_callback(self, request, response):
        """Print the current building model with landmarks."""
        summary = self.building.get_summary()
        self.get_logger().info(f"\n{summary}")
        response.success = True
        response.message = summary
        return response


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

