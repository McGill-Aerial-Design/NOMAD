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
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from cv_bridge import CvBridge

import numpy as np
import cv2
import math
import os
import time
import traceback
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
    face_label: str            # face name, or 'ground' / 'roof'
    height_agl: float
    horiz_from_left: float
    east: float
    north: float
    up: float
    description: str
    timestamp: float
    confidence: float
    image_path: Optional[str] = None


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
        # Z from local pose is AGL if EKF origin is at ground level
        self.drone_alt = msg.pose.position.z
        self.has_local_pose = True

    def _zed_odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.drone_local_east = pose.position.x
        self.drone_local_north = pose.position.y
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
        self, px: int, py: int, depth_image: np.ndarray
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
            for half_window in (12, 20, 30):
                depth = _sample_depth(half_window=half_window, max_range_m=35.0)
                if depth is not None:
                    break

        if depth is None:
            return None

        # Pixel to camera-frame 3D (OpenCV convention: Z forward, X right, Y down)
        cam_x = (px - self.camera_cx) * depth / self.camera_fx
        cam_y = (py - self.camera_cy) * depth / self.camera_fy
        cam_z = depth

        # Apply servo pitch rotation (rotate around camera X axis)
        # Servo pitch: positive = tilt up, negative = tilt down
        pitch_rad = math.radians(self.servo_pitch_deg)
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
        self, px: int, py: int, depth_image: np.ndarray
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project pixel to world ENU coordinates (relative to building center).
        """
        local = self._pixel_to_3d_local(px, py, depth_image)
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

        Output follows the format in CONOPS Appendix D:
          "<Color> target on the <face> face of the building, <height>m
           above ground, <horizontal reference>."

        plane_kind is 'wall', 'ground', or 'roof'.
        """
        height_rounded = round(height_agl, 1)
        ref_name, ref_dist, ref_phrase = self.building.find_nearest_reference(
            face, horiz_from_left, height_agl
        )
        face_name = face.name

        if plane_kind == "ground" or height_agl < 0.3:
            return (
                f"{color.value.capitalize()} target on the ground near the "
                f"{face_name} face of the building, {ref_phrase}."
            )
        if plane_kind == "roof":
            return (
                f"{color.value.capitalize()} target on the roof near the "
                f"{face_name} face of the building, {ref_phrase}."
            )
        return (
            f"{color.value.capitalize()} target on the {face_name} face "
            f"of the building, {height_rounded}m above ground, "
            f"{ref_phrase}."
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

        # Snapshot current data
        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()

        # Run HSV circle detection
        circles = self.circle_detector.detect(rgb)

        if len(circles) == 0:
            response.success = False
            response.message = "No colored circles detected in current frame."
            self.get_logger().warn("No circles found.")
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
            cv2.imwrite(img_path, annotated)
            saved_images.append(img_filename)
            self.get_logger().info(f"Saved detection image: {img_filename}")

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

        new_targets = []
        duplicate_count = 0
        depth_fail_count = 0
        face_fail_count = 0
        for det in circles:
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

            # Back-project to 3D world coordinates
            world = self._pixel_to_world_enu(det.cx, det.cy, depth)
            if world is None:
                self.get_logger().warn(
                    f"Could not get depth for circle at ({det.cx}, {det.cy}), skipping."
                )
                depth_fail_count += 1
                continue

            east, north, up = world

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

            # Project onto face
            horiz_from_left, height_agl = self.building.project_point_onto_face(
                east, north, up, face
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
            cv2.putText(
                annotated,
                f"{final_color.value} ({det.confidence:.2f})",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.imwrite(img_path, annotated)

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
            )
            self.targets.append(record)
            new_targets.append(record)
            self.next_target_index += 1

            self.get_logger().info(
                f"TARGET {record.target_id}: {description} "
                f"(plane={plane_hit.label}, distance={plane_hit.distance:.2f}m)"
            )

        if new_targets:
            response.success = True
            descs = [f"Target {t.target_id}: {t.description}" for t in new_targets]
            response.message = f"Added {len(new_targets)} target(s):\n" + "\n".join(descs)
        else:
            # A filter-only result means HSV detection is working, but no NEW
            # targets were produced for this capture request.
            response.success = True
            response.message = (
                f"Detected {len(circles)} circle(s), but no new targets added "
                f"(duplicates={duplicate_count}, depth_filtered={depth_fail_count}, "
                f"face_filtered={face_fail_count})."
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

        lines = []
        for t in self.targets:
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
        for t in self.targets:
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
        corners_path = os.path.join(self.output_dir, "building_corners.json")
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
