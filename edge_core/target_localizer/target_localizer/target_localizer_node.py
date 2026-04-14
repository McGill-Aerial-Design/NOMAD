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
     d. Project onto building face
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

Parameters (set via YAML config):
  - building.center_lat, building.center_lon
  - building.length, building.width, building.height
  - building.orientation_deg
  - team_name
  - output_dir
  - yolo_model_path
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

from .building_model import BuildingModel, FaceID, gps_to_local
from .detectors import CircleDetector, LandmarkDetector, ColorVerifier, TargetColor


@dataclass
class TargetRecord:
    """A confirmed target with 3D position and description."""

    target_id: int
    color: TargetColor
    face: FaceID
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
        self.declare_parameter("building.length", 10.0)
        self.declare_parameter("building.width", 6.0)
        self.declare_parameter("building.height", 5.0)
        self.declare_parameter("building.orientation_deg", 0.0)
        self.declare_parameter("team_name", "MAD")
        self.declare_parameter("output_dir", "/home/mad/targets")
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("dedup_radius_m", 0.5)
        self.declare_parameter("landmark_detect_rate_hz", 2.0)
        self.declare_parameter("auto_landmark_detection", True)

        # Load parameters
        self.team_name = self.get_parameter("team_name").value
        self.output_dir = self.get_parameter("output_dir").value
        self.dedup_radius = self.get_parameter("dedup_radius_m").value

        os.makedirs(self.output_dir, exist_ok=True)

        # ----- Building model ----- #
        self.building = BuildingModel(
            center_lat=self.get_parameter("building.center_lat").value,
            center_lon=self.get_parameter("building.center_lon").value,
            length=self.get_parameter("building.length").value,
            width=self.get_parameter("building.width").value,
            height=self.get_parameter("building.height").value,
            orientation_deg=self.get_parameter("building.orientation_deg").value,
        )
        self.get_logger().info(
            f"Building model initialized:\n{self.building.get_summary()}"
        )

        # ----- Detectors ----- #
        self.circle_detector = CircleDetector(
            min_radius_px=8,
            max_radius_px=400,
            min_circularity=0.60,
            min_solidity=0.70,
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
        self.next_target_id: int = 1

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

        # Get median depth in a small window around the pixel
        half_w = 5
        y1 = max(0, py - half_w)
        y2 = min(depth_image.shape[0], py + half_w)
        x1 = max(0, px - half_w)
        x2 = min(depth_image.shape[1], px + half_w)

        roi = depth_image[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < 20.0)]
        if len(valid) == 0:
            return None

        depth = float(np.median(valid))

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
        face: "Face",
        horiz_from_left: float,
        height_agl: float,
    ) -> str:
        """
        Generate a natural-language target description using the building
        model and detected landmarks.

        Output follows the format specified in ConOps Appendix D:
          "<Color> target on the <face> face of the building, <height>m
           above ground, <horizontal reference>."
        """
        # Round to decimetres per ConOps rules
        height_rounded = round(height_agl, 1)

        # Find nearest reference point (landmark or corner)
        ref_name, ref_dist, ref_phrase = self.building.find_nearest_reference(
            face, horiz_from_left, height_agl
        )

        # Face name
        face_name = face.face_id.value

        # Build description
        if "corner" in ref_name:
            desc = (
                f"{color.value.capitalize()} target on the {face_name} face "
                f"of the building, {height_rounded}m above ground, "
                f"{ref_phrase}."
            )
        else:
            desc = (
                f"{color.value.capitalize()} target on the {face_name} face "
                f"of the building, {height_rounded}m above ground, "
                f"{ref_phrase}."
            )

        # Special case: ground-level target near the building
        if height_agl < 0.3:
            desc = (
                f"{color.value.capitalize()} target on the ground near the "
                f"{face_name} face of the building, {ref_phrase}."
            )

        # Special case: target not on a wall (detected on ground away from building)
        # This would be caught upstream, but handle gracefully
        return desc

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

        # Save images of all detected circles for Mission Planner display
        saved_images = []
        for i, det in enumerate(circles):
            # Save detection image
            img_filename = f"target_{i:02d}.jpg"
            img_path = os.path.join(capture_dir, img_filename)

            # Draw bounding box on image copy
            annotated = rgb.copy()
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{det.color.value} ({det.confidence:.2f})",
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

        active_face, face_source = self._resolve_observed_face()
        if active_face is None:
            response.success = True
            response.message = (
                f"Detected {len(circles)} circle(s) and saved {len(saved_images)} image(s), "
                "but could not determine the viewed building face "
                "(GPS/local pose context unavailable)."
            )
            self.get_logger().warn(
                "Saved %s detection images but face could not be resolved (gps_fix=%s, has_local_pose=%s)",
                len(saved_images),
                self.has_gps_fix,
                self.has_local_pose,
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

            # Determine building face once per capture request and reuse it.
            face = active_face

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
                final_color, face, horiz_from_left, height_agl
            )

            # Save target image (already saved detection images above, this is for confirmed targets)
            img_filename = f"target_{self.next_target_id:02d}.jpg"
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
                target_id=self.next_target_id,
                color=final_color,
                face=face.face_id,
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
            self.next_target_id += 1

            self.get_logger().info(f"TARGET {record.target_id}: {description}")

        if new_targets:
            response.success = True
            descs = [t.description for t in new_targets]
            prefix = ""
            if face_source == "local_pose":
                prefix = "GPS unavailable; using local pose fallback. "
            response.message = (
                prefix + f"Added {len(new_targets)} target(s):\n" + "\n".join(descs)
            )
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
                f"  Face: {t.face.value}\n"
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
