"""
NOMAD Edge Core - Task 2 Spray Controller (Hybrid Manual+Autonomous)

Workflow:
  Operator manually positions drone within 3m of target.
  Upon spray trigger, system autonomously approaches to 2m and executes spray sequence.

State Machine:
  IDLE -> APPROACH (3m->2m) -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE

Features:
  - Operator manual positioning (WASD) to enter 3m engagement zone
  - Autonomous approach via MAVLink velocity commands (3m to 2m)
  - Pre-spray image capture for circle change verification
  - Circle change verification: color-agnostic before/after comparison (>20% change)
  - Visual servoing: servo pitch + ballistic drop compensation
  - Google Drive upload: posts proof photo autonomously
  - Calibration data loaded from ~/.nomad/calibration/spray_calibration.json
  - Obstacle avoidance sector exclusion during approach

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Optional

logger = logging.getLogger("edge_core.spray_controller")


class SprayState(Enum):
    """Spray sequence states."""
    IDLE = "idle"
    APPROACH = "approach"    # Autonomous 3m->2m approach via Nav2
    AIM = "aim"              # Fine alignment with visual servoing
    SPRAY = "spray"          # Activating water pump
    VERIFY = "verify"        # Circle change detection (before/after comparison)
    UPLOAD = "upload"        # Photo capture + Google Drive upload
    COMPLETE = "complete"    # Sequence done, ready for next target
    FAILED = "failed"        # Sequence failed (after retry)
    ABORTED = "aborted"      # Operator or safety abort


@dataclass
class SprayTarget:
    """Target to spray (operator must position drone manually)."""
    target_id: int
    x: float  # world-frame position (NED)
    y: float
    z: float
    label: str = ""
    confidence: float = 0.0


@dataclass
class SprayStatus:
    """Current spray sequence status."""
    state: str = "idle"
    target_id: int = -1
    target_label: str = ""
    distance_to_target: float = 0.0
    servo_angle: float = 90.0
    spray_count: int = 0
    verification_passed: bool = False
    upload_url: str = ""
    error: Optional[str] = None
    # Nav2 approach tracking
    nav2_goal_id: Optional[str] = None
    nav2_approach_active: bool = False
    approach_method: str = ""  # "nav2" or "velocity"
    # Stats
    targets_engaged: int = 0
    targets_succeeded: int = 0
    targets_failed: int = 0

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "target_id": self.target_id,
            "target_label": self.target_label,
            "distance_to_target": round(self.distance_to_target, 2),
            "servo_angle": round(self.servo_angle, 1),
            "spray_count": self.spray_count,
            "verification_passed": self.verification_passed,
            "upload_url": self.upload_url,
            "error": self.error,
            "nav2_goal_id": self.nav2_goal_id,
            "nav2_approach_active": self.nav2_approach_active,
            "approach_method": self.approach_method,
            "targets_engaged": self.targets_engaged,
            "targets_succeeded": self.targets_succeeded,
            "targets_failed": self.targets_failed,
        }


# Ballistic drop compensation table (SP-004)
# Default values — overridden by calibration file if available
BALLISTIC_DROP_TABLE = {
    1.0: 0.0,    # 1m - negligible drop
    2.0: 2.0,    # 2m - ~2 degrees down
    3.0: 5.0,    # 3m - ~5 degrees down
    4.0: 8.0,    # 4m - ~8 degrees down
    5.0: 12.0,   # 5m - ~12 degrees down
}

# Calibration file path — produced by bench calibration system (aeac_ws)
CALIBRATION_FILE = os.path.expanduser("~/.nomad/calibration/spray_calibration.json")


def _load_calibration() -> dict:
    """Load bench calibration data from JSON file if available.

    Expected format:
        {
            "ballistic_drop_table": {"2.0": 2.5, "3.0": 6.0, ...},
            "spray_duration_ms": 600,
            "servo_gain": 0.08,
            "aim_tolerance_px": 25,
            "calibration_date": "2026-05-10",
            "notes": "Bench test results"
        }
    """
    try:
        if os.path.exists(CALIBRATION_FILE):
            with open(CALIBRATION_FILE) as f:
                data = json.load(f)
            logger.info(f"Loaded spray calibration from {CALIBRATION_FILE}")
            return data
    except Exception as e:
        logger.warning(f"Failed to load spray calibration from {CALIBRATION_FILE}: {e}")
    return {}


# Apply calibration overrides at module load time
_calibration = _load_calibration()
if "ballistic_drop_table" in _calibration:
    BALLISTIC_DROP_TABLE = {
        float(k): float(v)
        for k, v in _calibration["ballistic_drop_table"].items()
    }
    logger.info(f"Using calibrated ballistic drop table: {BALLISTIC_DROP_TABLE}")


def _interpolate_drop(distance_m: float) -> float:
    """Interpolate ballistic drop angle for a given distance."""
    if distance_m <= 1.0:
        return 0.0
    if distance_m >= 5.0:
        return 12.0
    keys = sorted(BALLISTIC_DROP_TABLE.keys())
    for i in range(len(keys) - 1):
        if keys[i] <= distance_m <= keys[i + 1]:
            t = (distance_m - keys[i]) / (keys[i + 1] - keys[i])
            return (
                BALLISTIC_DROP_TABLE[keys[i]] * (1 - t)
                + BALLISTIC_DROP_TABLE[keys[i + 1]] * t
            )
    return 0.0


class SprayController:
    """
    Task 2 Hybrid Spray Controller.

    Operator positions drone within 3m via WASD, then system autonomously
    approaches to 2m (Nav2) and executes the spray sequence.
    """

    # Engagement parameters
    TRIGGER_MAX_DISTANCE_M = 3.0
    APPROACH_STOP_DISTANCE_M = 2.0
    APPROACH_SPEED_MPS = 0.5
    APPROACH_TIMEOUT_S = 20.0

    # Nav2 approach parameters
    NAV2_GOAL_SETTLE_TIME_S = 1.0
    NAV2_STATUS_POLL_INTERVAL_S = 0.2

    # Aiming parameters
    AIM_TOLERANCE_PX = 30

    # Spray parameters
    SPRAY_DURATION_MS = 500
    SPRAY_SETTLE_TIME_S = 0.5
    MAX_SPRAY_ATTEMPTS = 2

    # Visual servoing parameters
    IMAGE_CENTER_X = 640
    IMAGE_CENTER_Y = 360
    SERVO_GAIN = 0.1

    def __init__(
        self,
        nav_controller: Any = None,
        servo_controller: Any = None,
        state_manager: Any = None,
        mode_manager: Any = None,
    ):
        self._nav = nav_controller
        self._servo = servo_controller
        self._state = state_manager
        self._mode_mgr = mode_manager
        self._status = SprayStatus()
        self._lock = threading.RLock()
        self._current_target: Optional[SprayTarget] = None
        self._spray_count = 0
        self._thread: Optional[threading.Thread] = None
        self._abort_event = threading.Event()

        # Nav2 callbacks (set by main.py)
        self._send_nav2_goal_fn: Optional[Callable[[dict], dict]] = None
        self._get_nav2_status_fn: Optional[Callable[[], dict]] = None
        self._cancel_nav2_goal_fn: Optional[Callable[[], None]] = None
        # Obstacle avoidance sector exclusion callback
        self._set_excluded_sectors_fn: Optional[Callable[[set[int]], None]] = None

        # Photo / verify / upload callbacks (set externally)
        self._capture_photo_fn: Optional[Callable[[], Optional[str]]] = None
        self._verify_hsv_fn: Optional[Callable[[str], bool]] = None
        self._verify_circle_change_fn: Optional[Callable[[str, str], bool]] = None
        self._upload_fn: Optional[Callable[[str, str], str]] = None
        self._get_detection_bbox_fn: Optional[Callable[[int], Optional[tuple]]] = None

        # Pre-spray image path for circle change verification
        self._pre_spray_image_path: Optional[str] = None

        logger.info("Spray controller initialized (Nav2 approach + circle verify)")

    @property
    def status(self) -> SprayStatus:
        with self._lock:
            return self._status

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._status.state not in (
                "idle", "complete", "failed", "aborted"
            )

    # ------------------------------------------------------------------ #
    # Callback setters
    # ------------------------------------------------------------------ #

    def set_capture_photo_fn(self, fn: Callable) -> None:
        self._capture_photo_fn = fn

    def set_verify_hsv_fn(self, fn: Callable) -> None:
        """Legacy HSV verify callback (fallback if circle change unavailable)."""
        self._verify_hsv_fn = fn

    def set_verify_circle_change_fn(self, fn: Callable[[str, str], bool]) -> None:
        """Set callback for circle change verification.

        fn(pre_spray_path, post_spray_path) -> bool
        Returns True if circle change exceeds 20% threshold.
        """
        self._verify_circle_change_fn = fn

    def set_upload_fn(self, fn: Callable) -> None:
        self._upload_fn = fn

    def set_detection_bbox_fn(self, fn: Callable) -> None:
        self._get_detection_bbox_fn = fn

    def set_nav2_goal_fn(self, fn: Callable[[dict], dict]) -> None:
        """Set callback to send Nav2 navigation goal."""
        self._send_nav2_goal_fn = fn

    def set_nav2_status_fn(self, fn: Callable[[], dict]) -> None:
        """Set callback to read current Nav2 navigation status."""
        self._get_nav2_status_fn = fn

    def set_nav2_cancel_fn(self, fn: Callable[[], None]) -> None:
        """Set callback to cancel current Nav2 goal."""
        self._cancel_nav2_goal_fn = fn

    def set_excluded_sectors_fn(self, fn: Callable[[set[int]], None]) -> None:
        """Set callback to update obstacle avoidance excluded sectors."""
        self._set_excluded_sectors_fn = fn

    # ------------------------------------------------------------------ #
    # Trigger / Abort
    # ------------------------------------------------------------------ #

    def trigger(self, target: SprayTarget) -> dict:
        """Trigger autonomous spray sequence on target."""
        with self._lock:
            if self.is_active:
                return {"success": False, "error": "Spray sequence already active"}

            drone_pos = self._get_drone_position()
            if drone_pos is None:
                return {"success": False, "error": "Cannot determine drone position"}

            dx = target.x - drone_pos[0]
            dy = target.y - drone_pos[1]
            dz = target.z - drone_pos[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance > self.TRIGGER_MAX_DISTANCE_M:
                return {
                    "success": False,
                    "error": (
                        f"Drone too far ({distance:.1f}m > "
                        f"{self.TRIGGER_MAX_DISTANCE_M}m). Position manually "
                        f"within {self.TRIGGER_MAX_DISTANCE_M}m before triggering."
                    ),
                }

            skip_approach = distance < self.APPROACH_STOP_DISTANCE_M
            self._current_target = target
            self._spray_count = 0
            self._abort_event.clear()
            self._pre_spray_image_path = None

            self._status.state = (
                SprayState.AIM.value if skip_approach
                else SprayState.APPROACH.value
            )
            self._status.target_id = target.target_id
            self._status.target_label = target.label
            self._status.distance_to_target = distance
            self._status.spray_count = 0
            self._status.verification_passed = False
            self._status.error = None
            self._status.nav2_goal_id = None
            self._status.nav2_approach_active = False
            self._status.approach_method = ""
            self._status.targets_engaged += 1

        # Run sequence in background thread (outside lock)
        self._thread = threading.Thread(
            target=self._run_sequence,
            args=(skip_approach,),
            daemon=True,
        )
        self._thread.start()

        logger.info(
            f"Spray sequence triggered: target {target.target_id} "
            f"({target.label}) at {distance:.1f}m"
            + (" [SKIP APPROACH]" if skip_approach else "")
        )
        return {
            "success": True,
            "target_id": target.target_id,
            "distance": round(distance, 2),
            "skip_approach": skip_approach,
        }

    def abort(self) -> dict:
        """Abort the current spray sequence."""
        self._abort_event.set()

        # Cancel Nav2 goal if active
        if self._cancel_nav2_goal_fn and self._status.nav2_approach_active:
            try:
                self._cancel_nav2_goal_fn()
                logger.info("Nav2 goal cancelled on spray abort")
            except Exception as e:
                logger.warning(f"Nav2 cancel failed on abort: {e}")

        # Stop drone movement
        if self._nav:
            self._nav.stop_movement()

        # Clear obstacle avoidance exclusions
        if self._set_excluded_sectors_fn:
            try:
                self._set_excluded_sectors_fn(set())
            except Exception:
                pass

        with self._lock:
            self._status.state = SprayState.ABORTED.value
            self._status.nav2_approach_active = False
        logger.info("Spray sequence aborted")
        return {"success": True, "message": "Spray sequence aborted"}

    def update_nav2_result(self, goal_id: str, status: str, message: str = "") -> None:
        """Called by API when Nav2 reports a goal result."""
        with self._lock:
            if (
                self._status.nav2_goal_id == goal_id
                and self._status.nav2_approach_active
            ):
                self._status.nav2_approach_active = False
                logger.info(
                    f"Nav2 approach result: goal={goal_id} status={status} "
                    f"msg={message}"
                )

    # ------------------------------------------------------------------ #
    # Sequence runner
    # ------------------------------------------------------------------ #

    def _run_sequence(self, skip_approach: bool = False) -> None:
        """Run autonomous spray sequence."""
        try:
            target = self._current_target
            if target is None:
                return

            # --- APPROACH (3m -> 2m via Nav2) ---
            if not skip_approach:
                self._set_state(SprayState.APPROACH)
                if not self._approach_target(target):
                    return

            # --- AIM ---
            self._set_state(SprayState.AIM)
            if not self._aim_at_target(target):
                return

            # --- CAPTURE PRE-SPRAY SNAPSHOT ---
            self._capture_pre_spray(target)

            # --- SPRAY (with retry) ---
            for attempt in range(self.MAX_SPRAY_ATTEMPTS):
                self._spray_count = attempt + 1
                with self._lock:
                    self._status.spray_count = self._spray_count

                self._set_state(SprayState.SPRAY)
                if not self._spray_target():
                    return

                # --- VERIFY ---
                self._set_state(SprayState.VERIFY)
                time.sleep(self.SPRAY_SETTLE_TIME_S)
                passed = self._verify_spray()
                if passed:
                    with self._lock:
                        self._status.verification_passed = True
                    break
                elif attempt < self.MAX_SPRAY_ATTEMPTS - 1:
                    logger.info(
                        f"Spray attempt {attempt + 1} failed verification, "
                        "retrying..."
                    )
                    self._set_state(SprayState.AIM)
                    self._aim_at_target(target)

            # --- UPLOAD ---
            self._set_state(SprayState.UPLOAD)
            self._capture_and_upload(target)

            # --- COMPLETE ---
            with self._lock:
                if self._status.verification_passed:
                    self._status.targets_succeeded += 1
                else:
                    self._status.targets_failed += 1
                    logger.warning(
                        f"Target {target.target_id} failed after "
                        f"{self.MAX_SPRAY_ATTEMPTS} attempts"
                    )
            self._set_state(SprayState.COMPLETE)

        except Exception as e:
            logger.error(f"Spray sequence error: {e}")
            self._set_state(SprayState.FAILED, error=str(e))

    def _set_state(self, state: SprayState, error: Optional[str] = None) -> None:
        with self._lock:
            self._status.state = state.value
            if error:
                self._status.error = error

    def _check_abort(self) -> bool:
        return self._abort_event.is_set()

    def _get_drone_position(self) -> Optional[tuple[float, float, float]]:
        """Get current drone NED position from state manager."""
        if not self._state:
            return None
        state = self._state.get_state()
        x = getattr(state, 'vio_x', None)
        y = getattr(state, 'vio_y', None)
        z = getattr(state, 'vio_z', None)
        if x is not None and y is not None and z is not None:
            return (x, y, z)
        return None

    # ------------------------------------------------------------------ #
    # APPROACH (Nav2 primary, velocity fallback)
    # ------------------------------------------------------------------ #

    def _compute_approach_pose(self, target: SprayTarget) -> dict:
        """Compute the 2m approach pose for Nav2 NavigateToPose goal."""
        drone_pos = self._get_drone_position()
        if drone_pos is None:
            dx, dy, dz = target.x, target.y, target.z
        else:
            dx = target.x - drone_pos[0]
            dy = target.y - drone_pos[1]
            dz = target.z - drone_pos[2]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 0.1:
            return {"x": target.x, "y": target.y, "z": target.z, "yaw": 0.0}

        nx, ny, nz = dx / dist, dy / dist, dz / dist
        approach_dist = self.APPROACH_STOP_DISTANCE_M
        approach_x = target.x - nx * approach_dist
        approach_y = target.y - ny * approach_dist
        approach_z = target.z - nz * approach_dist
        yaw = math.atan2(dy, dx)
        return {"x": approach_x, "y": approach_y, "z": approach_z, "yaw": yaw}

    def _approach_target(self, target: SprayTarget) -> bool:
        """Autonomous approach from 3m to 2m. Returns False if aborted.

        Uses direct velocity commands (not Nav2) because:
        - Nav2 is a 2D planner designed for ground robots — it ignores Z
        - The approach is only ~1m (3m→2m), obstacle avoidance is unnecessary
        - Velocity commands handle all 3 axes natively via MAVLink GUIDED
        - Nav2 has been reported to crash with vertical-level targets
        """
        self._set_approach_sectors(target, exclude=True)
        return self._approach_via_velocity(target)

    def _approach_via_nav2(self, target: SprayTarget) -> Optional[bool]:
        """Approach using Nav2 NavigateToPose.

        Returns: True=succeeded, False=aborted, None=should fall back.
        """
        if not self._send_nav2_goal_fn:
            return None

        approach_pose = self._compute_approach_pose(target)
        goal_dict = {"type": "navigate_to_pose", "pose": approach_pose}

        try:
            result = self._send_nav2_goal_fn(goal_dict)
        except Exception as e:
            logger.error(f"Failed to send Nav2 goal: {e}")
            return None

        if not result.get("success", False):
            logger.warning(f"Nav2 goal rejected: {result.get('error', 'unknown')}")
            return None

        goal_id = result.get("goal_id", "")
        with self._lock:
            self._status.nav2_goal_id = goal_id
            self._status.nav2_approach_active = True
            self._status.approach_method = "nav2"

        logger.info(f"Nav2 approach goal sent: id={goal_id}")

        # Wait for Nav2 result
        approach_start = time.time()
        while not self._check_abort():
            if time.time() - approach_start > self.APPROACH_TIMEOUT_S:
                logger.warning(f"Nav2 approach timeout ({self.APPROACH_TIMEOUT_S}s)")
                if self._cancel_nav2_goal_fn:
                    try:
                        self._cancel_nav2_goal_fn()
                    except Exception:
                        pass
                with self._lock:
                    self._status.nav2_approach_active = False
                # Timeout: proceed from current position
                self._set_approach_sectors(target, exclude=False)
                return True

            # Check if Nav2 result was reported
            with self._lock:
                if not self._status.nav2_approach_active:
                    # Result received via update_nav2_result()
                    if self._get_nav2_status_fn:
                        nav2_status = self._get_nav2_status_fn()
                        final_status = nav2_status.get("status", "unknown")
                    else:
                        final_status = "unknown"

                    if final_status == "succeeded":
                        logger.info("Nav2 approach succeeded")
                        time.sleep(self.NAV2_GOAL_SETTLE_TIME_S)
                        self._update_distance_to_target(target)
                        self._set_approach_sectors(target, exclude=False)
                        return True
                    elif final_status == "cancelled":
                        if self._check_abort():
                            return False
                        return None  # Non-abort cancel, fall back
                    else:
                        logger.warning(f"Nav2 approach result: {final_status}")
                        return None

            self._update_distance_to_target(target)
            time.sleep(self.NAV2_STATUS_POLL_INTERVAL_S)

        # Aborted
        with self._lock:
            self._status.nav2_approach_active = False
        self._set_approach_sectors(target, exclude=False)
        return False

    def _approach_via_velocity(self, target: SprayTarget) -> bool:
        """Fallback approach using direct velocity commands."""
        if not self._nav:
            logger.error("NavController not available for approach")
            self._set_state(SprayState.FAILED, error="NavController unavailable")
            return False

        with self._lock:
            self._status.approach_method = "velocity"

        approach_start = time.time()
        while not self._check_abort():
            if time.time() - approach_start > self.APPROACH_TIMEOUT_S:
                logger.warning("Velocity approach timeout - continuing")
                return True

            drone_pos = self._get_drone_position()
            if drone_pos is None:
                time.sleep(0.1)
                continue

            dx = target.x - drone_pos[0]
            dy = target.y - drone_pos[1]
            dz = target.z - drone_pos[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            with self._lock:
                self._status.distance_to_target = distance

            if distance < self.APPROACH_STOP_DISTANCE_M:
                logger.info(f"Velocity approach complete at {distance:.2f}m")
                if self._nav:
                    self._nav.stop_movement()
                self._set_approach_sectors(target, exclude=False)
                return True

            norm = max(distance, 0.1)
            speed = min(self.APPROACH_SPEED_MPS, distance * 0.5)
            vx = (dx / norm) * speed
            vy = (dy / norm) * speed
            vz = (dz / norm) * speed * 0.3
            self._nav.send_velocity(vx, vy, vz, 0)
            time.sleep(0.1)

        return False

    def _update_distance_to_target(self, target: SprayTarget) -> None:
        drone_pos = self._get_drone_position()
        if drone_pos is None:
            return
        dx = target.x - drone_pos[0]
        dy = target.y - drone_pos[1]
        dz = target.z - drone_pos[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        with self._lock:
            self._status.distance_to_target = distance

    def _set_approach_sectors(
        self, target: SprayTarget, *, exclude: bool
    ) -> None:
        """Set or clear obstacle avoidance sector exclusions for approach."""
        if not self._set_excluded_sectors_fn:
            return
        if not exclude:
            self._set_excluded_sectors_fn(set())
            return

        drone_pos = self._get_drone_position()
        if drone_pos is None:
            return
        dx = target.x - drone_pos[0]
        dy = target.y - drone_pos[1]
        angle_deg = math.degrees(math.atan2(dy, dx)) % 360
        center_sector = int(angle_deg / 5) % 72

        sectors = set()
        for offset in range(-2, 3):
            sectors.add((center_sector + offset) % 72)
        self._set_excluded_sectors_fn(sectors)
        logger.debug(f"Excluded obstacle sectors {sectors} for approach")

    # ------------------------------------------------------------------ #
    # AIM (visual servoing)
    # ------------------------------------------------------------------ #

    def _aim_at_target(self, target: SprayTarget) -> bool:
        """Visual servoing to center target in camera via servo pitch."""
        if not self._get_detection_bbox_fn:
            logger.warning("No detection bbox function - skipping aim")
            return True

        aim_start = time.time()
        max_aim_time = 15.0
        while not self._check_abort():
            if time.time() - aim_start > max_aim_time:
                logger.warning("Aim timeout - proceeding with current alignment")
                return True

            bbox = self._get_detection_bbox_fn(target.target_id)
            if bbox is None:
                time.sleep(0.1)
                continue

            cx, cy, w, h = bbox
            err_x = cx - self.IMAGE_CENTER_X
            err_y = cy - self.IMAGE_CENTER_Y

            if (
                abs(err_x) < self.AIM_TOLERANCE_PX
                and abs(err_y) < self.AIM_TOLERANCE_PX
            ):
                return True

            if self._servo:
                pitch_adjust = err_y * self.SERVO_GAIN
                drone_pos = self._get_drone_position()
                if drone_pos:
                    dist = math.sqrt(
                        (target.x - drone_pos[0]) ** 2
                        + (target.y - drone_pos[1]) ** 2
                    )
                    drop_angle = _interpolate_drop(dist)
                    pitch_adjust += drop_angle

                current_angle = self._status.servo_angle
                new_angle = max(0, min(180, current_angle - pitch_adjust))
                self._servo.set_camera_tilt(new_angle)
                with self._lock:
                    self._status.servo_angle = new_angle

            time.sleep(0.1)

        return False

    # ------------------------------------------------------------------ #
    # CAPTURE PRE-SPRAY
    # ------------------------------------------------------------------ #

    def _capture_pre_spray(self, target: SprayTarget) -> None:
        """Capture a pre-spray image for circle change verification.

        Called before the first spray attempt. Only captures once per sequence.
        """
        if self._pre_spray_image_path is not None:
            return  # Already captured

        try:
            if self._capture_photo_fn:
                path = self._capture_photo_fn()
                if path:
                    self._pre_spray_image_path = path
                    logger.info(f"Pre-spray image captured: {path}")
                else:
                    logger.warning("Pre-spray capture returned no path")
            else:
                logger.warning("No capture_photo_fn for pre-spray snapshot")
        except Exception as e:
            logger.error(f"Pre-spray capture error: {e}")

    # ------------------------------------------------------------------ #
    # SPRAY
    # ------------------------------------------------------------------ #

    def _spray_target(self) -> bool:
        """Activate water pump."""
        if not self._servo:
            logger.error("ServoController not available for spray")
            return True

        logger.info(f"Spraying target (attempt {self._spray_count})")
        success = self._servo.trigger_water_shooter(
            duration_ms=self.SPRAY_DURATION_MS
        )
        if not success:
            logger.warning("Water shooter trigger returned failure")
        return True  # proceed regardless

    # ------------------------------------------------------------------ #
    # VERIFY (circle change detection, HSV fallback)
    # ------------------------------------------------------------------ #

    def _verify_spray(self) -> bool:
        """Post-spray circle change verification.

        Captures a post-spray image and compares with the pre-spray image.
        Detects circles color-agnostically in both images, matches them
        by proximity, and computes pixel change ratio within matched circles.
        If > 20% of pixels changed significantly, the spray hit.

        Falls back to legacy HSV verification if circle change callback
        is not available.
        """
        # Primary: circle change verification
        if self._verify_circle_change_fn and self._pre_spray_image_path:
            try:
                post_path = None
                if self._capture_photo_fn:
                    post_path = self._capture_photo_fn()
                if post_path:
                    result = self._verify_circle_change_fn(
                        self._pre_spray_image_path, post_path
                    )
                    logger.info(
                        f"Circle change verify: pre={self._pre_spray_image_path} "
                        f"post={post_path} -> {'PASS' if result else 'FAIL'}"
                    )
                    return result
                else:
                    logger.warning("Could not capture post-spray photo")
                    return False
            except Exception as e:
                logger.error(f"Circle change verification error: {e}")
                return False

        # Fallback: legacy HSV verification (if available)
        if self._verify_hsv_fn:
            logger.info("Falling back to legacy HSV verification")
            try:
                photo_path = None
                if self._capture_photo_fn:
                    photo_path = self._capture_photo_fn()
                if photo_path:
                    return self._verify_hsv_fn(photo_path)
                else:
                    logger.warning("Could not capture photo for HSV verify")
                    return False
            except Exception as e:
                logger.error(f"HSV verification error: {e}")
                return False

        # No verification function at all - assume pass
        logger.warning("No verify function available - assuming pass")
        return True

    # ------------------------------------------------------------------ #
    # UPLOAD
    # ------------------------------------------------------------------ #

    def _capture_and_upload(self, target: SprayTarget) -> None:
        """Capture photo and upload to Google Drive.

        Filename: Task_2_MAD_target_<n>.jpg (matches CONOPS Section 5.2.4)
        """
        try:
            photo_path = None
            if self._capture_photo_fn:
                photo_path = self._capture_photo_fn()
            if not photo_path:
                logger.warning("Could not capture photo for upload")
                return

            filename = f"Task_2_MAD_target_{target.target_id}.jpg"
            if self._upload_fn:
                url = self._upload_fn(photo_path, filename)
                with self._lock:
                    self._status.upload_url = url or ""
                logger.info(f"Uploaded {filename} -> {url}")
            else:
                logger.warning("No upload function - photo saved locally only")
        except Exception as e:
            logger.error(f"Upload error: {e}")


# ---------------------------------------------------------------------- #
# Apply calibration overrides to class parameters (after class definition)
# ---------------------------------------------------------------------- #
if "spray_duration_ms" in _calibration:
    SprayController.SPRAY_DURATION_MS = int(_calibration["spray_duration_ms"])
    logger.info(f"Using calibrated spray duration: {SprayController.SPRAY_DURATION_MS}ms")
if "servo_gain" in _calibration:
    SprayController.SERVO_GAIN = float(_calibration["servo_gain"])
    logger.info(f"Using calibrated servo gain: {SprayController.SERVO_GAIN}")
if "aim_tolerance_px" in _calibration:
    SprayController.AIM_TOLERANCE_PX = int(_calibration["aim_tolerance_px"])
    logger.info(f"Using calibrated aim tolerance: {SprayController.AIM_TOLERANCE_PX}px")
