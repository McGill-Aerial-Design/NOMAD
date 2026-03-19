"""
NOMAD Edge Core - Autonomous Spray Controller (SP-001 to SP-008)

State machine for Task 2 autonomous target engagement:
    IDLE -> APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE

Requirements:
    SP-001: Trigger from > 2m distance (parallel to target plane)
    SP-002: Fully autonomous after trigger
    SP-003: Visual servoing (drone + servo) to center target
    SP-004: Nozzle pitch + ballistic drop compensation
    SP-005: Obstacle avoidance sector exclusion during approach
    SP-006: Ground target vertical descent (min alt 0.8m)
    SP-007: Post-spray photo + HSV verification + Google Drive upload
    SP-008: Re-spray once on verification failure

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional

logger = logging.getLogger("edge_core.spray_controller")


class SprayState(Enum):
    """Spray sequence states."""
    IDLE = "idle"
    APPROACH = "approach"       # Moving toward target
    AIM = "aim"                 # Fine alignment with visual servoing
    SPRAY = "spray"             # Activating water pump
    VERIFY = "verify"           # HSV color check on sprayed target
    UPLOAD = "upload"           # Photo capture + Google Drive upload
    COMPLETE = "complete"       # Sequence done, ready for next target
    FAILED = "failed"           # Sequence failed (after retry)
    ABORTED = "aborted"         # Operator or safety abort


@dataclass
class SprayTarget:
    """Target to engage."""
    target_id: int
    x: float            # world-frame position
    y: float
    z: float
    label: str = ""
    confidence: float = 0.0
    is_ground: bool = False  # True if target is on the ground (SP-006)


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
            "targets_engaged": self.targets_engaged,
            "targets_succeeded": self.targets_succeeded,
            "targets_failed": self.targets_failed,
        }


# Ballistic drop compensation table (SP-004)
# Maps engagement distance (m) to additional pitch-down angle (degrees)
# Based on water nozzle at ~2 bar pressure, measured empirically
BALLISTIC_DROP_TABLE = {
    1.0: 0.0,   # 1m - negligible drop
    2.0: 2.0,   # 2m - ~2 degrees down
    3.0: 5.0,   # 3m - ~5 degrees down
    4.0: 8.0,   # 4m - ~8 degrees down
    5.0: 12.0,  # 5m - ~12 degrees down
}


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
    Autonomous spray sequence controller for Task 2.

    Orchestrates the full engage sequence: approach, aim, spray, verify, upload.
    Uses NavController for drone movement and ServoController for nozzle aiming.
    """

    # Engagement parameters
    TRIGGER_MIN_DISTANCE_M = 2.0    # SP-001: min distance to trigger
    APPROACH_SPEED_MPS = 0.5        # approach velocity
    AIM_TOLERANCE_PX = 30           # pixel tolerance for target centering
    SPRAY_DURATION_MS = 500         # water pump duration per spray
    SPRAY_SETTLE_TIME_S = 0.5       # wait after spray before verify
    GROUND_TARGET_MIN_ALT_M = 0.8   # SP-006: minimum altitude for ground targets
    GROUND_TARGET_HOVER_ALT_M = 1.2 # SP-006: hover altitude above ground target
    MAX_SPRAY_ATTEMPTS = 2          # SP-008: max spray attempts

    # Visual servoing parameters
    IMAGE_CENTER_X = 640            # assume 1280x720 image
    IMAGE_CENTER_Y = 360
    SERVO_GAIN = 0.1                # degrees per pixel error

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

        # Callbacks (set externally)
        self._capture_photo_fn: Optional[Callable[[], Optional[str]]] = None
        self._verify_hsv_fn: Optional[Callable[[str], bool]] = None
        self._upload_fn: Optional[Callable[[str, str], str]] = None
        self._get_detection_bbox_fn: Optional[Callable[[int], Optional[tuple]]] = None
        self._set_excluded_sectors_fn: Optional[Callable[[set], None]] = None

        logger.info("Spray controller initialized")

    @property
    def status(self) -> SprayStatus:
        with self._lock:
            return self._status

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._status.state not in ("idle", "complete", "failed", "aborted")

    def set_capture_photo_fn(self, fn: Callable) -> None:
        self._capture_photo_fn = fn

    def set_verify_hsv_fn(self, fn: Callable) -> None:
        self._verify_hsv_fn = fn

    def set_upload_fn(self, fn: Callable) -> None:
        self._upload_fn = fn

    def set_detection_bbox_fn(self, fn: Callable) -> None:
        self._get_detection_bbox_fn = fn

    def set_excluded_sectors_fn(self, fn: Callable) -> None:
        self._set_excluded_sectors_fn = fn

    def trigger(self, target: SprayTarget) -> dict:
        """
        Trigger autonomous spray sequence on a target (SP-001).

        Args:
            target: Target to engage.

        Returns:
            dict with success status.
        """
        with self._lock:
            if self.is_active:
                return {"success": False, "error": "Spray sequence already active"}

        # SP-001: Validate trigger distance
        drone_pos = self._get_drone_position()
        if drone_pos is None:
            return {"success": False, "error": "Cannot determine drone position"}

        dx = target.x - drone_pos[0]
        dy = target.y - drone_pos[1]
        dz = target.z - drone_pos[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < self.TRIGGER_MIN_DISTANCE_M:
            return {
                "success": False,
                "error": f"Too close to target ({distance:.1f}m < {self.TRIGGER_MIN_DISTANCE_M}m). "
                         f"Must be > {self.TRIGGER_MIN_DISTANCE_M}m per SP-001.",
            }

        # Start the sequence
        self._current_target = target
        self._spray_count = 0
        self._abort_event.clear()

        with self._lock:
            self._status.state = SprayState.APPROACH.value
            self._status.target_id = target.target_id
            self._status.target_label = target.label
            self._status.distance_to_target = distance
            self._status.spray_count = 0
            self._status.verification_passed = False
            self._status.error = None
            self._status.targets_engaged += 1

        # Switch to spray approach mode (SP-005)
        if self._mode_mgr:
            self._mode_mgr.switch_mode("spray_approach")

        # Calculate and exclude target sector from obstacle avoidance (SP-005)
        self._update_excluded_sector(target)

        # Run sequence in background thread (SP-002)
        self._thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._thread.start()

        logger.info(
            f"Spray sequence triggered: target {target.target_id} "
            f"({target.label}) at {distance:.1f}m"
        )

        return {
            "success": True,
            "target_id": target.target_id,
            "distance": round(distance, 2),
        }

    def abort(self) -> dict:
        """Abort the current spray sequence."""
        self._abort_event.set()
        with self._lock:
            self._status.state = SprayState.ABORTED.value
        # Stop drone movement
        if self._nav:
            self._nav.stop_movement()
        # Clear excluded sectors
        if self._set_excluded_sectors_fn:
            self._set_excluded_sectors_fn(set())
        logger.info("Spray sequence aborted")
        return {"success": True, "message": "Spray sequence aborted"}

    def _run_sequence(self) -> None:
        """
        Run the full autonomous spray sequence (SP-002).

        States: APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE
        """
        try:
            target = self._current_target
            if target is None:
                return

            # --- APPROACH ---
            self._set_state(SprayState.APPROACH)
            if not self._approach_target(target):
                return

            # --- AIM ---
            self._set_state(SprayState.AIM)
            if not self._aim_at_target(target):
                return

            # --- SPRAY (with retry per SP-008) ---
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
                        f"SP-008: Spray attempt {attempt + 1} failed verification, "
                        f"retrying..."
                    )
                    # Re-aim before retry
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
                        f"SP-008: Target {target.target_id} failed after "
                        f"{self.MAX_SPRAY_ATTEMPTS} attempts"
                    )

            self._set_state(SprayState.COMPLETE)

        except Exception as e:
            logger.error(f"Spray sequence error: {e}")
            self._set_state(SprayState.FAILED, error=str(e))

        finally:
            # Clean up: clear excluded sectors, return to previous mode
            if self._set_excluded_sectors_fn:
                self._set_excluded_sectors_fn(set())

    def _set_state(self, state: SprayState, error: str = None) -> None:
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

    def _approach_target(self, target: SprayTarget) -> bool:
        """
        Approach the target (SP-003, SP-006).

        For wall targets: fly forward toward target.
        For ground targets: descend vertically above target (SP-006).
        """
        if not self._nav:
            logger.error("NavController not available for approach")
            self._set_state(SprayState.FAILED, error="NavController unavailable")
            return False

        approach_start = time.time()
        max_approach_time = 30.0  # safety timeout

        while not self._check_abort():
            if time.time() - approach_start > max_approach_time:
                logger.error("Approach timeout")
                self._set_state(SprayState.FAILED, error="Approach timeout")
                return False

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

            # Check if close enough to start aiming
            if distance < 2.5:  # 2.5m engagement distance
                self._nav.stop_movement()
                return True

            # SP-006: Ground target - vertical descent
            if target.is_ground:
                horiz_dist = math.sqrt(dx * dx + dy * dy)
                if horiz_dist < 0.5:
                    # Above target - descend
                    alt = -drone_pos[2]  # NED Z is down, so altitude = -z
                    if alt > self.GROUND_TARGET_MIN_ALT_M:
                        self._nav.send_velocity(0, 0, -0.3, 0)  # descend
                    else:
                        self._nav.stop_movement()
                        return True
                else:
                    # Move horizontally above target first
                    speed = min(self.APPROACH_SPEED_MPS, horiz_dist)
                    vx = (dx / horiz_dist) * speed
                    vy = (dy / horiz_dist) * speed
                    self._nav.send_velocity(vx, vy, 0, 0)
            else:
                # Wall target - fly toward it
                speed = min(self.APPROACH_SPEED_MPS, distance * 0.3)
                norm = max(distance, 0.1)
                vx = (dx / norm) * speed
                vy = (dy / norm) * speed
                vz = (dz / norm) * speed * 0.5  # slower vertical
                self._nav.send_velocity(vx, vy, vz, 0)

            time.sleep(0.1)  # 10 Hz control loop

        return False

    def _aim_at_target(self, target: SprayTarget) -> bool:
        """
        Visual servoing to center target in camera (SP-003, SP-004).

        Commands both drone lateral adjustments and servo pitch to
        center the target bounding box in the camera frame.
        """
        if not self._get_detection_bbox_fn:
            logger.warning("No detection bbox function - skipping aim")
            return True  # proceed without aiming

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

            cx, cy, w, h = bbox  # center x, center y, width, height

            # Error from image center
            err_x = cx - self.IMAGE_CENTER_X
            err_y = cy - self.IMAGE_CENTER_Y

            # Check if centered enough
            if abs(err_x) < self.AIM_TOLERANCE_PX and abs(err_y) < self.AIM_TOLERANCE_PX:
                if self._nav:
                    self._nav.stop_movement()
                return True

            # SP-003: Command drone lateral adjustment
            if self._nav and abs(err_x) > self.AIM_TOLERANCE_PX:
                vy = err_x * 0.001  # proportional lateral velocity
                vy = max(-0.3, min(0.3, vy))
                self._nav.send_velocity(0, vy, 0, 0)

            # SP-003 + SP-004: Command servo pitch
            if self._servo:
                # Vertical error -> servo pitch adjustment
                pitch_adjust = err_y * self.SERVO_GAIN

                # SP-004: Add ballistic drop compensation
                drone_pos = self._get_drone_position()
                if drone_pos:
                    dist = math.sqrt(
                        (target.x - drone_pos[0]) ** 2
                        + (target.y - drone_pos[1]) ** 2
                    )
                    drop_angle = _interpolate_drop(dist)
                    pitch_adjust += drop_angle

                # Convert to servo angle (90 = level, lower = pitch down)
                current_angle = self._status.servo_angle
                new_angle = max(0, min(180, current_angle - pitch_adjust))
                self._servo.set_camera_tilt(new_angle)

                with self._lock:
                    self._status.servo_angle = new_angle

            time.sleep(0.1)

        return False

    def _spray_target(self) -> bool:
        """Activate water pump (SP-002)."""
        if not self._servo:
            logger.error("ServoController not available for spray")
            return True  # don't fail the whole sequence

        logger.info(f"Spraying target (attempt {self._spray_count})")

        success = self._servo.trigger_water_shooter(
            duration_ms=self.SPRAY_DURATION_MS
        )

        if not success:
            logger.warning("Water shooter trigger returned failure")

        return True  # proceed regardless

    def _verify_spray(self) -> bool:
        """
        Post-spray HSV verification (SP-007).

        Checks if target color has shifted from purple to blue,
        indicating successful spray.
        """
        if not self._verify_hsv_fn:
            logger.warning("No HSV verify function - assuming pass")
            return True

        try:
            # Capture current frame and verify
            photo_path = None
            if self._capture_photo_fn:
                photo_path = self._capture_photo_fn()

            if photo_path:
                return self._verify_hsv_fn(photo_path)
            else:
                logger.warning("Could not capture photo for verification")
                return False

        except Exception as e:
            logger.error(f"Verification error: {e}")
            return False

    def _capture_and_upload(self, target: SprayTarget) -> None:
        """
        Capture photo and upload to Google Drive (SP-007).

        Filename format: Task_2_MAD_target_<n>.jpg
        """
        try:
            # Capture photo
            photo_path = None
            if self._capture_photo_fn:
                photo_path = self._capture_photo_fn()

            if not photo_path:
                logger.warning("Could not capture photo for upload")
                return

            # Upload to Google Drive
            filename = f"Task_2_MAD_target_{target.target_id}.jpg"

            if self._upload_fn:
                url = self._upload_fn(photo_path, filename)
                with self._lock:
                    self._status.upload_url = url or ""
                logger.info(f"SP-007: Uploaded {filename} -> {url}")
            else:
                logger.warning("No upload function configured - photo saved locally only")

        except Exception as e:
            logger.error(f"Upload error: {e}")

    def _update_excluded_sector(self, target: SprayTarget) -> None:
        """
        Calculate and set excluded obstacle avoidance sector (SP-005).

        The sector containing the target is excluded from OBSTACLE_DISTANCE
        so the drone can approach without avoidance interference.
        """
        drone_pos = self._get_drone_position()
        if drone_pos is None or self._set_excluded_sectors_fn is None:
            return

        dx = target.x - drone_pos[0]
        dy = target.y - drone_pos[1]

        # Angle to target (NED: atan2(east, north))
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad) % 360

        # Exclude a 30-degree wide sector centered on the target
        sector_width = 30
        center_sector = int(angle_deg / 5) % 72
        excluded = set()
        for offset in range(-sector_width // 10, sector_width // 10 + 1):
            excluded.add((center_sector + offset) % 72)

        self._set_excluded_sectors_fn(excluded)
        logger.info(
            f"SP-005: Excluded sectors {excluded} "
            f"(target at {angle_deg:.0f} deg)"
        )
