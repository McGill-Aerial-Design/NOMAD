"""
NOMAD Edge Core - Task 2 Spray Controller (Hybrid Manual+Autonomous)

Workflow:
  Operator manually positions until the ZED sees the target.
  Upon spray trigger, system autonomously approaches, aligns, and executes spray sequence.

State Machine:
  IDLE -> APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE

Features:
  - Operator manual positioning (WASD) until target is visible
  - Autonomous approach via MAVLink velocity commands
  - Pre-spray image capture for circle change verification
  - Circle change verification: color-agnostic before/after comparison (>20% change)
  - Visual servoing: calibrated aim pixel, ZED range, velocity/yaw-rate control
  - Google Drive upload: posts proof photo autonomously
  - Calibration data loaded from ~/.nomad/calibration/spray_calibration.json
  - Obstacle avoidance sector exclusion during approach

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import copy
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
    APPROACH = "approach"    # Autonomous visible-target approach
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
    image_only: bool = False
    range_m: Optional[float] = None


@dataclass
class SprayStatus:
    """Current spray sequence status."""
    state: str = "idle"
    target_id: int = -1
    target_number: int = 0
    target_label: str = ""
    distance_to_target: float = 0.0
    servo_angle: float = 90.0
    spray_count: int = 0
    verification_passed: bool = False
    upload_url: str = ""
    error: Optional[str] = None
    approach_method: str = ""  # "image" or "velocity" for the current Task 2 path
    autonomy_action: str = ""
    command_vx_mps: float = 0.0
    command_vy_mps: float = 0.0
    command_vz_mps: float = 0.0
    command_yaw_rate_radps: float = 0.0
    aim_error_x_px: Optional[float] = None
    aim_error_y_px: Optional[float] = None
    range_error_m: Optional[float] = None
    # Was this run triggered with the CONOPS Q&A #10 autonomy gate (i.e. the
    # operator pressed Auto Spray with require_autonomy=True)? Stays True
    # for the duration of the run so the UI can render it correctly.
    require_autonomy: bool = False
    # Becomes True if the pilot flips the RC mode switch out of GUIDED while
    # the autonomy sequence is running. The sequence aborts cleanly and this
    # flag tells the GCS that the 20-pt autonomy claim is forfeit for THIS
    # target - a fresh target can still be claimed.
    autonomy_compromised: bool = False
    # Stats
    targets_engaged: int = 0
    targets_succeeded: int = 0
    targets_failed: int = 0

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "target_id": self.target_id,
            "target_number": self.target_number,
            "target_label": self.target_label,
            "distance_to_target": round(self.distance_to_target, 2),
            "servo_angle": round(self.servo_angle, 1),
            "spray_count": self.spray_count,
            "verification_passed": self.verification_passed,
            "upload_url": self.upload_url,
            "error": self.error,
            "approach_method": self.approach_method,
            "autonomy_action": self.autonomy_action,
            "command_vx_mps": round(self.command_vx_mps, 3),
            "command_vy_mps": round(self.command_vy_mps, 3),
            "command_vz_mps": round(self.command_vz_mps, 3),
            "command_yaw_rate_radps": round(self.command_yaw_rate_radps, 3),
            "aim_error_x_px": (
                None if self.aim_error_x_px is None else round(self.aim_error_x_px, 1)
            ),
            "aim_error_y_px": (
                None if self.aim_error_y_px is None else round(self.aim_error_y_px, 1)
            ),
            "range_error_m": (
                None if self.range_error_m is None else round(self.range_error_m, 2)
            ),
            "require_autonomy": self.require_autonomy,
            "autonomy_compromised": self.autonomy_compromised,
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

DEFAULT_SPRAY_CALIBRATION = {
    # Fixed firing geometry. Calibrate these at the wall, then make the
    # aircraft reproduce the same view before every shot.
    "target_camera_range_m": 3.8,
    "range_tolerance_m": 0.25,
    "trigger_max_distance_m": 5.5,
    "aim_pixel_x": 640,
    "aim_pixel_y": 390,
    "aim_tolerance_px": 25,
    "servo_fire_angle_deg": 82.0,
    "spray_duration_ms": 500,
    "water_pump_relay_number": 0,
    # Visual-servo controller gains. Commands are velocity/yaw-rate setpoints;
    # ArduPilot owns the actual pitch/roll attitude control.
    "forward_gain": 0.45,
    "lateral_gain": 0.0010,
    "altitude_gain": 0.0010,
    "yaw_gain": 0.0025,
    "use_yaw_alignment": True,
    "max_forward_speed_mps": 0.45,
    "max_lateral_speed_mps": 0.25,
    "max_altitude_speed_mps": 0.20,
    "max_yaw_rate_radps": 0.35,
    "lock_hold_ms": 700,
    "align_timeout_s": 20.0,
}


def _load_calibration() -> dict:
    """Load bench calibration data from JSON file if available.

    Expected format:
        {
            "target_camera_range_m": 3.8,
            "range_tolerance_m": 0.25,
            "trigger_max_distance_m": 5.5,
            "aim_pixel_x": 640,
            "aim_pixel_y": 390,
            "servo_fire_angle_deg": 82.0,
            "spray_duration_ms": 600,
            "yaw_gain": 0.0025,
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
_spray_calibration = copy.deepcopy(DEFAULT_SPRAY_CALIBRATION)
_spray_calibration.update({
    k: v
    for k, v in _calibration.items()
    if k in DEFAULT_SPRAY_CALIBRATION
})
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

    Operator positions until the ZED sees the target, then the system
    autonomously approaches, aligns to a calibrated firing view, and sprays.
    """

    # Engagement parameters
    TRIGGER_MAX_DISTANCE_M = float(_spray_calibration["trigger_max_distance_m"])
    APPROACH_STOP_DISTANCE_M = 2.0
    APPROACH_SPEED_MPS = 0.5
    APPROACH_TIMEOUT_S = 20.0

    # CONOPS 5.2.4 / Q&A #10: full autonomy points require the
    # autonomous approach to start from more than 2 m. If the operator
    # triggers a "force autonomy" spray inside this radius, the AIM
    # state will run immediately and the autonomy criterion fails. We
    # gate triggers with require_autonomy=True at 2.5 m to leave
    # margin for ZED depth jitter.
    AUTONOMY_MIN_RANGE_M = 2.5

    # Aiming parameters
    AIM_TOLERANCE_PX = int(_spray_calibration["aim_tolerance_px"])

    # Spray parameters
    SPRAY_DURATION_MS = int(_spray_calibration["spray_duration_ms"])
    WATER_PUMP_RELAY_NUMBER = int(_spray_calibration["water_pump_relay_number"])
    SPRAY_SETTLE_TIME_S = 0.5
    MAX_SPRAY_ATTEMPTS = 2

    # Visual servoing parameters
    IMAGE_CENTER_X = 640
    IMAGE_CENTER_Y = 360
    SERVO_GAIN = 0.1

    TARGET_CAMERA_RANGE_M = float(_spray_calibration["target_camera_range_m"])
    RANGE_TOLERANCE_M = float(_spray_calibration["range_tolerance_m"])
    AIM_PIXEL_X = float(_spray_calibration["aim_pixel_x"])
    AIM_PIXEL_Y = float(_spray_calibration["aim_pixel_y"])
    SERVO_FIRE_ANGLE_DEG = float(_spray_calibration["servo_fire_angle_deg"])
    FORWARD_GAIN = float(_spray_calibration["forward_gain"])
    LATERAL_GAIN = float(_spray_calibration["lateral_gain"])
    ALTITUDE_GAIN = float(_spray_calibration["altitude_gain"])
    YAW_GAIN = float(_spray_calibration["yaw_gain"])
    USE_YAW_ALIGNMENT = bool(_spray_calibration["use_yaw_alignment"])
    MAX_FORWARD_SPEED_MPS = float(_spray_calibration["max_forward_speed_mps"])
    MAX_LATERAL_SPEED_MPS = float(_spray_calibration["max_lateral_speed_mps"])
    MAX_ALTITUDE_SPEED_MPS = float(_spray_calibration["max_altitude_speed_mps"])
    MAX_YAW_RATE_RADPS = float(_spray_calibration["max_yaw_rate_radps"])
    LOCK_HOLD_MS = int(_spray_calibration["lock_hold_ms"])
    ALIGN_TIMEOUT_S = float(_spray_calibration["align_timeout_s"])

    # Stereo parallax correction. Detection runs on the ZED left image, but
    # the spray nozzle is colocated with the ZED *right* camera. The target's
    # x in the right image differs from the left image by fx*baseline/Z, so we
    # subtract that shift from err_x to aim in the nozzle frame. Value is
    # fx_pub (px) * baseline (m) for the *published* stream — depends on
    # ZED_GRAB_RESOLUTION and ZED_PUB_DOWNSCALE_FACTOR. Source of truth is
    # NOMAD_STEREO_FX_BASELINE_PX_M in config/nomad.env; default 32 px·m
    # matches ZED2i HD720 + downscale 2.0 (fx≈267 px, baseline 0.12 m).
    STEREO_FX_BASELINE_PX_M = float(
        os.environ.get(
            "NOMAD_STEREO_FX_BASELINE_PX_M",
            _spray_calibration.get("stereo_fx_baseline_px_m", 32.0),
        )
    )

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

        # Obstacle avoidance sector exclusion callback
        self._set_excluded_sectors_fn: Optional[Callable[[set[int]], None]] = None

        # Tracks whether the current run was triggered with require_autonomy.
        # Approach/aim loops use this to decide if a non-GUIDED flight mode
        # should abort the sequence (operator override).
        self._active_require_autonomy: bool = False

        # Photo / verify / upload callbacks (set externally)
        self._capture_photo_fn: Optional[Callable[[], Optional[str]]] = None
        self._verify_hsv_fn: Optional[Callable[[str], bool]] = None
        self._verify_circle_change_fn: Optional[Callable[[str, str], bool]] = None
        self._upload_fn: Optional[Callable[[str, str], str]] = None
        self._get_detection_bbox_fn: Optional[Callable[[int], Optional[tuple]]] = None

        # Pre/post-spray image paths for circle change verification.
        # Reused by the artifact manager so we don't double-capture.
        self._pre_spray_image_path: Optional[str] = None
        self._post_spray_image_path: Optional[str] = None

        # Optional artifact-session callbacks so an external manager
        # (task2_spray_artifacts) can record before/after images and video
        # for the autonomous flow. The hooks receive the already-captured
        # snapshot path so the artifact manager doesn't re-capture.
        self._artifact_start_fn: Optional[Callable[[Optional[str]], None]] = None
        self._artifact_stop_fn: Optional[Callable[[Optional[str]], None]] = None

        logger.info("Spray controller initialized (ZED-guided spray + circle verify)")

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

    def set_artifact_callbacks(
        self,
        start_fn: Optional[Callable[[Optional[str]], None]],
        stop_fn:  Optional[Callable[[Optional[str]], None]],
    ) -> None:
        """Register artifact-session hooks for the autonomous flow.

        start_fn(before_path) is called once the pre-spray snapshot is on disk
        (or with None if capture failed). stop_fn(after_path) is called when
        the spray sequence ends, with the post-spray verification snapshot
        path (or None when no post-snapshot was taken — e.g. early failure).
        """
        self._artifact_start_fn = start_fn
        self._artifact_stop_fn  = stop_fn

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

    def set_excluded_sectors_fn(self, fn: Callable[[set[int]], None]) -> None:
        """Set callback to update obstacle avoidance excluded sectors."""
        self._set_excluded_sectors_fn = fn

    # ------------------------------------------------------------------ #
    # Runtime calibration
    # ------------------------------------------------------------------ #

    @classmethod
    def get_calibration(cls) -> dict:
        """Return the field-tunable spray calibration currently in use."""
        return {
            "target_camera_range_m": cls.TARGET_CAMERA_RANGE_M,
            "range_tolerance_m": cls.RANGE_TOLERANCE_M,
            "trigger_max_distance_m": cls.TRIGGER_MAX_DISTANCE_M,
            "aim_pixel_x": cls.AIM_PIXEL_X,
            "aim_pixel_y": cls.AIM_PIXEL_Y,
            "aim_tolerance_px": cls.AIM_TOLERANCE_PX,
            "servo_fire_angle_deg": cls.SERVO_FIRE_ANGLE_DEG,
            "spray_duration_ms": cls.SPRAY_DURATION_MS,
            "water_pump_relay_number": cls.WATER_PUMP_RELAY_NUMBER,
            "forward_gain": cls.FORWARD_GAIN,
            "lateral_gain": cls.LATERAL_GAIN,
            "altitude_gain": cls.ALTITUDE_GAIN,
            "yaw_gain": cls.YAW_GAIN,
            "use_yaw_alignment": cls.USE_YAW_ALIGNMENT,
            "max_forward_speed_mps": cls.MAX_FORWARD_SPEED_MPS,
            "max_lateral_speed_mps": cls.MAX_LATERAL_SPEED_MPS,
            "max_altitude_speed_mps": cls.MAX_ALTITUDE_SPEED_MPS,
            "max_yaw_rate_radps": cls.MAX_YAW_RATE_RADPS,
            "lock_hold_ms": cls.LOCK_HOLD_MS,
            "align_timeout_s": cls.ALIGN_TIMEOUT_S,
            "calibration_file": CALIBRATION_FILE,
        }

    @classmethod
    def update_calibration(cls, updates: dict, *, persist: bool = True) -> dict:
        """Apply Mission Planner field calibration values at runtime."""
        numeric_fields = {
            "target_camera_range_m": ("TARGET_CAMERA_RANGE_M", 0.5, 8.0),
            "range_tolerance_m": ("RANGE_TOLERANCE_M", 0.05, 1.0),
            "trigger_max_distance_m": ("TRIGGER_MAX_DISTANCE_M", 1.0, 8.0),
            "aim_pixel_x": ("AIM_PIXEL_X", 0.0, 4000.0),
            "aim_pixel_y": ("AIM_PIXEL_Y", 0.0, 3000.0),
            "aim_tolerance_px": ("AIM_TOLERANCE_PX", 2.0, 250.0),
            "servo_fire_angle_deg": ("SERVO_FIRE_ANGLE_DEG", 0.0, 180.0),
            "spray_duration_ms": ("SPRAY_DURATION_MS", 50.0, 5000.0),
            "water_pump_relay_number": ("WATER_PUMP_RELAY_NUMBER", 0.0, 15.0),
            "forward_gain": ("FORWARD_GAIN", 0.0, 2.0),
            "lateral_gain": ("LATERAL_GAIN", -0.02, 0.02),
            "altitude_gain": ("ALTITUDE_GAIN", -0.02, 0.02),
            "yaw_gain": ("YAW_GAIN", -0.02, 0.02),
            "max_forward_speed_mps": ("MAX_FORWARD_SPEED_MPS", 0.05, 2.0),
            "max_lateral_speed_mps": ("MAX_LATERAL_SPEED_MPS", 0.05, 1.0),
            "max_altitude_speed_mps": ("MAX_ALTITUDE_SPEED_MPS", 0.05, 1.0),
            "max_yaw_rate_radps": ("MAX_YAW_RATE_RADPS", 0.05, 2.0),
            "lock_hold_ms": ("LOCK_HOLD_MS", 100.0, 5000.0),
            "align_timeout_s": ("ALIGN_TIMEOUT_S", 2.0, 60.0),
        }

        for key, (attr, min_v, max_v) in numeric_fields.items():
            if key not in updates:
                continue
            try:
                value = float(updates[key])
            except (TypeError, ValueError):
                continue
            value = max(min_v, min(max_v, value))
            if attr in ("AIM_TOLERANCE_PX", "SPRAY_DURATION_MS", "LOCK_HOLD_MS", "WATER_PUMP_RELAY_NUMBER"):
                setattr(cls, attr, int(round(value)))
            else:
                setattr(cls, attr, value)

        if "use_yaw_alignment" in updates:
            cls.USE_YAW_ALIGNMENT = bool(updates["use_yaw_alignment"])

        current = cls.get_calibration()
        if persist:
            try:
                os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
                persisted = {
                    k: v
                    for k, v in current.items()
                    if k != "calibration_file"
                }
                with open(CALIBRATION_FILE, "w") as f:
                    json.dump(persisted, f, indent=2)
                os.chmod(CALIBRATION_FILE, 0o600)
            except Exception as e:
                logger.warning(f"Failed to persist spray calibration: {e}")
        logger.info(f"Updated spray calibration: {current}")
        return current

    # ------------------------------------------------------------------ #
    # Trigger / Abort
    # ------------------------------------------------------------------ #

    def trigger(self, target: SprayTarget, *, require_autonomy: bool = False) -> dict:
        """Trigger spray sequence on target.

        Args:
            target: SprayTarget to engage.
            require_autonomy: When True, refuse the trigger if the
                drone is already inside ``AUTONOMY_MIN_RANGE_M``. This
                protects the CONOPS Q&A #10 autonomy gate (autonomous
                approach must start from beyond 2 m). The Mission
                Planner "Auto Spray (autonomy gate)" button sets this
                True; manual sprays leave it False.
        """
        with self._lock:
            if self.is_active:
                return {"success": False, "error": "Spray sequence already active"}

            drone_pos = self._get_drone_position()
            if drone_pos is None and not target.image_only:
                return {"success": False, "error": "Cannot determine drone position"}

            if target.image_only:
                range_m = None
                try:
                    if target.range_m is not None:
                        candidate_range = float(target.range_m)
                        if math.isfinite(candidate_range):
                            range_m = candidate_range
                except (TypeError, ValueError):
                    range_m = None
                if range_m is None:
                    return {
                        "success": False,
                        "error": (
                            "Target has no valid depth/range estimate. Wait for "
                            "ZED depth on the detection or move to a view where "
                            "the circle center has valid depth before triggering spray."
                        ),
                    }
                distance = range_m if range_m is not None else self.TARGET_CAMERA_RANGE_M
            else:
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

            # CONOPS Q&A #10 autonomy gate: the autonomous approach must
            # cross the 2 m envelope, otherwise the auto-extinguish 20 pts
            # are forfeited. Refuse the trigger when the operator asked
            # for an autonomy-claim run but the drone is already too
            # close to demonstrate the approach.
            if require_autonomy and distance < self.AUTONOMY_MIN_RANGE_M:
                return {
                    "success": False,
                    "error": (
                        f"Too close for Auto Spray: {distance:.2f}m "
                        f"(need >= {self.AUTONOMY_MIN_RANGE_M:.1f}m). "
                        "Back up or use Manual Spray."
                    ),
                    "distance": round(distance, 2),
                    "min_required_m": self.AUTONOMY_MIN_RANGE_M,
                }

            # Image-only with a known range still warrants a real approach
            # whenever we're outside firing standoff. The Task 2 shape
            # detector now provides per-circle depth, so we can servo
            # forward instead of assuming we're already in range.
            if target.image_only:
                skip_approach = (
                    (target.range_m is None)
                    or (distance < self.APPROACH_STOP_DISTANCE_M)
                )
            else:
                skip_approach = distance < self.APPROACH_STOP_DISTANCE_M
            self._current_target = target
            self._spray_count = 0
            self._abort_event.clear()
            self._pre_spray_image_path = None
            self._post_spray_image_path = None

            self._status.state = (
                SprayState.AIM.value if skip_approach
                else SprayState.APPROACH.value
            )
            self._status.target_id = target.target_id
            self._status.target_number = self._status.targets_engaged + 1
            self._status.target_label = target.label
            self._status.distance_to_target = distance
            self._status.spray_count = 0
            self._status.verification_passed = False
            self._status.error = None
            self._status.approach_method = ""
            self._status.autonomy_action = "Queued"
            self._status.command_vx_mps = 0.0
            self._status.command_vy_mps = 0.0
            self._status.command_vz_mps = 0.0
            self._status.command_yaw_rate_radps = 0.0
            self._status.aim_error_x_px = None
            self._status.aim_error_y_px = None
            self._status.range_error_m = None
            self._status.require_autonomy = bool(require_autonomy)
            self._status.autonomy_compromised = False
            self._status.targets_engaged += 1

        # When the operator pressed Auto Spray (autonomy gate), force the
        # autopilot into GUIDED so the velocity commands the controller is
        # about to send actually take effect. The pilot's RC mode switch
        # remains the safety override - see _ensure_guided_or_abort.
        if require_autonomy and self._nav is not None:
            try:
                if not self._nav.enable_guided_mode():
                    logger.warning("Auto Spray: failed to send GUIDED mode change")
                # Don't block forever; the in-loop check will catch a missed transition.
                self._nav.wait_for_guided(timeout_s=2.0)
            except Exception as e:
                logger.warning(f"Auto Spray: GUIDED transition error: {e}")

        # Run sequence in background thread (outside lock)
        self._thread = threading.Thread(
            target=self._run_sequence,
            args=(skip_approach, bool(require_autonomy)),
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
            self._status.autonomy_action = "Aborted"
            self._status.command_vx_mps = 0.0
            self._status.command_vy_mps = 0.0
            self._status.command_vz_mps = 0.0
            self._status.command_yaw_rate_radps = 0.0
        logger.info("Spray sequence aborted")
        return {"success": True, "message": "Spray sequence aborted"}

    # ------------------------------------------------------------------ #
    # Sequence runner
    # ------------------------------------------------------------------ #

    def _run_sequence(self, skip_approach: bool = False, require_autonomy: bool = False) -> None:
        """Run autonomous spray sequence."""
        try:
            target = self._current_target
            if target is None:
                return

            # Stash the autonomy flag on the controller so the approach/aim
            # loops can read it via _ensure_guided_or_abort without changing
            # every signature.
            self._active_require_autonomy = require_autonomy

            # --- APPROACH (visible target -> calibrated firing range) ---
            if not skip_approach:
                self._set_state(SprayState.APPROACH)
                self._set_autonomy_command("Approaching target")
                if not self._approach_target(target):
                    if not self._check_abort():
                        self._set_state(SprayState.FAILED, error="Approach failed")
                    return

            # --- AIM ---
            self._set_state(SprayState.AIM)
            self._set_autonomy_command("Aiming at target")
            if not self._aim_at_target(target):
                if not self._check_abort():
                    self._set_state(SprayState.FAILED, error="Aim failed")
                return

            # --- CAPTURE PRE-SPRAY SNAPSHOT ---
            self._set_autonomy_command("Capturing pre-spray image")
            self._capture_pre_spray(target)

            # --- SPRAY (with retry) ---
            for attempt in range(self.MAX_SPRAY_ATTEMPTS):
                self._spray_count = attempt + 1
                with self._lock:
                    self._status.spray_count = self._spray_count

                self._set_state(SprayState.SPRAY)
                self._set_autonomy_command("Spraying")
                if not self._spray_target():
                    with self._lock:
                        self._status.targets_failed += 1
                    self._set_state(
                        SprayState.FAILED,
                        error="Water shooter trigger failed",
                    )
                    return

                # --- VERIFY ---
                self._set_state(SprayState.VERIFY)
                self._set_autonomy_command("Verifying color change")
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
            self._set_autonomy_command("Uploading proof image")
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
        finally:
            self._active_require_autonomy = False

    def _set_state(self, state: SprayState, error: Optional[str] = None) -> None:
        with self._lock:
            self._status.state = state.value
            if error:
                self._status.error = error
            if state in (SprayState.COMPLETE, SprayState.FAILED, SprayState.ABORTED):
                self._status.command_vx_mps = 0.0
                self._status.command_vy_mps = 0.0
                self._status.command_vz_mps = 0.0
                self._status.command_yaw_rate_radps = 0.0
                self._status.aim_error_x_px = None
                self._status.aim_error_y_px = None
                self._status.range_error_m = None
                if state == SprayState.COMPLETE:
                    self._status.autonomy_action = "Complete"
                elif state == SprayState.FAILED:
                    self._status.autonomy_action = "Failed"
                else:
                    self._status.autonomy_action = "Aborted"
        # When the autonomous sequence ends, finalise any open artifact
        # session so the Submit panel can pick it up via /last_artifacts.
        if state in (SprayState.COMPLETE, SprayState.FAILED, SprayState.ABORTED):
            if self._artifact_stop_fn is not None:
                try:
                    self._artifact_stop_fn(self._post_spray_image_path)
                except Exception as e:
                    logger.warning(f"Artifact stop hook failed: {e}")

    def _check_abort(self) -> bool:
        return self._abort_event.is_set()

    def _set_autonomy_command(
        self,
        action: str,
        *,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
        aim_error_x: Optional[float] = None,
        aim_error_y: Optional[float] = None,
        range_error: Optional[float] = None,
    ) -> None:
        """Publish the current autonomous intent for the GCS video overlay."""
        with self._lock:
            self._status.autonomy_action = action
            self._status.command_vx_mps = float(vx)
            self._status.command_vy_mps = float(vy)
            self._status.command_vz_mps = float(vz)
            self._status.command_yaw_rate_radps = float(yaw_rate)
            self._status.aim_error_x_px = aim_error_x
            self._status.aim_error_y_px = aim_error_y
            self._status.range_error_m = range_error

    def _flight_mode(self) -> str:
        """Current ArduPilot flight mode string, or empty when unknown."""
        if not self._state:
            return ""
        try:
            return self._state.get_state().flight_mode or ""
        except Exception:
            return ""

    def _ensure_guided_or_abort(self, *, require_autonomy: bool) -> bool:
        """Verify we are still in GUIDED. Pilot RC override is detected here.

        If the operator flips the RC mode switch (LOITER/STABILIZE/ALT_HOLD)
        the autopilot stops accepting our velocity commands. We DON'T fight
        the pilot - we mark the autonomy run compromised, stop sending
        commands, and let the sequence unwind. This is the safety override
        path the user asked for: the pilot retains ultimate authority via
        the mode switch on the transmitter.

        Only meaningful while ``require_autonomy=True``. Manual sprays
        already accept whatever mode the pilot is in.
        """
        if not require_autonomy:
            return True
        mode = self._flight_mode()
        # While we're commanding velocity we expect GUIDED. Empty/UNKNOWN
        # means the heartbeat hasn't been seen yet - treat as transient.
        if mode and mode != "GUIDED":
            msg = (
                f"Autonomy aborted: pilot switched to {mode}. "
                "The 20-pt claim is forfeit for this target; pick another."
            )
            logger.warning(
                "Spray autonomy: flight mode is %s (not GUIDED) - operator override detected, aborting",
                mode,
            )
            with self._lock:
                self._status.autonomy_compromised = True
            self._abort_event.set()
            if self._nav:
                try:
                    self._nav.stop_movement()
                except Exception:
                    pass
            self._set_state(SprayState.ABORTED, error=msg)
            return False
        return True

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

    def _aim_err_x(self, cx: float, range_m: Optional[float]) -> float:
        """Aim error in the right-camera (nozzle) image frame.

        cx and AIM_PIXEL_X are in left-image pixels. Right-image x = left - fx*baseline/Z.
        AIM_PIXEL_X was calibrated at TARGET_CAMERA_RANGE_M, so its baked-in
        parallax is fx*baseline/TARGET_CAMERA_RANGE_M.
        """
        raw = cx - self.AIM_PIXEL_X
        if range_m is None or range_m < 0.3:
            return raw
        parallax_dx = self.STEREO_FX_BASELINE_PX_M * (
            1.0 / float(range_m) - 1.0 / self.TARGET_CAMERA_RANGE_M
        )
        return raw - parallax_dx

    def _get_drone_yaw_rad(self) -> float:
        """Current drone yaw (radians, NED: 0=North, CW positive). Falls back to heading_deg, then 0."""
        if not self._state:
            return 0.0
        state = self._state.get_state()
        yaw = getattr(state, 'vio_yaw', None)
        if yaw is not None:
            return float(yaw)
        heading_deg = getattr(state, 'heading_deg', None)
        if heading_deg is not None:
            return math.radians(float(heading_deg))
        return 0.0

    # ------------------------------------------------------------------ #
    # APPROACH (direct velocity)
    # ------------------------------------------------------------------ #

    def _approach_target(self, target: SprayTarget) -> bool:
        """Autonomous approach from trigger range to the coarse firing standoff.

        Nav2 has been removed (it was a 2D ground-robot planner that crashed
        on vertical targets). The spray flow now uses pure visual servoing:
        image-space approach when we have a bbox + depth, otherwise direct
        velocity in the drone's world frame.
        """
        self._set_approach_sectors(target, exclude=True)
        try:
            if self._get_detection_bbox_fn is not None:
                return self._approach_via_image(target)
            return self._approach_via_velocity(target)
        finally:
            self._set_approach_sectors(target, exclude=False)

    def _approach_via_image(self, target: SprayTarget) -> bool:
        """Image-space approach for image_only targets with known range.

        Drives forward (and lightly nudges yaw/lateral/altitude to keep the
        circle on the calibrated aim pixel) until the visual servo reports
        range < APPROACH_STOP_DISTANCE_M. Falls through to AIM on timeout so
        the operator isn't stranded mid-flight.
        """
        if not self._nav:
            logger.error("NavController not available for image approach")
            self._set_state(SprayState.FAILED, error="NavController unavailable")
            return False
        if not self._get_detection_bbox_fn:
            logger.warning("No detection bbox function — skipping image approach")
            return True

        with self._lock:
            self._status.approach_method = "image"

        # Camera tilt to firing pose so the visual range_m is comparable to
        # the calibrated TARGET_CAMERA_RANGE_M during aim.
        if self._servo:
            fire_angle = max(0.0, min(180.0, self.SERVO_FIRE_ANGLE_DEG))
            self._servo.set_camera_tilt(fire_angle)
            with self._lock:
                self._status.servo_angle = fire_angle
            self._set_autonomy_command(f"Servo to firing angle {fire_angle:.0f} deg")

        approach_start = time.time()
        last_command_time = 0.0
        no_detection_streak = 0

        while not self._check_abort():
            # Pilot RC override safety check - bail out if we left GUIDED.
            if not self._ensure_guided_or_abort(require_autonomy=self._active_require_autonomy):
                return False

            now = time.time()
            if now - approach_start > self.APPROACH_TIMEOUT_S:
                logger.warning("Image approach timeout — proceeding to aim")
                if self._nav:
                    self._nav.stop_movement()
                self._set_autonomy_command("Approach timeout - holding")
                self._set_approach_sectors(target, exclude=False)
                return True

            detection = self._get_detection_for_aim(target)
            if detection is None:
                no_detection_streak += 1
                if self._nav and now - last_command_time > 0.25:
                    self._nav.stop_movement()
                    self._set_autonomy_command("Holding - target lost")
                    last_command_time = now
                if no_detection_streak > 30:  # ~3s without a detection
                    logger.warning("Image approach lost target — bailing to aim")
                    self._set_approach_sectors(target, exclude=False)
                    return True
                time.sleep(0.1)
                continue
            no_detection_streak = 0

            cx = float(detection["cx"])
            cy = float(detection["cy"])
            camera_range_m = detection.get("range_m")
            if camera_range_m is None:
                # No depth on this frame — hold and wait for one.
                if self._nav:
                    self._nav.stop_movement()
                self._set_autonomy_command("Holding - waiting for depth")
                time.sleep(0.1)
                continue

            range_m = float(camera_range_m)
            with self._lock:
                self._status.distance_to_target = range_m

            if range_m <= self.APPROACH_STOP_DISTANCE_M:
                logger.info(f"Image approach complete at {range_m:.2f}m")
                if self._nav:
                    self._nav.stop_movement()
                self._set_autonomy_command("Approach complete - holding")
                self._set_approach_sectors(target, exclude=False)
                return True

            # Forward velocity scales with how far we still have to go,
            # capped by the configured approach speed. Same lateral/yaw/
            # altitude correction shape as _aim_at_target so the circle
            # stays visible while we close in.
            err_x = self._aim_err_x(cx, range_m)
            err_y = cy - self.AIM_PIXEL_Y
            forward_excess = range_m - self.APPROACH_STOP_DISTANCE_M
            vx = self._clamp(forward_excess * 0.5, 0.0, self.APPROACH_SPEED_MPS)
            vy = 0.0
            yaw_rate = 0.0
            # NavController expects ROS convention: +vy=left, +vz=up, +yaw=CCW.
            # Target right of aim (err_x>0) → yaw right (CW, negative) or strafe
            # right (negative vy). Target low in frame (err_y>0) → descend
            # (negative vz). Gains remain positive in calibration.
            if self.USE_YAW_ALIGNMENT:
                yaw_rate = self._clamp(
                    -err_x * self.YAW_GAIN,
                    -self.MAX_YAW_RATE_RADPS,
                    self.MAX_YAW_RATE_RADPS,
                )
            else:
                vy = self._clamp(
                    -err_x * self.LATERAL_GAIN,
                    -self.MAX_LATERAL_SPEED_MPS,
                    self.MAX_LATERAL_SPEED_MPS,
                )
            vz = self._clamp(
                -err_y * self.ALTITUDE_GAIN,
                -self.MAX_ALTITUDE_SPEED_MPS,
                self.MAX_ALTITUDE_SPEED_MPS,
            )
            self._nav.send_velocity(vx, vy, vz, yaw_rate)
            self._set_autonomy_command(
                "Approaching target",
                vx=vx,
                vy=vy,
                vz=vz,
                yaw_rate=yaw_rate,
                aim_error_x=err_x,
                aim_error_y=err_y,
                range_error=forward_excess,
            )
            last_command_time = now
            time.sleep(0.1)

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
            if not self._ensure_guided_or_abort(require_autonomy=self._active_require_autonomy):
                return False

            if time.time() - approach_start > self.APPROACH_TIMEOUT_S:
                logger.warning("Velocity approach timeout - continuing")
                self._set_autonomy_command("Approach timeout - holding")
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
                self._set_autonomy_command("Approach complete - holding")
                self._set_approach_sectors(target, exclude=False)
                return True

            norm = max(distance, 0.1)
            speed = min(self.APPROACH_SPEED_MPS, distance * 0.5)
            # World NED delta (dx=North, dy=East, dz=Down) -> body FLU velocity
            # (vx=Forward, vy=Left, vz=Up) for NavController.send_velocity.
            yaw = self._get_drone_yaw_rad()
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            fwd = cos_y * dx + sin_y * dy
            left = sin_y * dx - cos_y * dy
            up = -dz
            vx = (fwd / norm) * speed
            vy = (left / norm) * speed
            vz = (up / norm) * speed * 0.3
            self._nav.send_velocity(vx, vy, vz, 0)
            self._set_autonomy_command(
                "Approaching target",
                vx=vx,
                vy=vy,
                vz=vz,
                range_error=max(distance - self.APPROACH_STOP_DISTANCE_M, 0.0),
            )
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
        # Bearing to target in world NED (0=North, CW positive). Convert to
        # body-frame bearing (0=drone forward) since obstacle_distance_bridge
        # publishes sectors in MAV_FRAME_BODY_FRD.
        world_bearing_rad = math.atan2(dy, dx)
        body_bearing_rad = world_bearing_rad - self._get_drone_yaw_rad()
        angle_deg = math.degrees(body_bearing_rad) % 360
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
        """Visual-servo to the calibrated water landing pixel and range.

        The camera may sit several meters behind the nozzle because of the
        spray arm. We therefore do not chase image center; we reproduce a
        calibrated firing geometry: target at AIM_PIXEL_X/Y and
        TARGET_CAMERA_RANGE_M. The controller sends body velocity/yaw commands
        and lets ArduPilot own attitude stabilization.
        """
        if not self._get_detection_bbox_fn:
            logger.warning("No detection bbox function - skipping aim")
            return True

        if self._servo:
            fire_angle = max(0.0, min(180.0, self.SERVO_FIRE_ANGLE_DEG))
            self._servo.set_camera_tilt(fire_angle)
            with self._lock:
                self._status.servo_angle = fire_angle

        aim_start = time.time()
        lock_started_at: Optional[float] = None
        last_command_time = 0.0

        while not self._check_abort():
            if not self._ensure_guided_or_abort(require_autonomy=self._active_require_autonomy):
                return False

            now = time.time()
            if now - aim_start > self.ALIGN_TIMEOUT_S:
                logger.warning("Aim timeout - proceeding with current alignment")
                if self._nav:
                    self._nav.stop_movement()
                self._set_autonomy_command("Aim timeout - holding")
                return True

            detection = self._get_detection_for_aim(target)
            if detection is None:
                lock_started_at = None
                if self._nav and now - last_command_time > 0.25:
                    self._nav.stop_movement()
                    self._set_autonomy_command("Holding - target not visible")
                    last_command_time = now
                time.sleep(0.1)
                continue

            cx = float(detection["cx"])
            cy = float(detection["cy"])
            camera_range_m = detection.get("range_m")
            err_x = self._aim_err_x(cx, camera_range_m)
            err_y = cy - self.AIM_PIXEL_Y
            range_error = 0.0
            if camera_range_m is not None:
                range_error = float(camera_range_m) - self.TARGET_CAMERA_RANGE_M

            pixel_locked = (
                abs(err_x) < self.AIM_TOLERANCE_PX
                and abs(err_y) < self.AIM_TOLERANCE_PX
            )
            range_locked = (
                camera_range_m is None
                or abs(range_error) < self.RANGE_TOLERANCE_M
            )

            if pixel_locked and range_locked:
                if lock_started_at is None:
                    lock_started_at = now
                    if self._nav:
                        self._nav.stop_movement()
                    self._set_autonomy_command(
                        "Aim locked - holding",
                        aim_error_x=err_x,
                        aim_error_y=err_y,
                        range_error=range_error,
                    )
                elif (now - lock_started_at) * 1000.0 >= self.LOCK_HOLD_MS:
                    if self._nav:
                        self._nav.stop_movement()
                    self._set_autonomy_command(
                        "Aim locked",
                        aim_error_x=err_x,
                        aim_error_y=err_y,
                        range_error=range_error,
                    )
                    logger.info(
                        "Spray aim lock acquired: "
                        f"pixel=({cx:.0f},{cy:.0f}) err=({err_x:.0f},{err_y:.0f}) "
                        f"range={camera_range_m if camera_range_m is not None else 'n/a'}"
                    )
                    return True
                time.sleep(0.05)
                continue

            lock_started_at = None

            if self._nav:
                # Positive range_error means target is too far, move forward.
                vx = self._clamp(
                    range_error * self.FORWARD_GAIN,
                    -self.MAX_FORWARD_SPEED_MPS,
                    self.MAX_FORWARD_SPEED_MPS,
                )
                # NavController expects ROS convention (+vy=left, +vz=up,
                # +yaw=CCW). Positive err_x means target appears right of aim,
                # which requires a right-going correction → negative vy or
                # negative (CW) yaw_rate. Positive err_y means target appears
                # low in frame, which requires descending → negative vz.
                vy = 0.0
                yaw_rate = 0.0
                if self.USE_YAW_ALIGNMENT:
                    yaw_rate = self._clamp(
                        -err_x * self.YAW_GAIN,
                        -self.MAX_YAW_RATE_RADPS,
                        self.MAX_YAW_RATE_RADPS,
                    )
                else:
                    vy = self._clamp(
                        -err_x * self.LATERAL_GAIN,
                        -self.MAX_LATERAL_SPEED_MPS,
                        self.MAX_LATERAL_SPEED_MPS,
                    )
                vz = self._clamp(
                    -err_y * self.ALTITUDE_GAIN,
                    -self.MAX_ALTITUDE_SPEED_MPS,
                    self.MAX_ALTITUDE_SPEED_MPS,
                )

                self._nav.send_velocity(vx, vy, vz, yaw_rate)
                self._set_autonomy_command(
                    "Aiming at target",
                    vx=vx,
                    vy=vy,
                    vz=vz,
                    yaw_rate=yaw_rate,
                    aim_error_x=err_x,
                    aim_error_y=err_y,
                    range_error=range_error,
                )
                last_command_time = now

            time.sleep(0.1)

        return False

    def _get_detection_for_aim(self, target: SprayTarget) -> Optional[dict]:
        """Normalize the detection callback output for the aim controller."""
        try:
            raw = self._get_detection_bbox_fn(target)  # type: ignore[misc]
        except TypeError:
            raw = self._get_detection_bbox_fn(target.target_id)  # type: ignore[misc]

        if raw is None:
            return None

        if isinstance(raw, dict):
            cx = raw.get("cx")
            cy = raw.get("cy")
            if cx is None or cy is None:
                bbox_x = raw.get("bbox_x", raw.get("x"))
                bbox_y = raw.get("bbox_y", raw.get("y"))
                bbox_w = raw.get("bbox_w", raw.get("w", 0))
                bbox_h = raw.get("bbox_h", raw.get("h", 0))
                if bbox_x is not None and bbox_y is not None:
                    cx = float(bbox_x) + float(bbox_w or 0) / 2.0
                    cy = float(bbox_y) + float(bbox_h or 0) / 2.0
            if cx is None or cy is None:
                return None
            range_m = raw.get("range_m")
            if range_m is None:
                coords = [
                    raw.get("det_x", raw.get("x_m", raw.get("x"))),
                    raw.get("det_y", raw.get("y_m", raw.get("y"))),
                    raw.get("det_z", raw.get("z_m", raw.get("z"))),
                ]
                try:
                    if all(v is not None for v in coords):
                        range_m = math.sqrt(sum(float(v) ** 2 for v in coords))
                except (TypeError, ValueError):
                    range_m = None
            return {"cx": float(cx), "cy": float(cy), "range_m": range_m}

        if isinstance(raw, tuple) and len(raw) >= 2:
            cx, cy = raw[0], raw[1]
            range_m = raw[4] if len(raw) >= 5 else None
            return {"cx": float(cx), "cy": float(cy), "range_m": range_m}

        return None

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    # ------------------------------------------------------------------ #
    # CAPTURE PRE-SPRAY
    # ------------------------------------------------------------------ #

    def _capture_pre_spray(self, target: SprayTarget) -> None:
        """Capture a pre-spray image for circle change verification.

        Called before the first spray attempt. Only captures once per sequence.
        """
        if self._pre_spray_image_path is not None:
            return  # Already captured

        captured_path: Optional[str] = None
        try:
            if self._capture_photo_fn:
                captured_path = self._capture_photo_fn()
                if captured_path:
                    self._pre_spray_image_path = captured_path
                    logger.info(f"Pre-spray image captured: {captured_path}")
                else:
                    logger.warning("Pre-spray capture returned no path")
            else:
                logger.warning("No capture_photo_fn for pre-spray snapshot")
        except Exception as e:
            logger.error(f"Pre-spray capture error: {e}")

        # Hand the same path to the artifact manager so it doesn't re-capture.
        if self._artifact_start_fn is not None:
            try:
                self._artifact_start_fn(captured_path)
            except Exception as e:
                logger.warning(f"Artifact start hook failed: {e}")

    # ------------------------------------------------------------------ #
    # SPRAY
    # ------------------------------------------------------------------ #

    def _spray_target(self) -> bool:
        """Activate water pump."""
        if not self._servo:
            logger.error("ServoController not available for spray")
            return False

        logger.info(f"Spraying target (attempt {self._spray_count})")
        try:
            if not self._servo.configure_water_pump_relay(
                int(self.WATER_PUMP_RELAY_NUMBER)
            ):
                logger.warning("Failed to configure water pump relay")
                return False
        except Exception as e:
            logger.warning(f"Failed to configure water pump relay: {e}")
            return False
        success = self._servo.trigger_water_shooter(
            duration_ms=self.SPRAY_DURATION_MS
        )
        if not success:
            logger.warning("Water shooter trigger returned failure")
            return False
        return True

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
                    self._post_spray_image_path = post_path
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
                    self._post_spray_image_path = photo_path
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

        Filename: target_<n>.jpg, with a 1-based target number for the
        competition Drive folder.
        """
        try:
            photo_path = None
            if self._capture_photo_fn:
                photo_path = self._capture_photo_fn()
            if not photo_path:
                logger.warning("Could not capture photo for upload")
                return

            with self._lock:
                target_number = self._status.target_number
            target_number = max(1, int(target_number or 1))
            filename = f"target_{target_number}.jpg"
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
