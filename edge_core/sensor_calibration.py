"""
NOMAD Edge Core - ZED 2i Sensor Calibration

Provides IMU and magnetometer calibration for the ZED 2i camera.

Calibration Types:
    1. Magnetometer: Hard-iron and soft-iron calibration via data collection
       during rotation. Essential for accurate heading.
    2. IMU (Accelerometer): Gravity alignment check and bias estimation.
    3. IMU (Gyroscope): Zero-rate offset calibration (stationary).

Usage:
    # Via API (triggered from Mission Planner):
    POST /api/calibration/magnetometer/start   -> begin mag cal
    GET  /api/calibration/magnetometer/status   -> check progress
    POST /api/calibration/magnetometer/stop     -> finish and compute
    GET  /api/calibration/imu/check             -> quick IMU health check

    # Standalone on Jetson:
    python3 -m edge_core.sensor_calibration --type mag
    python3 -m edge_core.sensor_calibration --type imu

Target: Python 3.x | ZED SDK 4.x | Jetson Orin Nano
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger("edge_core.sensor_calibration")

# Calibration data directory
CALIBRATION_DIR = Path(__file__).parent.parent / "config" / "calibration"


class CalibrationState(str, Enum):
    """Calibration process state."""

    IDLE = "idle"
    COLLECTING = "collecting"
    COMPUTING = "computing"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass
class MagCalibrationResult:
    """Result of magnetometer calibration."""

    hard_iron: Tuple[float, float, float]  # Offset (x, y, z) in uT
    soft_iron: List[List[float]]  # 3x3 correction matrix
    fitness: float  # Fit quality (0-1, higher is better)
    samples_used: int
    timestamp: str
    duration_s: float

    def to_dict(self) -> dict:
        return {
            "hard_iron": list(self.hard_iron),
            "soft_iron": self.soft_iron,
            "fitness": self.fitness,
            "samples_used": self.samples_used,
            "timestamp": self.timestamp,
            "duration_s": self.duration_s,
        }


@dataclass
class IMUCheckResult:
    """Result of IMU health check."""

    accel_bias: Tuple[float, float, float]  # m/s^2
    gyro_bias: Tuple[float, float, float]  # rad/s
    accel_noise: float  # Standard deviation m/s^2
    gyro_noise: float  # Standard deviation rad/s
    gravity_magnitude: float  # Should be ~9.81 m/s^2
    gravity_error_pct: float  # % deviation from 9.81
    temperature: float  # Celsius
    healthy: bool
    issues: List[str]
    timestamp: str

    def to_dict(self) -> dict:
        return {
            "accel_bias": list(self.accel_bias),
            "gyro_bias": list(self.gyro_bias),
            "accel_noise": self.accel_noise,
            "gyro_noise": self.gyro_noise,
            "gravity_magnitude": self.gravity_magnitude,
            "gravity_error_pct": self.gravity_error_pct,
            "temperature": self.temperature,
            "healthy": self.healthy,
            "issues": self.issues,
            "timestamp": self.timestamp,
        }


class MagCalibrationSession:
    """
    Magnetometer calibration session.

    Collects raw magnetometer readings while the user rotates the camera
    in all orientations, then computes hard-iron offset and soft-iron
    correction matrix using ellipsoid fitting.

    Hard-iron: constant offset from nearby magnetic material.
    Soft-iron: scaling/axis distortion from nearby magnetic material.
    """

    def __init__(self, min_samples: int = 300, target_samples: int = 500):
        self._state = CalibrationState.IDLE
        self._min_samples = min_samples
        self._target_samples = target_samples
        self._lock = threading.RLock()

        # Raw data collection
        self._mag_samples: List[Tuple[float, float, float]] = []
        self._start_time: Optional[float] = None

        # Result
        self._result: Optional[MagCalibrationResult] = None
        self._error: Optional[str] = None

        # ZED camera handle (set externally)
        self._zed = None
        self._collection_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    @property
    def state(self) -> CalibrationState:
        return self._state

    @property
    def sample_count(self) -> int:
        return len(self._mag_samples)

    @property
    def progress(self) -> float:
        """Progress 0.0 to 1.0."""
        if self._target_samples == 0:
            return 0.0
        return min(1.0, len(self._mag_samples) / self._target_samples)

    @property
    def result(self) -> Optional[MagCalibrationResult]:
        return self._result

    @property
    def error(self) -> Optional[str]:
        return self._error

    def start(self, zed_camera=None) -> bool:
        """
        Start magnetometer data collection.

        Args:
            zed_camera: pyzed.sl.Camera instance (if None, opens a new one)
        """
        with self._lock:
            if self._state == CalibrationState.COLLECTING:
                return True  # Already collecting

            self._state = CalibrationState.COLLECTING
            self._mag_samples.clear()
            self._result = None
            self._error = None
            self._start_time = time.time()
            self._stop_event.clear()

            if zed_camera:
                self._zed = zed_camera
                self._owns_camera = False
            else:
                self._owns_camera = True
                if not self._open_camera():
                    self._state = CalibrationState.FAILED
                    self._error = "Failed to open ZED camera"
                    return False

            # Start collection thread
            self._collection_thread = threading.Thread(
                target=self._collect_loop, daemon=True
            )
            self._collection_thread.start()
            logger.info(
                "Magnetometer calibration started - rotate camera in all orientations"
            )
            return True

    def stop(self) -> Optional[MagCalibrationResult]:
        """
        Stop collection and compute calibration.

        Returns:
            MagCalibrationResult if successful, None on failure
        """
        self._stop_event.set()
        if self._collection_thread:
            self._collection_thread.join(timeout=5.0)

        if self._owns_camera and self._zed:
            try:
                self._zed.close()
            except Exception:
                pass
            self._zed = None

        if len(self._mag_samples) < self._min_samples:
            self._state = CalibrationState.FAILED
            self._error = (
                f"Not enough samples: {len(self._mag_samples)}/{self._min_samples}. "
                "Rotate the camera more during collection."
            )
            logger.error(self._error)
            return None

        # Compute calibration
        self._state = CalibrationState.COMPUTING
        result = self._compute_calibration()

        if result:
            self._result = result
            self._state = CalibrationState.COMPLETE
            self._save_calibration(result)
            logger.info(
                f"Magnetometer calibration complete: "
                f"hard_iron={result.hard_iron}, fitness={result.fitness:.3f}"
            )

            # Attempt to store calibration in camera EEPROM so the SDK
            # applies it automatically to get_magnetic_field_calibrated()
            if self._zed:
                try:
                    import pyzed.sl as sl
                    if not hasattr(self._zed, "store_calibration"):
                        logger.warning(
                            "Camera SDK does not expose store_calibration(); EEPROM write unsupported"
                        )
                    else:
                        store_status = self._zed.store_calibration()
                        if store_status == sl.ERROR_CODE.SUCCESS:
                            logger.info("Calibration parameters stored to camera EEPROM")
                        else:
                            logger.warning(f"Failed to store calibration to EEPROM: {store_status}")
                except Exception as e:
                    logger.warning(f"EEPROM store error: {e}")
        else:
            self._state = CalibrationState.FAILED

        return result

    def cancel(self) -> None:
        """Cancel ongoing calibration."""
        self._stop_event.set()
        if self._collection_thread:
            self._collection_thread.join(timeout=3.0)
        if self._owns_camera and self._zed:
            try:
                self._zed.close()
            except Exception:
                pass
            self._zed = None
        self._state = CalibrationState.IDLE
        self._mag_samples.clear()

    def get_status(self) -> dict:
        """Get current calibration status."""
        status = {
            "state": self._state.value,
            "samples": len(self._mag_samples),
            "target_samples": self._target_samples,
            "min_samples": self._min_samples,
            "progress": self.progress,
        }
        if self._start_time:
            status["elapsed_s"] = round(time.time() - self._start_time, 1)
        if self._error:
            status["error"] = self._error
        if self._result:
            status["result"] = self._result.to_dict()
        # Spatial coverage feedback
        if self._mag_samples:
            status["coverage"] = self._compute_coverage()
        return status

    def _open_camera(self) -> bool:
        """Open ZED camera for sensor-only access."""
        try:
            import pyzed.sl as sl

            self._zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.depth_mode = sl.DEPTH_MODE.NONE  # Sensor only
            init_params.camera_resolution = sl.RESOLUTION.VGA  # Minimal
            init_params.camera_fps = 15  # Low FPS, sensor data is independent

            status = self._zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self._error = f"Failed to open ZED camera: {status}"
                logger.error(self._error)
                return False

            logger.info("ZED camera opened for sensor calibration")
            return True

        except ImportError:
            self._error = "ZED SDK (pyzed) not installed"
            logger.error(self._error)
            return False
        except Exception as e:
            self._error = f"Camera open error: {e}"
            logger.error(self._error)
            return False

    def _collect_loop(self) -> None:
        """Background thread: collect magnetometer samples."""
        try:
            import pyzed.sl as sl

            sensors_data = sl.SensorsData()

            while not self._stop_event.is_set():
                if (
                    self._zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
                    == sl.ERROR_CODE.SUCCESS
                ):
                    mag = sensors_data.get_magnetometer_data()
                    # Use UNCALIBRATED data to avoid double-applying factory cal
                    mag_field = mag.get_magnetic_field_uncalibrated()

                    if mag_field is None:
                        # Fallback: some SDK versions may not have uncalibrated
                        mag_field = mag.get_magnetic_field_calibrated()

                    if mag_field is not None:
                        mx, my, mz = (
                            float(mag_field[0]),
                            float(mag_field[1]),
                            float(mag_field[2]),
                        )

                        # Skip zero/invalid readings
                        if abs(mx) < 0.001 and abs(my) < 0.001 and abs(mz) < 0.001:
                            continue

                        # Deduplicate: skip if too similar to last sample
                        with self._lock:
                            if self._mag_samples:
                                last = self._mag_samples[-1]
                                dist = math.sqrt(
                                    (mx - last[0]) ** 2
                                    + (my - last[1]) ** 2
                                    + (mz - last[2]) ** 2
                                )
                                if dist < 0.5:  # Skip near-duplicate readings
                                    time.sleep(0.02)
                                    continue

                            self._mag_samples.append((mx, my, mz))

                            if len(self._mag_samples) % 50 == 0:
                                logger.info(
                                    f"Mag cal: {len(self._mag_samples)}/{self._target_samples} samples"
                                )

                # ~50 Hz sample rate
                time.sleep(0.02)

        except Exception as e:
            self._error = f"Collection error: {e}"
            logger.error(self._error)
            self._state = CalibrationState.FAILED

    def _compute_calibration(self) -> Optional[MagCalibrationResult]:
        """
        Compute hard-iron and soft-iron calibration from collected samples.

        Uses full quadratic ellipsoid fitting with cross-axis terms:
        Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Iz = 1
        """
        try:
            with self._lock:
                data = np.array(self._mag_samples)
            n = data.shape[0]

            if n < self._min_samples:
                self._error = f"Too few samples: {n}"
                return None

            # Check spatial coverage
            coverage = self._compute_coverage()
            if coverage.get("octants_covered", 0) < 4:
                self._error = (
                    f"Insufficient spatial coverage: {coverage.get('octants_covered', 0)}/8 octants. "
                    "Rotate the camera more during collection."
                )
                return None

            # ---- Full quadratic ellipsoid fit ----
            # Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Iz = 1
            x, y, z = data[:, 0], data[:, 1], data[:, 2]
            D_mat = np.column_stack([x**2, y**2, z**2, x * y, x * z, y * z, x, y, z])

            ones = np.ones(n)

            # Solve least squares: D @ params = 1
            params, residuals, rank, sv = np.linalg.lstsq(D_mat, ones, rcond=None)
            A, B, C, D_cross, E_cross, F_cross, G, H, I_param = params

            # Build quadratic matrix Q:
            # [A, D/2, E/2]
            # [D/2, B, F/2]
            # [E/2, F/2, C]
            Q = np.array(
                [
                    [A, D_cross / 2, E_cross / 2],
                    [D_cross / 2, B, F_cross / 2],
                    [E_cross / 2, F_cross / 2, C],
                ]
            )

            # Check positive-definiteness
            eigvals = np.linalg.eigvalsh(Q)
            if np.any(eigvals <= 0):
                self._error = (
                    "Ellipsoid fit produced non-physical result (not positive-definite). "
                    "Need more spatial coverage during rotation."
                )
                return None

            # Linear part: p = [G, H, I]
            p = np.array([G, H, I_param])

            # Hard-iron offset: center = -Q^-1 @ p / 2
            Q_inv = np.linalg.inv(Q)
            center = -0.5 * Q_inv @ p
            hx, hy, hz = center

            # Soft-iron: decompose Q to correction matrix
            # Q = R @ diag(eigenvalues) @ R^T
            # Soft-iron matrix S normalizes the ellipsoid to a sphere
            eigvals_full, eigvecs = np.linalg.eigh(Q)

            # Scale to average radius
            val = center @ Q @ center + p @ center + 1
            if val <= 0:
                self._error = "Invalid ellipsoid parameters"
                return None

            semi_axes = np.sqrt(val / eigvals_full)
            avg_r = np.mean(semi_axes)

            # Correction matrix: rotates and scales to sphere
            scale_diag = np.diag(avg_r / semi_axes)
            soft_iron_matrix = eigvecs @ scale_diag @ eigvecs.T

            soft_iron = soft_iron_matrix.tolist()

            # Compute fitness: how well the calibrated data forms a sphere
            corrected = (data - center) @ soft_iron_matrix.T
            radii = np.linalg.norm(corrected, axis=1)
            mean_r = np.mean(radii)
            std_r = np.std(radii)
            fitness = max(0.0, 1.0 - (std_r / mean_r if mean_r > 1e-6 else 1.0))

            duration = time.time() - self._start_time if self._start_time else 0.0

            return MagCalibrationResult(
                hard_iron=(
                    round(float(hx), 4),
                    round(float(hy), 4),
                    round(float(hz), 4),
                ),
                soft_iron=[[round(float(v), 6) for v in row] for row in soft_iron],
                fitness=round(float(fitness), 4),
                samples_used=n,
                timestamp=datetime.now(timezone.utc).isoformat(),
                duration_s=round(duration, 1),
            )

        except Exception as e:
            self._error = f"Computation error: {e}"
            logger.error(self._error)
            return None

    def _compute_coverage(self) -> dict:
        """Estimate spatial coverage of collected samples."""
        if not self._mag_samples:
            return {"octants_covered": 0, "total_octants": 8}

        data = np.array(self._mag_samples)
        center = np.mean(data, axis=0)
        centered = data - center

        # Check which octants have samples
        octants = set()
        for p in centered:
            octant = (
                int(p[0] >= 0),
                int(p[1] >= 0),
                int(p[2] >= 0),
            )
            octants.add(octant)

        return {
            "octants_covered": len(octants),
            "total_octants": 8,
            "coverage_pct": round(len(octants) / 8.0 * 100, 0),
        }

    def _save_calibration(self, result: MagCalibrationResult) -> None:
        """Save calibration to file."""
        CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
        cal_file = CALIBRATION_DIR / "magnetometer_cal.json"

        cal_data = {
            "type": "magnetometer",
            "version": "1.0",
            "camera": "ZED 2i",
            **result.to_dict(),
        }

        with open(cal_file, "w") as f:
            json.dump(cal_data, f, indent=2)

        logger.info(f"Magnetometer calibration saved to {cal_file}")


class IMUCalibrationCheck:
    """
    IMU health and calibration check.

    Collects accelerometer and gyroscope data while the camera is
    stationary to verify:
    - Accelerometer reads ~9.81 m/s^2 gravity (bias check)
    - Gyroscope reads ~0 rad/s (zero-rate offset)
    - Sensor noise levels are within expected ranges
    """

    @staticmethod
    def run_check(zed_camera=None, duration_s: float = 5.0) -> IMUCheckResult:
        """
        Run IMU health check.

        The camera should be stationary and level during this check.

        Args:
            zed_camera: Existing pyzed.sl.Camera instance (or None to open new)
            duration_s: Duration to collect samples (default 5s)
        """
        owns_camera = False
        zed = zed_camera

        try:
            import pyzed.sl as sl

            if zed is None:
                owns_camera = True
                zed = sl.Camera()
                init_params = sl.InitParameters()
                init_params.depth_mode = sl.DEPTH_MODE.NONE
                init_params.camera_resolution = sl.RESOLUTION.VGA
                init_params.camera_fps = 15
                status = zed.open(init_params)
                if status != sl.ERROR_CODE.SUCCESS:
                    return IMUCheckResult(
                        accel_bias=(0, 0, 0),
                        gyro_bias=(0, 0, 0),
                        accel_noise=0,
                        gyro_noise=0,
                        gravity_magnitude=0,
                        gravity_error_pct=100,
                        temperature=0,
                        healthy=False,
                        issues=[f"Failed to open camera: {status}"],
                        timestamp=datetime.now(timezone.utc).isoformat(),
                    )

            accel_samples = []
            gyro_samples = []
            temperatures = []

            sensors_data = sl.SensorsData()
            start = time.time()

            logger.info(
                f"IMU check: collecting data for {duration_s}s (keep camera still)"
            )

            while time.time() - start < duration_s:
                if (
                    zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
                    == sl.ERROR_CODE.SUCCESS
                ):
                    imu = sensors_data.get_imu_data()

                    # Use raw acceleration (includes gravity) for gravity magnitude check
                    # get_linear_acceleration() is gravity-compensated in some SDK versions,
                    # so we try get_linear_acceleration_uncalibrated() first
                    accel = None
                    try:
                        accel = imu.get_linear_acceleration_uncalibrated()
                    except AttributeError:
                        pass
                    if accel is None:
                        # Fallback: use standard accessor
                        accel = imu.get_linear_acceleration()

                    gyro = imu.get_angular_velocity()

                    if accel is not None:
                        accel_samples.append(
                            (float(accel[0]), float(accel[1]), float(accel[2]))
                        )
                    if gyro is not None:
                        gyro_samples.append(
                            (float(gyro[0]), float(gyro[1]), float(gyro[2]))
                        )

                    # Temperature
                    try:
                        temp_data = sensors_data.get_temperature_data()
                        if temp_data:
                            temp_val = temp_data.get(sl.SENSOR_LOCATION.IMU)
                            if temp_val is not None and isinstance(
                                temp_val, (int, float)
                            ):
                                temperatures.append(float(temp_val))
                    except Exception:
                        pass

                time.sleep(0.01)  # ~100 Hz

        except ImportError:
            return IMUCheckResult(
                accel_bias=(0, 0, 0),
                gyro_bias=(0, 0, 0),
                accel_noise=0,
                gyro_noise=0,
                gravity_magnitude=0,
                gravity_error_pct=100,
                temperature=0,
                healthy=False,
                issues=["ZED SDK not installed"],
                timestamp=datetime.now(timezone.utc).isoformat(),
            )
        finally:
            if owns_camera and zed:
                try:
                    zed.close()
                except Exception:
                    pass

        if not accel_samples:
            return IMUCheckResult(
                accel_bias=(0, 0, 0),
                gyro_bias=(0, 0, 0),
                accel_noise=0,
                gyro_noise=0,
                gravity_magnitude=0,
                gravity_error_pct=100,
                temperature=0,
                healthy=False,
                issues=["No IMU data received"],
                timestamp=datetime.now(timezone.utc).isoformat(),
            )

        # Analyze accelerometer
        accel_arr = np.array(accel_samples)
        accel_mean = np.mean(accel_arr, axis=0)
        accel_std = np.std(accel_arr, axis=0)
        gravity_mag = float(np.linalg.norm(accel_mean))
        gravity_error = abs(gravity_mag - 9.81) / 9.81 * 100

        # Analyze gyroscope
        gyro_arr = np.array(gyro_samples) if gyro_samples else np.zeros((1, 3))
        gyro_mean = np.mean(gyro_arr, axis=0)
        gyro_std = np.std(gyro_arr, axis=0)

        # Temperature
        avg_temp = np.mean(temperatures) if temperatures else 0.0

        # Check for issues
        issues = []
        healthy = True

        if gravity_error > 5.0:
            issues.append(
                f"Gravity magnitude off by {gravity_error:.1f}% (expected ~9.81 m/s^2, got {gravity_mag:.3f})"
            )
            healthy = False

        gyro_bias_mag = float(np.linalg.norm(gyro_mean))
        if gyro_bias_mag > 0.05:  # > 0.05 rad/s (~3 deg/s)
            issues.append(
                f"Gyroscope bias too high: {gyro_bias_mag:.4f} rad/s (threshold: 0.05)"
            )
            healthy = False

        accel_noise_mag = float(np.mean(accel_std))
        if accel_noise_mag > 0.5:
            issues.append(f"Accelerometer noise high: {accel_noise_mag:.3f} m/s^2")
            if accel_noise_mag > 1.0:
                healthy = False

        gyro_noise_mag = float(np.mean(gyro_std))
        if gyro_noise_mag > 0.02:
            issues.append(f"Gyroscope noise high: {gyro_noise_mag:.4f} rad/s")

        if not issues:
            issues.append("All sensors within normal parameters")

        return IMUCheckResult(
            accel_bias=(
                round(float(accel_mean[0]), 4),
                round(float(accel_mean[1]), 4),
                round(float(accel_mean[2]), 4),
            ),
            gyro_bias=(
                round(float(gyro_mean[0]), 4),
                round(float(gyro_mean[1]), 4),
                round(float(gyro_mean[2]), 4),
            ),
            accel_noise=round(accel_noise_mag, 4),
            gyro_noise=round(gyro_noise_mag, 4),
            gravity_magnitude=round(gravity_mag, 4),
            gravity_error_pct=round(gravity_error, 2),
            temperature=round(float(avg_temp), 1),
            healthy=healthy,
            issues=issues,
            timestamp=datetime.now(timezone.utc).isoformat(),
        )


def load_magnetometer_calibration() -> Optional[MagCalibrationResult]:
    """Load saved magnetometer calibration from disk."""
    cal_file = CALIBRATION_DIR / "magnetometer_cal.json"
    if not cal_file.exists():
        return None

    try:
        with open(cal_file) as f:
            data = json.load(f)

        return MagCalibrationResult(
            hard_iron=tuple(data["hard_iron"]),
            soft_iron=data["soft_iron"],
            fitness=data["fitness"],
            samples_used=data["samples_used"],
            timestamp=data["timestamp"],
            duration_s=data["duration_s"],
        )
    except Exception as e:
        logger.error(f"Failed to load magnetometer calibration: {e}")
        return None


def apply_magnetometer_calibration(
    raw_mag: Tuple[float, float, float],
    cal: MagCalibrationResult,
) -> Tuple[float, float, float]:
    """
    Apply magnetometer calibration to raw reading.

    Args:
        raw_mag: Raw magnetometer reading (x, y, z) in uT
        cal: Calibration result with hard-iron and soft-iron

    Returns:
        Corrected magnetometer reading (x, y, z)
    """
    raw = np.array(raw_mag) - np.array(cal.hard_iron)
    corrected = np.array(cal.soft_iron) @ raw
    return (float(corrected[0]), float(corrected[1]), float(corrected[2]))


# ==================== Global Session ====================

_mag_session: Optional[MagCalibrationSession] = None


def get_mag_session() -> Optional[MagCalibrationSession]:
    """Get the global magnetometer calibration session."""
    global _mag_session
    return _mag_session


def start_mag_calibration(zed_camera=None) -> MagCalibrationSession:
    """Start a new magnetometer calibration session."""
    global _mag_session
    if _mag_session and _mag_session.state == CalibrationState.COLLECTING:
        return _mag_session  # Already running
    _mag_session = MagCalibrationSession()
    success = _mag_session.start(zed_camera)
    if not success:
        logger.error(f"Failed to start mag calibration: {_mag_session.error}")
    return _mag_session


def stop_mag_calibration() -> Optional[MagCalibrationResult]:
    """Stop the current magnetometer calibration and compute results."""
    global _mag_session
    if not _mag_session:
        return None
    return _mag_session.stop()


# ==================== IMU Heading (6-Position) Calibration ====================

# The 6 canonical orientations and the expected gravity axis for each.
# (label, description, expected_accel_sign) where expected is the dominant axis.
IMU_POSITIONS = [
    # (label, user instruction, expected_accel_sign)
    # expected_accel_sign = the direction the accelerometer reads when stationary in this pose.
    # ZED IMU frame: X=right, Y=down, Z=forward.
    # Accelerometer measures the reaction force that counteracts gravity (anti-gravity direction).
    # When a face is pointing DOWN, gravity acts along that axis, so the accel reads the OPPOSITE.
    #
    #   Pose               Camera-frame gravity    Accel reads
    #   front (Z down)     +Z                      -Z  →  (0, 0, -1)
    #   back  (Z up)       -Z                      +Z  →  (0, 0, +1)
    #   left  (X down)     +X                      -X  →  (-1, 0, 0)   ← right side resting on table
    #   right (-X down)    -X                      +X  →  (+1, 0, 0)   ← left side resting on table
    #   up    (Y down)     +Y                      -Y  →  (0, -1, 0)   ← normal upright position
    #   down  (-Y down)    -Y                      +Y  →  (0, +1, 0)   ← upside down
    ("front", "Place camera lens pointing DOWN (front facing ground)", (0, 0, -1)),
    ("back", "Place camera lens pointing UP (front facing sky)", (0, 0, +1)),
    ("left", "Place camera on its RIGHT side (left side facing up)", (-1, 0, 0)),
    ("right", "Place camera on its LEFT side (right side facing up)", (+1, 0, 0)),
    ("up", "Place camera upright (top facing up, normal position)", (0, -1, 0)),
    ("down", "Place camera upside-down (top facing ground)", (0, +1, 0)),
]


@dataclass
class IMUHeadingCalibrationResult:
    """Result of 6-position IMU heading calibration."""

    accel_bias: Tuple[float, float, float]  # (x, y, z) bias in m/s^2
    accel_scale: Tuple[float, float, float]  # (x, y, z) scale factors
    gyro_bias: Tuple[float, float, float]  # (x, y, z) gyro bias in rad/s
    positions_collected: int
    fitness: float  # 0-1, quality of fit
    timestamp: str
    duration_s: float
    eeprom_store_attempted: bool = False
    eeprom_store_success: Optional[bool] = None
    eeprom_store_detail: str = "not_attempted"

    def to_dict(self) -> dict:
        return {
            "accel_bias": list(self.accel_bias),
            "accel_scale": list(self.accel_scale),
            "gyro_bias": list(self.gyro_bias),
            "positions_collected": self.positions_collected,
            "fitness": self.fitness,
            "timestamp": self.timestamp,
            "duration_s": self.duration_s,
            "eeprom_store_attempted": self.eeprom_store_attempted,
            "eeprom_store_success": self.eeprom_store_success,
            "eeprom_store_detail": self.eeprom_store_detail,
        }


class IMUHeadingCalibration:
    """
    6-position IMU accelerometer calibration for heading accuracy.

    The user places the camera in 6 orientations (front, back, left, right,
    up, down) and collects data at each. This provides known gravity vectors
    in all 6 axis-aligned directions, allowing computation of per-axis
    accelerometer scale factors and biases.

    These corrections improve the gravity-compensated acceleration and
    attitude estimation, which directly improves heading accuracy.
    """

    def __init__(self, collect_duration: float = 3.0):
        self._collect_duration = collect_duration
        self._state = CalibrationState.IDLE
        self._current_position = 0  # Index into IMU_POSITIONS
        self._position_data: dict = {}  # position_label -> (accel_samples, gyro_samples)
        self._start_time: Optional[float] = None
        self._error: Optional[str] = None
        self._result: Optional[IMUHeadingCalibrationResult] = None
        self._lock = threading.RLock()
        self._zed = None
        self._owns_camera = False
        self._stop_event = threading.Event()

    @property
    def state(self) -> CalibrationState:
        return self._state

    @property
    def current_position(self) -> int:
        return self._current_position

    @property
    def result(self) -> Optional[IMUHeadingCalibrationResult]:
        return self._result

    @property
    def error(self) -> Optional[str]:
        return self._error

    def start(self, zed_camera=None) -> bool:
        """Begin a new 6-position calibration session."""
        with self._lock:
            self._state = CalibrationState.COLLECTING
            self._current_position = 0
            self._position_data.clear()
            self._result = None
            self._error = None
            self._start_time = time.time()
            self._stop_event.clear()

            if zed_camera:
                self._zed = zed_camera
                self._owns_camera = False
            else:
                self._owns_camera = True
                if not self._open_camera():
                    self._state = CalibrationState.FAILED
                    return False

            logger.info("IMU heading calibration started (6-position)")
            return True

    def collect_position(self) -> dict:
        """
        Collect data for the current position.

        Returns dict with success, position label, and sample count.
        The camera must be held still in the instructed orientation.
        """
        if self._state != CalibrationState.COLLECTING:
            return {"success": False, "error": "Not in collecting state"}

        if self._current_position >= len(IMU_POSITIONS):
            return {"success": False, "error": "All positions already collected"}

        label, description, _ = IMU_POSITIONS[self._current_position]

        try:
            import pyzed.sl as sl

            accel_samples = []
            gyro_samples = []
            sensors_data = sl.SensorsData()
            start = time.time()

            logger.info(f"Collecting position '{label}': {description}")

            while (
                time.time() - start < self._collect_duration
                and not self._stop_event.is_set()
            ):
                with self._lock:
                    if self._zed is None:
                        if self._stop_event.is_set():
                            break
                        self._error = "ZED camera not available"
                        return {"success": False, "error": self._error}
                    read_status = self._zed.get_sensors_data(
                        sensors_data, sl.TIME_REFERENCE.CURRENT
                    )

                if read_status == sl.ERROR_CODE.SUCCESS:
                    imu = sensors_data.get_imu_data()

                    accel = None
                    try:
                        accel = imu.get_linear_acceleration_uncalibrated()
                    except AttributeError:
                        pass
                    if accel is None:
                        accel = imu.get_linear_acceleration()

                    gyro = imu.get_angular_velocity()

                    if accel is not None:
                        accel_samples.append(
                            (float(accel[0]), float(accel[1]), float(accel[2]))
                        )
                    if gyro is not None:
                        gyro_samples.append(
                            (float(gyro[0]), float(gyro[1]), float(gyro[2]))
                        )

                time.sleep(0.01)

            if self._stop_event.is_set():
                return {"success": False, "error": "Calibration canceled"}

            if len(accel_samples) < 50:
                return {
                    "success": False,
                    "error": f"Too few samples: {len(accel_samples)}",
                }

            with self._lock:
                if (
                    self._stop_event.is_set()
                    or self._state != CalibrationState.COLLECTING
                ):
                    return {"success": False, "error": "Calibration canceled"}
                self._position_data[label] = (accel_samples, gyro_samples)
                self._current_position += 1
                positions_remaining = len(IMU_POSITIONS) - self._current_position

            logger.info(
                f"Position '{label}' collected: {len(accel_samples)} accel, {len(gyro_samples)} gyro samples"
            )

            return {
                "success": True,
                "position": label,
                "samples": len(accel_samples),
                "positions_remaining": positions_remaining,
            }

        except ImportError:
            self._error = "ZED SDK not installed"
            return {"success": False, "error": self._error}
        except Exception as e:
            self._error = str(e)
            return {"success": False, "error": self._error}

    def compute(self) -> Optional[IMUHeadingCalibrationResult]:
        """Compute calibration from all collected positions."""
        if len(self._position_data) < 6:
            self._error = f"Need 6 positions, only have {len(self._position_data)}"
            self._state = CalibrationState.FAILED
            return None

        self._state = CalibrationState.COMPUTING

        try:
            # Expected gravity at each position (9.81 * direction_sign)
            GRAVITY = 9.80665
            all_accel = []
            all_expected = []
            all_gyro = []

            for i, (label, _, expected_sign) in enumerate(IMU_POSITIONS):
                if label not in self._position_data:
                    self._error = f"Missing position: {label}"
                    self._state = CalibrationState.FAILED
                    return None

                accel_samples, gyro_samples = self._position_data[label]
                accel_arr = np.array(accel_samples)
                accel_mean = np.mean(accel_arr, axis=0)

                expected = np.array(expected_sign, dtype=np.float64) * GRAVITY
                all_accel.append(accel_mean)
                all_expected.append(expected)

                if gyro_samples:
                    gyro_arr = np.array(gyro_samples)
                    all_gyro.append(np.mean(gyro_arr, axis=0))

            measured = np.array(all_accel)  # (6, 3)
            expected = np.array(all_expected)  # (6, 3)

            # Per-axis least-squares: measured_i = scale_i * expected_i + bias_i
            scale = np.ones(3)
            bias = np.zeros(3)

            for axis in range(3):
                m = measured[:, axis]
                e = expected[:, axis]
                # Solve: m = scale * e + bias
                A = np.column_stack([e, np.ones(6)])
                params, _, _, _ = np.linalg.lstsq(A, m, rcond=None)
                scale[axis] = params[0]
                bias[axis] = params[1]

            # Gyro bias: average of all positions (should read ~0 when stationary)
            gyro_bias = np.mean(all_gyro, axis=0) if all_gyro else np.zeros(3)

            # Fitness: how well calibrated readings match expected gravity
            corrected = (measured - bias) / scale
            errors = np.linalg.norm(corrected - expected, axis=1)
            mean_error = np.mean(errors)
            fitness = max(0.0, 1.0 - mean_error / GRAVITY)

            duration = time.time() - self._start_time if self._start_time else 0.0

            result = IMUHeadingCalibrationResult(
                accel_bias=(
                    round(float(bias[0]), 6),
                    round(float(bias[1]), 6),
                    round(float(bias[2]), 6),
                ),
                accel_scale=(
                    round(float(scale[0]), 6),
                    round(float(scale[1]), 6),
                    round(float(scale[2]), 6),
                ),
                gyro_bias=(
                    round(float(gyro_bias[0]), 6),
                    round(float(gyro_bias[1]), 6),
                    round(float(gyro_bias[2]), 6),
                ),
                positions_collected=len(self._position_data),
                fitness=round(float(fitness), 4),
                timestamp=datetime.now(timezone.utc).isoformat(),
                duration_s=round(duration, 1),
            )

            self._result = result
            self._state = CalibrationState.COMPLETE
            logger.info(
                f"IMU heading calibration complete: fitness={result.fitness:.3f}"
            )

            # Attempt to store calibration in camera EEPROM
            if self._zed:
                result.eeprom_store_attempted = True
                try:
                    import pyzed.sl as sl
                    if not hasattr(self._zed, "store_calibration"):
                        result.eeprom_store_success = False
                        result.eeprom_store_detail = "store_calibration_not_supported_by_pyzed"
                        logger.warning(
                            "IMU heading EEPROM write unsupported: pyzed Camera has no store_calibration()"
                        )
                    else:
                        store_status = self._zed.store_calibration()
                        if store_status == sl.ERROR_CODE.SUCCESS:
                            result.eeprom_store_success = True
                            result.eeprom_store_detail = "stored_to_camera_eeprom"
                            logger.info("IMU heading calibration stored to camera EEPROM")
                        else:
                            result.eeprom_store_success = False
                            result.eeprom_store_detail = str(store_status)
                            logger.warning(f"Failed to store IMU heading calibration to EEPROM: {store_status}")
                except Exception as e:
                    result.eeprom_store_success = False
                    result.eeprom_store_detail = str(e)
                    logger.warning(f"EEPROM store error: {e}")

            self._save_calibration(result)

            return result

        except Exception as e:
            self._error = f"Computation error: {e}"
            logger.error(self._error)
            self._state = CalibrationState.FAILED
            return None

        finally:
            # Always release the ZED camera after calibration completes or fails
            # so the Isaac ROS container can open it for nvblox/OD.
            if self._owns_camera:
                with self._lock:
                    if self._zed:
                        try:
                            self._zed.close()
                        except Exception:
                            pass
                        self._zed = None

    def cancel(self):
        """Cancel the calibration."""
        self._stop_event.set()
        with self._lock:
            if self._owns_camera and self._zed:
                try:
                    self._zed.close()
                except Exception:
                    pass
                self._zed = None
            self._state = CalibrationState.IDLE
            self._current_position = 0
            self._position_data.clear()

    def get_status(self) -> dict:
        """Get current calibration status."""
        positions_done = list(self._position_data.keys())
        next_pos = None
        next_instruction = None
        if self._current_position < len(IMU_POSITIONS):
            label, desc, _ = IMU_POSITIONS[self._current_position]
            next_pos = label
            next_instruction = desc

        status = {
            "state": self._state.value,
            "current_step": self._current_position + 1,
            "total_steps": len(IMU_POSITIONS),
            "positions_done": positions_done,
            "next_position": next_pos,
            "next_instruction": next_instruction,
        }
        if self._start_time:
            status["elapsed_s"] = round(time.time() - self._start_time, 1)
        if self._error:
            status["error"] = self._error
        if self._result:
            status["result"] = self._result.to_dict()
        return status

    def _open_camera(self) -> bool:
        """Open ZED camera for sensor-only access."""
        try:
            import pyzed.sl as sl

            self._zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.depth_mode = sl.DEPTH_MODE.NONE
            init_params.camera_resolution = sl.RESOLUTION.VGA
            init_params.camera_fps = 15

            status = self._zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self._error = f"Failed to open ZED camera: {status}"
                return False

            return True
        except ImportError:
            self._error = "ZED SDK (pyzed) not installed"
            return False
        except Exception as e:
            self._error = f"Camera open error: {e}"
            return False

    def _save_calibration(self, result: IMUHeadingCalibrationResult):
        """Save calibration to file."""
        CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
        cal_file = CALIBRATION_DIR / "imu_heading_cal.json"

        cal_data = {
            "type": "imu_heading",
            "version": "1.0",
            "camera": "ZED 2i",
            **result.to_dict(),
        }

        with open(cal_file, "w") as f:
            json.dump(cal_data, f, indent=2)

        logger.info(f"IMU heading calibration saved to {cal_file}")


def load_imu_heading_calibration() -> Optional[IMUHeadingCalibrationResult]:
    """Load saved IMU heading calibration from disk."""
    cal_file = CALIBRATION_DIR / "imu_heading_cal.json"
    if not cal_file.exists():
        return None

    try:
        with open(cal_file) as f:
            data = json.load(f)
        return IMUHeadingCalibrationResult(
            accel_bias=tuple(data["accel_bias"]),
            accel_scale=tuple(data["accel_scale"]),
            gyro_bias=tuple(data["gyro_bias"]),
            positions_collected=data["positions_collected"],
            fitness=data["fitness"],
            timestamp=data["timestamp"],
            duration_s=data["duration_s"],
            eeprom_store_attempted=data.get("eeprom_store_attempted", False),
            eeprom_store_success=data.get("eeprom_store_success"),
            eeprom_store_detail=data.get("eeprom_store_detail", "not_attempted"),
        )
    except Exception as e:
        logger.error(f"Failed to load IMU heading calibration: {e}")
        return None


_imu_heading_session: Optional[IMUHeadingCalibration] = None


def get_imu_heading_session() -> Optional[IMUHeadingCalibration]:
    """Get the global IMU heading calibration session."""
    global _imu_heading_session
    return _imu_heading_session


def start_imu_heading_calibration(zed_camera=None) -> IMUHeadingCalibration:
    """Start a new 6-position IMU heading calibration session."""
    global _imu_heading_session
    _imu_heading_session = IMUHeadingCalibration()
    success = _imu_heading_session.start(zed_camera)
    if not success:
        logger.error(
            f"Failed to start IMU heading calibration: {_imu_heading_session.error}"
        )
    return _imu_heading_session


# ==================== CLI Entry Point ====================


def main():
    """CLI entry point for sensor calibration."""
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="ZED 2i Sensor Calibration")
    parser.add_argument(
        "--type",
        choices=["mag", "imu", "all"],
        default="all",
        help="Calibration type: mag (magnetometer), imu (health check), or all",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Magnetometer collection duration in seconds (default: 60)",
    )
    parser.add_argument(
        "--imu-duration",
        type=float,
        default=5.0,
        help="IMU check duration in seconds (default: 5)",
    )
    args = parser.parse_args()

    if args.type in ("imu", "all"):
        print("\n=== IMU Health Check ===")
        print("Keep the camera stationary and level...")
        result = IMUCalibrationCheck.run_check(duration_s=args.imu_duration)
        print(
            f"  Gravity: {result.gravity_magnitude:.3f} m/s^2 (error: {result.gravity_error_pct:.1f}%)"
        )
        print(f"  Accel bias: {result.accel_bias}")
        print(f"  Gyro bias: {result.gyro_bias}")
        print(f"  Accel noise: {result.accel_noise:.4f} m/s^2")
        print(f"  Gyro noise: {result.gyro_noise:.4f} rad/s")
        print(f"  Temperature: {result.temperature:.1f} C")
        print(f"  Healthy: {result.healthy}")
        for issue in result.issues:
            print(f"  - {issue}")

    if args.type in ("mag", "all"):
        print(f"\n=== Magnetometer Calibration ({args.duration}s) ===")
        print("Rotate the camera slowly in ALL orientations (figure-8 pattern)")
        print("Cover all 8 octants for best results.\n")

        session = MagCalibrationSession()
        session.start()

        try:
            start = time.time()
            while time.time() - start < args.duration:
                status = session.get_status()
                coverage = status.get("coverage", {})
                print(
                    f"\r  Samples: {status['samples']}/{status['target_samples']} "
                    f"| Coverage: {coverage.get('octants_covered', 0)}/8 octants "
                    f"| Elapsed: {status.get('elapsed_s', 0):.0f}s",
                    end="",
                    flush=True,
                )
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n  Interrupted by user")

        print("\n  Computing calibration...")
        result = session.stop()

        if result:
            print(f"  Hard-iron offset: {result.hard_iron}")
            print(f"  Soft-iron matrix: {result.soft_iron}")
            print(f"  Fitness: {result.fitness:.3f}")
            print(f"  Samples used: {result.samples_used}")
            print(f"  Saved to: {CALIBRATION_DIR / 'magnetometer_cal.json'}")
        else:
            print(f"  Calibration failed: {session.error}")


if __name__ == "__main__":
    main()
