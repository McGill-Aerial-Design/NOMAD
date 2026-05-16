"""
NOMAD Edge Core - Main Entry Point.

Initializes and runs the drone-side services including:
- MAVLink interface for flight controller communication
- FastAPI server for REST/WebSocket API
- Time synchronization service

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import atexit
import logging
import math
import os
import signal
import subprocess
import sys
from typing import Any

# Ensure user-local libs (e.g. libturbojpeg for pyzed) are discoverable
_local_lib = os.path.expanduser("~/.local/lib")
if os.path.isdir(_local_lib):
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    if _local_lib not in _ld:
        os.environ["LD_LIBRARY_PATH"] = f"{_local_lib}:{_ld}" if _ld else _local_lib
    # Pre-load libturbojpeg so pyzed can find it in the current process
    import ctypes
    _turbojpeg = os.path.join(_local_lib, "libturbojpeg.so.0")
    if os.path.isfile(_turbojpeg):
        try:
            ctypes.cdll.LoadLibrary(_turbojpeg)
        except OSError:
            pass

import time
import uvicorn
import asyncio

from .api import ( create_app,
                  set_isaac_bridge,
                  set_health_monitor,
                  set_tailscale_manager,
                  set_network_monitor,
                  set_nav_controller,
)
from .operational_mode import init_mode_manager, OperationalModeManager
from .spray_controller import SprayController, SprayTarget

from .logging_service import cleanup_old_logs
from .video_stream_manager import init_video_stream_manager

from .mavlink_interface import MavlinkService
from .nav_controller import NavController
from .state import StateManager
from .time_manager import TimeSyncService, TimeSyncStatus
from .health_monitor import JetsonHealthMonitor
from tailscale.src.tailscale_manager import init_tailscale_manager
from tailscale.src.network_monitor import init_network_monitor

# Conditional import for Isaac ROS bridge (ROS2 environment only)
try:
    from .isaac_ros_bridge import IsaacROSBridge, init_isaac_bridge, get_isaac_bridge
    ISAAC_ROS_AVAILABLE = True
except ImportError:
    ISAAC_ROS_AVAILABLE = False
    IsaacROSBridge = None  # type: ignore

# Conditional import for servo controller (PWM control)
try:
    from .servo_controller import init_servo_controller, shutdown_servo_controller, get_servo_controller
    SERVO_AVAILABLE = True
except ImportError:
    SERVO_AVAILABLE = False

# RC channel to servo bridge
try:
    from .rc_servo_bridge import init_rc_servo_bridge, shutdown_rc_servo_bridge, get_rc_servo_bridge
    RC_SERVO_BRIDGE_AVAILABLE = True
except ImportError:
    RC_SERVO_BRIDGE_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("edge_core")


# Initialize services
state_manager = StateManager.instance()
mavlink_service = MavlinkService(state_manager)

# Time synchronization service
time_sync_service: TimeSyncService | None = None

# Health monitor for Jetson metrics
health_monitor: JetsonHealthMonitor | None = None

# Navigation controller with velocity watchdog (0.5s timeout)
nav_controller: NavController | None = None

# Isaac ROS bridge (Task 2 only - requires ROS2 environment)
isaac_bridge: "IsaacROSBridge | None" = None

# Servo controller for camera tilt and water shooter
servo_controller_initialized: bool = False

tailscale_manager = None
network_monitor = None

# Operational mode manager (Section 9)
mode_manager: "OperationalModeManager | None" = None

# Spray controller (SP-001 to SP-008)
spray_controller: "SprayController | None" = None

def get_app():
    """Get or create the FastAPI application."""
    return create_app(state_manager)


# Create app instance for uvicorn
app = get_app()


def cleanup() -> None:
    """Cleanup on shutdown."""
    global time_sync_service, isaac_bridge, health_monitor, nav_controller, servo_controller_initialized, spray_controller
    logger.info("Shutting down Edge Core...")

    # Abort any active spray sequence
    if spray_controller and spray_controller.is_active:
        try:
            spray_controller.abort()
        except Exception:
            pass

    # Stop detection thread (if running)
    try:
        stop_fn = getattr(app.state, '_stop_detection_fn', None)
        if stop_fn:
            stop_fn()
    except Exception:
        pass

    spray_controller = None

    # Shutdown RC servo bridge
    if RC_SERVO_BRIDGE_AVAILABLE:
        try:
            shutdown_rc_servo_bridge()
        except Exception:
            pass

    # Shutdown servo controller (safety - disable PWM outputs)
    if servo_controller_initialized and SERVO_AVAILABLE:
        try:
            shutdown_servo_controller()
            servo_controller_initialized = False
            logger.info("Servo controller stopped")
        except Exception as e:
            logger.error(f"Error shutting down servo controller: {e}")

    # Stop navigation controller first (safety - stops velocity watchdog)
    if nav_controller:
        nav_controller.stop()
        logger.info("Navigation controller stopped")

    # Stop Isaac ROS bridge (depends on ROS being active)
    if isaac_bridge:
        isaac_bridge.stop()
        logger.info("Isaac ROS bridge stopped")

    # Stop health monitor
    if health_monitor:
        health_monitor.stop()
        logger.info("Health monitor stopped")

    # Stop time sync service
    if time_sync_service:
        time_sync_service.stop()
        logger.info("Time sync service stopped")

    mavlink_service.stop()
    logger.info("Cleanup complete")


# Register cleanup
atexit.register(cleanup)


def run(
    host: str = "0.0.0.0",
    port: int = 8000,
    log_level: str = "info",
) -> None:
    """
    Run the Edge Core server.

    Args:
        host: Host address to bind to
        port: Port number
        log_level: Logging level
    """
    global time_sync_service, isaac_bridge, health_monitor, nav_controller
    global tailscale_manager, network_monitor

    logger.info("=" * 50)
    logger.info("NOMAD Edge Core Starting")
    logger.info("=" * 50)
    logger.info(f"Host: {host}:{port}")
    logger.info("=" * 50)

    # Cleanup old logs before starting services (non-blocking, fail-safe)
    try:
        deleted = cleanup_old_logs()
        if deleted > 0:
            logger.info(f"Cleaned up {deleted} old log files")
    except Exception as e:
        logger.warning(f"Log cleanup failed (non-critical): {e}")

    # Initialize Jetson health monitor
    health_monitor = JetsonHealthMonitor(poll_interval=2.0)
    health_monitor.set_state_manager(state_manager)
    health_monitor.start()
    set_health_monitor(app, health_monitor)
    logger.info("Health monitor started")

    tailscale_manager = init_tailscale_manager(
        hostname="nomad-jetson",
        on_status_change=lambda info: logger.info(
            f"Tailscale: {info.status.value}"
        ),
    )
    set_tailscale_manager(app, tailscale_manager)

    gcs_ip = os.environ.get("GCS_IP", "100.76.127.17")
    network_monitor = init_network_monitor(gcs_tailscale_ip=gcs_ip)
    set_network_monitor(app, network_monitor)

    @app.on_event("startup")
    async def _start_network_services():
        await tailscale_manager.start()
        await network_monitor.start()
        logger.info("Tailscale manager + network monitor started")


    # Initialize Isaac ROS bridge (Task 2 only - requires NOMAD_ENABLE_ISAAC_ROS=true)
    enable_isaac = os.environ.get("NOMAD_ENABLE_ISAAC_ROS", "false").lower() == "true"
    if enable_isaac and ISAAC_ROS_AVAILABLE:
        try:
            isaac_bridge = init_isaac_bridge()
            isaac_bridge.start()
            set_isaac_bridge(app, isaac_bridge)
            logger.info("Isaac ROS bridge started")
        except Exception as e:
            logger.error(f"Failed to start Isaac ROS bridge: {e}")
            isaac_bridge = None
    elif enable_isaac and not ISAAC_ROS_AVAILABLE:
        logger.warning("Isaac ROS enabled but rclpy not available - skipping bridge")
    else:
        logger.info("Isaac ROS bridge disabled (set NOMAD_ENABLE_ISAAC_ROS=true to enable)")

    # Nav2 integration flag: controls whether navigation controller bridges ROS nav2 commands
    # Set NOMAD_ENABLE_NAV2=false to disable nav2 integration (default: true when Isaac ROS enabled)
    # This flag can be used to disable nav2 in production for safety/testing
    enable_nav2 = os.environ.get("NOMAD_ENABLE_NAV2", "true" if enable_isaac else "false").lower() == "true"
    if enable_nav2:
        logger.info("Nav2 integration enabled (NOMAD_ENABLE_NAV2=true or implicit from NOMAD_ENABLE_ISAAC_ROS)")
    else:
        logger.info("Nav2 integration disabled (set NOMAD_ENABLE_NAV2=true to enable)")

    # Mesh bridge is not auto-started; mesh data arrives via ros_http_bridge
    # (POST /api/task/2/slam/mesh/update -> GET /api/task/2/slam/mesh)

    # Video stream manager. The /api/video/start and /api/video/stop endpoints
    # remain the only way to bring the bridge up/down. The systemd unit
    # nomad-video-bridge.service is the SINGLE OWNER of that lifecycle; the
    # in-process auto-start thread was removed to eliminate ownership races.
    # A crash-recovery watchdog still runs (see VideoStreamManager).
    init_video_stream_manager(container_name="nomad_isaac_ros")
    logger.info("Video stream manager initialized (owner: nomad-video-bridge.service)")

    # ZED camera is owned by the ROS2 wrapper inside the Isaac ROS container
    # (for nvblox, video bridge, etc.). Task 1 captures use the RTSP stream.

    # Initialize servo controller for camera tilt and water shooter
    global servo_controller_initialized
    enable_servos = os.environ.get("NOMAD_ENABLE_SERVOS", "true").lower() == "true"
    # Camera tilt servo can be driven from the flight controller PWM outputs.
    # Use environment variable NOMAD_CAMERA_TILT_SERVO_CHANNEL to select channel (1-indexed).
    try:
        camera_tilt_channel = int(os.environ.get("NOMAD_CAMERA_TILT_SERVO_CHANNEL", "6"))
    except ValueError:
        camera_tilt_channel = 6
        logger.warning("Invalid NOMAD_CAMERA_TILT_SERVO_CHANNEL value, using default 6")

    if enable_servos and SERVO_AVAILABLE:
        try:
            if init_servo_controller(mavlink_service=mavlink_service, camera_tilt_channel=camera_tilt_channel):
                servo_controller_initialized = True
                logger.info("Servo controller initialized for camera tilt and water shooter")
            else:
                logger.warning("Servo controller initialization failed - PWM pins or MAVLink may not be configured")
        except Exception as e:
            logger.error(f"Failed to initialize servo controller: {e}")
    elif not SERVO_AVAILABLE:
        logger.warning("Servo controller module not available")
    else:
        logger.info("Servo controller disabled (set NOMAD_ENABLE_SERVOS=true to enable)")

    # Initialize time synchronization service
    def on_time_sync_change(status: TimeSyncStatus) -> None:
        """Callback when time sync status changes."""
        if status.synced:
            logger.info(f"Time synchronized via {status.source.name}")
        else:
            logger.warning(f"Time synchronization lost (offset: {status.offset_seconds:.3f}s)")

    time_sync_service = TimeSyncService(
        state_manager=state_manager,
        on_sync_change=on_time_sync_change,
    )
    time_sync_service.start()
    
    # Log initial sync status
    sync_status = time_sync_service.status
    if sync_status.synced:
        logger.info(f"Time sync: {sync_status.source.name}")
    else:
        logger.warning("Time not synchronized - will use GPS time when available")

    # Start MAVLink service
    mavlink_service.set_time_sync_service(time_sync_service)
    
    # Initialize RC-to-servo bridge (maps ELRS controller knob to nozzle servo)
    try:
        rc_channel = int(os.environ.get("NOMAD_RC_SERVO_CHANNEL", "6"))
    except ValueError:
        rc_channel = 6
        logger.warning("Invalid NOMAD_RC_SERVO_CHANNEL value, using default channel 6")
    enable_rc_servo = os.environ.get("NOMAD_ENABLE_RC_SERVO", "true").lower() == "true"
    if enable_rc_servo and servo_controller_initialized and RC_SERVO_BRIDGE_AVAILABLE:
        try:
            bridge = init_rc_servo_bridge(
                servo_controller=get_servo_controller(),
                rc_channel=rc_channel,
                enabled=True,
            )
            if bridge:
                mavlink_service.set_rc_servo_bridge(bridge)
                logger.info(f"RC servo bridge started (channel {rc_channel} -> nozzle servo)")
            else:
                logger.warning("RC servo bridge failed to start")
        except Exception as e:
            logger.error(f"Failed to start RC servo bridge: {e}")
    elif not enable_rc_servo:
        logger.info("RC servo bridge disabled (set NOMAD_ENABLE_RC_SERVO=true to enable)")
    
    mavlink_service.start()
    logger.info("MAVLink service started")

    # Start navigation controller with velocity watchdog (SAFETY: 0.5s timeout)
    # For nav2-enabled systems, this bridges ROS velocity commands to MAVLink
    # For nav2-disabled systems, this provides API-based navigation (e.g., GPS targets, velocity commands)
    # This ensures commands timeout and vehicle stops if connection is lost
    nav_controller = NavController(mavlink_service, state_manager, nav2_enabled=enable_nav2)
    nav_controller.start()
    set_nav_controller(app, nav_controller)
    if enable_nav2:
        logger.info("Navigation controller started (nav2 mode, velocity watchdog: 0.5s timeout)")
    else:
        logger.info("Navigation controller started (API mode, velocity watchdog: 0.5s timeout)")

    # Initialize operational mode manager (Section 9)
    global mode_manager, spray_controller
    servo_ctrl = get_servo_controller() if servo_controller_initialized and SERVO_AVAILABLE else None
    mode_manager = init_mode_manager(
        servo_controller=servo_ctrl,
        state_manager=state_manager,
    )
    app.state.mode_manager = mode_manager

    # nvblox restart callback: kill composable node, overlay correct config, relaunch
    def _restart_nvblox(config_name: str) -> bool:
        """Restart nvblox inside Isaac ROS container with a different config."""
        container = "nomad_isaac_ros"
        if config_name == "indoor":
            src_cfg = "/workspaces/isaac_ros-dev/config/nvblox_indoor.yaml"
        else:
            src_cfg = "/workspaces/isaac_ros-dev/config/nvblox_performance.yaml"
        try:
            # Kill existing nvblox launch (the PID written by start script)
            subprocess.run(
                ["docker", "exec", container, "bash", "-c",
                 "kill $(cat /tmp/zed_nvblox.pid 2>/dev/null) 2>/dev/null; sleep 2"],
                timeout=10, capture_output=True,
            )
            # Overlay config onto installed nvblox_base.yaml and relaunch
            relaunch_script = (
                "source /opt/ros/humble/setup.bash 2>/dev/null; "
                "source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; "
                "export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH; "
                f'NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory; '
                f"print(get_package_share_directory('nvblox_examples_bringup'))\" 2>/dev/null)/config/nvblox/nvblox_base.yaml; "
                f"cp {src_cfg} $NVBLOX_BASE; "
                'NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py; '
                'if [ -f "$NOMAD_LAUNCH" ]; then '
                '  ros2 launch "$NOMAD_LAUNCH" & echo $! > /tmp/zed_nvblox.pid; '
                'else '
                '  ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 & echo $! > /tmp/zed_nvblox.pid; '
                'fi'
            )
            result = subprocess.run(
                ["docker", "exec", "-d", container, "bash", "-c", relaunch_script],
                timeout=15, capture_output=True,
            )
            success = result.returncode == 0
            if success:
                logger.info(f"nvblox restarted with config: {config_name}")
            else:
                logger.error(f"nvblox restart failed: {result.stderr.decode()}")
            return success
        except Exception as e:
            logger.error(f"nvblox restart error: {e}")
            return False

    mode_manager.set_nvblox_restart_fn(_restart_nvblox)
    logger.info("Operational mode manager initialized")

    # Initialize spray controller (SP-001 to SP-008)
    spray_controller = SprayController(
        nav_controller=nav_controller,
        servo_controller=servo_ctrl,
        state_manager=state_manager,
        mode_manager=mode_manager,
    )
    app.state.spray_controller = spray_controller

    # Wire spray callbacks
    # set_capture_photo_fn: capture frame from video bridge HTTP snapshot
    # Photos saved to persistent directory (not /tmp) for competition evidence
    SPRAY_CAPTURE_DIR = os.path.expanduser("~/.nomad/spray_captures")

    def _capture_photo() -> str | None:
        """Capture a frame from the ZED video bridge and save persistently.

        Saves to ~/.nomad/spray_captures/ with timestamped filenames
        so images survive reboots and can be reviewed after flights.
        """
        try:
            import requests as _requests
            import numpy as _np
            from datetime import datetime

            os.makedirs(SPRAY_CAPTURE_DIR, exist_ok=True)
            bridge_port = int(os.environ.get("NOMAD_BRIDGE_HTTP_PORT", "9200"))
            snap_url = f"http://172.17.0.1:{bridge_port}/snapshot"
            resp = _requests.get(snap_url, timeout=3)
            if resp.status_code == 200 and resp.headers.get("Content-Type", "").startswith("image"):
                import cv2
                arr = _np.frombuffer(resp.content, dtype=_np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = f"spray_capture_{ts}.jpg"
                    path = os.path.join(SPRAY_CAPTURE_DIR, filename)
                    cv2.imwrite(path, img)
                    logger.info(f"Photo saved: {path}")
                    return path
        except Exception as e:
            logger.error(f"Photo capture failed: {e}")
        return None

    spray_controller.set_capture_photo_fn(_capture_photo)

    # Spray artifact manager — provides /api/task/2/spray/manual/* and
    # last_artifacts for the Mission Planner Submit panel. Reuses the same
    # capture function so the before/after snapshots match the autonomous flow.
    try:
        from .task2_spray_artifacts import get_artifact_manager
        _art_mgr = get_artifact_manager()
        _art_mgr.set_capture_photo_fn(_capture_photo)
        spray_controller.set_artifact_callbacks(
            start_fn=lambda before_path: _art_mgr.start_session(
                source="autonomous", before_path=before_path,
            ),
            stop_fn=lambda after_path: _art_mgr.stop_session(after_path=after_path),
        )
    except Exception as e:
        logger.warning(f"Spray artifact manager init failed: {e}")

    # set_verify_hsv_fn: check if target region shifted from purple to blue
    def _verify_hsv(photo_path: str) -> bool:
        """Analyze sprayed target for purple-to-blue color shift."""
        try:
            import cv2
            import numpy as _np
            img = cv2.imread(photo_path)
            if img is None:
                return False
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Center 30% of image (where target should be after aiming)
            h, w = hsv.shape[:2]
            cx, cy = w // 2, h // 2
            rw, rh = w // 6, h // 6
            roi = hsv[cy - rh:cy + rh, cx - rw:cx + rw]
            # Blue range in OpenCV HSV: H 100-130, S > 80, V > 50
            blue_mask = cv2.inRange(roi, _np.array([100, 80, 50]), _np.array([130, 255, 255]))
            blue_ratio = _np.count_nonzero(blue_mask) / max(blue_mask.size, 1)
            passed = blue_ratio > 0.15  # 15% blue pixels indicates successful spray
            logger.info(f"HSV verify: blue_ratio={blue_ratio:.2f} passed={passed}")
            return passed
        except Exception as e:
            logger.error(f"HSV verification failed: {e}")
            return False

    spray_controller.set_verify_hsv_fn(_verify_hsv)

    # set_verify_circle_change_fn: color-agnostic circle detection before/after
    # Compares circles in pre/post spray images. If >20% pixel change, pass.
    def _verify_circle_change(pre_spray_path: str, post_spray_path: str) -> bool:
        """Compare pre/post spray images using color-agnostic circle detection.

        Primary verifier (ColorAgnosticCircleVerifier) does any-shape detection
        plus a colour-agnostic mean Lab deltaE / pixel-change delta inside the
        matched ROI — passes on >=20% change regardless of hue. The HSV-gated
        CircleChangeVerifier remains available as an opt-in fallback for
        Task 2 runs where we know the paper chemistry is purple→blue: set
        NOMAD_SPRAY_VERIFY=hsv to enable it.
        """
        try:
            from .task2_circle_verify import (
                ColorAgnosticCircleVerifier,
                CircleChangeVerifier,
            )
            verify_mode = (os.environ.get("NOMAD_SPRAY_VERIFY") or "").strip().lower()
            if verify_mode == "hsv":
                verifier = CircleChangeVerifier(
                    change_threshold=0.20,
                    pixel_diff_threshold=30,
                    match_distance_px=50,
                )
            else:
                verifier = ColorAgnosticCircleVerifier(
                    change_threshold=0.20,
                    pixel_diff_threshold=30,
                    match_distance_px=50,
                )
            before = verifier.capture_snapshot_from_file(pre_spray_path)
            after = verifier.capture_snapshot_from_file(post_spray_path)
            result = verifier.compare(before, after)
            logger.info(
                f"Circle verify: change={result.change_ratio:.1%} "
                f"verified={result.verified} matched={result.matched_pairs} "
                f"detail={result.details}"
            )
            return result.verified
        except Exception as e:
            logger.error(f"Circle change verification failed: {e}")
            return False

    spray_controller.set_verify_circle_change_fn(_verify_circle_change)

    # Wire Google Drive upload for autonomous photo submission (CONOPS 5.2.4).
    # Uses existing gdrive_upload.py module — requires OAuth2 setup on Jetson:
    #   python -m edge_core.gdrive_upload --setup <client_secret.json>
    # Set GDRIVE_FOLDER_ID env var to the team's competition folder.
    try:
        from .gdrive_upload import upload_to_gdrive, gdrive_ready

        if gdrive_ready():
            def _upload_photo(local_path: str, filename: str) -> str:
                """Upload spray photo to Google Drive."""
                file_id = upload_to_gdrive(local_path, filename)
                if file_id:
                    logger.info(f"Google Drive upload: {filename} -> id={file_id}")
                else:
                    logger.warning(f"Google Drive upload failed for {filename}")
                return file_id

            spray_controller.set_upload_fn(_upload_photo)
            logger.info("Google Drive upload enabled (autonomous)")
        else:
            logger.warning(
                "Google Drive not configured — photos saved locally only. "
                "Run: python -m edge_core.gdrive_upload --setup <client_secret.json>"
            )
    except ImportError as e:
        logger.warning(f"Google Drive upload unavailable: {e}")

    # set_excluded_sectors_fn: store on app.state for obstacle distance endpoint
    app.state.excluded_sectors: set[int] = set()

    def _set_excluded_sectors(sectors: set[int]) -> None:
        app.state.excluded_sectors = sectors

    spray_controller.set_excluded_sectors_fn(_set_excluded_sectors)

    # Nav2 was removed; spray controller now relies on visual servoing for
    # the approach phase, so no nav2 goal/status/cancel wiring is needed here.

    # ------------------------------------------------------------------ #
    # Continuous detection thread for visual servoing AIM phase.
    # Runs at ~5 Hz in background, caches the latest detection so the
    # spray controller gets instant responses instead of blocking on
    # an HTTP snapshot per detection call.
    # ------------------------------------------------------------------ #
    import threading as _threading

    _latest_detection: dict | None = None
    _detection_lock = _threading.Lock()
    _detection_running = False

    def _detection_loop() -> None:
        """Background detection thread — continuously detects targets at ~5 Hz.

        Uses NOMAD's built-in circle detector (contrast/shape-based, no YOLO).
        Caches the latest detection result for the spray controller and exposes
        it through /api/detections so Mission Planner can select it.
        """
        nonlocal _latest_detection, _detection_running
        _detection_running = True
        logger.info("Detection thread started (~5 Hz)")

        # Lazy imports (only done once when thread starts)
        try:
            from .task2_circle_verify import Task2CircleDetector
            import cv2
            import numpy as _np
            import requests as _requests
        except ImportError as e:
            logger.error(f"Detection thread: missing dependency: {e}")
            _detection_running = False
            return

        detector = Task2CircleDetector(
            min_radius_px=3,    # Lowered for small targets at distance
            hough_param2=30,    # More sensitive for small circles
            blur_kernel=3,      # Less blur to preserve small features
        )
        bridge_port = int(os.environ.get("NOMAD_BRIDGE_HTTP_PORT", "9200"))
        snap_url = f"http://172.17.0.1:{bridge_port}/snapshot"
        # simple_video_bridge refreshes /snapshot every ~2s; polling faster
        # just re-decodes and re-detects the same JPEG. Skip when the payload
        # hash hasn't moved so we don't burn Jetson CPU on duplicate work.
        last_payload_hash: int | None = None

        while _detection_running:
            try:
                resp = _requests.get(snap_url, timeout=1)
                if resp.status_code != 200:
                    time.sleep(0.5)
                    continue

                payload_hash = hash(resp.content)
                if payload_hash == last_payload_hash:
                    time.sleep(0.5)
                    continue
                last_payload_hash = payload_hash

                arr = _np.frombuffer(resp.content, dtype=_np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is None:
                    time.sleep(0.5)
                    continue

                circles = detector.detect(img)
                current_detection = None
                with _detection_lock:
                    if circles:
                        best = max(
                            circles, key=lambda c: c.confidence * c.radius
                        )
                        bbox_w = float(best.radius * 2)
                        bbox_h = float(best.radius * 2)
                        bbox_x = float(best.cx - best.radius)
                        bbox_y = float(best.cy - best.radius)
                        current_detection = {
                            "target_id": 1,
                            "label": "task2_circle",
                            "confidence": float(best.confidence * 100.0),
                            "bbox_x": bbox_x,
                            "bbox_y": bbox_y,
                            "bbox_w": bbox_w,
                            "bbox_h": bbox_h,
                            "cx": float(best.cx),
                            "cy": float(best.cy),
                            "radius_px": float(best.radius),
                            "source": "snapshot_circle",
                            "image_only": True,
                        }
                        drone_state = state_manager.get_state()
                        drone_pos = (
                            getattr(drone_state, "vio_x", None),
                            getattr(drone_state, "vio_y", None),
                            getattr(drone_state, "vio_z", None),
                        )
                        heading_deg = getattr(drone_state, "heading_deg", None)
                        if all(v is not None for v in drone_pos) and heading_deg is not None:
                            # Approximate a selectable target location straight
                            # ahead. Final alignment is image-based; this only
                            # lets the existing trigger/approach API work when
                            # custom object detection is disabled.
                            approx_range = float(SprayController.TARGET_CAMERA_RANGE_M)
                            yaw = math.radians(float(heading_deg))
                            current_detection["x"] = float(drone_pos[0]) + approx_range * math.cos(yaw)
                            current_detection["y"] = float(drone_pos[1]) + approx_range * math.sin(yaw)
                            current_detection["z"] = float(drone_pos[2])
                            current_detection["range_m"] = approx_range
                        _latest_detection = current_detection
                    else:
                        _latest_detection = None
                with app.state.detection_state_lock:
                    existing_depth_detection = None
                    existing_age_s = None
                    try:
                        existing_age_s = time.time() - float(app.state.detection_last_update)
                    except (TypeError, ValueError):
                        existing_age_s = None
                    for det in getattr(app.state, "detected_objects", []):
                        if (
                            isinstance(det, dict)
                            and det.get("source") == "target_localizer_depth"
                            and det.get("range_m") is not None
                        ):
                            existing_depth_detection = det
                            break

                    # Do not let the lightweight RGB snapshot detector clobber
                    # the ROS-side circle+depth detection. The depth path is the
                    # collision-critical source for forward/backward motion.
                    preserve_depth_detection = (
                        existing_depth_detection is not None
                        and existing_age_s is not None
                        and existing_age_s <= 1.0
                    )
                    if current_detection is not None:
                        if not preserve_depth_detection:
                            app.state.detected_objects = [current_detection]
                            app.state.detection_last_update = time.time()
                    elif (
                        getattr(app.state, "detected_objects", [])
                        and app.state.detected_objects[0].get("source") == "snapshot_circle"
                    ):
                        app.state.detected_objects = []
                        app.state.detection_last_update = time.time()
            except Exception as e:
                logger.error(f"Detection loop error: {e}")
            time.sleep(0.5)  # ~2 Hz; bridge snapshot only refreshes every 2s

    def _get_detection_bbox(target_or_id) -> dict | tuple | None:
        """Return the freshest detection for spray visual servoing.

        Prefer the shape-based Task 2 circle detector. If another detection
        source is publishing through /api/detections, use the freshest bbox and
        range data available.
        """
        # Image-only targets (Task 2 shape detector) live in the bridge —
        # they never enter app.state.detected_objects, so prefer the bridge's
        # current task2 circle list when the spray target is image_only.
        image_only = bool(getattr(target_or_id, "image_only", False))
        if image_only:
            try:
                from .video_stream_manager import get_video_stream_manager as _gvm
                _mgr = _gvm()
                if _mgr is not None:
                    raw = _mgr.get_overlay_detections(source="task2")
                    shape_dets = raw.get("detections", []) if isinstance(raw, dict) else []
                    best = None
                    best_score = float("inf")
                    for d in shape_dets:
                        try:
                            bw = float(d.get("bbox_w", 0) or 0)
                            bh = float(d.get("bbox_h", 0) or 0)
                        except (TypeError, ValueError):
                            continue
                        if bw <= 0 or bh <= 0:
                            continue
                        # Pick the highest-confidence circle currently visible.
                        score = -float(d.get("confidence", 0) or 0)
                        if score < best_score:
                            best = d
                            best_score = score
                    if best is not None:
                        bx = float(best.get("bbox_x", 0) or 0)
                        by = float(best.get("bbox_y", 0) or 0)
                        bw = float(best.get("bbox_w", 0) or 0)
                        bh = float(best.get("bbox_h", 0) or 0)
                        rng = best.get("range_m")
                        try:
                            rng_f = float(rng) if rng is not None else None
                        except (TypeError, ValueError):
                            rng_f = None
                        return {
                            "cx": bx + bw / 2.0,
                            "cy": by + bh / 2.0,
                            "bbox_x": bx,
                            "bbox_y": by,
                            "bbox_w": bw,
                            "bbox_h": bh,
                            "range_m": rng_f,
                            "confidence": best.get("confidence", 0),
                            "source": "task2_shape",
                        }
            except Exception as e:
                logger.debug(f"Task 2 shape detection lookup failed: {e}")

        try:
            target_x = getattr(target_or_id, "x", None)
            target_y = getattr(target_or_id, "y", None)
            target_z = getattr(target_or_id, "z", None)
            with app.state.detection_state_lock:
                current = list(getattr(app.state, "detected_objects", []))

            best = None
            best_score = float("inf")
            for det in current:
                bbox_w = float(det.get("bbox_w", 0) or 0)
                bbox_h = float(det.get("bbox_h", 0) or 0)
                if bbox_w <= 0 or bbox_h <= 0:
                    continue
                if target_x is not None and target_y is not None and target_z is not None:
                    try:
                        dx = float(det.get("x", 0)) - float(target_x)
                        dy = float(det.get("y", 0)) - float(target_y)
                        dz = float(det.get("z", 0)) - float(target_z)
                        score = dx * dx + dy * dy + dz * dz
                    except (TypeError, ValueError):
                        score = 0.0
                else:
                    score = -float(det.get("confidence", 0) or 0)
                if score < best_score:
                    best = det
                    best_score = score

            if best is not None:
                bbox_x = float(best.get("bbox_x", 0) or 0)
                bbox_y = float(best.get("bbox_y", 0) or 0)
                bbox_w = float(best.get("bbox_w", 0) or 0)
                bbox_h = float(best.get("bbox_h", 0) or 0)
                det_x = best.get("x")
                det_y = best.get("y")
                det_z = best.get("z")
                range_m = best.get("range_m")
                try:
                    det_x_f = float(det_x) if det_x is not None else None
                    det_y_f = float(det_y) if det_y is not None else None
                    det_z_f = float(det_z) if det_z is not None else None
                    if range_m is None and all(v is not None for v in (det_x_f, det_y_f, det_z_f)):
                        range_m = (det_x_f * det_x_f + det_y_f * det_y_f + det_z_f * det_z_f) ** 0.5
                except (TypeError, ValueError):
                    det_x_f = det_y_f = det_z_f = None
                    range_m = None
                return {
                    "cx": bbox_x + bbox_w / 2.0,
                    "cy": bbox_y + bbox_h / 2.0,
                    "bbox_x": bbox_x,
                    "bbox_y": bbox_y,
                    "bbox_w": bbox_w,
                    "bbox_h": bbox_h,
                    "range_m": range_m,
                    "x_m": det_x_f,
                    "y_m": det_y_f,
                    "z_m": det_z_f,
                    "confidence": best.get("confidence", 0),
                    "source": best.get("source", "detections_api"),
                }
        except Exception as e:
            logger.debug(f"ZED detection lookup failed, using snapshot fallback: {e}")

        with _detection_lock:
            return _latest_detection

    def _stop_detection() -> None:
        """Stop the detection thread (called on shutdown)."""
        nonlocal _detection_running
        _detection_running = False

    # Start background detection thread
    _det_thread = _threading.Thread(
        target=_detection_loop, daemon=True, name="spray_detection"
    )
    _det_thread.start()
    spray_controller.set_detection_bbox_fn(_get_detection_bbox)
    # Store stop function for cleanup
    app.state._stop_detection_fn = _stop_detection

    logger.info("Spray controller initialized with velocity approach + circle verify + continuous detection")

    # Start health status broadcast (every 2 seconds)
    mavlink_service.start_health_broadcast(interval=2.0)
    logger.info("Health status broadcast started (2s interval)")

    # Handle shutdown signals -- guard cleanup to run exactly once across
    # signal handler, try/finally, and atexit paths.
    import threading
    _cleanup_lock = threading.Lock()
    _cleanup_done = False

    def _safe_cleanup() -> None:
        nonlocal _cleanup_done
        with _cleanup_lock:
            if _cleanup_done:
                return
            _cleanup_done = True
        cleanup()

    # Replace the atexit handler with the guarded version
    atexit.unregister(cleanup)
    atexit.register(_safe_cleanup)

    def signal_handler(signum: int, frame: Any) -> None:
        sig_name = signal.Signals(signum).name
        logger.info(f"Received signal {sig_name} ({signum}), shutting down...")
        _safe_cleanup()
        # Raise SystemExit so uvicorn performs its own graceful shutdown
        # (close sockets, drain connections) before the process exits.
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run FastAPI server
    try:
        uvicorn.run(app, host=host, port=port, log_level=log_level)
    finally:
        _safe_cleanup()


def main() -> None:
    """
    CLI entry point with argument parsing.
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description="NOMAD Edge Core - Drone-side processing system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    # Server arguments
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host address to bind to",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port number for REST API",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="info",
        choices=["debug", "info", "warning", "error"],
        help="Logging level",
    )
    
    # Simulation/Development arguments
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Enable simulation mode (mock hardware)",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Disable vision process",
    )
    parser.add_argument(
        "--no-task2",
        action="store_true",
        help="Disable Task 2 features",
    )
    parser.add_argument(
        "--servo-mode",
        type=str,
        default="gimbal",
        choices=["gimbal", "direct", "disabled"],
        help="Servo control mode",
    )
    
    args = parser.parse_args()
    
    # Set environment variables based on CLI args
    if args.sim:
        os.environ["NOMAD_SIM_MODE"] = "true"
    if args.no_vision:
        os.environ["NOMAD_ENABLE_VISION"] = "false"
    if args.no_task2:
        os.environ["TASK2_ENABLED"] = "false"
    if args.servo_mode != "gimbal":
        os.environ["SERVO_MODE"] = args.servo_mode
    
    # Run the server
    run(
        host=args.host,
        port=args.port,
        log_level=args.log_level,
    )


if __name__ == "__main__":
    main()
