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
import os
import signal
import sys
from pathlib import Path
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

import uvicorn
import asyncio

sys.path.insert (
    0, str(Path(__file__).resolve().parent.parent / "tailscale" / "src" )
)

from .api import ( create_app,
                  set_isaac_bridge,
                  set_health_monitor,
                  set_tailscale_manager,
                  set_network_monitor,
                  set_nav_controller,
)

from .logging_service import cleanup_old_logs
from .video_stream_manager import init_video_stream_manager

from .mavlink_interface import MavlinkService
from .nav_controller import NavController
from .state import StateManager
from .time_manager import TimeSyncService, TimeSyncStatus
from .health_monitor import JetsonHealthMonitor
from tailscale_manager import init_tailscale_manager
from network_monitor import init_network_monitor

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

def get_app():
    """Get or create the FastAPI application."""
    return create_app(state_manager)


# Create app instance for uvicorn
app = get_app()


def cleanup() -> None:
    """Cleanup on shutdown."""
    global time_sync_service, isaac_bridge, health_monitor, nav_controller, servo_controller_initialized
    logger.info("Shutting down Edge Core...")

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

    async def _start_network_services():
        await tailscale_manager.start()
        await network_monitor.start()

    asyncio.run(_start_network_services())
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

    # Mesh bridge is not auto-started; mesh data arrives via ros_http_bridge
    # (POST /api/task/2/slam/mesh/update -> GET /api/task/2/slam/mesh)

    # Initialize video stream manager with auto-start
    # This runs in background and will auto-start the video relay when container is ready
    enable_video_auto_start = os.environ.get("NOMAD_VIDEO_AUTO_START", "true").lower() == "true"
    init_video_stream_manager(
        container_name="nomad_isaac_ros",
        auto_start=enable_video_auto_start
    )
    logger.info(f"Video stream manager initialized (auto_start={enable_video_auto_start})")

    # ZED camera is owned by the ROS2 wrapper inside the Isaac ROS container
    # (for nvblox, video bridge, etc.). Task 1 captures use the RTSP stream.

    # Initialize servo controller for camera tilt and water shooter
    global servo_controller_initialized
    enable_servos = os.environ.get("NOMAD_ENABLE_SERVOS", "true").lower() == "true"
    if enable_servos and SERVO_AVAILABLE:
        try:
            if init_servo_controller():
                servo_controller_initialized = True
                logger.info("Servo controller initialized for camera tilt and water shooter")
            else:
                logger.warning("Servo controller initialization failed - PWM pins may not be configured")
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
    # This ensures commands timeout and vehicle stops if connection is lost
    nav_controller = NavController(mavlink_service, state_manager)
    nav_controller.start()
    set_nav_controller(app, nav_controller)
    logger.info("Navigation controller started (velocity watchdog: 0.5s timeout)")

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

