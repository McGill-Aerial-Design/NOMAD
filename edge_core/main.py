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
)


from .mavlink_interface import MavlinkService
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

# Isaac ROS bridge (Task 2 only - requires ROS2 environment)
isaac_bridge: "IsaacROSBridge | None" = None

tailscale_manager = None
network_monitor = None

def get_app():
    """Get or create the FastAPI application."""
    return create_app(state_manager)


# Create app instance for uvicorn
app = get_app()


def cleanup() -> None:
    """Cleanup on shutdown."""
    global time_sync_service, isaac_bridge, health_monitor
    logger.info("Shutting down Edge Core...")

    # Stop Isaac ROS bridge first (depends on ROS being active)
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
    global time_sync_service, isaac_bridge, health_monitor
    global tailscale_manager, network_monitor

    logger.info("=" * 50)
    logger.info("NOMAD Edge Core Starting")
    logger.info("=" * 50)
    logger.info(f"Host: {host}:{port}")
    logger.info("=" * 50)

    # Initialize Jetson health monitor
    health_monitor = JetsonHealthMonitor(poll_interval=2.0)
    health_monitor.start()
    set_health_monitor(health_monitor)
    logger.info("Health monitor started")

    tailscale_manager = init_tailscale_manager(
        hostname="nomad-jetson",
        on_status_change=lambda info: logger.info(
            f"Tailscale: {info.status.value}"
        ),
    )
    set_tailscale_manager(tailscale_manager)

    network_monitor = init_network_monitor(gcs_tailscale_ip="100.103.238.9" 
    ) #IP of Tailscale on my laptop
    set_network_monitor(network_monitor)

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
            set_isaac_bridge(isaac_bridge)
            logger.info("Isaac ROS bridge started")
        except Exception as e:
            logger.error(f"Failed to start Isaac ROS bridge: {e}")
            isaac_bridge = None
    elif enable_isaac and not ISAAC_ROS_AVAILABLE:
        logger.warning("Isaac ROS enabled but rclpy not available - skipping bridge")
    else:
        logger.info("Isaac ROS bridge disabled (set NOMAD_ENABLE_ISAAC_ROS=true to enable)")

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
    mavlink_service.start()
    logger.info("MAVLink service started")

    # Start health status broadcast (every 2 seconds)
    mavlink_service.start_health_broadcast(interval=2.0)
    logger.info("Health status broadcast started (2s interval)")

    # Handle shutdown signals
    def signal_handler(signum: int, frame: Any) -> None:
        logger.info(f"Received signal {signum}")
        cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run FastAPI server
    try:
        uvicorn.run(app, host=host, port=port, log_level=log_level)
    finally:
        cleanup()


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

