# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - Main Entry Point.

Initializes and runs the drone-side services under the NomadModule framework.
"""

from __future__ import annotations

import atexit
import logging
import os
import signal
from typing import Any

# Ensure user-local libs (e.g. libturbojpeg for pyzed) are discoverable
_local_lib = os.path.expanduser("~/.local/lib")
if os.path.isdir(_local_lib):
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    if _local_lib not in _ld:
        os.environ["LD_LIBRARY_PATH"] = f"{_local_lib}:{_ld}" if _ld else _local_lib
    import ctypes

    _turbojpeg = os.path.join(_local_lib, "libturbojpeg.so.0")
    if os.path.isfile(_turbojpeg):
        try:
            ctypes.cdll.LoadLibrary(_turbojpeg)
        except OSError:
            pass

import uvicorn

from .api import create_app
from .core import AppContext, wire_modules
from .services.state import StateManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("edge_core")

# Thread-safe global state manager singleton
state_manager = StateManager.instance()


def _is_sim_mode() -> bool:
    """True when NOMAD_SIM_MODE is set to a truthy value (set by ``--sim``)."""
    return os.environ.get("NOMAD_SIM_MODE", "").strip().lower() in {"1", "true", "yes", "on"}


def get_app():
    """Get or create the FastAPI application."""
    return create_app(state_manager)


# Module-level singleton created at import time. uvicorn's reload/worker modes
# import the app by string ("edge_core.main:app"), so this must exist at module
# scope — do not move it into run()/main() or uvicorn cannot find it.
app = get_app()


def cleanup() -> None:
    """Tear down all active modules and services."""
    logger.info("Shutting down Edge Core...")
    if hasattr(app.state, "module_registry") and app.state.module_registry:
        try:
            app.state.module_registry.stop_all()
            logger.info("All modular services stopped")
        except Exception as e:
            logger.error("Error stopping modules: %s", e)
    logger.info("Cleanup complete")


def run(
    host: str = "0.0.0.0",
    port: int = 8000,
    log_level: str = "info",
) -> None:
    """Run the Edge Core FastAPI server and drive service lifecycles."""
    logger.info("=" * 50)
    logger.info("NOMAD Edge Core Starting")
    logger.info("=" * 50)

    # Initialize AppContext with our baseline state and config
    ctx = AppContext(app=app)
    ctx.register_service("state_manager", state_manager)

    # Register cleanup handlers safely
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

    atexit.register(_safe_cleanup)

    def signal_handler(signum: int, frame: Any) -> None:
        sig_name = signal.Signals(signum).name
        logger.info("Received signal %s (%s), shutting down...", sig_name, signum)
        _safe_cleanup()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Wire and start all discoverable modules (includes core services + routes).
    # In sim/dev we tolerate a half-wired boot so the API still comes up; on real
    # hardware a wiring failure is fatal — fail fast rather than fly half-armed.
    try:
        wire_modules(app, ctx=ctx)
    except Exception as exc:
        logger.critical("Module wiring failed: %s", exc, exc_info=True)
        if not _is_sim_mode():
            raise
        logger.warning("Continuing with a partially wired app (sim mode)")

    try:
        uvicorn.run(app, host=host, port=port, log_level=log_level)
    finally:
        _safe_cleanup()


def main() -> None:
    """CLI entry point with argument parsing."""
    import argparse

    parser = argparse.ArgumentParser(
        description="NOMAD Edge Core - Drone-side processing system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port number for REST API")
    parser.add_argument(
        "--log-level", type=str, default="info", choices=["debug", "info", "warning", "error"], help="Logging level"
    )
    parser.add_argument("--sim", action="store_true", help="Enable simulation mode")

    args = parser.parse_args()

    # CLI flags are translated into environment variables here, which is the
    # single contract by which modules/services discover their configuration:
    #   --sim -> NOMAD_SIM_MODE (sim vs real hardware paths)
    # Modules read these via AppContext / os.environ; see docs/configuration.md.
    if args.sim:
        os.environ["NOMAD_SIM_MODE"] = "true"

    run(host=args.host, port=args.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
