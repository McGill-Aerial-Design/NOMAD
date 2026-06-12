# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Runtime lifecycle for the Edge Core process."""

from __future__ import annotations

import atexit
import logging
import signal
import threading
from collections.abc import Callable
from typing import Any

import uvicorn
from fastapi import FastAPI

from .core import AppContext, wire_modules
from .env import env_bool
from .services.state import StateManager


def is_sim_mode() -> bool:
    """True when NOMAD_SIM_MODE is set to a truthy value."""
    return env_bool("NOMAD_SIM_MODE")


def cleanup(app: FastAPI, logger: logging.Logger) -> None:
    """Tear down all active modules and services."""
    logger.info("Shutting down Edge Core...")
    if hasattr(app.state, "module_registry") and app.state.module_registry:
        try:
            app.state.module_registry.stop_all()
            logger.info("All modular services stopped")
        except Exception as exc:
            logger.error("Error stopping modules: %s", exc)
    logger.info("Cleanup complete")


def install_cleanup_handlers(app: FastAPI, logger: logging.Logger) -> Callable[[], None]:
    cleanup_lock = threading.Lock()
    cleanup_done = False

    def safe_cleanup() -> None:
        nonlocal cleanup_done
        with cleanup_lock:
            if cleanup_done:
                return
            cleanup_done = True
        cleanup(app, logger)

    atexit.register(safe_cleanup)

    def signal_handler(signum: int, frame: Any) -> None:
        sig_name = signal.Signals(signum).name
        logger.info("Received signal %s (%s), shutting down...", sig_name, signum)
        safe_cleanup()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    return safe_cleanup


def wire_runtime_modules(app: FastAPI, state_manager: StateManager, logger: logging.Logger) -> None:
    ctx = AppContext(app=app)
    ctx.register_service("state_manager", state_manager)

    # In sim/dev we tolerate a half-wired boot so the API still comes up; on real
    # hardware a wiring failure is fatal.
    try:
        wire_modules(app, ctx=ctx)
    except Exception as exc:
        logger.critical("Module wiring failed: %s", exc, exc_info=True)
        if not is_sim_mode():
            raise
        logger.warning("Continuing with a partially wired app (sim mode)")


def run(
    app: FastAPI,
    state_manager: StateManager,
    logger: logging.Logger,
    host: str = "0.0.0.0",
    port: int = 8000,
    log_level: str = "info",
) -> None:
    """Run the Edge Core FastAPI server and drive service lifecycles."""
    logger.info("=" * 50)
    logger.info("NOMAD Edge Core Starting")
    logger.info("=" * 50)

    safe_cleanup = install_cleanup_handlers(app, logger)
    wire_runtime_modules(app, state_manager, logger)

    try:
        uvicorn.run(app, host=host, port=port, log_level=log_level)
    finally:
        safe_cleanup()
