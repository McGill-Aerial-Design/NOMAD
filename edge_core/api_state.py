# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""FastAPI app.state initialization for Edge Core."""

from __future__ import annotations

from fastapi import FastAPI

from .services.state import StateManager


def initialize_app_state(app: FastAPI, state_manager: StateManager) -> None:
    app.state.state_manager = state_manager
    app.state.health_monitor = None
    app.state.mavlink_service = None
    app.state.isaac_runtime_cache = {
        "timestamp": 0.0,
        "container_running": False,
    }
