# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status, health monitoring,
payload/actuation control, and video streaming.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import logging

from fastapi import FastAPI

from .api_auth import APIKeyMiddleware, AuthSettings
from .api_context import ApiRouteContext
from .api_cors import CorsSettings, configure_cors
from .api_route_registry import register_all_routes
from .api_state import initialize_app_state
from .services.state import StateManager

logger = logging.getLogger("edge_core.api")


def create_app(state_manager: StateManager) -> FastAPI:
    """
    Create the FastAPI application for Edge Core.

    Args:
        state_manager: StateManager instance for system state

    Returns:
        Configured FastAPI application
    """
    app = build_fastapi_app()
    configure_cors(app, CorsSettings.from_env())

    auth = AuthSettings.from_env(logger)
    app.add_middleware(APIKeyMiddleware, settings=auth, logger=logger)

    initialize_app_state(app, state_manager)
    register_all_routes(app, build_route_context(auth))

    return app


def build_fastapi_app() -> FastAPI:
    return FastAPI(
        title="NOMAD Edge Core API",
        description="Drone-side companion-computer API for NOMAD",
        version="1.0.0",
        docs_url="/docs",
        redoc_url="/redoc",
    )


def build_route_context(auth: AuthSettings) -> ApiRouteContext:
    return ApiRouteContext(
        logger=logger,
        nomad_api_key=auth.api_key,
        allow_insecure_remote=auth.allow_insecure_remote,
        require_terminal_api_key=auth.require_admin_api_key,
    )
