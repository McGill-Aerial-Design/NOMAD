# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Register baseline Edge Core API route modules."""

from __future__ import annotations

from fastapi import FastAPI

from .api_context import ApiRouteContext


def register_all_routes(app: FastAPI, ctx: ApiRouteContext) -> None:
    from .api_routes.services import register_services_routes
    from .api_routes.streaming import register_streaming_routes
    from .api_routes.system import register_system_routes
    from .api_routes.terminal import register_terminal_routes

    register_system_routes(app, ctx)
    register_terminal_routes(app, ctx)
    register_services_routes(app, ctx)
    register_streaming_routes(app, ctx)
