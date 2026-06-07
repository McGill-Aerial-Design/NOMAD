# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""A minimal, runnable NOMAD module.

This is the example referenced by ``docs/writing_a_module.md`` and exercised by
``tests/test_module_registry.py``. It is the smallest module that registers a
FastAPI route and participates in the ``start()``/``stop()`` lifecycle.

Load it explicitly (config / tests)::

    from edge_core.core import wire_modules
    wire_modules(app, specs=["sample_module.sample_module:SampleModule"])

or, when packaged and installed, advertise it on the ``nomad.modules``
entry-point group so it is auto-discovered::

    [project.entry-points."nomad.modules"]
    sample = "sample_module.sample_module:SampleModule"
"""

from __future__ import annotations

from typing import Any

from edge_core.core import AppContext, BaseModule, ModuleMetadata


class SampleModule(BaseModule):
    """Smallest useful module: one health route plus lifecycle state."""

    metadata = ModuleMetadata(
        name="sample",
        version="0.1.0",
        description="Minimal example NOMAD module.",
    )

    def __init__(self) -> None:
        self.started = False

    def configure(self, ctx: AppContext) -> None:
        # Grab whatever core services you need off the context here, e.g.
        # ``self.state = ctx.get_service("state")``.
        self.ctx = ctx

    def register_routes(self, app: Any) -> None:
        from fastapi import APIRouter

        router = APIRouter(prefix="/api/sample", tags=["sample"])

        @router.get("/ping")
        async def ping() -> dict[str, Any]:
            return {
                "module": self.metadata.name,
                "started": self.started,
                "reply": "pong",
            }

        app.include_router(router)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False
