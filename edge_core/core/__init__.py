# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""NOMAD module SDK.

This package provides the framework that turns NOMAD capabilities into
discoverable, optional **modules**:

* :class:`~edge_core.core.module.NomadModule` / :class:`~edge_core.core.module.BaseModule`
  — the module interface and a convenience base.
* :class:`~edge_core.core.context.AppContext` — services + config handed to modules.
* :class:`~edge_core.core.registry.ModuleRegistry` — discovery, dependency
  resolution, and lifecycle.

Modules register via the ``nomad.modules`` entry-point group::

    [project.entry-points."nomad.modules"]
    my_module = "my_pkg.my_module:MyModule"

This module system is transitional and will be removed during the C++ cutover.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Iterable
from contextlib import asynccontextmanager
from typing import Any

from .context import AppContext
from .module import BaseModule, ModuleMetadata, NomadModule
from .registry import ENTRY_POINT_GROUP, ModuleError, ModuleRegistry

logger = logging.getLogger(__name__)

__all__ = [
    "AppContext",
    "BaseModule",
    "ModuleMetadata",
    "NomadModule",
    "ModuleRegistry",
    "ModuleError",
    "ENTRY_POINT_GROUP",
    "wire_modules",
]


def _allow_list_from_env() -> list[str] | None:
    """Parse the optional NOMAD_MODULES allow-list (comma/space separated)."""
    raw = os.environ.get("NOMAD_MODULES", "").strip()
    names = [n for n in raw.replace(",", " ").split() if n]
    return names or None


def _create_registry(allow_list: Iterable[str] | None) -> ModuleRegistry:
    registry = ModuleRegistry(allow_list=list(allow_list) if allow_list is not None else None)
    try:
        registry.discover_entry_points()
    except Exception as exc:  # noqa: BLE001 - never let discovery crash startup
        logger.error("module discovery failed: %s", exc)
    return registry


def _install_module_lifespan(app: Any, registry: ModuleRegistry) -> None:
    previous_lifespan = app.router.lifespan_context

    @asynccontextmanager
    async def module_lifespan(app_: Any) -> Any:  # pragma: no cover - exercised at runtime
        async with previous_lifespan(app_):
            registry.start_all()
            try:
                yield
            finally:
                registry.stop_all()

    app.router.lifespan_context = module_lifespan


def wire_modules(
    app: Any,
    *,
    allow_list: Iterable[str] | None = None,
    ctx: AppContext | None = None,
) -> ModuleRegistry | None:
    """Discover, configure, and register enabled modules on the FastAPI ``app``."""
    if allow_list is None:
        allow_list = _allow_list_from_env()
    registry = _create_registry(allow_list)
    if len(registry) == 0:
        return None
    if ctx is None:
        ctx = AppContext(app=app)
    if not registry.wire_safe(ctx, app):
        return None
    _install_module_lifespan(app, registry)
    app.state.module_registry = registry
    logger.info("NOMAD modules wired: %s", ", ".join(registry.order))
    return registry
