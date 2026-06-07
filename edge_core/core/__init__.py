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

See ``examples/sample_module`` for a minimal, runnable example.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Iterable
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


def wire_modules(
    app: Any,
    *,
    allow_list: Iterable[str] | None = None,
    specs: Iterable[str] | None = None,
    ctx: AppContext | None = None,
) -> ModuleRegistry | None:
    """Discover, configure, and register enabled modules on the FastAPI ``app``.

    Additive and safe: when no modules are installed/enabled (today's default),
    this is a no-op and returns ``None`` without touching the app. Otherwise it
    wires routes immediately and registers startup/shutdown hooks that drive the
    modules' ``start()``/``stop()`` lifecycle.
    """
    if allow_list is None:
        allow_list = _allow_list_from_env()

    registry = ModuleRegistry(allow_list=list(allow_list) if allow_list is not None else None)
    try:
        registry.discover_entry_points()
    except Exception as exc:  # noqa: BLE001 - never let discovery crash startup
        logger.error("module discovery failed: %s", exc)
    if specs:
        try:
            registry.load_specs(specs)
        except Exception as exc:  # noqa: BLE001
            logger.error("module spec loading failed: %s", exc)

    if len(registry) == 0:
        return None

    if ctx is None:
        ctx = AppContext(app=app)

    enabled = registry.resolve_order(ctx)
    if not enabled:
        return None

    wired: list[str] = []
    for name in enabled:
        try:
            registry._modules[name].configure(ctx)
            registry._modules[name].register_routes(app)
            wired.append(name)
        except Exception as exc:  # noqa: BLE001
            logger.error("module %s wiring failed: %s", name, exc)

    if not wired:
        return None

    registry._order = wired

    @app.on_event("startup")
    async def _nomad_start_modules() -> None:  # pragma: no cover - exercised at runtime
        registry.start_all()

    @app.on_event("shutdown")
    async def _nomad_stop_modules() -> None:  # pragma: no cover - exercised at runtime
        registry.stop_all()

    app.state.module_registry = registry
    logger.info("NOMAD modules wired: %s", ", ".join(registry.order))
    return registry
