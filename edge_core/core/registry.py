# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ModuleRegistry — discover, order, and drive the lifecycle of NOMAD modules.

Discovery sources (any combination):

* the ``nomad.modules`` entry-point group (installed packages), via
  :meth:`ModuleRegistry.discover_entry_points`;
* explicit ``"package.module:attr"`` specs (config / tests), via
  :meth:`ModuleRegistry.load_specs`;
* direct registration of an instance, via :meth:`ModuleRegistry.register`.

Enabling is controlled by an optional allow-list (e.g. from a ``NOMAD_MODULES``
config entry) and per-module ``enable_flag`` env flags. Dependencies declared in
``ModuleMetadata.requires`` are resolved into a start order with a topological
sort; ``stop`` runs in reverse.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Iterable, Sequence
from importlib import import_module
from importlib import metadata as importlib_metadata
from typing import Any

from .context import AppContext
from .module import ModuleMetadata, NomadModule

logger = logging.getLogger(__name__)

ENTRY_POINT_GROUP = "nomad.modules"


class ModuleError(Exception):
    """Raised for duplicate names, missing dependencies, or dependency cycles."""


def _metadata(module: Any) -> ModuleMetadata:
    meta = getattr(module, "metadata", None)
    if not isinstance(meta, ModuleMetadata):
        raise ModuleError(f"module {module!r} has no ModuleMetadata `metadata` attribute")
    return meta


def _instantiate(obj: Any) -> Any:
    """Turn an entry-point/spec target into a module instance.

    Accepts a class (instantiated), a factory callable (called), or an
    already-built instance (used as-is).
    """
    if isinstance(obj, type):
        return obj()
    if callable(obj):
        return obj()
    return obj


def _default_entry_point_loader(group: str) -> Sequence[Any]:
    return list(importlib_metadata.entry_points(group=group))


class ModuleRegistry:
    def __init__(
        self,
        *,
        entry_point_group: str = ENTRY_POINT_GROUP,
        allow_list: Iterable[str] | None = None,
    ) -> None:
        self._group = entry_point_group
        # None => enable every discovered module (still subject to enable_flag).
        self._allow_list: set[str] | None = set(allow_list) if allow_list is not None else None
        self._modules: dict[str, NomadModule] = {}
        self._order: list[str] = []
        self._started: list[str] = []

    # -- introspection ------------------------------------------------------
    @property
    def modules(self) -> dict[str, NomadModule]:
        return dict(self._modules)

    @property
    def order(self) -> list[str]:
        return list(self._order)

    def __len__(self) -> int:
        return len(self._modules)

    # -- discovery ----------------------------------------------------------
    def register(self, module: NomadModule) -> str:
        """Register a module instance. Returns its name."""
        meta = _metadata(module)
        if meta.name in self._modules:
            raise ModuleError(f"duplicate module name: {meta.name!r}")
        self._modules[meta.name] = module
        return meta.name

    def discover_entry_points(self, *, loader: Callable[[str], Sequence[Any]] | None = None) -> list[str]:
        """Discover modules from the ``nomad.modules`` entry-point group.

        A bad entry point is logged and skipped rather than failing the whole app.
        """
        load = loader or _default_entry_point_loader
        names: list[str] = []
        for ep in load(self._group):
            try:
                target = ep.load()
                module = _instantiate(target)
                names.append(self.register(module))
            except Exception as exc:  # noqa: BLE001 - isolate one bad module
                ep_name = getattr(ep, "name", repr(ep))
                logger.error("skipping module entry point %s: %s", ep_name, exc)
        return names

    def load_specs(self, specs: Iterable[str]) -> list[str]:
        """Load modules from ``"package.module:attr"`` specs."""
        names: list[str] = []
        for spec in specs:
            mod_path, _, attr = spec.partition(":")
            obj: Any = import_module(mod_path)
            if attr:
                obj = getattr(obj, attr)
            names.append(self.register(_instantiate(obj)))
        return names

    # -- resolution ---------------------------------------------------------
    def _is_enabled(self, module: NomadModule, ctx: AppContext) -> bool:
        meta = _metadata(module)
        if self._allow_list is not None and meta.name not in self._allow_list:
            return False
        if meta.enable_flag is not None and not ctx.is_enabled(meta.enable_flag, default=meta.enabled_by_default):
            return False
        return True

    def resolve_order(self, ctx: AppContext) -> list[str]:
        """Return the enabled modules in dependency (topological) order."""
        enabled = {name: mod for name, mod in self._modules.items() if self._is_enabled(mod, ctx)}
        order: list[str] = []
        visiting: set[str] = set()
        done: set[str] = set()

        def visit(name: str, chain: tuple[str, ...]) -> None:
            if name in done:
                return
            if name in visiting:
                cycle = " -> ".join((*chain, name))
                raise ModuleError(f"dependency cycle: {cycle}")
            if name not in enabled:
                raise ModuleError(
                    f"module {chain[-1]!r} requires {name!r}, which is missing or disabled"
                    if chain
                    else f"module {name!r} is missing or disabled"
                )
            visiting.add(name)
            for dep in _metadata(enabled[name]).requires:
                visit(dep, (*chain, name))
            visiting.discard(name)
            done.add(name)
            order.append(name)

        for name in enabled:
            visit(name, ())
        self._order = order
        return order

    # -- lifecycle ----------------------------------------------------------
    def configure_all(self, ctx: AppContext) -> None:
        for name in self.resolve_order(ctx):
            self._modules[name].configure(ctx)

    def register_routes(self, app: Any) -> None:
        for name in self._order:
            self._modules[name].register_routes(app)

    def start_all(self) -> None:
        for name in self._order:
            self._modules[name].start()
            self._started.append(name)

    def stop_all(self) -> None:
        for name in reversed(self._started):
            try:
                self._modules[name].stop()
            except Exception as exc:  # noqa: BLE001 - one module must not block others
                logger.error("error stopping module %s: %s", name, exc)
        self._started.clear()

    def wire_safe(self, ctx: AppContext, app: Any) -> list[str]:
        """Resolve order, then configure + register routes with fault isolation.

        Unlike :meth:`configure_all` + :meth:`register_routes`, a module that
        raises during ``configure``/``register_routes`` is logged and skipped
        rather than aborting the rest. ``self.order`` is updated to the modules
        that wired successfully, and that list is returned. This is the single
        source of truth used by :func:`edge_core.core.wire_modules`.
        """
        wired: list[str] = []
        for name in self.resolve_order(ctx):
            try:
                self._modules[name].configure(ctx)
                self._modules[name].register_routes(app)
                wired.append(name)
            except Exception as exc:  # noqa: BLE001 - isolate one bad module
                logger.error("module %s wiring failed: %s", name, exc)
        self._order = wired
        return wired
