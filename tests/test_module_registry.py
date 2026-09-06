# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Transition coverage for the current Python module registry.

The registry is transitional and will be removed with Edge Core. These tests
protect current startup behavior until the C++ cutover is complete.
"""

from __future__ import annotations

import pytest

from edge_core.core import AppContext, BaseModule, ModuleMetadata, ModuleRegistry, wire_modules
from edge_core.core.registry import ModuleError


def make_module(
    name: str,
    *,
    requires: tuple[str, ...] = (),
    enable_flag: str | None = None,
    enabled_by_default: bool = True,
    log: list[tuple[str, str]] | None = None,
) -> BaseModule:
    class TestModule(BaseModule):
        metadata = ModuleMetadata(
            name=name,
            requires=requires,
            enable_flag=enable_flag,
            enabled_by_default=enabled_by_default,
        )

        def configure(self, ctx: AppContext) -> None:
            if log is not None:
                log.append(("configure", name))

        def start(self) -> None:
            if log is not None:
                log.append(("start", name))

        def stop(self) -> None:
            if log is not None:
                log.append(("stop", name))

    return TestModule()


class FakeEntryPoint:
    def __init__(self, name: str, target: object) -> None:
        self.name = name
        self.target = target

    def load(self) -> object:
        if isinstance(self.target, Exception):
            raise self.target
        return self.target


def test_register_rejects_duplicates_and_missing_metadata() -> None:
    registry = ModuleRegistry()
    registry.register(make_module("a"))
    with pytest.raises(ModuleError):
        registry.register(make_module("a"))
    with pytest.raises(ModuleError):
        registry.register(object())


def test_discover_entry_points_skips_bad_modules(caplog) -> None:
    class ModuleA(BaseModule):
        metadata = ModuleMetadata(name="a")

    def make_b() -> BaseModule:
        return make_module("b")

    entries = [
        FakeEntryPoint("a", ModuleA),
        FakeEntryPoint("b", make_b),
        FakeEntryPoint("bad", RuntimeError("boom")),
    ]
    registry = ModuleRegistry()

    names = registry.discover_entry_points(loader=lambda group: entries)

    assert set(names) == {"a", "b"}
    assert len(registry) == 2


def test_resolve_order_handles_dependencies_and_cycles() -> None:
    registry = ModuleRegistry()
    registry.register(make_module("a", requires=("b",)))
    registry.register(make_module("b", requires=("c",)))
    registry.register(make_module("c"))

    order = registry.resolve_order(AppContext())

    assert order.index("c") < order.index("b") < order.index("a")

    cycle = ModuleRegistry()
    cycle.register(make_module("a", requires=("b",)))
    cycle.register(make_module("b", requires=("a",)))
    with pytest.raises(ModuleError):
        cycle.resolve_order(AppContext())


def test_resolve_order_filters_allow_list_and_enable_flags() -> None:
    registry = ModuleRegistry(allow_list=["a"])
    registry.register(make_module("a", enable_flag="NOMAD_ENABLE_A"))
    registry.register(make_module("b"))

    assert registry.resolve_order(AppContext(config={})) == ["a"]
    assert registry.resolve_order(AppContext(config={"NOMAD_ENABLE_A": "false"})) == []


def test_lifecycle_starts_in_order_and_stops_in_reverse() -> None:
    log: list[tuple[str, str]] = []
    registry = ModuleRegistry()
    registry.register(make_module("a", requires=("b",), log=log))
    registry.register(make_module("b", log=log))
    context = AppContext()

    registry.wire_safe(context, app=None)
    registry.start_all()
    registry.stop_all()

    assert [name for action, name in log if action == "start"] == ["b", "a"]
    assert [name for action, name in log if action == "stop"] == ["a", "b"]


def test_context_reads_services_and_config() -> None:
    service = object()
    context = AppContext(services={"state": service}, config={"NOMAD_X": "yes"})

    assert context.get_service("state") is service
    assert context.require_service("state") is service
    assert context.is_enabled("NOMAD_X")
    with pytest.raises(KeyError):
        context.require_service("missing")


def test_context_reads_app_state() -> None:
    class State:
        mavlink = object()

    class App:
        state = State()

    assert AppContext(app=App()).get_service("mavlink") is App.state.mavlink


def test_wire_modules_is_noop_when_no_module_is_available() -> None:
    fastapi = pytest.importorskip("fastapi")
    app = fastapi.FastAPI()

    assert wire_modules(app, allow_list=["not-installed"]) is None
