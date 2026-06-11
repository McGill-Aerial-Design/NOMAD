# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Behavior specs for the NOMAD module SDK (edge_core.core).

These tests double as executable documentation for module authors. They run on
the hardware-free sim path (no Jetson/CUDA/ZED/ROS).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from edge_core.core import (
    AppContext,
    BaseModule,
    ModuleMetadata,
    ModuleRegistry,
    wire_modules,
)
from edge_core.core.registry import ModuleError

REPO_ROOT = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def make_module(
    name: str,
    *,
    requires: tuple[str, ...] = (),
    enable_flag: str | None = None,
    enabled_by_default: bool = True,
    log: list[tuple[str, str]] | None = None,
) -> BaseModule:
    class _Module(BaseModule):
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

    return _Module()


class _FakeEntryPoint:
    def __init__(self, name: str, target: object) -> None:
        self.name = name
        self._target = target

    def load(self) -> object:
        if isinstance(self._target, Exception):
            raise self._target
        return self._target


# --------------------------------------------------------------------------- #
# registration & discovery
# --------------------------------------------------------------------------- #
def test_register_and_len():
    reg = ModuleRegistry()
    assert reg.register(make_module("a")) == "a"
    assert len(reg) == 1


def test_duplicate_name_rejected():
    reg = ModuleRegistry()
    reg.register(make_module("a"))
    with pytest.raises(ModuleError):
        reg.register(make_module("a"))


def test_module_without_metadata_rejected():
    reg = ModuleRegistry()
    with pytest.raises(ModuleError):
        reg.register(object())


def test_discover_entry_points_handles_class_factory_and_bad_ep(caplog):
    class ModA(BaseModule):
        metadata = ModuleMetadata(name="a")

    def factory_b():
        return make_module("b")

    eps = [
        _FakeEntryPoint("a", ModA),  # class -> instantiated
        _FakeEntryPoint("b", factory_b),  # factory -> called
        _FakeEntryPoint("bad", RuntimeError("boom")),  # bad -> skipped, logged
    ]
    reg = ModuleRegistry()
    names = reg.discover_entry_points(loader=lambda group: eps)
    assert set(names) == {"a", "b"}
    assert len(reg) == 2  # the bad one was skipped, not fatal


def test_load_specs_loads_sample_module():
    sys.path.insert(0, str(REPO_ROOT / "examples"))
    try:
        reg = ModuleRegistry()
        reg.load_specs(["sample_module.sample_module:SampleModule"])
        assert "sample" in reg.modules
    finally:
        sys.path.remove(str(REPO_ROOT / "examples"))


# --------------------------------------------------------------------------- #
# dependency resolution
# --------------------------------------------------------------------------- #
def test_resolve_order_topological():
    reg = ModuleRegistry()
    reg.register(make_module("a", requires=("b",)))
    reg.register(make_module("b", requires=("c",)))
    reg.register(make_module("c"))
    order = reg.resolve_order(AppContext())
    assert order.index("c") < order.index("b") < order.index("a")


def test_missing_dependency_raises():
    reg = ModuleRegistry()
    reg.register(make_module("a", requires=("missing",)))
    with pytest.raises(ModuleError):
        reg.resolve_order(AppContext())


def test_dependency_cycle_raises():
    reg = ModuleRegistry()
    reg.register(make_module("a", requires=("b",)))
    reg.register(make_module("b", requires=("a",)))
    with pytest.raises(ModuleError):
        reg.resolve_order(AppContext())


# --------------------------------------------------------------------------- #
# enable / disable
# --------------------------------------------------------------------------- #
def test_allow_list_filters_modules():
    reg = ModuleRegistry(allow_list=["a"])
    reg.register(make_module("a"))
    reg.register(make_module("b"))
    assert reg.resolve_order(AppContext()) == ["a"]


def test_enable_flag_opt_out_default_on():
    reg = ModuleRegistry()
    reg.register(make_module("a", enable_flag="NOMAD_ENABLE_A", enabled_by_default=True))
    # flag unset -> enabled
    assert reg.resolve_order(AppContext(config={})) == ["a"]
    # flag false -> disabled
    assert reg.resolve_order(AppContext(config={"NOMAD_ENABLE_A": "false"})) == []


def test_enable_flag_opt_in_default_off():
    reg = ModuleRegistry()
    reg.register(make_module("a", enable_flag="NOMAD_ENABLE_A", enabled_by_default=False))
    assert reg.resolve_order(AppContext(config={})) == []
    assert reg.resolve_order(AppContext(config={"NOMAD_ENABLE_A": "true"})) == ["a"]


# --------------------------------------------------------------------------- #
# lifecycle ordering
# --------------------------------------------------------------------------- #
def test_lifecycle_runs_in_dependency_order_and_stops_reversed():
    log: list[tuple[str, str]] = []
    reg = ModuleRegistry()
    reg.register(make_module("a", requires=("b",), log=log))
    reg.register(make_module("b", log=log))
    ctx = AppContext()
    reg.wire_safe(ctx, app=None)
    reg.start_all()
    reg.stop_all()
    starts = [name for action, name in log if action == "start"]
    stops = [name for action, name in log if action == "stop"]
    assert starts == ["b", "a"]
    assert stops == ["a", "b"]  # reverse of start


# --------------------------------------------------------------------------- #
# AppContext
# --------------------------------------------------------------------------- #
def test_context_service_and_config():
    ctx = AppContext(services={"state": object()}, config={"NOMAD_X": "yes"})
    assert ctx.get_service("state") is not None
    assert ctx.require_service("state") is not None
    with pytest.raises(KeyError):
        ctx.require_service("missing")
    assert ctx.is_enabled("NOMAD_X") is True
    assert ctx.is_enabled("NOMAD_MISSING", default=True) is True


def test_context_falls_back_to_app_state():
    class _State:
        mavlink = object()

    class _App:
        state = _State()

    ctx = AppContext(app=_App())
    assert ctx.get_service("mavlink") is _App.state.mavlink


# --------------------------------------------------------------------------- #
# end-to-end: wire the sample module into a real FastAPI app
# --------------------------------------------------------------------------- #
def test_wire_sample_module_into_fastapi_app(monkeypatch):
    fastapi = pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    sys.path.insert(0, str(REPO_ROOT / "examples"))
    try:
        # Feed the sample module through the production discovery path.
        class _FakeEntryPoint:
            name = "sample"

            @staticmethod
            def load():
                from sample_module.sample_module import SampleModule

                return SampleModule

        import edge_core.core.registry as registry_mod

        monkeypatch.setattr(registry_mod, "_default_entry_point_loader", lambda group: [_FakeEntryPoint])

        app = fastapi.FastAPI()
        registry = wire_modules(app)
        assert registry is not None and "sample" in registry.modules

        # TestClient as a context manager fires startup/shutdown -> start()/stop().
        with TestClient(app) as client:
            resp = client.get("/api/sample/ping")
            assert resp.status_code == 200
            body = resp.json()
            assert body["module"] == "sample"
            assert body["started"] is True
            assert body["reply"] == "pong"
    finally:
        sys.path.remove(str(REPO_ROOT / "examples"))


def test_wire_modules_noop_when_empty():
    fastapi = pytest.importorskip("fastapi")
    app = fastapi.FastAPI()
    # No specs, no installed entry points in the test env -> no-op.
    assert wire_modules(app, allow_list=["nonexistent"]) is None
