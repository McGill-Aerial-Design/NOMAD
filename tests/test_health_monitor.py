# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.health_monitor.

The monitor reads Jetson sysfs/proc nodes and shells out to ``tailscale``, none
of which exist in the test env. Each reader is exercised against a fake file
system (``open``/``os.path.exists`` scoped to the module) with ``os.statvfs``,
``subprocess.check_output`` and ``time.sleep`` stubbed, so the parsing and
status logic is covered without a Jetson.
"""

from __future__ import annotations

import io
import os
from types import SimpleNamespace

from edge_core.services import health_monitor as hm
from edge_core.services.health_monitor import (
    JetsonHealth,
    JetsonHealthMonitor,
)

ZONES = JetsonHealthMonitor.THERMAL_ZONES
GPU_LOAD = "/sys/devices/platform/bus@0/17000000.gpu/load"
GPU_FREQ = "/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq"
SCALING = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
MAXFREQ = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
FAN = "/sys/devices/pwm-fan/target_pwm"


class FakeFS:
    """A minimal path->content map standing in for the Jetson sysfs/proc tree."""

    def __init__(self, files: dict):
        self.files = files

    def exists(self, path):
        return path in self.files

    def open(self, path, *args, **kwargs):
        if path not in self.files:
            raise FileNotFoundError(path)
        content = self.files[path]
        if isinstance(content, Exception):
            raise content
        return io.StringIO(content)


def _install_fs(monkeypatch, files: dict) -> FakeFS:
    fs = FakeFS(files)
    real_exists = os.path.exists
    # Scope `open` to the module so only health_monitor's reads are faked.
    monkeypatch.setattr(hm, "open", fs.open, raising=False)
    # exists() delegates to the real one for anything we do not manage.
    monkeypatch.setattr(os.path, "exists", lambda p: p in fs.files or real_exists(p))
    monkeypatch.setattr(hm.time, "sleep", lambda *_: None)
    return fs


# --------------------------------------------------------------------------- #
# pure helpers
# --------------------------------------------------------------------------- #


def test_to_dict_carries_every_field():
    d = JetsonHealth(cpu_temp_c=42.0, status="ok", tailscale_ip="100.64.0.1").to_dict()
    assert d["cpu_temp"] == 42.0
    assert d["status"] == "ok"
    assert d["tailscale_ip"] == "100.64.0.1"
    assert set(d) >= {"cpu_temp", "gpu_load", "memory_used_pct", "throttled", "network_latency_ms"}


def test_thermal_zone_thresholds():
    m = JetsonHealthMonitor()
    assert m._get_thermal_zone(96, 0) == "critical"
    assert m._get_thermal_zone(0, 95) == "critical"
    assert m._get_thermal_zone(86, 0) == "warning"
    assert m._get_thermal_zone(72, 0) == "warm"
    assert m._get_thermal_zone(40, 50) == "normal"


def test_compute_status_precedence():
    m = JetsonHealthMonitor()
    assert m._compute_status(JetsonHealth(thermal_zone="critical")) == "critical"
    assert m._compute_status(JetsonHealth(thermal_zone="warning")) == "warning"
    assert m._compute_status(JetsonHealth(throttled=True)) == "warning"
    assert m._compute_status(JetsonHealth(memory_used_pct=95)) == "warning"
    assert m._compute_status(JetsonHealth(disk_used_pct=99)) == "warning"
    assert m._compute_status(JetsonHealth(cpu_temp_c=50)) == "ok"
    assert m._compute_status(JetsonHealth()) == "unknown"


# --------------------------------------------------------------------------- #
# individual readers
# --------------------------------------------------------------------------- #


def test_read_temperature_valid_missing_and_error(monkeypatch):
    _install_fs(monkeypatch, {ZONES["cpu"]: "45500", ZONES["board"]: ValueError("bad")})
    m = JetsonHealthMonitor()
    assert m._read_temperature("cpu") == 45.5
    assert m._read_temperature("gpu") == 0.0  # path not present
    assert m._read_temperature("board") == 0.0  # read raises -> 0.0
    assert m._read_temperature("nonexistent-zone") == 0.0


def test_get_cpu_load_scales_by_core_count(monkeypatch):
    _install_fs(monkeypatch, {"/proc/loadavg": "2.0 1.0 0.5 1/100 1234"})
    monkeypatch.setattr(hm.os, "cpu_count", lambda: 4)
    assert JetsonHealthMonitor()._get_cpu_load() == 50.0


def test_get_cpu_load_caps_at_100_and_handles_error(monkeypatch):
    _install_fs(monkeypatch, {"/proc/loadavg": "8.0 1 1 1/1 1"})
    monkeypatch.setattr(hm.os, "cpu_count", lambda: 2)
    assert JetsonHealthMonitor()._get_cpu_load() == 100.0
    _install_fs(monkeypatch, {})  # loadavg missing -> open raises -> 0.0
    assert JetsonHealthMonitor()._get_cpu_load() == 0.0


def test_get_gpu_info_averages_load_and_reads_freq(monkeypatch):
    _install_fs(monkeypatch, {GPU_LOAD: "500", GPU_FREQ: "918000000"})
    info = JetsonHealthMonitor()._get_gpu_info()
    assert info["load"] == 50.0  # 500/10 tenths-of-percent, averaged
    assert info["freq"] == 918.0  # Hz -> MHz


def test_get_gpu_info_defaults_when_absent(monkeypatch):
    _install_fs(monkeypatch, {})
    assert JetsonHealthMonitor()._get_gpu_info() == {"load": 0.0, "freq": 0.0}


def test_get_gpu_info_swallows_read_error(monkeypatch):
    # The load node exists but errors on read -> exception is swallowed, defaults kept.
    _install_fs(monkeypatch, {GPU_LOAD: ValueError("bad sample")})
    assert JetsonHealthMonitor()._get_gpu_info() == {"load": 0.0, "freq": 0.0}


def test_get_memory_info_parses_meminfo(monkeypatch):
    meminfo = "MemTotal: 8000000 kB\nMemFree: 2000000 kB\nBuffers: 1000000 kB\nCached: 1000000 kB\n"
    _install_fs(monkeypatch, {"/proc/meminfo": meminfo})
    info = JetsonHealthMonitor()._get_memory_info()
    assert info["total"] == 8000000 / 1024
    assert info["used"] == 4000000 / 1024  # total - free - buffers - cached
    assert info["used_pct"] == 50.0


def test_get_memory_info_handles_missing(monkeypatch):
    _install_fs(monkeypatch, {})
    assert JetsonHealthMonitor()._get_memory_info() == {"total": 0.0, "used": 0.0, "used_pct": 0.0}


def test_get_disk_info_from_statvfs(monkeypatch):
    monkeypatch.setattr(
        hm.os,
        "statvfs",
        lambda path: SimpleNamespace(f_blocks=1000, f_frsize=1024**3, f_bavail=400),
        raising=False,
    )
    info = JetsonHealthMonitor()._get_disk_info()
    assert info["total"] == 1000.0
    assert info["free"] == 400.0
    assert info["used"] == 600.0
    assert info["used_pct"] == 60.0


def test_get_disk_info_handles_error(monkeypatch):
    def boom(_):
        raise OSError("no statvfs")

    monkeypatch.setattr(hm.os, "statvfs", boom, raising=False)
    assert JetsonHealthMonitor()._get_disk_info() == {"total": 0.0, "used": 0.0, "free": 0.0, "used_pct": 0.0}


def _hwmon(n: int, leaf: str) -> str:
    # Match os.path.join's OS-native separator so the fake keys line up with the
    # paths _get_power_info builds at runtime (backslash on Windows, / on Linux).
    return os.path.join(f"/sys/class/hwmon/hwmon{n}", leaf)


def test_get_power_info_reads_ina3221(monkeypatch):
    _install_fs(
        monkeypatch,
        {
            _hwmon(1, "name"): "ina3221\n",
            _hwmon(1, "in1_input"): "5000",
            _hwmon(1, "curr1_input"): "2000",
        },
    )
    info = JetsonHealthMonitor()._get_power_info()
    assert info["draw"] == 10.0  # 5000 mV * 2000 mA / 1e6
    assert info["budget"] == 15.0


def test_get_power_info_skips_non_ina3221_sensor(monkeypatch):
    # hwmon1 is some other sensor; hwmon2 is the ina3221.
    _install_fs(
        monkeypatch,
        {
            _hwmon(1, "name"): "coretemp\n",
            _hwmon(2, "name"): "ina3221\n",
            _hwmon(2, "in1_input"): "4000",
            _hwmon(2, "curr1_input"): "1000",
        },
    )
    assert JetsonHealthMonitor()._get_power_info()["draw"] == 4.0


def test_get_power_info_skips_sensor_with_unreadable_name(monkeypatch):
    # hwmon1's name node errors on read -> continue; hwmon2 is the real sensor.
    _install_fs(
        monkeypatch,
        {
            _hwmon(1, "name"): OSError("name read failed"),
            _hwmon(2, "name"): "ina3221\n",
            _hwmon(2, "in1_input"): "3000",
            _hwmon(2, "curr1_input"): "1000",
        },
    )
    assert JetsonHealthMonitor()._get_power_info()["draw"] == 3.0


def test_get_power_info_swallows_value_read_error(monkeypatch):
    _install_fs(
        monkeypatch,
        {
            _hwmon(1, "name"): "ina3221\n",
            _hwmon(1, "in1_input"): ValueError("garbage"),
            _hwmon(1, "curr1_input"): "1000",
        },
    )
    assert JetsonHealthMonitor()._get_power_info()["draw"] == 0.0


def test_get_power_info_defaults_when_no_sensor(monkeypatch):
    _install_fs(monkeypatch, {})
    assert JetsonHealthMonitor()._get_power_info() == {"draw": 0.0, "budget": 15.0}


def test_check_throttled(monkeypatch):
    _install_fs(monkeypatch, {SCALING: "1000000", MAXFREQ: "1500000"})
    assert JetsonHealthMonitor()._check_throttled() is True  # 1.0 < 0.9*1.5
    _install_fs(monkeypatch, {SCALING: "1500000", MAXFREQ: "1500000"})
    assert JetsonHealthMonitor()._check_throttled() is False
    _install_fs(monkeypatch, {})  # paths absent
    assert JetsonHealthMonitor()._check_throttled() is False


def test_get_fan_speed(monkeypatch):
    _install_fs(monkeypatch, {FAN: "255"})
    assert JetsonHealthMonitor()._get_fan_speed() == 100.0
    _install_fs(monkeypatch, {})
    assert JetsonHealthMonitor()._get_fan_speed() == 0.0


# --------------------------------------------------------------------------- #
# tailscale status (subprocess + cache)
# --------------------------------------------------------------------------- #


def test_tailscale_status_running(monkeypatch):
    payload = '{"BackendState": "Running", "Self": {"TailscaleIPs": ["100.64.0.5", "fd7a::1"]}}'
    monkeypatch.setattr(hm.subprocess, "check_output", lambda *a, **k: payload.encode())
    info = JetsonHealthMonitor()._get_tailscale_status()
    assert info == {"connected": True, "ip": "100.64.0.5"}


def test_tailscale_status_is_cached(monkeypatch):
    calls = {"n": 0}

    def fake(*a, **k):
        calls["n"] += 1
        return b'{"BackendState": "Running", "Self": {"TailscaleIPs": ["100.64.0.5"]}}'

    monkeypatch.setattr(hm.subprocess, "check_output", fake)
    m = JetsonHealthMonitor()
    first = m._get_tailscale_status()
    second = m._get_tailscale_status()  # served from cache, no second subprocess
    assert first == second
    assert calls["n"] == 1


def test_tailscale_status_handles_subprocess_error(monkeypatch):
    def boom(*a, **k):
        raise OSError("tailscale missing")

    monkeypatch.setattr(hm.subprocess, "check_output", boom)
    assert JetsonHealthMonitor()._get_tailscale_status() == {"connected": False, "ip": None}


# --------------------------------------------------------------------------- #
# _update_health orchestration
# --------------------------------------------------------------------------- #


def _full_fs(monkeypatch):
    return _install_fs(
        monkeypatch,
        {
            ZONES["cpu"]: "60000",
            ZONES["gpu"]: "50000",
            "/proc/loadavg": "1.0 1 1 1/1 1",
            GPU_LOAD: "300",
            GPU_FREQ: "900000000",
            "/proc/meminfo": "MemTotal: 8000000 kB\nMemFree: 4000000 kB\nBuffers: 0 kB\nCached: 0 kB\n",
            SCALING: "1500000",
            MAXFREQ: "1500000",
            FAN: "127",
            os.path.join("/sys/class/hwmon/hwmon1", "name"): "ina3221\n",
            os.path.join("/sys/class/hwmon/hwmon1", "in1_input"): "5000",
            os.path.join("/sys/class/hwmon/hwmon1", "curr1_input"): "2000",
        },
    )


def test_update_health_populates_and_pushes_state(monkeypatch):
    _full_fs(monkeypatch)
    monkeypatch.setattr(hm.os, "cpu_count", lambda: 4)
    monkeypatch.setattr(
        hm.os, "statvfs", lambda p: SimpleNamespace(f_blocks=100, f_frsize=1024**3, f_bavail=90), raising=False
    )
    monkeypatch.setattr(hm.subprocess, "check_output", lambda *a, **k: b'{"BackendState": "Stopped"}')

    pushed = {}
    m = JetsonHealthMonitor()
    m.set_state_manager(SimpleNamespace(update_state=lambda **kw: pushed.update(kw)))
    m._update_health()

    h = m.health
    assert h.cpu_temp_c == 60.0
    assert h.gpu_temp_c == 50.0
    assert h.gpu_load_pct == 30.0
    assert h.power_draw_w == 10.0
    assert h.status == "ok"
    assert pushed["cpu_temp_c"] == 60.0 and pushed["throttled"] is False


def test_update_health_smooths_gpu_load_with_ema(monkeypatch):
    _full_fs(monkeypatch)
    m = JetsonHealthMonitor()
    m._update_health()  # first sample seeds the EMA
    assert m._gpu_load_ema == 30.0
    m._update_health()  # second sample exercises the EMA blend branch
    assert m._gpu_load_ema == 30.0


def test_update_health_survives_state_manager_error(monkeypatch):
    _full_fs(monkeypatch)
    monkeypatch.setattr(hm.subprocess, "check_output", lambda *a, **k: b"{}")

    def boom(**kw):
        raise RuntimeError("state push failed")

    m = JetsonHealthMonitor()
    m.set_state_manager(SimpleNamespace(update_state=boom))
    m._update_health()  # exception is swallowed; health still updated
    assert m.health.cpu_temp_c == 60.0


def test_health_all_zero_reads_unknown_status(monkeypatch):
    # Exercises the real production path: the monitoring loop calls
    # _update_health() and the API serves monitor.health.to_dict(). With nothing
    # present, every metric reads zero and the status classifies as "unknown".
    _install_fs(monkeypatch, {})
    monkeypatch.setattr(hm.subprocess, "check_output", lambda *a, **k: (_ for _ in ()).throw(OSError()))
    monitor = JetsonHealthMonitor()
    monitor._update_health()
    d = monitor.health.to_dict()
    assert d["status"] == "unknown"
    assert d["cpu_temp"] == 0.0


# --------------------------------------------------------------------------- #
# lifecycle + module
# --------------------------------------------------------------------------- #


def test_health_property_and_set_state_manager():
    m = JetsonHealthMonitor()
    assert isinstance(m.health, JetsonHealth)
    sentinel = object()
    m.set_state_manager(sentinel)
    assert m._state_manager is sentinel


def test_start_runs_loop_then_stop(monkeypatch):
    monkeypatch.setattr(hm.time, "sleep", lambda *_: None)
    m = JetsonHealthMonitor(poll_interval=0.0)
    calls = {"n": 0}

    def fake_update():
        calls["n"] += 1
        m._stop_event.set()

    monkeypatch.setattr(m, "_update_health", fake_update)
    m.start()
    m._thread.join(timeout=2.0)
    m.stop()
    assert calls["n"] >= 1


def test_run_swallows_update_errors(monkeypatch):
    monkeypatch.setattr(hm.time, "sleep", lambda *_: None)
    m = JetsonHealthMonitor()
    calls = {"n": 0}

    def fake_update():
        calls["n"] += 1
        m._stop_event.set()
        raise RuntimeError("update blew up")

    monkeypatch.setattr(m, "_update_health", fake_update)
    m._run()  # the error is caught; the loop exits on the stop event
    assert calls["n"] == 1


def test_start_idempotent_when_already_alive(monkeypatch):
    m = JetsonHealthMonitor()
    sentinel = SimpleNamespace(is_alive=lambda: True)
    m._thread = sentinel
    m.start()
    assert m._thread is sentinel


def test_stop_without_thread_is_noop():
    JetsonHealthMonitor().stop()  # must not raise when never started


def test_health_module_configure_start_stop(monkeypatch):
    monkeypatch.setattr(hm.JetsonHealthMonitor, "_run", lambda self: None)
    registered = {}
    app = SimpleNamespace(state=SimpleNamespace())
    ctx = SimpleNamespace(
        app=app,
        require_service=lambda name: SimpleNamespace(update_state=lambda **kw: None),
        register_service=lambda name, svc: registered.__setitem__(name, svc),
    )

    module = hm.HealthMonitorModule()
    module.configure(ctx)
    assert "health_monitor" in registered
    assert app.state.health_monitor is registered["health_monitor"]

    module.start()
    module.stop()
