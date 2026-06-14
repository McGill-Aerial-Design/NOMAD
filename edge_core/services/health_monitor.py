# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - Hardware Health Monitor

Provides system health monitoring for Jetson Orin Nano including:
- CPU/GPU temperature and load
- Memory usage
- Disk space
- Power draw
- Thermal throttling status

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from dataclasses import dataclass

from edge_core.core import BaseModule, ModuleMetadata

logger = logging.getLogger("edge_core.health_monitor")


@dataclass
class JetsonHealth:
    """Health metrics for Jetson Orin Nano."""

    # CPU metrics
    cpu_temp_c: float = 0.0
    cpu_load_pct: float = 0.0
    cpu_freq_mhz: float = 0.0

    # GPU metrics
    gpu_temp_c: float = 0.0
    gpu_load_pct: float = 0.0
    gpu_freq_mhz: float = 0.0

    # Memory
    memory_total_mb: float = 0.0
    memory_used_mb: float = 0.0
    memory_used_pct: float = 0.0

    # Disk
    disk_total_gb: float = 0.0
    disk_used_gb: float = 0.0
    disk_free_gb: float = 0.0
    disk_used_pct: float = 0.0

    # Power
    power_draw_w: float = 0.0
    power_budget_w: float = 0.0

    # Thermal status
    throttled: bool = False
    thermal_zone: str = "normal"
    fan_speed_pct: float = 0.0

    # Network (Tailscale)
    tailscale_connected: bool = False
    tailscale_ip: str | None = None
    network_latency_ms: float = 0.0

    # Overall status
    status: str = "unknown"

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "cpu_temp": self.cpu_temp_c,
            "cpu_load": self.cpu_load_pct,
            "cpu_freq": self.cpu_freq_mhz,
            "gpu_temp": self.gpu_temp_c,
            "gpu_load": self.gpu_load_pct,
            "gpu_freq": self.gpu_freq_mhz,
            "memory_total_mb": self.memory_total_mb,
            "memory_used_mb": self.memory_used_mb,
            "memory_used_pct": self.memory_used_pct,
            "disk_total_gb": self.disk_total_gb,
            "disk_used_gb": self.disk_used_gb,
            "disk_free_gb": self.disk_free_gb,
            "disk_used_pct": self.disk_used_pct,
            "power_draw_w": self.power_draw_w,
            "power_budget_w": self.power_budget_w,
            "throttled": self.throttled,
            "thermal_zone": self.thermal_zone,
            "fan_speed_pct": self.fan_speed_pct,
            "tailscale_connected": self.tailscale_connected,
            "tailscale_ip": self.tailscale_ip,
            "network_latency_ms": self.network_latency_ms,
            "status": self.status,
        }


class JetsonHealthMonitor:
    """
    Health monitor for Jetson Orin Nano.

    Polls system metrics at regular intervals and provides
    health status to the API.
    """

    # Thermal zone paths (Jetson Orin Nano)
    THERMAL_ZONES = {
        "cpu": "/sys/devices/virtual/thermal/thermal_zone0/temp",
        "gpu": "/sys/devices/virtual/thermal/thermal_zone1/temp",
        "board": "/sys/devices/virtual/thermal/thermal_zone2/temp",
    }

    def __init__(
        self,
        poll_interval: float = 5.0,
    ):
        self._poll_interval = poll_interval

        self._health = JetsonHealth()
        self._lock = threading.RLock()
        self._state_manager = None  # Set via set_state_manager()

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # EMA-smoothed GPU load to reduce jitter between poll cycles
        self._gpu_load_ema: float | None = None
        self._GPU_LOAD_EMA_ALPHA: float = 0.2

        # Cached results to avoid spawning subprocesses every poll cycle
        self._tailscale_cache: dict = {}
        self._tailscale_cache_ts: float = 0.0
        self._TAILSCALE_CACHE_TTL: float = 15.0

    @property
    def health(self) -> JetsonHealth:
        """Get current health metrics."""
        with self._lock:
            return self._health

    def set_state_manager(self, state_manager) -> None:
        """Set StateManager reference so health metrics are pushed to system state."""
        self._state_manager = state_manager

    def start(self) -> None:
        """Start health monitoring."""
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        logger.info("Jetson health monitor started")

    def stop(self) -> None:
        """Stop health monitoring."""
        self._stop_event.set()

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        logger.info("Jetson health monitor stopped")

    def _run(self) -> None:
        """Main monitoring loop."""
        while not self._stop_event.is_set():
            try:
                self._update_health()
            except Exception as e:
                logger.error(f"Health update error: {e}")

            time.sleep(self._poll_interval)

    def _update_health(self) -> None:
        """Update all health metrics."""
        health = JetsonHealth()

        # CPU/GPU temperatures
        health.cpu_temp_c = self._read_temperature("cpu")
        health.gpu_temp_c = self._read_temperature("gpu")

        # CPU load
        health.cpu_load_pct = self._get_cpu_load()

        # GPU load and frequency
        gpu_info = self._get_gpu_info()
        raw_gpu_load = gpu_info.get("load", 0.0)
        if self._gpu_load_ema is None:
            self._gpu_load_ema = raw_gpu_load
        else:
            self._gpu_load_ema = (
                self._GPU_LOAD_EMA_ALPHA * raw_gpu_load + (1.0 - self._GPU_LOAD_EMA_ALPHA) * self._gpu_load_ema
            )
        health.gpu_load_pct = round(self._gpu_load_ema, 1)
        health.gpu_freq_mhz = gpu_info.get("freq", 0.0)

        # Memory
        mem_info = self._get_memory_info()
        health.memory_total_mb = mem_info.get("total", 0.0)
        health.memory_used_mb = mem_info.get("used", 0.0)
        health.memory_used_pct = mem_info.get("used_pct", 0.0)

        # Disk
        disk_info = self._get_disk_info()
        health.disk_total_gb = disk_info.get("total", 0.0)
        health.disk_used_gb = disk_info.get("used", 0.0)
        health.disk_free_gb = disk_info.get("free", 0.0)
        health.disk_used_pct = disk_info.get("used_pct", 0.0)

        # Power
        power_info = self._get_power_info()
        health.power_draw_w = power_info.get("draw", 0.0)
        health.power_budget_w = power_info.get("budget", 15.0)  # Orin Nano default

        # Thermal status
        health.throttled = self._check_throttled()
        health.thermal_zone = self._get_thermal_zone(health.cpu_temp_c, health.gpu_temp_c)
        health.fan_speed_pct = self._get_fan_speed()

        # Tailscale status
        ts_info = self._get_tailscale_status()
        health.tailscale_connected = ts_info.get("connected", False)
        health.tailscale_ip = ts_info.get("ip")

        # Overall status
        health.status = self._compute_status(health)

        with self._lock:
            self._health = health

        # Push key metrics to StateManager so MAVLink health broadcast has data
        if self._state_manager:
            try:
                self._state_manager.update_state(
                    cpu_temp_c=health.cpu_temp_c,
                    gpu_temp_c=health.gpu_temp_c,
                    gpu_load_pct=health.gpu_load_pct,
                    power_draw_w=health.power_draw_w,
                    disk_free_gb=health.disk_free_gb,
                    memory_used_pct=health.memory_used_pct,
                    throttled=health.throttled,
                )
            except Exception as e:
                logger.debug(f"Failed to push health to state manager: {e}")

    def _read_temperature(self, zone: str) -> float:
        """Read temperature from thermal zone."""
        path = self.THERMAL_ZONES.get(zone)
        if not path or not os.path.exists(path):
            return 0.0

        try:
            with open(path) as f:
                # Temperature is in millidegrees
                return float(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def _get_cpu_load(self) -> float:
        """Get CPU load percentage."""
        try:
            # Use /proc/loadavg for 1-minute average
            with open("/proc/loadavg") as f:
                load = float(f.read().split()[0])
                # Get CPU count
                cpu_count = os.cpu_count() or 1
                return min(100.0, (load / cpu_count) * 100)
        except Exception:
            return 0.0

    def _get_gpu_info(self) -> dict:
        """Get GPU load and frequency from sysfs (Jetson-native, no subprocess).

        The Jetson GPU load sysfs file reports an instantaneous 0-1000 value
        (tenths of a percent). A single read often catches the GPU between
        work batches, returning 0 even when utilisation is high. Averaging
        multiple samples over a ~300 ms window produces a stable, representative
        reading.
        """
        result = {"load": 0.0, "freq": 0.0}

        # Jetson sysfs paths — Orin Nano first, then TX2/Xavier fallbacks.
        # Do NOT use nvidia-smi here: on Jetson it spawns a heavy process every call.
        load_candidates = [
            "/sys/devices/platform/bus@0/17000000.gpu/load",
            "/sys/devices/gpu.0/load",
        ]
        freq_candidates = [
            "/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq",
            "/sys/devices/gpu.0/devfreq/17000000.gp10b/cur_freq",
        ]

        try:
            for load_path in load_candidates:
                if os.path.exists(load_path):
                    samples = 15
                    total = 0.0
                    for _ in range(samples):
                        with open(load_path) as f:
                            total += float(f.read().strip())
                        time.sleep(0.02)
                    result["load"] = (total / samples) / 10.0
                    break

            for freq_path in freq_candidates:
                if os.path.exists(freq_path):
                    with open(freq_path) as f:
                        result["freq"] = float(f.read().strip()) / 1_000_000.0
                    break

        except Exception:
            pass

        return result

    def _get_memory_info(self) -> dict:
        """Get memory usage information."""
        result = {"total": 0.0, "used": 0.0, "used_pct": 0.0}

        try:
            with open("/proc/meminfo") as f:
                meminfo = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(":")
                        value = float(parts[1])
                        meminfo[key] = value

            total_kb = meminfo.get("MemTotal", 0)
            free_kb = meminfo.get("MemFree", 0)
            buffers_kb = meminfo.get("Buffers", 0)
            cached_kb = meminfo.get("Cached", 0)

            used_kb = total_kb - free_kb - buffers_kb - cached_kb

            result["total"] = total_kb / 1024
            result["used"] = used_kb / 1024
            result["used_pct"] = (used_kb / total_kb * 100) if total_kb > 0 else 0

        except Exception:
            pass

        return result

    def _get_disk_info(self, path: str = "/") -> dict:
        """Get disk usage information."""
        result = {"total": 0.0, "used": 0.0, "free": 0.0, "used_pct": 0.0}

        try:
            stat = os.statvfs(path)  # type: ignore[attr-defined]
            total = stat.f_blocks * stat.f_frsize
            free = stat.f_bavail * stat.f_frsize
            used = total - free

            result["total"] = total / (1024**3)
            result["used"] = used / (1024**3)
            result["free"] = free / (1024**3)
            result["used_pct"] = (used / total * 100) if total > 0 else 0

        except Exception:
            pass

        return result

    def _get_power_info(self) -> dict:
        """Get power consumption information."""
        result = {"draw": 0.0, "budget": 15.0}

        # Jetson Orin Nano power sensor via INA3221 hwmon interface
        # hwmon1 is typically the INA3221 sensor
        hwmon_paths = [
            "/sys/class/hwmon/hwmon1",
            "/sys/class/hwmon/hwmon2",
        ]

        for hwmon in hwmon_paths:
            # Check if this is the ina3221 sensor
            name_path = os.path.join(hwmon, "name")
            if os.path.exists(name_path):
                try:
                    with open(name_path) as f:
                        if "ina3221" not in f.read().strip().lower():
                            continue
                except Exception:
                    continue

            # Read channel 1 (VDD_IN) which is total system power
            # Power = Voltage (mV) × Current (mA) / 1,000,000 = Watts
            try:
                voltage_path = os.path.join(hwmon, "in1_input")
                current_path = os.path.join(hwmon, "curr1_input")

                if os.path.exists(voltage_path) and os.path.exists(current_path):
                    with open(voltage_path) as f:
                        voltage_mv = float(f.read().strip())
                    with open(current_path) as f:
                        current_ma = float(f.read().strip())

                    power_w = (voltage_mv * current_ma) / 1_000_000.0
                    result["draw"] = power_w
                    break

            except Exception as e:
                logger.debug(f"Power read error: {e}")
                continue

        return result

    def _check_throttled(self) -> bool:
        """Check if system is thermally throttled."""
        throttle_path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        max_freq_path = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"

        try:
            if os.path.exists(throttle_path) and os.path.exists(max_freq_path):
                with open(throttle_path) as f:
                    current = float(f.read().strip())
                with open(max_freq_path) as f:
                    max_freq = float(f.read().strip())
                # Throttled if current freq is less than 90% of max
                return current < (max_freq * 0.9)
        except Exception:
            pass

        return False

    def _get_thermal_zone(self, cpu_temp: float, gpu_temp: float) -> str:
        """Determine thermal zone based on temperatures."""
        max_temp = max(cpu_temp, gpu_temp)

        if max_temp >= 95:
            return "critical"
        elif max_temp >= 85:
            return "warning"
        elif max_temp >= 70:
            return "warm"
        else:
            return "normal"

    def _get_fan_speed(self) -> float:
        """Get PWM fan speed percentage."""
        pwm_paths = [
            "/sys/devices/pwm-fan/target_pwm",
            "/sys/class/hwmon/hwmon0/pwm1",
        ]

        for path in pwm_paths:
            if os.path.exists(path):
                try:
                    with open(path) as f:
                        pwm = float(f.read().strip())
                        return (pwm / 255.0) * 100
                except Exception:
                    pass

        return 0.0

    def _get_tailscale_status(self) -> dict:
        """Get Tailscale VPN status, cached for 15 s to avoid subprocess churn."""
        now = time.time()
        if now - self._tailscale_cache_ts < self._TAILSCALE_CACHE_TTL and self._tailscale_cache:
            return self._tailscale_cache

        result: dict = {"connected": False, "ip": None}
        try:
            output = subprocess.check_output(
                ["tailscale", "status", "--json"],
                timeout=3,
                stderr=subprocess.DEVNULL,
            ).decode()

            import json

            data = json.loads(output)
            result["connected"] = data.get("BackendState") == "Running"
            self_info = data.get("Self", {})
            ips = self_info.get("TailscaleIPs", [])
            if ips:
                result["ip"] = ips[0]

        except Exception:
            pass

        self._tailscale_cache = result
        self._tailscale_cache_ts = now
        return result

    def _compute_status(self, health: JetsonHealth) -> str:
        """Compute overall health status."""
        if health.thermal_zone == "critical":
            return "critical"
        elif health.thermal_zone == "warning" or health.throttled:
            return "warning"
        elif health.memory_used_pct > 90 or health.disk_used_pct > 95:
            return "warning"
        elif health.cpu_temp_c > 0:  # We're getting valid readings
            return "ok"
        else:
            return "unknown"


class HealthMonitorModule(BaseModule):
    """Bridges JetsonHealthMonitor into the modular SDK framework."""

    metadata = ModuleMetadata(
        name="health",
        version="1.0.0",
        description="Hardware health monitoring (CPU, GPU, memory, disk, power, thermal)",
        enable_flag="NOMAD_AUTOSTART_HEALTH_MONITOR",
        enabled_by_default=True,
    )

    def __init__(self) -> None:
        self._monitor: JetsonHealthMonitor | None = None

    def configure(self, ctx) -> None:
        state_mgr = ctx.require_service("state_manager")
        self._monitor = JetsonHealthMonitor()
        self._monitor.set_state_manager(state_mgr)
        ctx.register_service("health_monitor", self._monitor)
        ctx.app.state.health_monitor = self._monitor

    def start(self) -> None:
        if self._monitor:
            self._monitor.start()
            logger.info("Health monitor module started")

    def stop(self) -> None:
        if self._monitor:
            self._monitor.stop()
            logger.info("Health monitor module stopped")
