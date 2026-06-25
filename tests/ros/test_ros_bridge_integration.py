# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Self-contained integration test for the NOMAD ROS-HTTP bridge.

Runs INSIDE the ``nomad-sim-ros:latest`` container where rclpy, the bridge, and
the synthetic publisher are all available.  A lightweight in-process HTTP stub
replaces Edge Core so no external services are required.

Launch via::

    pixi run test-ros-integration
    # which runs: docker run --rm -v $PWD/tests:/opt/nomad/tests \\
    #             nomad-sim-ros:latest python3 -m pytest tests/ros/ -v

What is exercised end-to-end:
  1. ``ZedSimPublisher`` publishes Odometry / IMU / MagneticField / Marker /
     Float32 topics over in-process DDS.
  2. ``ROSHTTPBridge`` receives those topics, converts them, and POSTs to the
     stub HTTP server (replaces Edge Core).
  3. The stub records every request so the test can assert on counts, values,
     and motion.

Assertions:
  - At least 3 VIO (POST /api/vio/update) updates received.
  - The last VIO update has ``confidence > 0.5``.
  - The VIO x *or* y position differs between the first and last update
    (proves the simulated vehicle is moving).
  - At least 1 mesh update (POST /api/slam/mesh/update) received.
  - At least 1 servo update (POST /api/servo/camera/tilt) received.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import pytest

# Skip cleanly when rclpy is absent (normal pixi run test on the host — no
# ROS2 installed).  This line MUST come before any rclpy import.
rclpy = pytest.importorskip("rclpy")

# ---------------------------------------------------------------------------
# Stub HTTP server
# ---------------------------------------------------------------------------


class _StubState:
    """Thread-safe store for captured HTTP requests."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.vio_updates: list[dict] = []
        self.mesh_count: int = 0
        self.servo_count: int = 0

    def record_vio(self, body: bytes) -> None:
        try:
            data = json.loads(body.decode("utf-8"))
        except Exception:
            data = {}
        with self._lock:
            self.vio_updates.append(data)

    def record_mesh(self) -> None:
        with self._lock:
            self.mesh_count += 1

    def record_servo(self) -> None:
        with self._lock:
            self.servo_count += 1

    def snapshot(self) -> tuple[list[dict], int, int]:
        with self._lock:
            return list(self.vio_updates), self.mesh_count, self.servo_count


def _make_handler(state: _StubState):
    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:  # silence access log
            pass

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/api/servo/camera/tilt":
                body = json.dumps({"angle": 90.0}).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length > 0 else b""

            parsed = urlparse(self.path)
            path = parsed.path

            if path == "/api/vio/update":
                state.record_vio(raw)
            elif path == "/api/slam/mesh/update":
                state.record_mesh()
            elif path == "/api/servo/camera/tilt":
                state.record_servo()

            body = json.dumps({"success": True}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return _Handler


# ---------------------------------------------------------------------------
# Pytest fixture: stub server + bridge + publisher lifecycle
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def ros_bridge_session():
    """Start the stub server, rclpy nodes, spin for 10 s, then tear down.

    Generous spin time (10 s) accounts for DDS peer-discovery (can take 1-3 s)
    plus several send intervals (bridge sends VIO at 30 Hz, mesh via background
    thread).  Total wall-clock budget is capped at ~15 s via the fixture.
    """
    from rclpy.executors import MultiThreadedExecutor

    from edge_core.ros_http_bridge.node import ROSHTTPBridge
    from tools.sim.zed_sim_publisher import ZedSimPublisher

    # -- stub HTTP server on an ephemeral port --------------------------------
    state = _StubState()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler(state))
    stub_port = httpd.server_address[1]
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    # -- rclpy init -----------------------------------------------------------
    rclpy.init()

    publisher: ZedSimPublisher | None = None
    bridge: ROSHTTPBridge | None = None
    executor = None

    try:
        publisher = ZedSimPublisher(rate_hz=30.0)
        bridge = ROSHTTPBridge(
            host="127.0.0.1",
            port=stub_port,
            vio_topic="/zed/zed_node/odom",
            imu_topic="/zed/zed_node/imu/data",
            mag_topic="/zed/zed_node/imu/mag",
            cmd_vel_topic="/cmd_vel",
            mesh_topic="/nvblox_node/color_layer_marker",
            servo_topic="/nomad/servo/nozzle_angle",
            send_rate_hz=30.0,
            enable_nav_control=False,  # avoids pymavlink MAVLink controller
            enable_mesh=True,
            enable_servo=True,
            use_imu_attitude=True,
            use_mag_heading=True,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(bridge)

        # Spin in a background thread for 10 s so DDS discovery and multiple
        # send intervals complete before the test assertions run.
        spin_stop = threading.Event()

        def _spin():
            deadline = time.monotonic() + 10.0
            while not spin_stop.is_set() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)

        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()
        spin_thread.join(timeout=12.0)  # wall-clock cap

        yield state

    finally:
        spin_stop.set()
        if bridge is not None:
            bridge.destroy_node()
        if publisher is not None:
            publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        httpd.shutdown()


# ---------------------------------------------------------------------------
# Tests — each assertion is a separate function for clear failure messages
# ---------------------------------------------------------------------------


def test_vio_updates_received(ros_bridge_session: _StubState) -> None:
    """At least 3 VIO POSTs must arrive at the stub server."""
    vio_updates, _, _ = ros_bridge_session.snapshot()
    assert len(vio_updates) >= 3, (
        f"Expected >= 3 VIO updates, got {len(vio_updates)}. Check DDS discovery or spin duration."
    )


def test_vio_confidence_above_threshold(ros_bridge_session: _StubState) -> None:
    """The last VIO update must carry confidence > 0.5."""
    vio_updates, _, _ = ros_bridge_session.snapshot()
    assert vio_updates, "No VIO updates captured — see test_vio_updates_received"
    last = vio_updates[-1]
    confidence = last.get("confidence", 0.0)
    assert confidence > 0.5, (
        f"VIO confidence {confidence:.4f} <= 0.5. Check ZedSimPublisher covariance or bridge vio_confidence()."
    )


def test_vio_position_changes(ros_bridge_session: _StubState) -> None:
    """The x or y position must differ between the first and last VIO update."""
    vio_updates, _, _ = ros_bridge_session.snapshot()
    assert len(vio_updates) >= 2, f"Need >= 2 VIO updates to compare motion, got {len(vio_updates)}"
    first = vio_updates[0]
    last = vio_updates[-1]
    x_moved = abs(last.get("x", 0.0) - first.get("x", 0.0)) > 1e-6
    y_moved = abs(last.get("y", 0.0) - first.get("y", 0.0)) > 1e-6
    assert x_moved or y_moved, (
        f"VIO position did not change: first={first!r}, last={last!r}. "
        "The publisher flies a circle — x or y must differ."
    )


def test_mesh_update_received(ros_bridge_session: _StubState) -> None:
    """At least 1 POST to /api/slam/mesh/update must arrive."""
    _, mesh_count, _ = ros_bridge_session.snapshot()
    assert mesh_count >= 1, (
        f"Expected >= 1 mesh update, got {mesh_count}. "
        "Check enable_mesh=True and that Marker messages are being published."
    )


def test_servo_update_received(ros_bridge_session: _StubState) -> None:
    """At least 1 POST to /api/servo/camera/tilt must arrive."""
    _, _, servo_count = ros_bridge_session.snapshot()
    assert servo_count >= 1, (
        f"Expected >= 1 servo update, got {servo_count}. "
        "Check enable_servo=True and that Float32 messages are being published."
    )


def test_nav_controller_starts_when_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    """The ROS bridge must open its MAVLink velocity controller when nav is enabled."""
    from edge_core.ros_http_bridge import node as bridge_node

    events: list[str] = []

    class _FakeMavlinkVelocityController:
        def __init__(self, *args: object, **kwargs: object) -> None:
            events.append("init")

        def start(self) -> bool:
            events.append("start")
            return True

        def stop(self) -> None:
            events.append("stop")

        def note_vio(self, confidence: float, healthy: bool = True) -> None:
            pass

        def submit(self, vx: float, vy: float, vz: float, yaw_rate: float) -> bool:
            return True

    monkeypatch.setattr(bridge_node, "MavlinkVelocityController", _FakeMavlinkVelocityController)

    if not rclpy.ok():
        rclpy.init()

    bridge = None
    try:
        bridge = bridge_node.ROSHTTPBridge(
            host="127.0.0.1",
            port=1,
            vio_topic="/zed/zed_node/odom",
            imu_topic="/zed/zed_node/imu/data",
            mag_topic="/zed/zed_node/imu/mag",
            cmd_vel_topic="/cmd_vel",
            mesh_topic="/nvblox_node/color_layer_marker",
            servo_topic="/nomad/servo/nozzle_angle",
            send_rate_hz=30.0,
            enable_nav_control=True,
            enable_mesh=False,
            enable_servo=False,
            use_imu_attitude=False,
            use_mag_heading=False,
        )
        assert events[:2] == ["init", "start"]
    finally:
        if bridge is not None:
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    assert events[-1] == "stop"
