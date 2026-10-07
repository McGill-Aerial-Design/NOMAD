# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Shared ROS and MAVLink fixtures for read-only adapter integration tests."""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field

import pytest
import mavlink_wire as wire

# Skip cleanly when rclpy is absent (normal pixi run test on the host).
rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import BatteryState, NavSatFix  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402


@dataclass
class VehicleState:
    """Shared state between the receive-only responder and test assertions."""

    stop: threading.Event = field(default_factory=threading.Event)
    outbound_message_ids: list[int] = field(default_factory=list)
    setpoints: list[float] = field(default_factory=list)
    command_ids: list[int] = field(default_factory=list)
    node_connected: bool = False
    fix_messages: list[NavSatFix] = field(default_factory=list)
    battery_messages: list[BatteryState] = field(default_factory=list)
    send_position: bool = True
    send_gps: bool = True
    send_battery: bool = True
    repeat_position_timestamp: bool = False
    repeat_gps_timestamp: bool = False
    invalid_position: bool = False
    gps_fix_type: int = 3
    battery_voltage_mv: int = 12600
    battery_remaining_percent: int = 50


def find_free_udp_port() -> int:
    """Allocate a loopback UDP port and release it for the observer."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


class MavlinkResponder:
    """A minimal ArduPilot-style MAVLink peer with independently pausable data."""

    def __init__(self, state: VehicleState, node_port: int) -> None:
        self.state = state
        self.node_port = node_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.05)
        self.parser = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.position_boot_ms = 0
        self.gps_time_usec = 0
        self.thread: threading.Thread | None = None
        self._node_address = ("127.0.0.1", self.node_port)

    def _send(self, message) -> None:
        self.parser.seq = (self.parser.seq + 1) & 0xFF
        self.socket.sendto(message.pack(self.parser), self._node_address)

    def _send_heartbeat(self) -> None:
        self._send(
            self.parser.heartbeat_encode(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                0,
                4,
                mavlink.MAV_STATE_ACTIVE,
            )
        )

    def _send_gps(self) -> None:
        if not self.state.repeat_gps_timestamp:
            self.gps_time_usec += 100_000
        self._send(
            self.parser.gps_raw_int_encode(
                time_usec=self.gps_time_usec,
                fix_type=self.state.gps_fix_type,
                lat=int(42.3898 * 1e7),
                lon=int(-71.1476 * 1e7),
                alt=14100,
                eph=100,
                epv=100,
                vel=0,
                cog=0,
                satellites_visible=10,
            )
        )

    def _send_position(self) -> None:
        if not self.state.repeat_position_timestamp:
            self.position_boot_ms += 100
        latitude = 91.0 if self.state.invalid_position else 42.3898
        self._send(
            self.parser.global_position_int_encode(
                time_boot_ms=self.position_boot_ms,
                lat=int(latitude * 1e7),
                lon=int(-71.1476 * 1e7),
                alt=14100,
                relative_alt=8000,
                vx=0,
                vy=0,
                vz=0,
                hdg=0,
            )
        )

    def _send_battery(self) -> None:
        self._send(
            self.parser.sys_status_encode(
                onboard_control_sensors_present=0xFFFFFFFF,
                onboard_control_sensors_enabled=0xFFFFFFFF,
                onboard_control_sensors_health=0xFFFFFFFF,
                load=500,
                voltage_battery=self.state.battery_voltage_mv,
                current_battery=-1000,
                battery_remaining=self.state.battery_remaining_percent,
                drop_rate_comm=0,
                errors_comm=0,
                errors_count1=0,
                errors_count2=0,
                errors_count3=0,
                errors_count4=0,
            )
        )

    def _telemetry_burst(self) -> None:
        self._send_heartbeat()
        if self.state.send_gps:
            self._send_gps()
        if self.state.send_position:
            self._send_position()
        if self.state.send_battery:
            self._send_battery()

    def _run(self) -> None:
        while not self.state.stop.is_set():
            self._telemetry_burst()
            try:
                data, _ = self.socket.recvfrom(65535)
            except OSError:
                continue
            if not data:
                continue
            self.state.outbound_message_ids.extend(wire.decode_message_ids(data))
            self.state.setpoints.extend(wire.decode_velocity_setpoints(data))
            self.state.command_ids.extend(command_id for command_id, _ in wire.decode_commands(data))

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.state.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.socket.close()


def _node_command(port: int) -> list[str]:
    return [
        "/bin/bash",
        "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /ws/install/setup.bash && "
        "ros2 run nomad_ros nomad_vehicle_node --ros-args "
        f"-p observation_endpoint:=udpin:127.0.0.1:{port} "
        "-p expected_system_id:=1 -p publish_rate_hz:=10.0",
    ]


def _launch_node(port: int) -> tuple[subprocess.Popen, list[str], threading.Thread]:
    """Start nomad_vehicle_node plus a stdout drain thread."""
    node_process = subprocess.Popen(
        _node_command(port),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    log_lines: list[str] = []

    def _drain_log() -> None:
        assert node_process.stdout is not None
        for line in node_process.stdout:
            log_lines.append(line.rstrip())

    drain_thread = threading.Thread(target=_drain_log, daemon=True)
    drain_thread.start()
    return node_process, log_lines, drain_thread


def _create_ros_control(state: VehicleState):
    """Subscribe to observer topics and publish the removed cmd_vel input."""
    rclpy.init()
    control_node = rclpy.create_node("nomad_ros_test")
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    spin_stop = threading.Event()
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    cmd_publisher = control_node.create_publisher(TwistStamped, "/nomad/cmd_vel", qos)
    control_node.create_subscription(NavSatFix, "/nomad/fix", state.fix_messages.append, QoSProfile(depth=5))
    control_node.create_subscription(BatteryState, "/nomad/battery", state.battery_messages.append, QoSProfile(depth=5))
    control_node.create_subscription(
        Bool, "/nomad/connected", lambda message: setattr(state, "node_connected", message.data), 1
    )

    def _spin() -> None:
        while not spin_stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    return control_node, spin_stop, spin_thread, cmd_publisher


def _wait_for_node_connect(log_lines: list[str], state: VehicleState) -> None:
    """Wait until the observer logs that it selected the telemetry peer."""
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        if any("connected to the read-only MAVLink telemetry observer" in line for line in log_lines):
            state.node_connected = True
            return
        time.sleep(0.2)


def _start_ros_session():
    state = VehicleState()
    responder = MavlinkResponder(state, find_free_udp_port())
    responder.start()
    node_process, log_lines, drain_thread = _launch_node(responder.node_port)
    control = _create_ros_control(state)
    _wait_for_node_connect(log_lines, state)
    return state, responder, node_process, log_lines, drain_thread, control


def _stop_ros_session(session) -> None:
    state, responder, node_process, log_lines, drain_thread, control = session
    control_node, spin_stop = control[0], control[1]
    spin_stop.set()
    print("\n--- node log tail ---")
    print("\n".join(log_lines[-10:]))
    os.killpg(node_process.pid, signal.SIGTERM)
    try:
        node_process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(node_process.pid, signal.SIGKILL)
    drain_thread.join(timeout=2.0)
    responder.stop()
    control_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def ros_session_values():
    """Yield the ROS/MAVLink observation session and always tear it down."""
    state, responder, node_process, log_lines, drain_thread, control = _start_ros_session()
    try:
        yield {
            "state": state,
            "responder": responder,
            "control_node": control[0],
            "spin_stop": control[1],
            "cmd_publisher": control[3],
            "log_lines": log_lines,
            "node_process": node_process,
            "connected": state.node_connected,
        }
    finally:
        _stop_ros_session((state, responder, node_process, log_lines, drain_thread, control))


def _publish_cmd_vel(session, duration: float, vx: float, rate: float = 10.0) -> None:
    deadline = time.monotonic() + duration
    interval = 1.0 / rate
    while time.monotonic() < deadline:
        twist = TwistStamped()
        twist.header.stamp = session["control_node"].get_clock().now().to_msg()
        twist.twist.linear.x = float(vx)
        session["cmd_publisher"].publish(twist)
        time.sleep(interval)
