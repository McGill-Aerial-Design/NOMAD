# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""End-to-end integration test for the C++ ROS 2 vehicle adapter (nomad_ros).

Runs INSIDE the ``nomad-sim-ros:latest`` image where ``nomad_vehicle_node`` is
built into the colcon workspace and rclpy + pymavlink are available. An
in-process pymavlink UDP responder plays the ArduPilot vehicle (heartbeats,
GPS/SYS_STATUS telemetry); the test publishes VIO health/confidence/source
and ``/nomad/cmd_vel`` topics and asserts on what the responder actually
receives.

This is the C++ replacement for the retired Python ``ros_http_bridge``
integration test (docs/migration.md Phase 7, deletion-gate step 1).

Launch via::

    pixi run test-ros-integration
    # docker run --rm -v $PWD/tests:/opt/nomad/tests \\
    #     nomad-sim-ros:latest python3 -m pytest tests/ros/ -v

What is exercised end-to-end:
  1. ``nomad_vehicle_node`` connects to a MAVLink UDP peer and latches the
     vehicle (heartbeat from system 1).
  2. The node publishes typed telemetry derived from the peer's frames.
  3. The fail-closed VIO gate: a velocity command with no VIO feed or a
     mismatched VIO source is refused and no setpoint reaches the vehicle.
  4. With a healthy, fresh, high-confidence VIO feed from the configured
     source, the same command is
     translated FLU -> core FRD and a SET_POSITION_TARGET_LOCAL_NED setpoint
     reaches the vehicle with the commanded forward velocity.
  5. The /nomad/arm, /nomad/disarm, /nomad/land and /nomad/rtl trigger
     services send the command through the core, verify the resulting
     authoritative vehicle state (armed bit / mode from heartbeats) and
     surface ACK rejections as service failures.
"""

from __future__ import annotations

import socket
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field

import pytest
import mavlink_wire as wire

# Skip cleanly when rclpy is absent (normal pixi run test on the host — no
# ROS2 installed).  This line MUST come before any rclpy import.
rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402

_GUIDED_CUSTOM_MODE = 4
_CUSTOM_MODE_LAND = 9
_CUSTOM_MODE_RTL = 6


@dataclass
class VehicleState:
    """Shared state between the responder threads and the test assertions."""

    stop: threading.Event = field(default_factory=threading.Event)
    setpoints: list[float] = field(default_factory=list)
    node_connected: bool = False
    fix_messages: int = 0
    armed: bool = True  # the velocity tests command an armed GUIDED vehicle
    custom_mode: int = _GUIDED_CUSTOM_MODE
    reject_next: bool = False  # when set, the next command is ACKed as FAILED
    command_ids: list[int] = field(default_factory=list)


def find_free_udp_port() -> int:
    """Allocate an ephemeral loopback port and release it.

    The node (udpin listener) then binds that port; the race window between
    close and bind is negligible for a test.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


class MavlinkResponder:
    """A minimal ArduPilot-style MAVLink peer on a loopback UDP socket."""

    def __init__(self, state: VehicleState, node_port: int) -> None:
        """Bind a sender socket and stream telemetry to the node's listener."""
        self.state = state
        self.node_port = node_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))  # ephemeral sender port, != node port
        self.socket.settimeout(0.1)
        self.parser = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.thread: threading.Thread | None = None
        self._node_address = ("127.0.0.1", self.node_port)

    def _send(self, message) -> None:
        self.socket.sendto(message.pack(self.parser), self._node_address)

    def _send_heartbeat(self) -> None:
        base_mode = 0x80 if self.state.armed else 0
        self._send(
            self.parser.heartbeat_encode(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode,
                self.state.custom_mode,
                mavlink.MAV_STATE_ACTIVE,
            )
        )

    def _send_gps(self) -> None:
        self._send(
            self.parser.gps_raw_int_encode(
                time_usec=0,
                fix_type=3,
                lat=int(42.3898 * 1e7),
                lon=int(-71.1476 * 1e7),
                alt=14000,
                eph=100,
                epv=100,
                vel=0,
                cog=0,
                satellites_visible=10,
            )
        )

    def _send_position(self) -> None:
        self._send(
            self.parser.global_position_int_encode(
                time_boot_ms=0,
                lat=int(42.3898 * 1e7),
                lon=int(-71.1476 * 1e7),
                alt=14000,
                relative_alt=8000,
                vx=0,
                vy=0,
                vz=0,
                hdg=0,
            )
        )

    def _send_sys_status(self) -> None:
        self._send(
            self.parser.sys_status_encode(
                onboard_control_sensors_present=0xFFFFFFFF,
                onboard_control_sensors_enabled=0xFFFFFFFF,
                onboard_control_sensors_health=0xFFFFFFFF,
                load=500,
                voltage_battery=12600,
                current_battery=-1000,
                battery_remaining=100,
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
        self._send_gps()
        self._send_position()
        self._send_sys_status()

    def _ack_command(self, command_id: int, result: int) -> None:
        # Broadcast ACK targets let MAVSDK match the vehicle source and command ID.
        acknowledgement = self.parser.command_ack_encode(command_id, result, 0, 0, 0, 0)
        self._send(acknowledgement)

    def _handle_commands(self, commands: list[tuple[int, float]]) -> None:
        for command_id, param1 in commands:
            self.state.command_ids.append(command_id)
            # MAVSDK may issue background protocol requests while the test is
            # preparing a service call. Keep the fault injection attached to a
            # vehicle command so an unrelated request cannot consume it.
            if self.state.reject_next and command_id in {
                wire.ARM_DISARM_COMMAND,
                wire.LAND_COMMAND,
                wire.RTL_COMMAND,
            }:
                self.state.reject_next = False
                self._ack_command(command_id, wire.MAV_RESULT_FAILED)
                continue
            if command_id == wire.ARM_DISARM_COMMAND:
                self.state.armed = param1 >= 1.0
                self._ack_command(command_id, wire.MAV_RESULT_ACCEPTED)
            elif command_id == wire.LAND_COMMAND:
                self.state.custom_mode = _CUSTOM_MODE_LAND
                self._ack_command(command_id, wire.MAV_RESULT_ACCEPTED)
            elif command_id == wire.RTL_COMMAND:
                self.state.custom_mode = _CUSTOM_MODE_RTL
                self._ack_command(command_id, wire.MAV_RESULT_ACCEPTED)

    def _run(self) -> None:
        while not self.state.stop.is_set():
            # Keep the link alive and the vehicle state fresh (20 Hz).
            self._telemetry_burst()
            try:
                data, _ = self.socket.recvfrom(65535)
            except OSError:
                continue
            if not data:
                continue
            self.state.setpoints.extend(wire.decode_velocity_setpoints(data))
            commands = wire.decode_commands(data)
            if commands:
                self._handle_commands(commands)

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.state.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.socket.close()


def _node_command(port: int, integrated_flight_mode: bool = False) -> list[str]:
    integrated = "true" if integrated_flight_mode else "false"
    return [
        "/bin/bash",
        "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /ws/install/setup.bash && "
        f"ros2 run nomad_ros nomad_vehicle_node "
        f"--ros-args -p endpoint:=udpin:127.0.0.1:{port} -p publish_rate_hz:=10.0 "
        f"-p vio_source:=test_vio -p integrated_flight_mode:={integrated}",
    ]


def _launch_node(
    port: int, integrated_flight_mode: bool = False
) -> tuple[subprocess.Popen, list[str], threading.Thread]:
    """Start nomad_vehicle_node plus a stdout drain thread."""
    node_process = subprocess.Popen(
        _node_command(port, integrated_flight_mode),
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


def _create_ros_publishers(control_node):
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    return (
        control_node.create_publisher(TwistStamped, "/nomad/cmd_vel", qos),
        control_node.create_publisher(Bool, "/nomad/vio_health", qos),
        control_node.create_publisher(Float32, "/nomad/vio_confidence", qos),
        control_node.create_publisher(String, "/nomad/vio_source", qos),
    )


def _create_ros_control(
    state: VehicleState,
) -> tuple[object, threading.Event, threading.Thread, object, object, object, object]:
    """Create the rclpy control node with its publishers and a spin thread."""
    rclpy.init()
    control_node = rclpy.create_node("nomad_ros_test")
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    spin_stop = threading.Event()
    publishers = _create_ros_publishers(control_node)
    control_node.create_subscription(
        NavSatFix,
        "/nomad/fix",
        lambda _message: setattr(state, "fix_messages", state.fix_messages + 1),
        QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE),
    )

    def _spin() -> None:
        while not spin_stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    return (control_node, spin_stop, spin_thread, *publishers)


def _wait_for_node_connect(log_lines: list[str], state: VehicleState) -> None:
    """Wait until the node logs that it latched the vehicle."""
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        if any(
            "connected to the NOMAD core vehicle" in line or "connected as an observation-only ROS adapter" in line
            for line in log_lines
        ):
            state.node_connected = True
            return
        time.sleep(0.2)


def _start_ros_session(integrated_flight_mode: bool = False):
    state = VehicleState()
    responder = MavlinkResponder(state, find_free_udp_port())
    responder.start()
    node_process, log_lines, drain_thread = _launch_node(responder.node_port, integrated_flight_mode)
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
    """Yield the real ROS/MAVLink session and always tear it down."""
    state, responder, node_process, log_lines, drain_thread, control = _start_ros_session()
    (
        control_node,
        spin_stop,
        spin_thread,
        cmd_publisher,
        vio_health_publisher,
        vio_confidence_publisher,
        vio_source_publisher,
    ) = control
    try:
        yield {
            "state": state,
            "responder": responder,
            "control_node": control_node,
            "cmd_publisher": cmd_publisher,
            "vio_health_publisher": vio_health_publisher,
            "vio_confidence_publisher": vio_confidence_publisher,
            "vio_source_publisher": vio_source_publisher,
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


def _call_trigger_service(control_node, name: str) -> tuple[bool, str]:
    """Call a /nomad/<name> Trigger service and return (success, message)."""
    client = control_node.create_client(Trigger, f"/nomad/{name}")
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            pytest.fail(f"service /nomad/{name} never became available")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        if not future.done():
            pytest.fail(f"service call /nomad/{name} timed out")
        response = future.result()
        return bool(response.success), str(response.message)
    finally:
        control_node.destroy_client(client)


def _publish_vio(session, duration: float, rate: float = 10.0, source: str = "test_vio") -> None:
    deadline = time.monotonic() + duration
    interval = 1.0 / rate
    while time.monotonic() < deadline:
        session["vio_health_publisher"].publish(Bool(data=True))
        session["vio_confidence_publisher"].publish(Float32(data=1.0))
        session["vio_source_publisher"].publish(String(data=source))
        time.sleep(interval)
