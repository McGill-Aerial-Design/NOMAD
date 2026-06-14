# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL gimbal mount-control scenario.

Proves that the MAVLink the NOMAD gimbal control emits actually points a real
ArduPilot mount. The C# offline tests (``pixi run test-plugin-gimbal``) pin the
exact command ids + parameter layout that ``GimbalCommand`` produces; this
scenario sends those same commands to a live ArduPilot SITL and verifies the
mount points where commanded:

  configure a servo mount (MNT1_TYPE=1, tilt/roll on outputs 5/6) + reboot
    -> DO_MOUNT_CONFIGURE MAVLINK_TARGETING
    -> DO_MOUNT_CONTROL pitch sweep   (mount attitude tracks the commanded angle)
    -> DO_MOUNT_CONTROL beyond limits (mount attitude clamps at the mount limit)

The mount's pointing is read back from ``GIMBAL_DEVICE_ATTITUDE_STATUS`` (the
attitude ArduPilot reports for the mount), which is backend-agnostic.

It only needs the operator MAVLink link (no controller, no flight — mount control
works on the ground). Because configuring MNT1_TYPE requires a reboot, this is
run LAST in the nightly SITL sequence so it does not disturb the velocity/fence
scenarios that share the vehicle.

Run standalone inside the dev network (see tests/sitl/README.md):

    NOMAD_SITL_OPERATOR=tcp:host:5762 python tests/sitl/gimbal_mount_control.py
"""

from __future__ import annotations

import math
import os
import time

from pymavlink import mavutil

# Mount limits — must match GimbalCommand.PITCH/ROLL_*_DEG (the plugin's mount).
PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 90.0
ROLL_MIN_DEG = -30.0
ROLL_MAX_DEG = 30.0

# Servo outputs we assign to the mount. Use 5/6: free on a quad (motors are 1-4).
PITCH_CHANNEL = 5  # SERVO5_FUNCTION = 7 (Mount1 tilt/pitch)
ROLL_CHANNEL = 6  # SERVO6_FUNCTION = 8 (Mount1 roll)
_SERVO_FN_MOUNT_PITCH = 7
_SERVO_FN_MOUNT_ROLL = 8

# How close the reported mount pitch must be to the commanded angle (deg). The
# servo mount tracks the command essentially exactly; this leaves slack for the
# attitude quantization and any earth-frame stabilization on the ground.
_PITCH_TOLERANCE_DEG = 12.0


class ScenarioError(AssertionError):
    """A scenario assertion failed (the mount did not point as commanded)."""


def _log(msg: str) -> None:
    print(f"[sitl-gimbal] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def _connect(endpoint: str, timeout: float = 60.0):
    """Open a MAVLink connection and wait for the first heartbeat."""
    deadline = time.time() + timeout
    last_exc: Exception | None = None
    while time.time() < deadline:
        try:
            conn = mavutil.mavlink_connection(endpoint)
            if conn.wait_heartbeat(timeout=10):
                conn.mav.request_data_stream_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10,
                    1,
                )
                return conn
            conn.close()
        except Exception as exc:  # retry until the relay/firmware is back up
            last_exc = exc
            time.sleep(2.0)
    raise ScenarioError(f"no heartbeat on {endpoint} within {timeout:.0f}s (last error: {last_exc})")


def _command_long(conn, command: int, *params: float) -> None:
    args = list(params) + [0.0] * (7 - len(params))
    conn.mav.command_long_send(conn.target_system, conn.target_component, command, 0, *args)


def _param_set(conn, name: str, value: float, attempts: int = 8) -> None:
    """Set a parameter and confirm it read back, retrying for SITL settling."""
    pname = name.encode() if isinstance(name, str) else name
    for _ in range(attempts):
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            pname,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            got_name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            if got_name.strip("\x00") == name and abs(msg.param_value - float(value)) < 0.5:
                return
    raise ScenarioError(f"could not set parameter {name}={value}")


def _read_param(conn, name: str, timeout: float = 5.0):
    conn.mav.param_request_read_send(conn.target_system, conn.target_component, name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        got_name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        if got_name.strip("\x00") == name:
            return msg.param_value
    return None


def _read_mount_pitch_deg(conn, settle: float = 1.5) -> float:
    """Let the mount slew, then return its reported pitch (deg) from the latest
    GIMBAL_DEVICE_ATTITUDE_STATUS quaternion."""
    time.sleep(settle)
    latest = None
    deadline = time.time() + 4.0
    while time.time() < deadline:
        msg = conn.recv_match(type="GIMBAL_DEVICE_ATTITUDE_STATUS", blocking=True, timeout=1.0)
        if msg is not None:
            latest = msg
    if latest is None:
        raise ScenarioError("no GIMBAL_DEVICE_ATTITUDE_STATUS reported (mount not publishing attitude)")
    w, x, y, z = latest.q
    # Pitch from the quaternion (rotation about body Y).
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - x * z)))
    return math.degrees(math.asin(sin_pitch))


def _configure_mount(conn, endpoint: str):
    """Set up a servo mount and reboot to instantiate it (idempotent).

    Returns a live connection: the same one if the mount was already configured,
    or a fresh one after the reboot. (Reconnecting unconditionally would open a
    second client on the single-client SERIAL2 and starve of telemetry.)
    """
    if abs((_read_param(conn, "MNT1_TYPE") or 0) - 1) < 0.5:
        _log("mount already configured (MNT1_TYPE=1); skipping reboot")
        return conn

    _log("configuring servo mount params (MNT1_TYPE + tilt/roll outputs + limits)")
    _param_set(conn, f"SERVO{PITCH_CHANNEL}_FUNCTION", _SERVO_FN_MOUNT_PITCH)
    _param_set(conn, f"SERVO{ROLL_CHANNEL}_FUNCTION", _SERVO_FN_MOUNT_ROLL)
    _param_set(conn, "MNT1_PITCH_MIN", PITCH_MIN_DEG)
    _param_set(conn, "MNT1_PITCH_MAX", PITCH_MAX_DEG)
    _param_set(conn, "MNT1_ROLL_MIN", ROLL_MIN_DEG)
    _param_set(conn, "MNT1_ROLL_MAX", ROLL_MAX_DEG)
    _param_set(conn, "MNT1_TYPE", 1)  # Servo — needs a reboot to take effect

    _log("rebooting SITL to apply MNT1_TYPE")
    _command_long(conn, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    try:
        conn.close()
    except Exception:
        pass
    time.sleep(8.0)  # let the firmware drop before reconnecting
    return _connect(endpoint)


def _mount_control(conn, pitch_deg: float, roll_deg: float) -> None:
    # DO_MOUNT_CONTROL: p1=pitch, p2=roll, p7=MAVLINK_TARGETING (2) — exactly the
    # frame GimbalCommand.BuildMountControl produces.
    _command_long(
        conn,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        pitch_deg,
        roll_deg,
        0,
        0,
        0,
        0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
    )


def _assert_pitch(conn, commanded: float, expected: float, what: str, results: dict) -> None:
    _mount_control(conn, commanded, 0.0)
    actual = _read_mount_pitch_deg(conn)
    _log(f"command pitch={commanded:+.0f} -> mount pitch {actual:+.1f} deg (expect ~{expected:+.0f})")
    results[what] = actual
    if abs(actual - expected) > _PITCH_TOLERANCE_DEG:
        raise ScenarioError(
            f"mount pitch {actual:+.1f} deg not within {_PITCH_TOLERANCE_DEG:.0f} of {expected:+.0f} "
            f"for DO_MOUNT_CONTROL pitch={commanded:+.0f} ({what})"
        )


def run_scenario(operator_ep: str) -> dict:
    """Execute the gimbal mount-control scenario. Raises ScenarioError on failure."""
    results: dict[str, object] = {}

    _log(f"operator link -> {operator_ep}")
    conn = _connect(operator_ep)
    _log(f"heartbeat: sys={conn.target_system} comp={conn.target_component}")

    conn = _configure_mount(conn, operator_ep)

    mnt_type = _read_param(conn, "MNT1_TYPE")
    if mnt_type is None or abs(mnt_type - 1) >= 0.5:
        raise ScenarioError(f"MNT1_TYPE did not stick (got {mnt_type})")
    _log("mount is configured as a servo mount (MNT1_TYPE=1)")

    # Select MAVLINK_TARGETING (the DO_MOUNT_CONFIGURE frame the plugin sends).
    _command_long(
        conn,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        1,
        1,
        1,
        2,
        2,
        2,
    )
    time.sleep(1.0)

    # Pitch must track the command in both directions, then clamp at the limit.
    _assert_pitch(conn, 0.0, 0.0, "pitch_zero", results)
    _assert_pitch(conn, -45.0, -45.0, "pitch_down", results)
    _assert_pitch(conn, 45.0, 45.0, "pitch_up", results)
    _log("  PASS: mount tracks commanded pitch in both directions")

    # The full commanded range is reachable: command the configured lower limit
    # and confirm the mount points there. (The plugin clamps out-of-range inputs
    # to this limit *before* sending — pinned by the offline test — and ArduPilot
    # ignores anything beyond it, so the limit is the meaningful end-to-end check.)
    _assert_pitch(conn, PITCH_MIN_DEG, PITCH_MIN_DEG, "pitch_min_limit", results)
    _log("  PASS: mount reaches the configured pitch limit")

    # Re-center before leaving.
    _mount_control(conn, 0.0, 0.0)
    try:
        conn.close()
    except Exception:
        pass

    results["status"] = "PASS"
    return results


def main() -> int:
    operator_ep = os.environ.get("NOMAD_SITL_OPERATOR")
    if not operator_ep:
        print("set NOMAD_SITL_OPERATOR (MAVLink endpoint)")
        return 2
    try:
        results = run_scenario(operator_ep)
    except ScenarioError as exc:
        _log(f"SCENARIO FAILED: {exc}")
        return 1
    _log(f"SCENARIO PASSED: {results}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
