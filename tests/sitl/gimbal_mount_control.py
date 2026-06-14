# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL gimbal mount-control scenario.

Proves that the MAVLink the NOMAD gimbal control emits actually points a real
ArduPilot mount. The C# offline tests (``pixi run test-plugin-gimbal``) pin the
exact command ids + parameter layout that ``GimbalCommand`` produces; this
scenario sends those same commands to a live ArduPilot SITL and verifies the
mount responds:

  configure a servo mount (MNT1_TYPE=1, tilt/roll on AUX outputs) + reboot
    -> DO_MOUNT_CONFIGURE MAVLINK_TARGETING
    -> DO_MOUNT_CONTROL pitch sweep   (servo output tracks the commanded angle)
    -> DO_MOUNT_CONTROL beyond limits (servo output clamps at the mount limit)

It only needs the operator MAVLink link (no controller, no flight — mount control
works on the ground). Because configuring MNT1_TYPE requires a reboot, this is
run LAST in the nightly SITL sequence so it does not disturb the velocity/fence
scenarios that share the vehicle.

Run standalone inside the dev network (see tests/sitl/README.md):

    NOMAD_SITL_OPERATOR=tcp:host:5762 python tests/sitl/gimbal_mount_control.py
"""

from __future__ import annotations

import os
import time

from pymavlink import mavutil

# Mount limits — must match GimbalCommand.PITCH/ROLL_*_DEG (the plugin's mount).
PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 90.0
ROLL_MIN_DEG = -30.0
ROLL_MAX_DEG = 30.0

# AUX servo outputs we assign to the mount (free on a quad; motors are 1-4).
PITCH_CHANNEL = 9  # SERVO9_FUNCTION = 7 (Mount1 tilt/pitch)
ROLL_CHANNEL = 10  # SERVO10_FUNCTION = 8 (Mount1 roll)
_SERVO_FN_MOUNT_PITCH = 7
_SERVO_FN_MOUNT_ROLL = 8

# A commanded angle change must move the servo output by at least this much (us)
# to count as "the mount responded"; clamped commands must stay within the
# tolerance of the limit's output.
_MOVE_THRESHOLD_US = 80
_CLAMP_TOLERANCE_US = 40


class ScenarioError(AssertionError):
    """A scenario assertion failed (the mount did not respond as commanded)."""


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


def _read_servo_us(conn, channel: int, settle: float = 1.5) -> int:
    """Let the servo slew, then return the latest reported PWM (us) for a channel."""
    time.sleep(settle)
    field = f"servo{channel}_raw"
    latest = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = conn.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1.0)
        if msg is not None and hasattr(msg, field):
            latest = getattr(msg, field)
    if latest is None:
        raise ScenarioError(f"no SERVO_OUTPUT_RAW.{field} reported")
    return int(latest)


def _configure_mount(conn) -> None:
    """Set up a servo mount and reboot to instantiate it (idempotent)."""
    if abs((_read_param(conn, "MNT1_TYPE") or 0) - 1) < 0.5:
        _log("mount already configured (MNT1_TYPE=1); skipping reboot")
        return

    _log("configuring servo mount params (MNT1_TYPE + AUX tilt/roll + limits)")
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


def run_scenario(operator_ep: str) -> dict:
    """Execute the gimbal mount-control scenario. Raises ScenarioError on failure."""
    results: dict[str, object] = {}

    _log(f"operator link -> {operator_ep}")
    conn = _connect(operator_ep)
    _log(f"heartbeat: sys={conn.target_system} comp={conn.target_component}")

    _configure_mount(conn)
    conn = _connect(operator_ep)  # reconnect after the reboot (no-op if not rebooted)

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

    # --- baseline at 0 deg --------------------------------------------------
    _mount_control(conn, 0.0, 0.0)
    pwm0 = _read_servo_us(conn, PITCH_CHANNEL)
    _log(f"pitch=0 deg -> SERVO{PITCH_CHANNEL}={pwm0} us")
    results["pwm_zero"] = pwm0

    # --- pitch down: the servo must move proportionally ---------------------
    _mount_control(conn, -45.0, 0.0)
    pwm_neg = _read_servo_us(conn, PITCH_CHANNEL)
    _log(f"pitch=-45 deg -> SERVO{PITCH_CHANNEL}={pwm_neg} us (delta {pwm_neg - pwm0:+d})")
    results["pwm_neg45"] = pwm_neg
    if abs(pwm_neg - pwm0) < _MOVE_THRESHOLD_US:
        raise ScenarioError(
            f"mount did not respond to DO_MOUNT_CONTROL pitch=-45 "
            f"(servo moved {abs(pwm_neg - pwm0)} us < {_MOVE_THRESHOLD_US})"
        )

    # --- pitch up: opposite direction from pitch down -----------------------
    _mount_control(conn, 45.0, 0.0)
    pwm_pos = _read_servo_us(conn, PITCH_CHANNEL)
    _log(f"pitch=+45 deg -> SERVO{PITCH_CHANNEL}={pwm_pos} us (delta {pwm_pos - pwm0:+d})")
    results["pwm_pos45"] = pwm_pos
    if (pwm_pos - pwm0) * (pwm_neg - pwm0) >= 0:
        raise ScenarioError(
            "servo output did not track command direction "
            f"(+45 delta {pwm_pos - pwm0:+d}, -45 delta {pwm_neg - pwm0:+d} should be opposite)"
        )
    _log("  PASS: servo output tracks commanded pitch in both directions")

    # --- clamp: command beyond the limit, expect saturation at the limit ----
    _mount_control(conn, PITCH_MIN_DEG, 0.0)
    pwm_lim = _read_servo_us(conn, PITCH_CHANNEL)
    _mount_control(conn, PITCH_MIN_DEG - 30.0, 0.0)  # 30 deg past the min
    pwm_over = _read_servo_us(conn, PITCH_CHANNEL)
    _log(f"pitch={PITCH_MIN_DEG} vs {PITCH_MIN_DEG - 30}: SERVO {pwm_lim} vs {pwm_over} us")
    results["pwm_limit"] = pwm_lim
    results["pwm_over_limit"] = pwm_over
    if abs(pwm_over - pwm_lim) > _CLAMP_TOLERANCE_US:
        raise ScenarioError(
            f"mount did not clamp beyond the pitch limit "
            f"(limit {pwm_lim} us, over-limit {pwm_over} us, diff > {_CLAMP_TOLERANCE_US})"
        )
    _log("  PASS: mount saturates at the pitch limit (command clamped)")

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
