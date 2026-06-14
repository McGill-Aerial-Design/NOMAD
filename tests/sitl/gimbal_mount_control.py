# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL gimbal mount-control scenario.

Proves that the MAVLink the NOMAD gimbal control emits actually points a real
ArduPilot mount. The C# offline tests (``pixi run test-plugin-gimbal``) pin the
exact command ids + parameter layout that ``GimbalCommand`` produces; this
scenario sends those same commands to a live ArduPilot SITL and verifies the
mount moves where commanded:

  configure a servo mount (MNT1_TYPE=1, tilt/roll on outputs 5/6) + reboot
    -> DO_MOUNT_CONFIGURE MAVLINK_TARGETING
    -> DO_MOUNT_CONTROL pitch sweep   (the mount tracks the commanded angle)
    -> DO_MOUNT_CONTROL to the limit  (the mount reaches the configured limit)

The mount's response is read back from whichever signal the firmware provides:
the reported attitude (``GIMBAL_DEVICE_ATTITUDE_STATUS`` / ``MOUNT_ORIENTATION``)
when available, otherwise the servo output (``SERVO_OUTPUT_RAW`` on the mount's
pitch channel). Some ArduCopter 4.6 SITL builds drive the servo but do not
publish the gimbal-device attitude, so the scenario accepts either.

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

# Servo outputs we assign to the mount. Use 5/6: free on a quad (motors are 1-4)
# and reported in the base (port 0) SERVO_OUTPUT_RAW message.
PITCH_CHANNEL = 5  # SERVO5_FUNCTION = 7 (Mount1 tilt/pitch)
ROLL_CHANNEL = 6  # SERVO6_FUNCTION = 8 (Mount1 roll)
_SERVO_FN_MOUNT_PITCH = 7
_SERVO_FN_MOUNT_ROLL = 8

# Attitude-mode tolerance (deg) and servo-mode movement threshold (us).
_PITCH_TOLERANCE_DEG = 12.0
_SERVO_MOVE_US = 80

# Message ids we explicitly request (some firmwares don't include the mount
# attitude in MAV_DATA_STREAM_ALL). Fall back to the literal ids if missing.
_MSGID_GIMBAL_ATTITUDE = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS", 285)
_MSGID_MOUNT_ORIENTATION = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_MOUNT_ORIENTATION", 265)


class ScenarioError(AssertionError):
    """A scenario assertion failed (the mount did not move as commanded)."""


def _log(msg: str) -> None:
    print(f"[sitl-gimbal] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def _request_streams(conn) -> None:
    """Ask the autopilot for telemetry, including the mount attitude messages."""
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    for msgid in (_MSGID_GIMBAL_ATTITUDE, _MSGID_MOUNT_ORIENTATION):
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msgid,
            100000,  # 100 ms = 10 Hz
            0,
            0,
            0,
            0,
            0,
        )


def _connect(endpoint: str, timeout: float = 60.0):
    """Open a MAVLink connection and latch onto the AUTOPILOT heartbeat.

    Latching the first heartbeat blindly can pick a GCS/MAVProxy component
    (system 0), which then never answers our stream requests. Wait for the
    autopilot specifically.
    """
    deadline = time.time() + timeout
    last_exc: Exception | None = None
    while time.time() < deadline:
        try:
            conn = mavutil.mavlink_connection(endpoint)
            if _latch_autopilot(conn, timeout=15.0):
                _request_streams(conn)
                return conn
            conn.close()
        except Exception as exc:  # retry until the relay/firmware is back up
            last_exc = exc
            time.sleep(2.0)
    raise ScenarioError(f"no autopilot heartbeat on {endpoint} within {timeout:.0f}s (last error: {last_exc})")


def _latch_autopilot(conn, timeout: float = 15.0) -> bool:
    """Set conn.target_* from the first non-GCS (autopilot) heartbeat."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
        if msg is None:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue  # skip GCS / companion heartbeats
        conn.target_system = msg.get_srcSystem()
        conn.target_component = msg.get_srcComponent()
        return True
    return False


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


def _sample(conn, window: float = 3.0):
    """Collect the latest mount response over a window.

    Returns (pitch_deg_or_None, servo_us_or_None): the reported mount pitch (from
    GIMBAL_DEVICE_ATTITUDE_STATUS / MOUNT_ORIENTATION) and the mount pitch
    channel's servo PWM (from SERVO_OUTPUT_RAW port 0), whichever the firmware
    streams.
    """
    pitch = None
    servo = None
    field = f"servo{PITCH_CHANNEL}_raw"
    start = time.time()
    while time.time() - start < window:
        msg = conn.recv_match(
            type=["GIMBAL_DEVICE_ATTITUDE_STATUS", "MOUNT_ORIENTATION", "SERVO_OUTPUT_RAW"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            w, x, y, z = msg.q
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - x * z)))))
        elif kind == "MOUNT_ORIENTATION":
            pitch = float(msg.pitch)
        elif kind == "SERVO_OUTPUT_RAW" and getattr(msg, "port", 0) == 0:
            val = getattr(msg, field, None)
            if val:
                servo = int(val)
    return pitch, servo


def _settle_pitch(conn, pitch_deg: float, settle: float = 1.5):
    """Command a pitch, let the mount slew, and return (pitch_deg, servo_us)."""
    _mount_control(conn, pitch_deg, 0.0)
    time.sleep(settle)
    return _sample(conn)


def _detect_mode(conn, timeout: float = 90.0) -> str:
    """Wait until the mount demonstrably responds; return 'att' or 'servo'.

    Prefers attitude (clean angles); falls back to the servo output once a
    commanded move actually changes it. Polls because the mount can take time to
    come up after the reboot.
    """
    # Test hook: force the servo-output path even when attitude is available, so
    # the servo verification can be exercised on a build that does publish
    # attitude (CI's build does not, and uses servo automatically).
    force_servo = os.environ.get("NOMAD_GIMBAL_FORCE_SERVO") == "1"

    deadline = time.time() + timeout
    while time.time() < deadline:
        _request_streams(conn)
        att0, sv0 = _settle_pitch(conn, 0.0)
        if att0 is not None and not force_servo:
            _log("verifying via mount attitude")
            return "att"
        att1, sv1 = _settle_pitch(conn, -45.0)
        if att1 is not None and not force_servo:
            _log("verifying via mount attitude")
            return "att"
        if sv0 is not None and sv1 is not None and abs(sv1 - sv0) >= _SERVO_MOVE_US:
            _log(f"verifying via servo output (pitch channel {PITCH_CHANNEL})")
            return "servo"
        _log("  waiting for the mount to respond...")

    seen: dict[str, int] = {}
    start = time.time()
    while time.time() - start < 3.0:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is not None:
            seen[msg.get_type()] = seen.get(msg.get_type(), 0) + 1
    raise ScenarioError(f"mount never responded within {timeout:.0f}s; messages seen: {sorted(seen)}")


def _configure_mount(conn, endpoint: str):
    """Set up a servo mount and reboot to instantiate it (idempotent).

    Returns a live connection: the same one if the mount was already configured,
    or a fresh one after the reboot.
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


def _verify_attitude(conn, results: dict) -> None:
    def check(commanded, expected, what):
        pitch, _ = _settle_pitch(conn, commanded)
        _log(f"command pitch={commanded:+.0f} -> mount pitch {pitch:+.1f} deg (expect ~{expected:+.0f})")
        results[what] = pitch
        if pitch is None or abs(pitch - expected) > _PITCH_TOLERANCE_DEG:
            raise ScenarioError(
                f"mount pitch {pitch} not within {_PITCH_TOLERANCE_DEG:.0f} of {expected:+.0f} ({what})"
            )

    check(0.0, 0.0, "pitch_zero")
    check(-45.0, -45.0, "pitch_down")
    check(45.0, 45.0, "pitch_up")
    _log("  PASS: mount tracks commanded pitch in both directions")
    check(PITCH_MIN_DEG, PITCH_MIN_DEG, "pitch_min_limit")
    _log("  PASS: mount reaches the configured pitch limit")


def _verify_servo(conn, results: dict) -> None:
    _, pwm0 = _settle_pitch(conn, 0.0)
    _, pwm_neg = _settle_pitch(conn, -45.0)
    _, pwm_pos = _settle_pitch(conn, 45.0)
    _, pwm_lim = _settle_pitch(conn, PITCH_MIN_DEG)
    _log(f"servo us: 0={pwm0} -45={pwm_neg} +45={pwm_pos} {PITCH_MIN_DEG:.0f}={pwm_lim}")
    results.update(servo_zero=pwm0, servo_neg=pwm_neg, servo_pos=pwm_pos, servo_limit=pwm_lim)
    if None in (pwm0, pwm_neg, pwm_pos, pwm_lim):
        raise ScenarioError("servo output stopped streaming mid-test")

    if abs(pwm_neg - pwm0) < _SERVO_MOVE_US:
        raise ScenarioError(f"mount did not move for pitch=-45 (servo {pwm_neg - pwm0:+d} us)")
    if (pwm_pos - pwm0) * (pwm_neg - pwm0) >= 0:
        raise ScenarioError(f"servo did not track direction (+45 {pwm_pos - pwm0:+d}, -45 {pwm_neg - pwm0:+d})")
    _log("  PASS: mount tracks commanded pitch in both directions")
    # The limit must reach at least as far as -45 in the same direction.
    if abs(pwm_lim - pwm0) + 1 < abs(pwm_neg - pwm0) or (pwm_lim - pwm0) * (pwm_neg - pwm0) < 0:
        raise ScenarioError(f"mount did not reach the limit (limit {pwm_lim - pwm0:+d} vs -45 {pwm_neg - pwm0:+d})")
    _log("  PASS: mount reaches the configured pitch limit")


def run_scenario(operator_ep: str) -> dict:
    """Execute the gimbal mount-control scenario. Raises ScenarioError on failure."""
    results: dict[str, object] = {}

    _log(f"operator link -> {operator_ep}")
    conn = _connect(operator_ep)
    _log(f"autopilot: sys={conn.target_system} comp={conn.target_component}")

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

    mode = _detect_mode(conn)
    if mode == "att":
        _verify_attitude(conn, results)
    else:
        _verify_servo(conn, results)

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
