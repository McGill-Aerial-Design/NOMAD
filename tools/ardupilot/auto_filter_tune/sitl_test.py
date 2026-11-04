# SPDX-License-Identifier: Apache-2.0
"""ArduCopter SITL check for auto_filter_tune.lua. Runs the real firmware; never touches hardware.

Usage (Linux/WSL): python3 sitl_test.py --ardupilot ~/ardupilot [scenario ...]
SITL motor vibration is RPM-scaled per motor with harmonics (SIM_VIB_MOT_*), ESC RPM comes from
SIM_ESC_TELEM, motor faults from SIM_ENGINE_FAIL/MUL and fixed resonances from SIM_VIB_FREQ_*.
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from pymavlink import mavutil

HERE = Path(__file__).resolve().parent
SPEEDUP = 10
# SITL motor noise on the 1st+2nd harmonic only (2-blade props). Gyro LPF high enough to see it.
VEHICLE = {
    "SCR_ENABLE": 1,
    "SIM_ESC_TELEM": 1,
    "SIM_VIB_MOT_MAX": 350,
    "SIM_VIB_MOT_HMNC": 3,
    "SIM_GYR1_RND": 20,
    "INS_GYRO_FILTER": 80,
    "INS_HNTCH_ENABLE": 0,
    "FFT_ENABLE": 0,
    "LOG_DISARMED": 0,
    "ARMING_MAGTHRESH": 0,  # SITL tip-overs on touchdown upset the mag-field check; unrelated to filters
}


class Sitl:
    def __init__(self, ardupilot, params):
        runs = Path.home() / "aft-sitl-runs"
        runs.mkdir(exist_ok=True)
        self.run = Path(tempfile.mkdtemp(prefix="run-", dir=runs))
        (self.run / "scripts").mkdir()
        shutil.copy(HERE / "auto_filter_tune.lua", self.run / "scripts")
        extra = self.run / "extra.parm"
        extra.write_text("".join(f"{k} {v}\n" for k, v in {**VEHICLE, **params}.items()))
        defaults = f"{ardupilot}/Tools/autotest/default_params/copter.parm,{extra}"
        cmd = [
            f"{ardupilot}/build/sitl/bin/arducopter",
            "--model",
            "quad",
            "--speedup",
            str(SPEEDUP),
            "--defaults",
            defaults,
            "-w",
            "-I0",
            "--home",
            "45.5,-73.6,50,0",
        ]
        self.log = open(self.run / "sitl.log", "w")
        self.proc = subprocess.Popen(cmd, cwd=self.run, stdout=self.log, stderr=subprocess.STDOUT)
        self.text = []
        self.mark = 0
        self.handled = 0
        self.connect()

    def connect(self):
        deadline = time.time() + 60
        while True:
            try:
                # no silent auto-reconnect: a reboot must be seen as a dropped link, see reconnect()
                self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=250)
                if self.mav.wait_heartbeat(timeout=10):
                    break
            except OSError:
                pass
            if time.time() > deadline:
                raise RuntimeError("no heartbeat from SITL")
            time.sleep(1)
        self.mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.last_heartbeat = time.time()
        self.dropped = False

    def pump(self, seconds, until=None):
        """Read messages for up to `seconds` wall time; return the first message matching `until`.

        A closed socket or 3 s without a heartbeat means the vehicle rebooted: sets `dropped`, returns None.
        """
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
            except OSError:
                msg = None
                self.last_heartbeat = 0
            if msg is None:
                if time.time() - self.last_heartbeat > 3:
                    self.dropped = True
                    return None
                continue
            if msg.get_type() == "HEARTBEAT":
                self.last_heartbeat = time.time()
            if msg.get_type() == "STATUSTEXT" and "auto_filter_tune.lua" in msg.text:
                raise AssertionError(f"script error: {msg.text}")
            if msg.get_type() == "STATUSTEXT" and msg.text.startswith("AFT+ ") and self.text:
                print("   ", msg.text)
                self.text[-1] += " " + msg.text[5:]
            elif msg.get_type() == "STATUSTEXT" and msg.text.startswith("AFT: "):
                print("   ", msg.text)
                self.text.append(msg.text)
            if until and until(msg):
                return msg
        return None

    def since_mark(self):
        return self.text[self.mark :]

    def saw(self, fragment):
        return any(fragment in t for t in self.since_mark())

    def wait_text(self, *fragments, seconds=30):
        """Wait for text containing any fragment, sent since the last flight or AFT_ACTION write."""

        def hits():
            return [t for t in self.since_mark() if any(f in t for f in fragments)]

        self.pump(seconds, lambda _m: hits())
        assert hits(), f"expected status text containing {fragments} within {seconds}s"
        return hits()[0]

    def get(self, name):
        self.mav.mav.param_request_read_send(1, 1, name.encode(), -1)
        msg = self.pump(10, lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name)
        assert msg, f"no value for {name}"
        return msg.param_value

    def set(self, name, value):
        if name == "AFT_ACTION":
            self.mark = len(self.text)
        for _attempt in range(3):
            self.mav.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            msg = self.pump(5, lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name)
            if msg and abs(msg.param_value - value) < 1e-3:
                return
        raise AssertionError(f"{name} not set to {value}")

    def command_reboot(self):
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        self.pump(30)
        assert self.dropped, "vehicle never rebooted"

    def reboot(self):
        self.command_reboot()
        self.settle()

    def settle(self, quiet=10):
        """Reboot whenever the script asks, until the vehicle stays up `quiet` s with nothing pending.

        SITL hangs when a script calls vehicle:reboot() (its reboot is an execv from the calling thread),
        so scenarios run with AFT_AUTO=1 and the reboot the script requests is done over MAVLink.
        """
        while True:
            if self.dropped:
                self.mav.close()
                self.connect()
            self.pump(quiet)
            if self.dropped:
                continue
            # requests can arrive while another wait is reading, so track unhandled ones, not new text
            if not any("reboot to load" in t for t in self.text[self.handled :]):
                return
            self.handled = len(self.text)
            self.command_reboot()

    def command(self, cmd, *args):
        self.mav.mav.command_long_send(1, 1, cmd, 0, *args, *([0] * (7 - len(args))))
        ack = self.pump(5, lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd)
        return ack is not None and ack.result == 0

    def fly_hover(self, hover_s=45):
        """GUIDED takeoff to 10 m, hover, LAND, disarm, then follow automatic reboots. Times are simulated."""
        self.mark = len(self.text)
        self.mav.set_mode("GUIDED")
        deadline = time.time() + 120
        while not self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            assert time.time() < deadline, "vehicle never armed (EKF/GPS not ready, or arming blocked)"
            self.pump(2)
        assert self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10), "takeoff rejected"
        alt = self.pump(60, lambda m: m.get_type() == "GLOBAL_POSITION_INT" and m.relative_alt > 9000)
        assert alt, "did not reach 9 m"
        self.pump(hover_s / SPEEDUP + 1)
        self.mav.set_mode("LAND")
        landed = self.pump(
            90,
            lambda m: (
                m.get_type() == "HEARTBEAT"
                and m.type == 2
                and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ),
        )
        pos = self.mav.messages.get("GLOBAL_POSITION_INT")
        if not landed and pos and pos.relative_alt < 300:
            # SITL ground contact occasionally tips the airframe on touchdown and the land detector
            # never completes; the vehicle is down, so disarm like a pilot would.
            print("    harness: forced disarm after SITL touchdown tip-over")
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
            landed = self.pump(10, lambda m: m.get_type() == "HEARTBEAT" and m.type == 2 and not m.base_mode & 128)
        alt = pos.relative_alt / 1000 if pos else "?"
        assert landed, f"did not land and disarm (alt {alt} m, mode {self.mav.flightmode})"
        self.settle()

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()  # a rebooted SITL re-execs and can ignore SIGTERM
            self.proc.wait(10)
        self.log.close()


def auto_full(sitl):
    """Hands-off: only hover until DONE; every stage must run and end in a kept or restored step."""
    for _flight in range(18):
        sitl.fly_hover()
        if sitl.saw("DONE"):
            break
    else:
        raise AssertionError("tune did not finish within 18 flights")
    log = "\n".join(sitl.text)
    # Whether a harmonic survives pruning depends on the low-pass in front of it (a 40 Hz LPF already
    # crushes the 372 Hz 2nd harmonic), so only the stage outcomes are pinned, not the final mask.
    for step in (
        "notch step VERIFIED",
        "prune step",
        "narrow step VERIFIED",  # the last narrowing (BW 9) sits within noise repeatability: may pass or fail
        "lpf step VERIFIED",
    ):
        assert step in log, f"missing '{step}'"
    assert sitl.get("INS_HNTCH_MODE") == 3, "ESC telemetry tracking expected"
    assert int(sitl.get("INS_HNTCH_HMNCS")) & 1, "pruning must keep the fundamental"
    assert int(sitl.get("INS_HNTCH_OPTS")) & 6 == 6, "per-motor notches updated at loop rate expected"
    assert sitl.get("INS_GYRO_FILTER") > 40, "low-pass stage should have raised the cutoff"


def manual_mode_waits(sitl):
    sitl.set("AFT_AUTO", 0)
    sitl.fly_hover()
    sitl.wait_text("set AFT_ACTION=1")
    assert sitl.get("INS_HNTCH_ENABLE") == 0, "manual mode must not change filters by itself"


def esc_fft_harmonic(sitl):
    sitl.fly_hover(60)
    hit = sitl.wait_text("FFT peak")
    assert "motor harmonic 1" in hit, f"SITL's dominant peak is the motor fundamental: {hit}"


def frame_resonance(sitl):
    """A strong fixed tone between motor harmonics should earn an FFT-driven second bank."""
    sitl.fly_hover(60)
    step = sitl.wait_text("notch step")
    assert sitl.saw("separate resonance"), "120 Hz tone not classified as resonance"
    assert "HNTC2_MODE=4" in step, f"expected FFT-driven second bank: {step}"


def motor_imbalance(sitl):
    sitl.fly_hover()
    hit = sitl.wait_text("REFUSED")
    assert "motor 1 output" in hit or "yaw" in hit, f"unexpected refusal reason: {hit}"
    assert sitl.get("INS_HNTCH_ENABLE") == 0, "refusal must not change filters"


def verify_fail_restores(sitl):
    sitl.fly_hover()
    assert sitl.saw("notch step"), "first flight should start the notch step"
    # A non-motor 45 Hz resonance the new notch cannot track makes the verification flight fail.
    sitl.set("SIM_VIB_FREQ_X", 45)
    sitl.fly_hover()
    sitl.wait_text("notch step failed")
    assert sitl.saw("restored"), "backup not restored"
    assert sitl.get("INS_HNTCH_ENABLE") == 0, "previous INS_HNTCH_ENABLE not restored"


def no_esc_uses_fft(sitl):
    sitl.fly_hover(60)
    step = sitl.wait_text("notch step")
    assert "HNTCH_MODE=4" in step and "HNTCH_FM_RAT=1" in step, f"expected FFT-driven notch: {step}"


def short_hover_refused(sitl):
    sitl.fly_hover(2)
    hit = sitl.wait_text("REFUSED")
    assert "steady hover" in hit, hit


SCENARIOS = {
    "auto_full": (auto_full, {"INS_GYRO_FILTER": 40}),
    "manual_mode_waits": (manual_mode_waits, {}),
    "esc_fft_harmonic": (esc_fft_harmonic, {"FFT_ENABLE": 1}),
    # Lua sees only the FFT's dominant hover peak, so the resonance must out-weigh the motor noise here.
    "frame_resonance": (
        frame_resonance,
        {"FFT_ENABLE": 1, "SIM_VIB_FREQ_X": 120, "SIM_VIB_FREQ_Y": 120, "SIM_VIB_MOT_MULT": 0.2},
    ),
    "motor_imbalance": (motor_imbalance, {"SIM_ENGINE_FAIL": 1, "SIM_ENGINE_MUL": 0.8}),
    "verify_fail_restores": (verify_fail_restores, {}),
    "no_esc_uses_fft": (no_esc_uses_fft, {"SIM_ESC_TELEM": 0, "FFT_ENABLE": 1}),
    "short_hover_refused": (short_hover_refused, {}),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("scenarios", nargs="*", default=list(SCENARIOS))
    args = parser.parse_args()
    results = {}
    for name in args.scenarios:
        test, params = SCENARIOS[name]
        print(f"== {name}")
        sitl = Sitl(args.ardupilot.expanduser(), params)
        try:
            sitl.pump(5)
            sitl.set("AFT_ENABLE", 1)
            sitl.set("AFT_AUTO", 1)  # see Sitl.settle: SITL cannot reboot from the scripting thread
            test(sitl)
            results[name] = "PASS"
        except (AssertionError, RuntimeError, OSError) as error:
            results[name] = f"FAIL: {type(error).__name__}: {error} (run dir {sitl.run})"
        finally:
            sitl.close()
    for name, result in results.items():
        print(f"{name}: {result}")
    sys.exit(0 if all(r == "PASS" for r in results.values()) else 1)


if __name__ == "__main__":
    main()
