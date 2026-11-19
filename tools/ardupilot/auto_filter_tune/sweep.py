# SPDX-License-Identifier: Apache-2.0
"""Airframe sweep for auto_filter_tune.lua: hover until DONE per airframe, then grade it from the logs.

Usage (Linux/WSL):
  python3 sweep.py --ardupilot ~/ardupilot --plane-ardupilot ~/nomad-simulation/sources/ardupilot \
      --out results.json [--script copy.lua] [--jobs 6] [--repeat 1] [case ...]

The grade does not use the script's own noise estimate. It compares batch-logged pre- and post-filter gyro
spectra on the first flight (starting filters) and on one extra flight after DONE (final filters), and computes
the final filter chain's phase lag at 10 Hz analytically. SITL is hardware-free and does not qualify real aircraft.
"""

import argparse
import contextlib
import glob
import json
import math
import multiprocessing
import time
from pathlib import Path

import numpy as np
import sitl_test
from pymavlink import DFReader, mavutil
from sitl_test import Sitl

# Batch-log raw and filtered gyro of IMU1, so the grade sees exactly what the rate loop sees.
LOGGING = {"INS_LOG_BAT_MASK": 1, "INS_LOG_BAT_OPT": 4, "LOG_BITMASK": 196607}
START = {"INS_GYRO_FILTER": 20}  # ArduPilot's default: an untuned aircraft starts here
GUSTY = {"SIM_WIND_SPD": 6, "SIM_WIND_DIR": 270, "SIM_WIND_TURB": 4}

# name: (vehicle, model, extra default files, params). Hover motor frequency = output x SIM_VIB_MOT_MAX.
# SIM_VIB_MOT_HMNC 3 = 1st+2nd harmonic (2-blade), 7 = 1st-3rd (3-blade).
FRAMES = {
    "fpv5_3blade": (
        "copter",
        "X:@ROMFS/models/freestyle.json",
        ["default_params/copter-X.parm", "models/freestyle.param"],
        {"SIM_VIB_MOT_MAX": 900, "SIM_VIB_MOT_HMNC": 7, "INS_HNTC2_ENABLE": 0, "INS_GYRO_FILTER": 80},
    ),
    "quad10_2blade": ("copter", "quad", [], {"SIM_VIB_MOT_MAX": 230, "SIM_VIB_MOT_HMNC": 3}),
    "hexa13_2blade": (
        "copter",
        "hexa",
        ["default_params/copter-hexa.parm"],
        {"SIM_VIB_MOT_MAX": 200, "SIM_VIB_MOT_HMNC": 3},
    ),
    # Large-prop octa (~48 Hz hover). SITL's Callisto model overshoots to 30 m at zero throttle output in
    # GUIDED, so the generic octa stands in with large-prop motor speeds.
    "octa_large": (
        "copter",
        "octa",
        ["default_params/copter-octa.parm"],
        {"SIM_VIB_MOT_MAX": 90, "SIM_VIB_MOT_HMNC": 3, "SIM_GYR1_RND": 10},
    ),
    # Vectored-yaw tilt-tricopter quadplane. ArduPlane's built-in physics stands in for the NOMAD Gazebo
    # fixture: the JSON backend carries no motor RPM, so it gets neither RPM-scaled vibration nor ESC RPM.
    "tilttri_plane": (
        "plane",
        "quadplane-tilttrivec",
        ["default_params/quadplane-tilttrivec.parm"],
        # SITL numbers ESC telemetry by motor slot, not output, so the tri's rear motor (SERVO8) never reports
        # RPM: this case exercises the FFT path. SERVO7 is quadplane.parm's unused Motor3.
        {
            "SIM_VIB_MOT_MAX": 200,
            "SIM_VIB_MOT_HMNC": 3,
            "MAV_GCS_SYSID": 250,
            "Q_A_RAT_YAW_P": 0.5,
            "SIM_ESC_TELEM": 0,
            "FFT_ENABLE": 1,
            "SERVO7_FUNCTION": 0,
        },
    ),
}
# 4 kHz gyro with the fast rate thread. Stock SITL registers its IMU at 1 kHz whatever INS_GYRO_RATE says: run these
# with --ardupilot pointing at a build of ArduPilot PR #34642 (SITL gyro rate follows INS_GYRO_RATE). Per that PR,
# SIM_RATE_HZ is raised to the gyro rate and FSTRATE_ENABLE=3 fixes the divisor, so catch-up bursts are not read
# as the thread running slow.
# At 10x a desktop cannot simulate 4 kHz plus the rate thread and drops rate-loop samples: SPEEDUP 2.
# The 5-inch freestyle JSON model does not load in that newer tree, so only the 10-inch quad runs at 4 kHz.
FAST = {"INS_FAST_SAMPLE": 1, "INS_GYRO_RATE": 2, "FSTRATE_ENABLE": 3, "SIM_RATE_HZ": 4000, "SPEEDUP": 2}
_quad10 = FRAMES["quad10_2blade"]
FRAMES["quad10_2blade_4khz"] = (*_quad10[:3], {**_quad10[3], **FAST})
FRAMES["quad10_2blade_4khz_h7"] = (*_quad10[:3], {**_quad10[3], **FAST, "AFT_NOTCH_MAX": 48})
CASES = {f"{f}_{w}": (f, w) for f in FRAMES for w in ("calm", "gusty")}


# ---------------------------------------------------------------------------------------------------------
# Flying
# ---------------------------------------------------------------------------------------------------------
def landed(m):
    return m.get_type() == "HEARTBEAT" and m.autopilot != 8 and not m.base_mode & 128


def fly_qhover(sitl, hover_s=45):
    """ArduPlane VTOL hover: QLOITER climb on RC throttle override, hold, QLAND (which disarms on touchdown)."""
    sitl.mark = len(sitl.text)
    mav = sitl.mav

    def throttle(pwm):
        mav.mav.rc_channels_override_send(1, 1, 0, 0, pwm, 0, 0, 0, 0, 0)

    sitl.mav.set_mode("QLOITER")
    throttle(1000)
    deadline = time.time() + 120
    while not sitl.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
        assert time.time() < deadline, "vehicle never armed (EKF/GPS not ready, or arming blocked)"
        throttle(1000)
        sitl.pump(2)
    end = time.time() + 60
    while time.time() < end:
        throttle(1800)
        if sitl.pump(0.5, lambda m: m.get_type() == "GLOBAL_POSITION_INT" and m.relative_alt > 9000):
            break
    else:
        raise AssertionError("did not reach 9 m")
    throttle(1500)
    for _ in range(hover_s // 5):  # refresh the override while holding
        sitl.wait_sim(5)
        throttle(1500)
    sitl.mav.set_mode("QLAND")
    end = time.time() + 90
    while time.time() < end:
        throttle(1500)
        if sitl.pump(0.5, landed):
            break
    else:
        raise AssertionError("did not land and disarm")
    throttle(0)  # release the override
    sitl.settle()


def fly(sitl, vehicle):
    with contextlib.suppress(Exception):
        sitl.mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    if vehicle == "plane":
        fly_qhover(sitl)
    else:
        sitl.fly_hover()


# ---------------------------------------------------------------------------------------------------------
# Grading
# ---------------------------------------------------------------------------------------------------------
def notch_response(f, fs, center, bw, att_db):
    """ArduPilot NotchFilter (AP_Filter NotchFilter.cpp) frequency response at f."""
    if center <= 0.5 * bw or center >= 0.48 * fs:
        return 1.0
    a = 10 ** (-att_db / 40)
    octaves = math.log2(center / (center - bw / 2)) * 2
    q = math.sqrt(2**octaves) / (2**octaves - 1)
    omega = 2 * math.pi * center / fs
    alpha = math.sin(omega) / (2 * q)
    b = [1 + alpha * a * a, -2 * math.cos(omega), 1 - alpha * a * a]
    den = [1 + alpha, -2 * math.cos(omega), 1 - alpha]
    z = np.exp(-2j * math.pi * f / fs)
    return (b[0] + b[1] * z + b[2] * z * z) / (den[0] + den[1] * z + den[2] * z * z)


def lpf_response(f, fs, cutoff):
    """ArduPilot LowPassFilter2p (2-pole Butterworth biquad) response at f."""
    if cutoff <= 0:
        return 1.0
    fr = fs / cutoff
    ohm = math.tan(math.pi / fr)
    c = 1 + 2 * math.cos(math.pi / 4) * ohm + ohm * ohm
    b0 = ohm * ohm / c
    a1 = 2 * (ohm * ohm - 1) / c
    a2 = (1 - 2 * math.cos(math.pi / 4) * ohm + ohm * ohm) / c
    z = np.exp(-2j * math.pi * f / fs)
    return b0 * (1 + 2 * z + z * z) / (1 + a1 * z + a2 * z * z)


def filter_lag_deg(p, motor_hz, f=10.0):
    """Phase lag of the gyro filter chain at f for parameters p, notches centred on the hover motor speeds."""
    fs = 1000 * 2 ** int(p.get("INS_GYRO_RATE", 0))
    h = lpf_response(f, fs, p.get("INS_GYRO_FILTER", 20))
    for bank in ("INS_HNTCH", "INS_HNTC2"):
        if p.get(bank + "_ENABLE", 0) < 1:
            continue
        freq, bw = p.get(bank + "_FREQ", 80), p.get(bank + "_BW", 40)
        att = p.get(bank + "_ATT", 40)
        opts, hmncs, mode = int(p.get(bank + "_OPTS", 0)), int(p.get(bank + "_HMNCS", 1)), int(p.get(bank + "_MODE", 1))
        per_motor = opts & 2 and mode == 3
        sources = motor_hz if per_motor else [np.mean(motor_hz) if mode in (3, 4) and motor_hz else freq]
        for src in sources:
            centre = max(src, freq)
            for k in range(16):
                if hmncs & (1 << k):
                    # bandwidth scales with the centre: Q is fixed by FREQ/BW
                    h = h * notch_response(f, fs, centre * (k + 1), bw * centre / freq, att)
    return -math.degrees(np.angle(h))


def grade_log(path):
    """Hover-only (>8 m) gyro spectra from batch samples: band RMS above 0.6 x motor fundamental, pre and post."""
    r = DFReader.DFReader_binary(str(path), zero_time_base=True)
    alt_ok, head, batches, rpm = False, None, {}, {}
    while True:
        m = r.recv_match(type=["POS", "ISBH", "ISBD", "ESC"])
        if m is None:
            break
        t = m.get_type()
        if t == "POS":
            alt_ok = m.RelHomeAlt > 8
        elif t == "ISBH":
            head = m if (m.type == 1 and alt_ok) else None
            if head:
                batches[m.N] = {"post": m.instance != 0, "rate": m.smp_rate, "mul": m.mul, "x": [], "y": []}
        elif t == "ISBD" and head and m.N in batches:
            batches[m.N]["x"] += list(m.x)
            batches[m.N]["y"] += list(m.y)
        elif t == "ESC" and alt_ok and m.RPM > 0:
            rpm.setdefault(m.Instance, []).append(m.RPM / 60)
    motor_hz = [float(np.median(v)) for v in rpm.values()]
    out = {"motor_hz": motor_hz}
    for kind in ("pre", "post"):
        psd, rate = [], None
        for b in batches.values():
            if b["post"] != (kind == "post") or len(b["x"]) < 256:
                continue
            rate = b["rate"]
            for axis in ("x", "y"):
                s = np.degrees(np.asarray(b[axis], float) / b["mul"])
                s = (s - s.mean()) * np.hanning(len(s))
                psd.append(np.abs(np.fft.rfft(s)) ** 2 / len(s))
        if not psd:
            return None
        n = min(len(p) for p in psd)
        mean = np.mean([p[:n] for p in psd], axis=0)
        freqs = np.fft.rfftfreq(2 * (n - 1), 1 / rate)
        if kind == "pre":
            # no ESC telemetry (FFT path): take the fundamental as the raw spectrum's strongest peak above 20 Hz
            f0 = min(motor_hz) if motor_hz else float(freqs[np.argmax(np.where(freqs > 20, mean, 0))])
        band = freqs > 0.6 * f0
        out[kind + "_rms"] = float(math.sqrt(mean[band].sum() * 2 / (n * 0.375)))
        peak = int(np.argmax(np.where(band, mean, 0)))
        out[kind + "_peak_hz"] = float(freqs[peak])
    out["atten_db"] = 20 * math.log10(out["post_rms"] / out["pre_rms"])
    return out


FILTER_PARAMS = [
    "INS_GYRO_FILTER",
    "INS_GYRO_RATE",
    *(
        f"{b}_{k}"
        for b in ("INS_HNTCH", "INS_HNTC2")
        for k in ("ENABLE", "MODE", "FREQ", "BW", "HMNCS", "OPTS", "REF", "ATT", "FM_RAT")
    ),
]


def filter_params(sitl):
    p = {}
    for name in FILTER_PARAMS:
        with contextlib.suppress(AssertionError):
            p[name] = sitl.get(name)
    return p


# ---------------------------------------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------------------------------------
def run_case(job):
    name, rep, args = job
    frame, wind = CASES[name]
    vehicle, model, defaults, params = FRAMES[frame]
    instance = multiprocessing.current_process()._identity[0] if multiprocessing.current_process()._identity else 0
    sitl_test.SCRIPT = Path(args["script"])
    tree = args["plane_ardupilot"] if vehicle == "plane" else args["ardupilot"]
    base = "default_params/quadplane.parm" if vehicle == "plane" else "default_params/copter.parm"
    result = {"case": name, "rep": rep, "flights": 0, "outcome": "?", "steps": []}
    logfile = Path(args["logdir"]) / f"{name}-{rep}.txt"
    started = time.time()
    with open(logfile, "w") as out, contextlib.redirect_stdout(out):
        aft = {k: v for k, v in params.items() if k.startswith("AFT_")}  # exist only once the script runs
        speedup = params.get("SPEEDUP", sitl_test.SPEEDUP)
        params = {k: v for k, v in params.items() if k not in aft and k != "SPEEDUP"}
        try:
            sitl = Sitl(
                tree,
                {**LOGGING, **START, **params, **(GUSTY if wind == "gusty" else {})},
                instance=args["instance_base"] + instance % 20,
                model=model,
                defaults=defaults,
                speedup=speedup,
                base=base,
                binary=f"{tree}/build/sitl/bin/ardu{vehicle}",
            )
        except RuntimeError as error:  # one case that cannot start must not end the sweep
            return {**result, "outcome": f"ERROR {error}", "wall_s": round(time.time() - started)}
        result["run"] = str(sitl.run)
        try:
            sitl.pump(5)
            sitl.set("AFT_ENABLE", 1)
            sitl.set("AFT_AUTO", 1)
            for k, v in aft.items():
                sitl.set(k, v)
            result["start"] = filter_params(sitl)
            for _ in range(args["max_flights"]):
                fly(sitl, vehicle)
                result["flights"] += 1
                if sitl.saw("DONE"):
                    result["outcome"] = "DONE"
                    break
                if sitl.saw("REFUSED"):
                    result["outcome"] = "REFUSED"
                    result["refusals"] = result.get("refusals", 0) + 1
                    if result["refusals"] >= 3:
                        break
            else:
                result["outcome"] = "MAX_FLIGHTS"
            result["final"] = filter_params(sitl)
            fly(sitl, vehicle)  # grading flight on the final filters
        except Exception as error:  # record and grade whatever was flown
            result["outcome"] = f"ERROR {type(error).__name__}: {error}"
        finally:
            sitl.close()
    result["text"] = [t for t in sitl.text if any(k in t for k in ("step", "REFUSED", "DONE", "motor ", "hover "))]
    result["steps"] = [t for t in result["text"] if "step" in t]
    logs = sorted(glob.glob(f"{sitl.run}/logs/0*.BIN"))
    if logs:
        result["grade_first"] = grade_log(logs[0])
        result["grade_last"] = grade_log(logs[-1])
        g = result["grade_last"] or result["grade_first"]
        if g:
            hz = g["motor_hz"]
            result["lag_start_deg"] = filter_lag_deg(result.get("start", {}), hz)
            result["lag_final_deg"] = filter_lag_deg(result.get("final", result.get("start", {})), hz)
    result["wall_s"] = round(time.time() - started)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ardupilot", type=Path, required=True)
    parser.add_argument("--plane-ardupilot", type=Path)
    parser.add_argument("--script", type=Path, default=sitl_test.SCRIPT)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--max-flights", type=int, default=16)
    parser.add_argument("--instance-base", type=int, default=0, help="SITL -I offset, to run sweeps side by side")
    parser.add_argument("cases", nargs="*", default=list(CASES))
    args = parser.parse_args()
    logdir = args.out.with_suffix("")
    logdir.mkdir(parents=True, exist_ok=True)
    shared = {
        "ardupilot": str(args.ardupilot.expanduser()),
        "plane_ardupilot": str((args.plane_ardupilot or args.ardupilot).expanduser()),
        "script": str(args.script.resolve()),
        "logdir": str(logdir),
        "max_flights": args.max_flights,
        "instance_base": args.instance_base,
    }
    jobs = [(c, r, shared) for r in range(args.repeat) for c in args.cases]
    results = []
    with multiprocessing.Pool(args.jobs) as pool:
        for res in pool.imap_unordered(run_case, jobs):
            results.append(res)
            print(summary_line(res), flush=True)
            args.out.write_text(json.dumps(results, indent=1))


def summary_line(r):
    a, b = r.get("grade_first") or {}, r.get("grade_last") or {}
    f = r.get("final", {})
    return (
        f"{r['case']:<26} {r['outcome'][:40]:<40} flights {r['flights']:>2} "
        f"post {a.get('post_rms', float('nan')):6.2f}->{b.get('post_rms', float('nan')):6.2f} dps "
        f"lag10 {r.get('lag_start_deg', float('nan')):5.1f}->{r.get('lag_final_deg', float('nan')):5.1f} deg "
        f"LPF {f.get('INS_GYRO_FILTER', 0):.0f} FREQ {f.get('INS_HNTCH_FREQ', 0):.0f} "
        f"BW {f.get('INS_HNTCH_BW', 0):.0f} H {f.get('INS_HNTCH_HMNCS', 0):.0f} O {f.get('INS_HNTCH_OPTS', 0):.0f} "
        f"wall {r['wall_s']}s"
    )


if __name__ == "__main__":
    main()
