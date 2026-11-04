# SPDX-License-Identifier: Apache-2.0
"""Bounded real-bridge protocol test. Does not connect to MP or any aircraft."""

import argparse
import json
import math
import os
import signal
import socket
import struct
import subprocess
import time
import uuid
from pathlib import Path

from build_manifest import verify


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve(strict=True)
    if (workspace / ".nomad-simulation-owned").read_text().strip() != "NOMAD simulation sources v1":
        raise ValueError("Unowned source workspace")
    binary = workspace / "build/last-letter/last_letter_ardupilot"
    manifest = verify(workspace, "last_letter", "13baa815e971abbcc568bea1e265026b44686273", binary)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 19202))
    run = workspace / "runs" / ("last-letter-" + uuid.uuid4().hex)
    run.mkdir(parents=True)
    result = {
        "schema": 1,
        "cohort": "last-letter-valid-frame-protocol",
        "passed": False,
        "build": manifest,
        "cases": [],
        "source_limits": {},
    }
    process = None

    def interrupted(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        with (run / "bridge.log").open("w") as log, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            process = subprocess.Popen(
                [
                    str(binary),
                    "--model",
                    "ardupilot_copter",
                    "--address",
                    "127.0.0.1",
                    "--port",
                    "19202",
                    "--models-dir",
                    str(workspace / "sources/last_letter/models"),
                    "--log",
                    str(run / "flight.mcap"),
                ],
                cwd=run,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            end = time.monotonic() + 10
            while "listening for ArduPilot" not in (run / "bridge.log").read_text():
                if process.poll() is not None or time.monotonic() >= end:
                    raise TimeoutError("last_letter startup deadline")
                time.sleep(0.02)
            peer.bind(("127.0.0.1", 0))
            peer.connect(("127.0.0.1", 19202))
            peer.settimeout(1)

            def packet(count, rate=400, channels=16):
                return struct.pack(
                    "<HHI" + str(channels) + "H", 18458 if channels == 16 else 29569, rate, count, *([1000] * channels)
                )

            def receive():
                raw = peer.recv(4096)
                value = json.loads(raw)
                vectors = [value[key] for key in ("position", "velocity", "quaternion", "velocity_wind")]
                vectors.extend(value["imu"][key] for key in ("gyro", "accel_body"))
                assert all(math.isfinite(x) for vector in vectors for x in vector), value
                assert abs(sum(x * x for x in value["quaternion"]) - 1) <= 1e-6, value
                assert math.isfinite(value["timestamp"]) and math.isfinite(value["airspeed"]), value
                return raw, value

            def step(count, expected, rate=400, channels=16):
                peer.send(packet(count, rate, channels))
                raw, value = receive()
                assert abs(value["timestamp"] - expected) <= 1e-9, value
                result["cases"].append(
                    {"frame": count, "rate": rate, "channels": channels, "timestamp": value["timestamp"]}
                )
                return raw

            first = step(0, 0.0025)
            assert step(0, 0.0025) == first, "Duplicate must resend identical state without stepping"
            step(1, 0.005)
            step(2, 0.010, 200)
            step(3, 0.015, 200, 32)
            for invalid in (b"\0" * 40, packet(4)[:10]):
                peer.send(invalid)
                peer.settimeout(0.2)
                try:
                    peer.recv(4096)
                    raise AssertionError("Malformed magic/short packet produced a state")
                except TimeoutError:
                    pass
            peer.settimeout(1)
            step(4, 0.020, 200)
            peer.settimeout(0.25)
            try:
                peer.recv(4096)
                raise AssertionError("Physics emitted unsolicited state while paused")
            except TimeoutError:
                pass
            peer.settimeout(1)
            step(5, 0.025, 200)
            # Record upstream validation limits separately from valid-frame qualification.
            peer.send(packet(6, 200, 32) + b"oversized")
            peer.settimeout(0.2)
            try:
                _, value = receive()
                result["source_limits"]["oversized_32_channel_datagram_accepted"] = True
                assert abs(value["timestamp"] - 0.030) <= 1e-9
            except TimeoutError:
                result["source_limits"]["oversized_32_channel_datagram_accepted"] = False
            peer.settimeout(1)
            previous = 0.030 if result["source_limits"]["oversized_32_channel_datagram_accepted"] else 0.025
            peer.send(packet(7, 0))
            _, zero = receive()
            delta = zero["timestamp"] - previous
            result["source_limits"]["zero_rate_uses_previous_timestep"] = abs(delta - 0.005) <= 1e-9
            assert result["source_limits"]["zero_rate_uses_previous_timestep"], zero
            result["source_limits"]["zero_rate_observed_timestamp"] = zero["timestamp"]
            step(0, 0.0025, 400)
            result["passed"] = True
    except Exception as error:
        result["failure"] = f"{type(error).__name__}: {error}"
    finally:
        if process is not None:
            start = time.monotonic()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                    result["passed"] = False
            result["exit_code"] = process.returncode
            result["cleanup_seconds"] = time.monotonic() - start
            log_path = run / "flight.mcap"
            if log_path.exists() and log_path.stat().st_size >= 16:
                with log_path.open("rb") as log:
                    header = log.read(8)
                    log.seek(-8, 2)
                    result["mcap_closed"] = header == log.read(8) == b"\x89MCAP0\r\n"
            else:
                result["mcap_closed"] = False
            result["passed"] = result["passed"] and process.returncode == 0 and result["mcap_closed"]
        (run / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {"passed": result["passed"], "result": str(run / "result.json"), "source_limits": result["source_limits"]}
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
