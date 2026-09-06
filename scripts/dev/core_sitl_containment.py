# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL containment scenario for the NOMAD-side geofence (SR-FEN-02, H-05).

Drives a real ArduPilot SITL vehicle through the **C++ core CLI** and proves
the projected keep-in fence closes the loop on a real autopilot:

  read home -> configure a keep-in fence box around it (NOMAD_FENCE_POLYGON)
    -> arm -> GUIDED -> takeoff (via the C++ CLI)
    -> goto target INSIDE the fence   (accepted; vehicle actually moves)
    -> goto target OUTSIDE the fence  (rejected before transmission; vehicle
       stays contained)

The vehicle command path is entirely the C++ core (mode, arm, takeoff, goto,
rtl, land); this script only configures the fence environment for the CLI,
spawns it, and polls the CLI's own ``status`` output for authoritative
telemetry. Replaces the deleted Python ``MavlinkService`` scenario — see the
Phase 7 deletion ledger in docs/migration.md.

Run against the dev stack: ``pixi run sitl-fence``.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from core_sitl_command_flow import ScenarioError, parse_status, run_cli, wait_for_altitude, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint

# Keep-in box half-extent around home (m) and margin (m). The out-of-fence
# target is well beyond the box; the in-fence target well within it.
FENCE_HALF_EXTENT_M = 100.0
FENCE_MARGIN_M = 5.0
INSIDE_TARGET_NORTH_M = 30.0
OUTSIDE_TARGET_NORTH_M = 300.0
# ~111.2 km per degree of latitude at these scales (matches the C++ core's
# projected geofence approximation).
METERS_PER_DEGREE_LAT = 110_540.0


def fence_env_around(home: tuple[float, float]) -> str:
    """A square keep-in boundary around home, as NOMAD_FENCE_POLYGON syntax."""
    half_extent_deg = FENCE_HALF_EXTENT_M / METERS_PER_DEGREE_LAT
    lat, lon = home
    corners = [
        (lat + half_extent_deg, lon - half_extent_deg),
        (lat + half_extent_deg, lon + half_extent_deg),
        (lat - half_extent_deg, lon + half_extent_deg),
        (lat - half_extent_deg, lon - half_extent_deg),
    ]
    return ";".join(f"{clat:.7f},{clon:.7f}" for clat, clon in corners)


def read_position(binary: Path, port: str) -> tuple[float, float]:
    """Return the last authoritative latitude/longitude from CLI status."""
    status = parse_status(run_cli(binary, port, "status"))
    try:
        return tuple(float(part) for part in status["position"].split(","))  # type: ignore[return-value]
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"no valid position in status; last status={status}") from error


def wait_for_displacement(
    binary: Path, port: str, home: tuple[float, float], minimum_north_m: float, timeout: float
) -> None:
    """Poll status until the vehicle has moved at least ``minimum_north_m`` north."""
    import time

    end = time.monotonic() + timeout
    last = ""
    while time.monotonic() < end:
        lat, lon = read_position(binary, port)
        north_m = (lat - home[0]) * METERS_PER_DEGREE_LAT
        if north_m >= minimum_north_m:
            return
        last = f"{lat:.7f},{lon:.7f} (north {north_m:.1f} m)"
        time.sleep(1.0)
    raise ScenarioError(f"timed out waiting for {minimum_north_m:.0f} m north displacement; last position={last}")


def run_containment(binary: Path, port: str) -> None:
    initial = wait_for_status(binary, port, {"connected": "true"}, 15.0)
    if initial.get("armed") == "true":
        raise ScenarioError("SITL must start disarmed for the containment scenario")

    home = read_position(binary, port)
    fence = fence_env_around(home)
    os.environ["NOMAD_FENCE_POLYGON"] = fence
    os.environ["NOMAD_FENCE_MARGIN_M"] = str(FENCE_MARGIN_M)
    print(f"fence configured: +/-{FENCE_HALF_EXTENT_M:.0f} m box around ({home[0]:.6f}, {home[1]:.6f})", flush=True)

    run_cli(binary, port, "mode", "4")
    wait_for_status(binary, port, {"mode": "4"}, 15.0)
    run_cli(binary, port, "arm")
    wait_for_status(binary, port, {"armed": "true"}, 15.0)
    run_cli(binary, port, "takeoff", "10")
    wait_for_altitude(binary, port, 8.0, 45.0)
    print(f"reached altitude, flying fence tests (home {home[0]:.6f},{home[1]:.6f})", flush=True)

    inside = (home[0] + INSIDE_TARGET_NORTH_M / METERS_PER_DEGREE_LAT, home[1])
    run_cli(binary, port, "goto", f"{inside[0]:.7f}", f"{inside[1]:.7f}", "10")
    wait_for_displacement(binary, port, home, INSIDE_TARGET_NORTH_M * 0.5, 45.0)
    print("PASS: in-fence target accepted and flown (loop closed)", flush=True)

    outside = (home[0] + OUTSIDE_TARGET_NORTH_M / METERS_PER_DEGREE_LAT, home[1])
    result = run_cli(binary, port, "goto", f"{outside[0]:.7f}", f"{outside[1]:.7f}", "10")
    print(f"out-of-fence goto output: {result.strip()!r}", flush=True)
    max_north = max_displacement_after_reject(binary, port, home, seconds=10.0)
    if max_north > FENCE_HALF_EXTENT_M:
        raise ScenarioError(f"vehicle left the fence after a rejected target (north={max_north:.1f} m)")
    print(f"PASS: out-of-fence target rejected; vehicle contained (max north {max_north:.1f} m)", flush=True)

    run_cli(binary, port, "rtl")
    run_cli(binary, port, "land")
    wait_for_status(binary, port, {"armed": "false"}, 120.0)
    run_cli(binary, port, "disarm")


def max_displacement_after_reject(binary: Path, port: str, home: tuple[float, float], seconds: float) -> float:
    import time

    end = time.monotonic() + seconds
    max_north = 0.0
    while time.monotonic() < end:
        lat, _ = read_position(binary, port)
        max_north = max(max_north, (lat - home[0]) * METERS_PER_DEGREE_LAT)
        time.sleep(0.5)
    return max_north


def main() -> int:
    try:
        port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_containment(binary, port)
    except (ValueError, ScenarioError) as error:
        print(f"C++ SITL containment failed: {error}", file=sys.stderr)
        return 1
    print("C++ SITL containment passed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
