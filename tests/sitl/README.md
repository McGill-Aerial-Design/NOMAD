<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# SITL scenario suite

End-to-end loop-closure tests that drive a **real ArduPilot SITL** vehicle and
exercise the safety-critical C++ core (velocity watchdog, projected geofence)
against it. This is the §4.5 verification the safety case calls for: proving
the mitigations in [../../docs/safety.md](../../docs/safety.md) close the loop
on an actual autopilot, not just in unit tests.

These tests are **skipped in normal CI** (no autopilot present). They run on
demand against the dev stack.

## What the scenarios prove

[velocity_loop_closure.py](velocity_loop_closure.py) runs:

| Step | Hazard / requirement | Assertion |
|------|----------------------|-----------|
| arm → GUIDED → takeoff | — | vehicle reaches altitude |
| velocity step (`vx=1.5`) | H-01 / SR-VEL | commanded motion actually happens (peak groundspeed ≥ 0.8 m/s, within the 2.0 clamp) |
| stop commanding | H-03/H-02 / SR-LNK-02, SR-VIO-02 | C++ watchdog zeroes velocity; vehicle stops on its own |
| switch to LOITER | H-04 / SR-VEL-05 | C++ core refuses setpoints outside GUIDED |

The vehicle command path is driven by the **C++ core CLI** (`nomad velocity`);
the scenario's operator link only observes the vehicle and flips modes for the
gate test.

[geofence containment](../../scripts/dev/core_sitl_containment.py) (`pixi run
sitl-fence`) runs — it proves the C++ core's projected keep-in fence closes the
loop on a real autopilot. The C++ core CLI drives the whole vehicle path
(mode/arm/takeoff/goto); the script configures `NOMAD_FENCE_POLYGON` around the
read-back home position and polls the CLI's `status` for authoritative
position:

| Step | Hazard / requirement | Assertion |
|------|----------------------|-----------|
| configure a 100 m keep-in box around home | H-05 / SR-FEN-02 | fence parsed by the C++ core (`src/safety/fence_config.cpp`) |
| arm → GUIDED → takeoff (C++ CLI) | — | vehicle reaches altitude |
| goto 30 m north (inside) | H-05 / SR-FEN-02 | target accepted and actually flown |
| goto 300 m north (outside) | H-05 / SR-FEN-02 | target rejected before transmission; vehicle stays inside the box |

[gimbal_mount_control.py](gimbal_mount_control.py) (`pixi run sitl-gimbal`) runs
— it proves the MAVLink the plugin's gimbal control emits actually points a real
mount (the GCS counterpart of `pixi run test-plugin-gimbal`, which pins the
command bytes). It needs only `NOMAD_SITL_OPERATOR`:

| Step | Assertion |
|------|-----------|
| configure a servo mount (`MNT1_TYPE=1`, AUX tilt/roll) + reboot | `MNT1_TYPE` sticks |
| `DO_MOUNT_CONFIGURE` MAVLINK_TARGETING + `DO_MOUNT_CONTROL` pitch −45° | servo output moves proportionally |
| `DO_MOUNT_CONTROL` pitch +45° | servo output moves the opposite way |
| `DO_MOUNT_CONTROL` pitch beyond the limit | servo output saturates at the limit (clamped) |

> It reboots the vehicle to apply `MNT1_TYPE`, so the nightly job runs it **last**,
> after the velocity and fence scenarios that share the vehicle.
>
> **Run it on a vehicle that hasn't flown.** State left by the velocity/fence
> flights (and persisted across the mount reboot) suppresses mount actuation —
> the mount stops driving its servo / publishing attitude. The nightly recreates
> the SITL container (`docker compose ... up -d --force-recreate sitl`, which
> re-wipes EEPROM via `-w`) before this scenario; locally, run it after a fresh
> `pixi run dev-up` (or recreate `sitl`) rather than after the other scenarios.
>
> As a defensive net the scenario still **skips** (exit 2; the nightly step logs a
> warning) if the mount never actuates, rather than failing — the command bytes
> are independently gated in CI by `pixi run test-plugin-gimbal`.

## How to run

1. Bring up the hardware-free stack (Edge Core + ArduPilot SITL):

   ```sh
   pixi run dev-up
   ```

   ArduPilot needs ~20-30 s to reach "ArduPilot Ready" (EKF/GPS).

2. Run the scenario. ArduPilot SITL exposes two independent MAVLink links on
   TCP **5762** (SERIAL1, operator) and **5763** (SERIAL2, controller). The
   scenario runs inside the compose network so it can reach the `sitl` service:

   ```sh
   pixi run sitl-scenario
   ```

   or directly (the C++ core binary runs on the host, fed by the SITL UDP
   copy on `NOMAD_CORE_SITL_PORT`):

   ```sh
   pixi run build-core
   NOMAD_SITL_OPERATOR=tcp:127.0.0.1:5762 \
     NOMAD_API_KEY=nomad-dev-sitl-key \
     python tests/sitl/velocity_loop_closure.py
   ```

   A pass ends with `SCENARIO PASSED: {...}`; any unmet safety behaviour raises
   `ScenarioError` and exits non-zero.

### Watch a run live in Mission Planner

The dev stack publishes the SITL vehicle's operator MAVLink link on host TCP
**5762** (the `mp_bridge` compose service forwards SITL's SERIAL1 link).
While any scenario or host-side C++ smoke test is running, open Mission
Planner and connect as a passive observer — you see the vehicle fly/arm/land
in real time without disturbing the test:

1. Mission Planner → **CONNECT** (top right).
2. Dropdown: **TCP** → host `127.0.0.1`, port `5762` → **Connect**.
3. The flight map shows the vehicle at the SITL home position (42.3898,
   -71.1476); arm/takeoff/goto/land steps appear live on the HUD.

The C++ smoke scripts (`pixi run core-sitl-*`) print this hint when they
start. TCP 5762 accepts multiple clients, so Mission Planner never steals the
link from the scenario or the core CLI (UDP 14570).

3. Tear down:

   ```sh
   pixi run dev-down
   ```

## C++ core smoke and command flow

The C++ migration gates use the same Docker SITL service but receive a dedicated
host UDP telemetry copy on `NOMAD_CORE_SITL_PORT` (default `14570`):

```sh
pixi run core-sitl-status
pixi run core-sitl-command-flow
```

The first command proves typed telemetry without Edge Core. The second runs the
C++ CLI through GUIDED, arm, takeoff, RTL, land, and disarm while polling status
between transitions. It requires a fresh, disarmed SITL vehicle and should run
before the legacy velocity/fence scenarios.

## As a pytest

[test_velocity_loop_closure.py](test_velocity_loop_closure.py) wraps the
scenario and **skips** unless `NOMAD_SITL_OPERATOR` is set and the C++ core
binary is built, so it is safe to leave in the default `pixi run test` run.

## Notes

- The scenario runs from the pixi env on the host: pymavlink for the operator
  link and the freshly built C++ core binary for the command path, so it always
  tests **current** code.
- The operator link (arm / mode / observe) is MAVLink; the velocity command
  path is the C++ core CLI on its own UDP copy of the stream, so the core's
  watchdog is exercised end-to-end against a real autopilot.
