<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# SITL scenario suite

End-to-end loop-closure tests that drive a **real ArduPilot SITL** vehicle and
exercise the safety-critical velocity core (`MavlinkVelocityController`) against
it. This is the §4.5 verification the safety case calls for: proving the
mitigations in [../../docs/safety/hazards.md](../../docs/safety/hazards.md) close
the loop on an actual autopilot, not just in unit tests.

These tests are **skipped in normal CI** (no autopilot present). They run on
demand against the dev stack.

## What the scenarios prove

[velocity_loop_closure.py](velocity_loop_closure.py) runs:

| Step | Hazard / requirement | Assertion |
|------|----------------------|-----------|
| arm → GUIDED → takeoff | — | vehicle reaches altitude |
| velocity step (`vx=1.5`) | H-01 / SR-VEL | commanded motion actually happens (peak groundspeed ≥ 0.8 m/s, within the 2.0 clamp) |
| stop commanding | H-03/H-02 / SR-LNK-02, SR-VIO-02 | watchdog zeroes velocity; vehicle stops on its own |
| switch to LOITER | H-04 / SR-VEL-05 | setpoints refused outside GUIDED |

[geofence_containment.py](geofence_containment.py) (`pixi run sitl-fence`) runs:

| Step | Hazard / requirement | Assertion |
|------|----------------------|-----------|
| configure a 100 m keep-in box around home | H-05 / SR-FEN-02 | fence parsed from `NOMAD_FENCE_POLYGON` |
| arm → GUIDED → takeoff | — | vehicle reaches altitude |
| global target 30 m north (inside) | H-05 / SR-FEN-02 | target accepted and actually flown |
| global target 300 m north (outside) | H-05 / SR-FEN-02 | target rejected; vehicle stays inside the box |
| LOCAL_NED target 300 m north | H-05 / SR-FEN-02 | target rejected |

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

   or directly:

   ```sh
   docker run --rm --network nomad-dev_default \
     -v "$PWD:/work" -w /work -e PYTHONPATH=/work \
     -e NOMAD_SITL_OPERATOR=tcp:nomad-dev-sitl-1:5762 \
     -e NOMAD_SITL_CONTROLLER=tcp:nomad-dev-sitl-1:5763 \
     --entrypoint python nomad-edge-dev:latest tests/sitl/velocity_loop_closure.py
   ```

   A pass ends with `SCENARIO PASSED: {...}`; any unmet safety behaviour raises
   `ScenarioError` and exits non-zero.

3. Tear down:

   ```sh
   pixi run dev-down
   ```

## As a pytest

[test_velocity_loop_closure.py](test_velocity_loop_closure.py) wraps the
scenario and **skips** unless `NOMAD_SITL_OPERATOR` and `NOMAD_SITL_CONTROLLER`
are set, so it is safe to leave in the default `pixi run test` run.

## Notes

- The scenario uses the freshly-built `nomad-edge-dev` image for pymavlink + the
  package, but mounts the working tree at `/work` so it always tests **current**
  code, not what was baked at image-build time.
- Two MAVLink links are required because one drives the vehicle (arm / mode /
  observe) while the other is the controller under test. SERIAL1/SERIAL2 give
  two independent views without a router.
