<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# FHA-lite — Functional Hazard Assessment

What NOMAD can *cause* that is dangerous. For each hazard: a severity, the
current mitigation, the SC code that implements it, the test that proves it, and
the honest residual risk. This table **is** the spine of the safety case — every
mitigation must name code and a test, or be marked **GAP**.

Severity scale (proportionate, not DAL-formal):
**Catastrophic** — likely injury / loss of aircraft over people ·
**Hazardous** — loss of aircraft / property damage ·
**Major** — controllability or mission degraded, recoverable.

| ID | Hazard | Severity | Current mitigation | SC code | Proof (test) | Residual risk |
|----|--------|----------|--------------------|---------|--------------|---------------|
| H-01 | **Uncommanded / excessive velocity** — a bad `/cmd_vel` drives the aircraft too fast or in the wrong axis. | Hazardous | Velocity clamps (XY ≤2.0, Z ≤1.0 m/s, yaw ≤1.0 rad/s) + finite check before send; FLU→FRD frame conversion verified. | `safety/limits.py::VelocityLimits.clamp_command`, `safety/envelope.py::evaluate` (frame convert) ([safety/](../../edge_core/safety/)) | `tests/test_safety_limits.py`, `tests/test_safety_envelope.py::test_allows_and_clamps_before_converting` / `::test_allows_and_frame_converts_when_all_gates_pass`; **SITL-proven** (commanded vx → real motion within clamp) | Clamp *limits* are constants, not yet derived from a requirement reviewed against the airframe. FC enforces its own params as backstop. |
| H-02 | **Stale-VIO flyaway** — velocity commanded while visual-inertial odometry is stale or low-confidence, so the aircraft flies blind. | Catastrophic | VIO-freshness + confidence + healthy gate refuses setpoints; watchdog zeroes velocity if VIO goes stale mid-motion. | `safety/gates.py::vio_ready`, `safety/watchdog.py::watchdog_decision` (via `envelope.evaluate`) | `tests/test_safety_gates.py::test_vio_ready_*`, `tests/test_safety_envelope.py::test_rejects_when_vio_*`, `tests/test_safety_watchdog.py::test_vio_stale_stops_with_reason`; **SITL-proven** (VIO-stale → vehicle stops) | Gate thresholds (`min_vio_confidence=0.3`, `vio_max_age_s=1.0`) tested but not yet derived from a reviewed requirement value. |
| H-03 | **Loss of link during GUIDED** — companion→FC link drops while streaming velocity; aircraft keeps last setpoint. | Hazardous | Heartbeat-freshness gate drops `cmd_vel` with no fresh FC heartbeat; command-timeout watchdog (0.5 s) zeroes velocity; FC's own GCS-failsafe is the independent backstop. | `safety/gates.py::heartbeat_fresh`, `safety/watchdog.py::watchdog_decision` | `tests/test_safety_envelope.py::test_rejects_when_heartbeat_stale`, `tests/test_safety_watchdog.py::test_command_timeout_stops_with_reason`; **SITL-proven** (stop commanding → vehicle stops, 1.5→0.04 m/s) | Relies on FC failsafe for the link-loss-after-last-good-command case. |
| H-04 | **Mode change out of GUIDED** — operator/FC leaves GUIDED but companion keeps sending velocity targets. | Major | Armed + GUIDED-mode gate parsed from FC HEARTBEAT; setpoints dropped unless armed and mode == GUIDED. HEARTBEAT is **filtered to the commanded autopilot** so a GCS heartbeat on the link can't flip the gate (SR-VEL-06). | `safety/envelope.py::evaluate` (armed/guided gates), `safety/gates.py::heartbeat_from_vehicle`, `mavlink_velocity._rx_loop` | `tests/test_safety_envelope.py::test_rejects_when_not_armed`, `::test_rejects_when_not_guided`, `tests/test_safety_gates.py::test_heartbeat_from_vehicle_rejects_gcs`; **SITL-proven** (LOITER → setpoints refused) | — |
| H-05 | **Geofence breach** — aircraft commanded outside the operating boundary. | Hazardous | Boundary defined + uploaded to FC; FC enforces fence independently. A NOMAD-side containment **primitive** now exists but is not yet wired. | C#: `BoundaryManager`, `MPFenceUploader` ([Geofence/](../../mission_planner/src/Geofence/)); Python primitive `safety/geofence.py` (unwired) | `tests/test_safety_geofence.py` proves the primitive. **GAP** — primitive not yet enforced in the command path; FC fence + manual verification remain the only active enforcement. | Enforcement is still FC-side + manual; Phase 3 wires the primitive in and proves it in SITL. |
| H-06 | **Unintended payload release / pump energized** — relay/servo actuated without operator intent, or pump left energized. | Major | Channel + PWM range validation (1–16, 500–2500 µs); pump duration clamped 0.05–5.0 s; **best-effort relay-off in `finally` and on failure** so the pump cannot be left on. | `ServoController.trigger_water_shooter`, `set_channel_pwm`, `MavlinkServo.set_pwm` ([servo.py](../../edge_core/modules/payload/servo.py)) | **GAP** — no unit test for the de-energize-on-failure / duration-clamp guarantees. | No explicit arm/confirm **interlock** on the release path yet (rearchitecture §3.3). |
| H-07 | **Failsafe suppression** — NOMAD floods STATUSTEXT/commands or holds a mode such that an FC failsafe is masked. | Hazardous | NOMAD never disables FC failsafes; commands are advisory; STATUSTEXT truncated to 50 chars; FC failsafes run independently of the companion. | `send_statustext`, `set_mode` (no failsafe-disable command exists in the surface) | **GAP** — no test asserting the command surface contains no failsafe-disabling MAVLink command. | By-construction today; worth an explicit "forbidden command" test. |
| H-08 | **Unauthorized command** — a command reaches the edge API without authorization. | Hazardous | API-key auth middleware: key required, loopback-dev fallback, explicit insecure-remote opt-in, length-checked internal-bridge token, exempt paths. | `api.py` auth middleware (Tier SR) | `tests/test_auth_middleware.py` | **GAP (target)** — §4.6: command-path endpoints should require auth *even on loopback* and log operator identity for audit. Not yet enforced. |

## How to read the GAPs

A **GAP** is a deliberately-visible piece of unfinished safety work, not a
defect being hidden. The Phase 3 work (rearchitecture plan §4.3, §4.5) is
precisely: turn every GAP above into a named test or SITL scenario.

**Phase 2** closed the velocity-path GAPs (H-01..H-04) with the pure,
100%-covered `edge_core/safety/` package + fault-injection unit tests. **Phase 3**
added the proof layer: the velocity path is now **loop-closure-proven against
real ArduPilot SITL** ([../../tests/sitl/](../../tests/sitl/)) — commanded
motion, watchdog stop, and the GUIDED gate all verified on a live autopilot.
That SITL run also surfaced and fixed a real defect (the gate could be flipped by
a GCS heartbeat sharing the link — now filtered, SR-VEL-06).

Remaining open items, in order:

1. **H-05 geofence** — containment primitive exists (`safety/geofence.py`) but is
   not yet wired into the command path; FC fence + manual remain the only active
   enforcement. Add a geofence-approach SITL scenario once wired.
2. **H-06 payload interlock** — release path has range/duration checks but no
   arm/confirm interlock, and the payload path is not yet extracted/tested.
