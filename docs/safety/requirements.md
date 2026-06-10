<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# Safety Requirements (SC tier)

Short, testable statements. Each is **verifiable** (a test can pass or fail on
it), traces to a hazard in [hazards.md](hazards.md), and traces to code + test in
[traceability.md](traceability.md). Status is one of:

- ✅ **Met** — implemented and proven by an existing test.
- 🟡 **Partial** — implemented in code but not yet proven by a dedicated test.
- 🔴 **Open** — not yet implemented.

Numbers are stable identifiers; do not renumber. New requirements append.

## Velocity (VEL) — hazard H-01, H-04

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-VEL-01 | Lateral (XY) velocity setpoints sent to the FC shall be clamped to ≤ 2.0 m/s in magnitude. | H-01 | ✅ |
| SR-VEL-02 | Vertical (Z) velocity setpoints shall be clamped to ≤ 1.0 m/s; yaw-rate setpoints to ≤ 1.0 rad/s. | H-01 | ✅ |
| SR-VEL-03 | Any non-finite (NaN/inf) component of a velocity command shall cause the entire command to be rejected (no setpoint sent). | H-01 | ✅ |
| SR-VEL-04 | ROS FLU body-frame input shall be converted to MAVLink FRD before transmission (negate y, z, yaw-rate). | H-01 | ✅ |
| SR-VEL-05 | Velocity setpoints shall be rejected unless the vehicle is armed **and** in GUIDED mode, as parsed from the FC HEARTBEAT. | H-04 | ✅ |
| SR-VEL-06 | The HEARTBEAT used to derive armed/flight-mode shall be filtered to the commanded autopilot — GCS and other-system heartbeats sharing the link shall be ignored. | H-04 | ✅ |

## VIO freshness (VIO) — hazard H-02

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-VIO-01 | Velocity setpoints shall be rejected when VIO is not healthy, below confidence threshold (default 0.3), or older than the freshness window (default `NOMAD_VIO_MAX_AGE_S`, 1.0 s). | H-02 | ✅ |
| SR-VIO-02 | If VIO becomes stale while the controller is actively commanding motion, the controller shall command zero velocity within one watchdog interval. | H-02 | ✅ |

## Link & watchdog (LNK) — hazard H-03

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-LNK-01 | Velocity setpoints shall be rejected when there is no fresh FC heartbeat (within `NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S`, default 3.0 s). | H-03 | ✅ |
| SR-LNK-02 | If no fresh `cmd_vel` arrives within the command-timeout window (0.5 s) while actively commanding, the controller shall command zero velocity. | H-03 | ✅ |
| SR-LNK-03 | On controller stop/shutdown, a zero-velocity command shall be issued before the link is closed. | H-03 | ✅ |

## Geofence (FEN) — hazard H-05

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-FEN-01 | The operating boundary shall be uploaded to and enforced by the FC fence before autonomous flight. | H-05 | 🟡 (FC-side; not NOMAD-tested) |
| SR-FEN-02 | NOMAD shall provide an independent containment check that rejects/clamps any position target outside the configured boundary. | H-05 | 🟡 (pure containment primitive `edge_core/safety/geofence.py` implemented + tested; **not yet wired/enforced** in the command path) |

## Payload (PAY) — hazard H-06

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-PAY-01 | Servo channel shall be validated to 1–16 and PWM to 500–2500 µs; out-of-range commands shall be rejected. | H-06 | 🟡 |
| SR-PAY-02 | Pump-on duration shall be clamped (0.05–5.0 s) and the relay shall be de-energized in a `finally` block and on any failure path, so the pump cannot be left energized. | H-06 | 🟡 |
| SR-PAY-03 | Payload release shall require an explicit operator confirm/interlock before actuation. | H-06 | 🔴 (no interlock yet) |

## Failsafe integrity & security (SEC) — hazards H-07, H-08

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-SEC-01 | The MAVLink command surface shall contain no command that disables an FC failsafe. | H-07 | 🟡 (true by construction; untested) |
| SR-SEC-02 | Edge API requests shall be authenticated per the auth middleware (key, loopback-dev fallback, explicit insecure-remote opt-in, length-checked internal token). | H-08 | ✅ |
| SR-SEC-03 | Command-path endpoints (velocity / RTH / payload) shall require authentication even on loopback, and each SC command shall be logged with operator identity for post-flight audit. | H-08 | 🔴 (target, §4.6) |

## Backlog

**Phase 2** closed the velocity-path gaps (SR-VEL-05, SR-VIO-01/02, SR-LNK-02 →
✅) by moving the decision logic into the pure, 100%-covered `edge_core/safety/`
package. **Phase 3** added the proof infrastructure: a traceability CI gate
([traceability.md](traceability.md) machine block +
`tests/test_safety_traceability.py`), a 100%-branch-coverage gate on the SC core
(`pixi run cov-safety`), and the SR-LNK-03 shutdown-stop test (→ ✅).

Remaining work:

- **SR-FEN-02 wiring** — wire `safety/geofence.py` into the command path and
  prove containment in SITL (the primitive is done; enforcement is not).
- **SR-PAY-01/02/03** — extract + test the payload range/duration/de-energize
  guarantees and add the arm/confirm interlock.
- **SR-SEC-01** — assert by test that the command surface exposes no
  failsafe-disabling MAVLink command.
- **SR-SEC-03** — auth-on-loopback for command paths + operator-audit logging.
- **SITL loop-closure** scenarios for the failsafe branches, run against the
  `pixi run dev-up` ArduPilot SITL stack (§4.5).

See [hazards.md](hazards.md) "How to read the GAPs".
