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
| SR-FEN-02 | NOMAD shall provide an independent containment check that rejects/clamps any position target outside the configured boundary. | H-05 | ✅ (pure decision `safety/geofence.py::evaluate_position`, enforced in `services/mavlink/commands.py` position-target senders; unit + adapter fault-injection tested. SITL containment scenario exists — `pixi run sitl-fence` — but has **not yet been run**; see H-05) |

## Payload (PAY) — hazard H-06

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-PAY-01 | Servo channel shall be validated to 1–16 and PWM to 500–2500 µs; out-of-range commands shall be rejected. | H-06 | ✅ (pure `safety/payload.py::validate_servo_command`, enforced in `servo.py`; tested) |
| SR-PAY-02 | Pump-on duration shall be clamped (0.05–5.0 s, non-finite rejected) and the relay shall be de-energized in a `finally` block and on any failure path, so the pump cannot be left energized. | H-06 | ✅ (`safety/payload.py::clamp_release_duration` + `servo.py` `finally`; fault-injection tested incl. exception mid-pulse) |
| SR-PAY-03 | Payload release shall require an explicit operator confirm/interlock before actuation: an arm within a short window, consumed by each release attempt. | H-06 | ✅ (`safety/payload.py` interlock state machine, enforced in `servo.py::trigger_water_shooter`; HTTP arm→trigger routes; GCS fire button requires a confirm click. Direct GCS→FC MAVLink relay commands are outside NOMAD's mediation — the transmitter switch / armed button click is the operator confirm there) |

## Failsafe integrity & security (SEC) — hazards H-07, H-08

| ID | Requirement | Hazard | Status |
|----|-------------|--------|--------|
| SR-SEC-01 | The MAVLink command surface shall contain no command that disables an FC failsafe (no parameter writes, no force-arm magic, no flight-termination/parachute commands). | H-07 | ✅ (deny-list scan over every MAVLink-owning module: `tests/test_safety_command_surface.py`) |
| SR-SEC-02 | Edge API requests shall be authenticated per the auth middleware (key, loopback-dev fallback, explicit insecure-remote opt-in, length-checked internal token). | H-08 | ✅ |
| SR-SEC-03 | Command-path endpoints (payload actuation; future velocity/mode routes) shall require authentication even on loopback — never the unauthenticated dev fallbacks — and every command-path request shall be audit-logged with client address and auth mode. | H-08 | ✅ (middleware `_COMMAND_PATH_PREFIXES`; tested in `tests/test_auth_middleware.py`. Identity is client address + auth mode — a single shared key cannot distinguish named operators) |

## Backlog

**Phase 2** closed the velocity-path gaps (SR-VEL-05, SR-VIO-01/02, SR-LNK-02 →
✅) by moving the decision logic into the pure, 100%-covered `edge_core/safety/`
package. **Phase 3** added the proof infrastructure: a traceability CI gate
([traceability.md](traceability.md) machine block +
`tests/test_safety_traceability.py`), a 100%-branch-coverage gate on the SC core
(`pixi run cov-safety`), and the SR-LNK-03 shutdown-stop test (→ ✅).

**Phase 4.1** wired the geofence: `safety/geofence.py::evaluate_position` (pure,
in the 100%-branch gate) is now enforced by the position-target senders in
`services/mavlink/commands.py`, configured via `NOMAD_FENCE_POLYGON` /
`NOMAD_FENCE_MARGIN_M` (malformed config fails closed). Unit + adapter
fault-injection tests prove it (SR-FEN-02 → ✅).

**Phase 4.2** extracted the payload path: pure `safety/payload.py`
(channel/PWM validation, duration clamp, arm→release interlock) enforced by the
`servo.py` adapter, with the de-energize-on-exception guarantee fault-injection
tested; `/api/servo/shooter/{arm,trigger}` give the interlock an operator
surface and the GCS fire button now requires a confirm click
(SR-PAY-01/02/03 → ✅).

**Phase 4.3** made security-as-safety checkable: a deny-list scan proves the
MAVLink surface contains no failsafe-disabling command (SR-SEC-01 → ✅), and
command-path endpoints require auth even on loopback with every request
audit-logged (SR-SEC-03 → ✅).

Remaining work:

- **SR-FEN-02 SITL evidence** — run the containment scenario
  (`tests/sitl/geofence_containment.py`, `pixi run sitl-fence`) against the
  dev stack; it is written but not yet executed.
- **SR-FEN-01** — FC-side fence upload remains manually verified.

See [hazards.md](hazards.md) "How to read the GAPs".
