<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Safety Case

This directory is the **living safety case** for NOMAD. It is the Phase 1
deliverable of [NOMAD_REARCHITECTURE_PLAN.md](../../NOMAD_REARCHITECTURE_PLAN.md):
classify the system by criticality and write down what the system can do that is
dangerous, before any safety-critical code is moved.

It deliberately borrows the *high-value habits* of aviation software practice
(DO-178C / ARP4754A / ARP4761 / DO-326A) **proportionately** — NOMAD is a
companion computer plus ground station, not a certified autopilot. The certified
flight-critical element is **ArduPilot on the flight controller (FC)**, which
NOMAD only *commands* (GUIDED velocity, RTH/landing, payload). The honest safety
job is to make those commands **well-behaved and revocable**, not to re-certify
the autopilot.

## What's here

| File | Purpose | Aviation analogue |
|------|---------|-------------------|
| [partition.md](partition.md) | The SC / SR / NC criticality tiers and which real files fall in each. The single most important architectural decision. | DAL allocation + partitioning (ARP4754A, DO-178C §2) |
| [hazards.md](hazards.md) | FHA-lite: what NOMAD can cause, severity, current mitigation, residual risk, and the code + test that proves each mitigation. | Functional Hazard Assessment (ARP4761) |
| [requirements.md](requirements.md) | Short, testable safety requirements (`SR-VEL-01: …`) for the SC tier. | Safety requirements (DO-178C §6) |
| [traceability.md](traceability.md) | requirement → code symbol → test, kept honest by CI in a later phase. | Bidirectional traceability (DO-178C §6) |

## How to use it

- **Before changing safety-critical code** (anything in the SC tier — see
  [partition.md](partition.md)): check whether a requirement in
  [requirements.md](requirements.md) constrains it, and that the
  [traceability.md](traceability.md) row for it still holds after your change.
- **When adding a new hazard surface** (a new actuator command path, a new
  failsafe): add a row to [hazards.md](hazards.md) and a requirement to
  [requirements.md](requirements.md) first, then the code, then the test.
- **Keep it honest.** A requirement with no test, or a hazard with no named
  mitigation, is a gap to flag — not a row to quietly delete. Where the current
  code does *not* yet implement a mitigation, this directory says so explicitly
  rather than describing the aspiration as fact.

## Status

**Phase 1 (classify & document) and Phase 2 (extract the SC core) are done.**
The safety-critical velocity decision logic now lives in the dependency-light,
pure-logic [`edge_core/safety/`](../../edge_core/safety/) package
(`limits`/`gates`/`watchdog`/`envelope`/`geofence`), at **100% line coverage**,
with `mavlink_velocity.py` reduced to a thin I/O adapter that defers to it. This
closed the velocity-path GAPs (H-01..H-04 / SR-VEL/VIO/LNK).

**Phase 3 (prove the SC core) is done** for the velocity path: a traceability CI
gate (`tests/test_safety_traceability.py` parses the normative block in
[traceability.md](traceability.md)), a 100%-branch-coverage gate on the SC core
(`pixi run cov-safety`), and an end-to-end **ArduPilot SITL loop-closure
scenario** ([../../tests/sitl/](../../tests/sitl/)) that proves H-01..H-04 on a
live autopilot. That SITL run also caught and fixed a real defect (GCS heartbeat
flipping the armed/mode gate → SR-VEL-06).

Still ahead: wiring the geofence primitive into the command path + a geofence
SITL scenario (SR-FEN-02 enforcement), extracting + testing the payload path and
its arm/confirm interlock (SR-PAY-*), and the security-as-safety targets
(SR-SEC-01/03). Remaining gaps are marked **GAP** / 🟡 / 🔴 throughout so they
are not mistaken for completed work.
