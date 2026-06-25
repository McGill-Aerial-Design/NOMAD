<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Safety Case

This directory is NOMAD's safety case: it classifies the system by criticality
and writes down what the system can do that is dangerous, so the dangerous parts
get the attention they need.

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
| [traceability.md](traceability.md) | requirement → code symbol → test, kept honest by CI (`tests/test_safety_traceability.py` parses the normative block and fails on a missing symbol/test). | Bidirectional traceability (DO-178C §6) |

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

## How it is enforced

All decision logic lives in the dependency-light, pure-logic
[`edge_core/safety/`](../../edge_core/safety/) package
(`limits`/`gates`/`watchdog`/`envelope`/`geofence`/`payload`) at 100% branch
coverage; the I/O adapters (`mavlink_velocity.py`, `services/mavlink/commands.py`,
`modules/payload/servo.py`) make no safety decisions of their own.

CI gates, all enforced on every PR:

- `pixi run cov-safety` — 100% branch coverage on the SC core.
- `tests/test_safety_traceability.py` — requirement → code → test sync against
  the normative block in [traceability.md](traceability.md).
- `tests/test_safety_partition.py` — the SC core imports stdlib + itself only;
  the rest of `edge_core/` reaches it only via its public API.
- `tests/test_safety_command_surface.py` — the MAVLink surface contains no
  failsafe-disabling command (SR-SEC-01).
- `tests/test_client_contract.py` — the C# client cannot drift from the API.
- `pixi run lint-safety` / scoped strict mypy — extra rule sets on SC files.

The velocity and geofence paths are also exercised against real ArduPilot SITL
([../../tests/sitl/](../../tests/sitl/), `pixi run sitl-fence`), which gates
nightly in [../../.github/workflows/sitl.yml](../../.github/workflows/sitl.yml).
SR-FEN-01 (FC-side fence upload) is verified manually before flight — inherent to
it being a flight-controller function. GCS-side return-to-home currently relies on
ArduPilot's own RTL/LAND modes; no NOMAD SC code owns that sequence.
