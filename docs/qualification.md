# Qualification status

Software tests establish only their asserted boundary. Run the retained gates
below for every release and retain exact source/build/firmware/configuration with
results. Full physical aircraft qualification remains open. [Safety](safety.md)
defines obligations; [TODO](../TODO.md) lists implementation work.

## Evidence levels

1. Unit/fake transport: policy, invalid/boundary input and deterministic fault paths.
2. Real process/loopback MAVLink: IPC authentication, replay/admission fencing,
   audit/restart, actual frame delivery and transport ownership.
3. Pinned ArduPilot SITL: controller acceptance and observed simulator behavior.
4. Physical bench/flight: actual receiver priority, output/trajectory, RF independence,
   power failure, takeover and approved operating limits. Levels 1–3 do not imply 4.

## Required software gates

| Gate | Coverage and limits |
|---|---|
| `test-core` | Fake-transport vehicle policy, command verification, telemetry/watchdogs, fence/velocity, QuadPlane phase/fault paths, authority/concurrency and actuator state |
| `test-python`, `lint`, `format-check`, `docs-build` | Retained harnesses, release/security/service regressions and repository consistency |
| `test-runtime-ipc`, `test-runtime-lifecycle` | Authenticated real runtime/CLI, stale/replayed contexts, clean/crash restart with durable history, lost/returning peer and no restored owner |
| `test-runtime-land` | Installed CLI/runtime LAND, one-hertz heartbeats, shared ACK/observation budget, explicit uncertainty, wire attempts and durable no-replay evidence |
| `test-mavsdk-connectivity`, `test-mavsdk-transport-qualification`, `test-mavsdk-authority-wire` | Selection, framing, ACK/state checks, session/connection retirement, retries/cancellation and final-send fencing at independent peers |
| `verify-mavsdk-provenance` | Exact reviewed dependency pins, generator/hash patches and selected-license bundle |
| Mission Planner tests in development guide | Immutable outcomes, cancellation/freshness/no replay, HID interlocks, advisory geometry, video disposal and host message boundary |
| `test-ground-router` | Receive-only permissions, physical-link routing/dedup/failover, monotonic timeout/freshness, transaction pinning and process lifecycle |
| `test-runtime-service-artifacts`, `test-release-lifecycle` | Protected systemd/SCM configuration, versioned archive/stage/activation/rollback and restoration failures; no privileged installation |
| `verify-core-package`, `verify-core-staged-install` | Exact production contents/identity, executables and legal notices; no test driver, SDK headers or operator state |

Runtime outcome, actuator, router and release checks are essential. Tests for deleted
migration/conversion and optional adapters are not release gates. Resource baselines
are removed; release acceptance still requires measured load/deadline behavior on its
actual target, including memory pressure, disk/audit failure and competing workloads.

## SITL qualification

[sitl.yml](../.github/workflows/sitl.yml) requires full Copter and QuadPlane jobs on
safety-sensitive changes and scheduled/manual runs. Its scope output is fail-closed.
The MAVSDK connectivity smoke is a separate limited check and cannot replace them.

Copter coverage includes command/mission, heartbeat cadence, velocity/watchdog,
geofence/containment, payload, zero delivery, link loss/recovery and runtime authority.
QuadPlane uses ArduPlane 4.7.1 (`dbe792162d06cab66c3475fd5556bf7a120f119e`),
`quadplane-tilttri`, `Q_ENABLE=2`: identity, VTOL takeoff, forward transition, two-point
route, recovery, transition back and QLAND. Success requires observed post-command
state; an ACK alone is never climb or touchdown. The driver/operator establishes
some setup states, including AUTO. This is not an autonomous Task 1 mission.
[Scenario instructions](../tests/sitl/README.md) define the runnable gates.

## Open release blockers

No production RC/ELRS channel map, aircraft-wide writer arbitration, physical
pilot takeover/handback, complete C2-loss policy or independent termination mechanism
is qualified. Beyond Copter LAND engagement, runtime IPC exposes no QuadPlane mutation or navigation,
boundary enforcement, VIO submission or mission API. Mission Planner boundary
outlines are advisory only. Native GCS and RC remain external writers.

Real perception/VIO fusion, competition server/traffic handling, wildlife export,
tracking/sample mechanisms, physical actuator feedback and full Task 1/2 integration
remain open. No firmware/board/sensor/compute placement is hardware-qualified.
Release signing/trusted publisher distribution and privileged systemd/SCM registration
require separate deployment acceptance. SHA-256 integrity alone does not authenticate
a publisher. Validate mixed component versions against both IPC and router contracts.
