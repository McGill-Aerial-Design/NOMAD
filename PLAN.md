# NOMAD Implementation Plan

> Working plan tracking the migration from the transitional Python-centered
> vehicle service to a small, testable C++20 core with independent clients and
> adapters. Canonical product and safety decisions live in `docs/` (`prd.md`,
> `architecture.md`, `safety.md`, `migration.md`); the running task ledger is
> `TODO.md`.

## Purpose

Track the implementation work required to move NOMAD to the target
architecture:

```text
CLI / Mission Planner / ROS 2 / Python tools
                    |
                    v
              NOMAD C++ core
                    |
              MAVLink transport
                    |
                 ArduPilot
```

## Product decisions

1. C++ owns vehicle behavior, telemetry models, missions, command validation,
   and MAVLink interaction.
2. Python remains for computer vision, machine learning, simulation, analysis,
   tests, and ground-side utilities.
3. ROS 2 and Mission Planner are adapters or clients; they do not own vehicle
   decisions.
4. ArduPilot remains responsible for stabilization, motor control, EKF,
   low-level navigation, and failsafes.
5. The code must remain understandable to a first-year engineering student.
6. Prefer ordinary types, explicit ownership, small functions, early returns,
   and minimal abstractions.
7. Safety-critical behavior must be testable without hardware before SITL or
   flight validation.
8. NOMAD targets ArduPilot on any flight controller (Cube, Pixhawk, etc.); all
   commands and payload outputs use standard board-independent MAVLink and
   ArduPilot servo/relay channels, never board-specific assumptions.

## Completed implementation slices

- Mission Planner output-command slice: core boundary grew `servo`, `relay`,
  `motor-test`, `gimbal-config`, `user-command` verbs (validated + ACK-
  verified, `src/vehicle/output.cpp`); `NomadCoreClient` gained the
  corresponding methods; OutputController (servo/relay/pulse) and
  MotorMusicCommand route through the core first; GimbalController.SetMode is
  core-first; dead plugin motor-test code deleted; contract tests extended on
  both sides (2026-09-04).
- Flight-controller-agnostic boundary (2026-09-04/05): verified the C++ core
  uses only board-independent MAVLink + ArduPilot servo/relay channels;
  generalized mavlink-router USB discovery to common ArduPilot boards; renamed
  `CubeOutputController` -> `OutputController`; removed the plugin's
  direct-MAVLink fallbacks for discrete outputs (fail closed through the core
  only); generalized the services-status wire key to
  `flight_controller_present`/`no_flight_controller`.
- ROS 2 adapter package `ros2/nomad_ros` (no ROS types in core headers):
  compiles and passes translation + integration tests in the Humble image,
  and the node connected to the live Copter 4.7.x SITL vehicle, published
  telemetry, and drove it via `/nomad/cmd_vel`. Two real bugs found and fixed:
  the node's VIO sample flag was never set (all velocity commands refused),
  and the core UDP transport starved heartbeat consumption under a slow
  sampler — both pinned by tests/evidence (2026-09-04).
- MAVLink codec generated from the pinned `third_party/ardupilot-mavlink`
  submodule (mavgen at the Copter 4.7.1 commit) instead of hand-maintained
  constants; golden frames byte-identical; full SITL battery re-passed on the
  generated codec (2026-09-05).
- Root CMake C++20 project with `nomad_core` library and thin CLI.
- MAVLink framing, CRC validation, UDP transport, heartbeat filtering,
  telemetry conversion, and fake transport tests.
- Verified vehicle operations: arm, disarm, mode, takeoff, land, RTL, and
  location.
- Mission model and synchronous executor with explicit GUIDED and disarm
  waits.
- Velocity envelope with finite checks, axis limits, frame conversion,
  heartbeat, armed, GUIDED, and VIO gates.
- Continuous velocity watchdog for command timeout, stale heartbeat, disarm,
  mode loss, stale VIO, low VIO confidence, and safe zero output.
- Projected global geofence validation and C++ ArduPilot fence
  upload/status verification.
- Servo and relay payload validation, bounded duration, consuming interlock,
  and de-energization cleanup.
- C++ safety traceability and command-surface policy tests.
- C++ telemetry, command-flow, mission, velocity-watchdog, geofence, payload,
  link-loss, and link-recovery SITL runners wired into Pixi and CI.
- Canonical documentation consolidated under `docs/`.
- Deletion progress: `edge_core/ros_http_bridge/`, `edge_core/safety/`,
  `edge_core/modules/payload/servo.py`, and
  `edge_core/services/payload_module.py` deleted after their replacement gates
  closed; Python-only safety gates removed from pixi/CI/mypy.

## Current verification state

- C++ Release build: passing; `pixi run test-core` 6/6.
- `goto <lat> <lon> <alt>` CLI verb: contract-tested, command-flow SITL
  scenario verifies the arrival position live. Proving it against Copter 4.7.0
  surfaced and fixed four real wire bugs (COMMAND_INT crc_extra, REPOSITION
  frame length, relative-altitude verification, ardupilotmega decode layouts);
  all decoder offsets pinned by unit tests against the dialect's own
  unpackers.
- Python test suite: 445 passed, 2 skipped.
- Ruff lint, formatting, size/function policy, strict docs build: passing.
- Live Docker/ArduPilot SITL battery (Copter 4.7.1 image, 2026-09-04/05):
  status, command-flow, mission, velocity-watchdog, geofence, payload,
  link-loss, link-recovery all pass through Pixi with the SR-SEC-02/03 API-key
  gate active. Transport-level zero-delivery proof remains a separate gate.
- GCS heartbeat (2026-09-05): the core now emits a 1 Hz standard GCS
  heartbeat while waiting for the vehicle, because ArduPilot/MAVProxy only
  stream a UDP leg after seeing one from that endpoint. Pinned by pymavlink
  golden-frame unit tests plus loopback tests proving the 1 Hz pre-latch
  emission, the `NOMAD_RELAY_ADDRESS` announcement override (for relays
  behind a separate gateway IP; malformed values fail connect() closed), and
  the override-stops-once-latched boundary (SR-LNK-04 in `docs/safety.md`).
  SITL relay-gate scenario added (`core-sitl-gcs-heartbeat`, wired into CI)
  with a negative control. Local Windows evidence: the relay gate opens,
  announcements arrive at ~1 Hz with correct GCS bytes, and the vehicle
  heartbeat is forwarded and consumed — but the degraded Docker Desktop UDP
  proxy trickles (~2 datagrams/5s, same for the `core-sitl-status` baseline)
  and cannot sustain telemetry, so the run fails closed with an accurate
  diagnostic. Record full-telemetry scenario evidence on Linux/CI.
- MAVSDK evaluation (2026-09-03): submodule pinned at `third_party/MAVSDK`,
  built cleanly on Windows, smoke client streamed telemetry from live SITL
  (`SMOKE_OK`). ArduPilot quirks found: `GUIDED` reports as `Offboard` in
  `Telemetry::FlightMode`; battery `remaining_percent` arrives unscaled.
  Full record and revisit triggers: `docs/architecture.md` (MAVLink library
  decision).

## Remaining work, ordered by risk

### 1. Complete safety evidence

- [Done] Deterministic relay-on/relay-off transport failure tests.
- [Done] Deterministic UDP shutdown ordering tests; live UDP fault injection
  remains open.
- [Done] C++ VIO validator feed (health, confidence, timestamp, source) with
  tests.
- [Done] SITL scenarios: command-flow, mission, velocity-watchdog, geofence,
  payload, link-loss, link-recovery on Copter 4.7.1.
- [Done] SR-FEN-01 live fence upload/readback accepted by ArduPilot.
- [Done] Recorded SITL evidence in `docs/safety.md` and `docs/migration.md`.
- Remaining: GCS-heartbeat SITL scenario evidence on Linux/CI (scenario
  implemented and CI-wired; local Windows relay inconclusive),
  transport-level zero-delivery proof, and hardware payload/VIO evidence.

### 2. Build adapter boundaries

- [Done] Separate ROS 2 adapter package `ros2/nomad_ros`, unit + integration
  tested in-image, live-vehicle validated.
- [Done] Mission Planner `NomadCoreClient` (MP-free core client spawning the
  CLI); `GuidedGoto` and the output/gimbal commands migrated through it;
  standalone contract tests (`pixi run test-plugin-core-client`, incl. the
  live CLI auth gate).
- [Done] SR-SEC-02/03 on the C++ boundary: actuation verbs require
  `NOMAD_API_KEY`, accepted/refused attempts emit audit lines, telemetry keeps
  a no-key local fallback, contract tests + traceability rows added, and the
  ROS 2 adapter documents SROS2 DDS security as its trust boundary.
- Remaining: move any still-remaining Mission Planner vehicle operations
  behind the core boundary; authenticated remote command handling and audit
  logging beyond the local CLI gate.

### 3. Cut over clients

- [Done] Inventory callers of the transitional Python vehicle-control path
  (`docs/migration.md` Phase 7 ledger).
- [Done] Client contract tests for the replacement boundary
  (`tests/test_core_client_contract.py`).
- [Confirmed 2026-09-05] No Python MAVLink vehicle-control callers remain:
  `edge_core` retains only service probes; the C# plugin's remaining REST calls
  are video/VIO/isaac/terminal/ZED camera tools. The `sitl-scenario` CI task
  already drives the C++ CLI (Python link is observer-only) and stays as
  loop-closure evidence. Remaining deletions are gated on the camera-tilt/
  spray Jetson camera-stack decision and on migrating the plugin's status/
  health surfaces off the Python module registry (see `docs/migration.md`
  Phase 7).

### 4. Hardening and release readiness

- Run plugin checks and ROS 2 integration tests in their supported
  environments.
- Review dependencies and remove any that do not reduce meaningful
  complexity.
- Split files when they approach 500 lines and keep functions near 40 logical
  lines.
- Verify shutdown, link loss, stale telemetry, invalid inputs, and ArduPilot
  failsafe preservation on every supported transport.
- Perform a hardware readiness review before any real flight.

## Deletion gates

Do not delete transitional code until all of the following are true:

- A C++ or adapter replacement owns the behavior.
- Normal, invalid, boundary, and failure tests exist.
- SITL evidence exists where MAVLink or flight behavior is involved.
- No client still calls the old implementation.
- Documentation names the replacement as current.
- Safety traceability points to the replacement code and tests.

## Working command sequence

```bash
pixi run build-core
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run docs-build
pixi run core-sitl-status
pixi run core-sitl-command-flow
pixi run core-sitl-mission
pixi run core-sitl-velocity-watchdog
```

## Decision log

- Keep stable requirements, statuses, and safety traceability in
  `docs/safety.md`; keep migration ownership and deletion gates in
  `docs/migration.md`. Do not create competing public architecture documents
  from these notes.
- One working ledger only: `PLAN.md` and `TODO.md`; no separate local-only
  copies.
- Do not commit secrets, local endpoints, machine paths, generated output, or
  temporary experiment results.
