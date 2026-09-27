# NOMAD work ledger

One ledger; one active item. Requirements/decisions live in [PRD](docs/prd.md),
current qualification scope and provenance in the [migration status matrix](docs/migration.md#current-qualification-status),
dated implementation evidence in [migration](docs/migration.md), and transport
details in [MAVSDK adoption](docs/mavsdk-adoption.md).

Status: [ ] open, [~] active, [x] complete. A task being implemented does not
close its integration or release gate.

## Current work

- [ ] G-M packaging/install evidence: the Release CMake target now produces
  NOMAD-only install trees plus reproducible ZIP/TGZ archives with the CLI,
  public headers, configuration template and dependency notices. The offline
  verifier rejects live configuration, checks both archive formats and runs the
  installed usage path. Versioned activation and rollback to a prior qualified
  build remain open. Falsification: an archive leaks MAVSDK development files,
  includes live configuration, fails clean extraction, or the installed CLI
  cannot run its offline usage path.

- [ ] G-M Phase A resource qualification: the published MAVSDK graph now has
  recursive clean-checkout provenance/build evidence, hosted Linux/Windows/ROS
  qualification and a live ArduPilot Copter SITL connect/status pass. The build
  task now records configure/build timing and footprint metrics, while live smoke
  records per-process-tree startup/RSS metrics; hosted jobs retain both artifacts.
  A clean local Windows checkout at the reviewed gitlink passes provenance,
  build, core/Python suites and deterministic peer cases. Retained hosted samples
  include Linux build-tree `216712508` bytes, Windows build-tree `446186671`
  bytes, Linux/Windows smoke executables of `4753832`/`2025472` bytes, Linux
  runtime peak RSS of `10297344` bytes, and a full hosted test run of about 18
  minutes. The merged-main SITL run retained a Linux build-tree sample of
  `216679160` bytes and runtime peak RSS samples of `10297344` and `9912320`
  bytes. These are observations, not approved budgets: the gate is explicitly
  deferred until a named owner approves build-tree, executable, memory, startup
  and CI-time limits and the retained artifacts are compared with them.
  Falsification: the reviewed pins stop reproducing, live qualification regresses,
  or measured resources exceed an approved threshold.

- [x] G-M aircraft operation capability boundary: heartbeat identity is classified into
  ArduPilot Copter, Plane, QuadPlane or Unknown, carried in core vehicle state,
  then evaluated by an explicit fail-closed operation policy before any aircraft
  command, setpoint, output, payload release or fence request reaches transport.
  The reference `quadplane-tilttri` profile is pinned to ArduPlane 4.7.1 commit
  `dbe792162d06cab66c3475fd5556bf7a120f119e`. Real SITL proved that profile
  reports `MAV_TYPE_FIXED_WING`, so NOMAD requires the ArduPilot identity plus
  `Q_ENABLE=1` or `2` before classifying it as QuadPlane. A missing or invalid
  parameter leaves identity unresolved. Local live evidence observes
  fresh position/GPS/attitude and GUIDED, QLOITER, QRTL and RTL reported modes.
  Only the Copter operational baseline is admitted; Plane, QuadPlane and Unknown
  command paths have focused non-transmission tests. Falsification: a new class
  inherits capability, any rejected request reaches transport, or Copter
  regression coverage fails.

- [x] G-M QuadPlane arm and VTOL takeoff qualification: qualify NOMAD arming,
  select one explicit VTOL takeoff mechanism for the pinned ArduPlane 4.7.1
  `quadplane-tilttri` profile, then verify armed state, the requested climb
  delta from the final pre-command relative altitude, and authoritative state
  with a fixed 0.5 m completion margin plus timeout/failure cases.
  Transition, route, return/recovery and landing are independent qualification
  slices; their current status is recorded in the later entries below.
  Falsification: an ACK is accepted as completion, the generic Copter takeoff
  path is reused silently, or a failed/partial climb reports success.

- [x] G-M QuadPlane transition qualification: the pinned ArduPlane 4.7.1
  `quadplane-tilttri` profile explicitly uses `Q_ENABLE=2`, `Q_ASSIST_SPEED=6`,
  `Q_TRANSITION_MS=5000`, and disabled ArduPilot transition-failure action.
  NOMAD qualifies exactly one mechanism: `MAV_CMD_DO_VTOL_TRANSITION` with
  `param1=MAV_VTOL_STATE_FW`, admitted only for QuadPlane in fresh `AUTO` /
  authoritative multicopter state. Live pinned SITL observed
  `MC -> TRANSITION_TO_FW -> FW`; completion required a newer fresh
  `EXTENDED_SYS_STATE.vtol_state=FW` observation after an accepted ACK.
  The primitive assumes `AUTO` has already been established by an
  operator/test authority; NOMAD still rejects arbitrary QuadPlane
  `set_mode` and cannot yet perform the complete autonomous
  `GUIDED` VTOL-takeoff -> `AUTO` -> transition sequence.
  Focused tests cover unsupported non-transmission, rejected ACK, ACK without
  completion, intermediate timeout, stale/missing state and link interruption.
  This forward-transition slice did not qualify fixed-wing route, recovery,
  transition-back, landing, link-loss strategy or hardware. Later sections
  record the separately qualified route, recovery, transition-back and QLAND
  landing slices. Link-loss/manual takeover, integrated Task 1 and hardware
  remain open.

- [x] G-M standalone-router status/config slice: PR #22 added a versioned local
  status/events/safe link-selection protocol and a standalone management
  client for `nomad-link-router.exe`, with stale/reconnect and loopback tests.
  The protocol does not carry flight command authority. Persistent C++ IPC and
  explicit command-authority handover remain separate work.

- [x] G-M QuadPlane fixed-wing route/navigation qualification: qualify two
  sequential `MAV_CMD_DO_REPOSITION` `COMMAND_INT` targets in already-confirmed
  GUIDED mode for the pinned ArduPlane 4.7.1 `quadplane-tilttri` profile, after
  the qualified AUTO forward transition. Each target needs fresh position
  newer than the ACK-boundary baseline and at least 10 m of additional progress
  before it can pass the horizontal and altitude bounds;
  only the final target completes the route. Deterministic MAVSDK wire checks
  and the exact-head hosted observer passed in [workflow run 35818612311](https://github.com/YoussGm3o8/NOMAD/actions/runs/35818612311)
  at implementation head `7f6206cbad51aade79ae86b20983d4e1fb818901`:
  two points, `AUTO`/fixed-wing at setup, `GUIDED` during route, independently
  observed progress `[1, 2]`, arrival distances 43.1 m and 44.7 m, and NOMAD
  completion at 14.7 s. That route slice did not itself qualify return/recovery,
  transition-back, VTOL landing, link-loss strategy or complete Task 1 flight;
  recovery and transition-back are qualified in the later entries below.
  Falsification: command acceptance, stale position or an intermediate
  waypoint is reported as route completion.

- [x] G-M QuadPlane return/recovery strategy qualification: one explicit
  `RecoveryPoint` uses fixed-wing GUIDED `MAV_CMD_DO_REPOSITION` after the
  qualified two-point route, with `Q_GUIDED_MODE=0` readback and no generic mode
  authority. Completion requires a fresh post-ACK position at least 10 m closer
  than the ACK-boundary position, within 45 m horizontally and 5 m of the
  requested relative-home altitude, while armed in GUIDED fixed-wing state.
  The pinned hosted [workflow run 35897872732](https://github.com/YoussGm3o8/NOMAD/actions/runs/35897872732)
  passed on implementation head `6157d13ff0a9e9516d862a194768f07d7bc3e44b`:
  the independent observer saw 242.3 m initial distance, 37.4 m minimum
  distance, 42.3 m completion distance, 3.6 m altitude error, and 11.2 s
  completion. Generic RTL/QRTL, VTOL landing, link-loss strategy, complete
  Task 1 flight and hardware remain unqualified.
  Falsification: an ACK, stale or pre-command position, insufficient progress,
  or interrupted session is reported as recovery.

- [x] G-M QuadPlane fixed-wing to VTOL transition qualification: pinned ArduPlane
  4.7.1 accepts `MAV_CMD_DO_VTOL_TRANSITION` (3000) with
  `param1=MAV_VTOL_STATE_MC` only in AUTO. An independent loiter setup moves the
  recovered armed GUIDED fixed-wing aircraft into AUTO; NOMAD then requires the
  stable 15–25 m/55 m transition-ready envelope before sending the request.
  Historical run details and implementation provenance remain in the dated
  [migration evidence](docs/migration.md). Current hosted scope is summarized in
  the [qualification status matrix](docs/migration.md#current-qualification-status).
  This proves transition to multicopter flight only; landing remains separate.
  Falsification: an ACK, pre-ACK multicopter state, stale telemetry, gate failure,
  non-multicopter final state, unstable position, disarm or unexpected mode is
  reported as transition completion.

- [x] G-M QuadPlane VTOL landing qualification: the pinned ArduPlane 4.7.1
  profile now has a narrow `quadplane-vtol-land` operation that enters QLAND
  (custom mode 20) only after profile, armed AUTO multicopter state, 15–25 m
  altitude, 5 m landing region, ≤1 m/s groundspeed, ≤0.25 m/s climb and a
  five-sample/two-second readiness dwell pass. Its ACK is only a command
  boundary; completion requires post-command QLAND, at least 5 m descent,
  fresh `EXTENDED_SYS_STATE=ON_GROUND`, disarm and five stable final samples.
  C++ and deterministic MAVSDK checks pass. The historical hosted result is in
  the [qualification status matrix](docs/migration.md#current-qualification-status);
  detailed observer measurements remain in the dated [migration evidence](docs/migration.md).
  This qualifies only the pinned SITL profile; generic `land`, RTL/QRTL and
  hardware remain blocked.

- [~] G-M QuadPlane link-loss/manual takeover qualification: define and prove
  the aircraft and operator response to lost link during supported QuadPlane
  states, without disabling ArduPilot failsafes. D09 topology/interfaces are now
  directed: primary ELRS RC/MAVLink, redundant LTE joystick C2, separate FPV,
  Arduino HID red termination button and independent transmitter chord.
  Qualify faults separately, source arbitration, stale-command inhibition,
  explicit handback and latched termination intent. Exact RC/ArduPilot mapping,
  approved automatic response and Q02 transition termination remain open; see
  [current evidence and remaining scope](docs/migration.md#current-baseline-checks-and-d09-direction-2026-09-27).

- [ ] G-M complete Task 1 flight qualification: integrate the separately
  qualified QuadPlane phases into one end-to-end flight and prove the full
  mission outcome.

- [ ] G-M QuadPlane hardware qualification: repeat the reviewed aircraft
  configuration and flight evidence on hardware with an authorized safety
  process.

- [x] G2 / SR-LNK-03 zero-delivery evidence: repair the live observer's MAVLink
  datagram parsing, prove nonzero-then-zero ordering independently, and rerun the
  complete hosted Copter SITL suite. The full Copter matrix provenance is in the
  [qualification status matrix](docs/migration.md#current-qualification-status);
  this repairs evidence
  collection only and does not weaken the core watchdog or substitute command
  acknowledgements for wire evidence. Falsification: the observer misses a valid
  packed setpoint, accepts zero without a preceding nonzero command, or live wire
  capture remains empty.

- [x] G2 / SR-LNK-04 GCS-heartbeat evidence: validate at least three measured
  intervals between 0.9 s and 1.3 s, route MAVSDK's `udpout:` source through the
  heartbeat-gated relay, preserve the dropped-announcement negative control, and
  rerun the current-head full SITL suite. The run and implementation provenance
  are in the [qualification status matrix](docs/migration.md#current-qualification-status).
  Falsification:
  cadence exceeds the owned limit, the relay opens without valid GCS heartbeats,
  or a future complete suite stops at this gate.

## Ordered implementation backlog

- [ ] G0: obtain organizer rulings Q02-Q09 and assign named decision/gate owners
  (D10). The 125-entry documentation reconciliation and Q01 decision are complete;
  unanswered interpretations keep the planning gate open.
- [x] G-M Phases B–E implementation: the narrow MAVSDK connection owns the
  existing core boundary, command/telemetry/velocity/watchdog, GCS heartbeat,
  zero-delivery, fence and parameter paths; the default production runtime uses
  it and the hand-written codec is removed. See the implementation inventory and
  cutover status in [migration](docs/migration.md). This records source and
  focused-test completion, not release qualification.
- [ ] G-M current-head integration and release gates: qualify supported vehicle
  classes (including QuadPlane), rerun the full current-head ROS/SITL matrix,
  and complete packaging/install/rollback and G8 evidence. Falsification: any
  required adapter or SITL gate fails, or the release artifact cannot be
  installed and rolled back safely.
- [ ] G-M: add ArduPlane/QuadPlane support coverage for Task 1 alongside Copter;
  the pinned ArduPlane 4.7.1 identity/telemetry/mode harness has local live
  evidence, but hosted evidence and independent Task 1 flight primitives remain
  open.
- [ ] G-M Phase F: upstream tested ArduPilot fixes; track patches, review status
  and maintenance owner. Upstream acceptance is separate from local parity.
- [ ] G2 / C04–C09, C13, C15: single persistent command owner, authenticated client
  contract, fresh telemetry, aircraft-specific semantics, cancellation and
  truthful final outcomes; migrate direct plugin policy and hazardous outputs.
- [ ] G3: runtime capability and profile tests; Task 1 GPU-groundstation video/CV
  path, Task 2 optional onboard path, minimal operation without perception.
- [ ] G4: opt-in AEAC 2027 module: transcribe/version the official portal wire
  contract, then implement armed telemetry and traffic cylinders with an
  independent timing mock, demonstrated operator avoidance, outage/replay and
  official-server acceptance.
- [ ] G5: VTOL 3-5 km lap route, single-battery reserve, deer/cluster/tag/anomaly
  report and timed TXT upload, traffic avoidance and verified flight-line landing.
- [ ] G6: under-250-g/8-cm tracker, single placement and continued 100 m offset,
  five-minute CSV path, intact egg/core/spheres on pad, explicit authorization;
  autonomous bonuses only after D05/Q06; swap recovery conditional on Q03.
- [ ] G7: authorized hardware evidence for selected controller, GNSS/RTK, camera,
  RF paths, all-up mass/endurance, payload feedback and independent all-mode
  termination; verified hard fence and Q02 phase ruling, 100 m AGL and approved FRR.
- [ ] G8: full task rehearsals using MAVSDK, security/observability, clean
  packaging/rollback and release review.

Independent read-only research and isolated mock/CV prototypes may prepare later
gates. Do not mark multiple work items active or bypass predecessor safety gates.

## Existing slices to preserve

- [x] C++ library/CLI, MAVSDK MAVLink transport, telemetry and basic
  Copter vehicle/mission operations exist.
- [x] Velocity configuration, watchdog, VIO source validation, fence and
  dedicated payload safety paths with unit tests exist.
- [x] Wire-level zero-delivery tests and named SITL harnesses exist; historical
  observer and Copter matrix results are in the [qualification status
  matrix](docs/migration.md#current-qualification-status). Aircraft-class,
  all-mode and hardware containment evidence remains open.
- [x] Mission Planner core client for goto/discrete outputs and ROS adapter exist;
  full command ownership migration remains open.
- [x] MAVSDK Phase A build/smoke target and fork wiring exist; the production
  cutover is implemented. Current-head integration and release gates remain open
  in G-M above.
- [x] MAVSDK Phase A selected-build inputs use immutable/strong-hash references,
  carry checked licence texts, and are published and pinned at the reviewed
  revision. Recursive hosted Linux/Windows, selected ROS and live Copter SITL
  qualification have passed; resource-budget approval is the remaining Phase A
  release blocker.
- [x] Three profile templates/config tests and retained Python video bridge exist;
  runtime and hardware capability evidence remains open.
- [x] Historical documentation review baseline (2026-09-08): test-core 9/9;
  test-python 251 passed, 3 skipped. Current repository check counts are in the
  [qualification status matrix](docs/migration.md#current-qualification-status).

## User decisions recorded on 2026-09-08

- Task 1: proposed lightweight VTOL, ground CV, no Jetson onboard; Pi Zero LTE
  backup and possible video transport.
- Task 2: heavier quad under the strict project 15 kg ceiling; optional Jetson if needed.
- Walksnail FPV camera; second camera TBD; no reliance on ZED.
- Flight-controller IMU; Cube Orange or custom ArduPilot controller; Here4 and
  RTK base selected in principle. Actual integrations remain unqualified.
- Custom tracker, possibly ESP32; payload mechanisms and feedback TBD.
- Traffic advisories and explicit action authorization first; revisit autonomy
  against the optional bonuses and actual traffic-avoidance obligation.
- MAVSDK is mandatory for competition and an early work priority with tests and
  focused reviewable changes. The CONOPS review changes documentation and
  traceability only.

## Decisions still open

### Direction update — 2026-09-27

The dated 2026-09-08 decisions above are historical. Current D09 topology and
human interfaces are directed in [PRD](docs/prd.md#c2-and-termination-direction-2026-09-27),
including LTE as redundant C2 and separate FPV. Task 2 now specifies Jetson Orin
Nano and intended robotic arm for tracker/egg/droppings; hardware remains
unqualified. D09 is not closed: arbitration, takeover/handback, dedicated
termination mapping, automatic response and common-mode qualification remain.
Verify CH5 arming against actual mode/auxiliary mappings. Q02 stays unresolved.

See D01–D11 in PRD: exact aircraft/firmware and profiles, core host/IPC/remote
transport, capture and synchronized sensor access, tag/sample design, RC and
termination mappings, numeric performance/safety budgets, server contract, owners and evidence
retention. The "15 kg class" description does not override the below-15-kg rule.

## Review follow-through

- [x] Q01 / U-FEN-01: retain the plugin's configurable inward soft-boundary inset
  (e.g. 5 m) for internal use; hard-boundary violation triggers termination.
  No plugin changes or second official soft polygon required.

- [x] Read all 36 CONOPS pages and preserve source hash, exact section/page IDs,
  retired preview crosswalk, interpretation register and source-backed gap matrix.
- [ ] Competition lead: Q02 aircraft-phase termination; Q03 mass/swaps; Q04 server and
  traffic semantics; Q05 clustering; Q06 autonomy authorization; Q07 measurement/
  exports; Q08 final administrative package; Q09 scoring/window edge cases.
- [ ] Team: D04 attachment versus box and sample mechanisms; D05 bonus priorities;
  D01/D09 firmware, capture, radio and power; D02 core host/client boundary;
  D08 budgets; D10 named owners; D11 private evidence and credential lifecycle.

Keep source fixes separate: plugin EmergencyLand/fence parameter ownership,
AGL/accuracy/link-quality model, fixed-wing/QuadPlane semantics and physical
termination need requirement/fault-path tests before implementation. Do not mix
scripts/hardware/servo_test.c's pre-existing SPDX issue into this reconciliation.
