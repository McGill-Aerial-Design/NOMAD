# MAVSDK adoption decision and parity gates

Decision originally recorded 2026-09-06; confirmed by the user on 2026-09-08:
MAVSDK ArduPilot support must be used during competition. Adoption is an early
implementation priority with unit tests, integration tests and focused reviewable
changes. It is gate G-M in [migration](migration.md), a prerequisite for release.

This is the transport decision record. Product phases, hardware and competition
requirements live in the canonical migration, architecture and PRD documents.
CONOPS v1.0 does not mandate MAVSDK; mandatory adoption is the project decision.
Its [source requirements](conops-requirements.md) define the acceptance context:
100 m AGL, all-mode termination, traffic separation and task evidence.

## Current status

The [migration qualification status](migration.md#current-qualification-status)
owns current aircraft scope, run and implementation provenance, repository
baseline counts and open release gates. This document keeps the transport decision
and its dated parity evidence.

MAVSDK is the transport. The Phase E cutover removed the hand-written codec
(`src/mavlink/{protocol,udp_connection,udp_commands,fence,params}.cpp`, the
generated dialect headers and the legacy codec test targets), so no build can
produce a NOMAD binary without a way to reach a vehicle, and the old path cannot
survive as a hidden runtime fallback. CMake has no MAVSDK on/off option: a
missing `third_party/MAVSDK` checkout is a configuration error. The CLI and
runtime use the transport through `nomad/mavlink/mavsdk_transport.hpp`. The ROS
2 adapter (`ros2/nomad_ros`) uses a separate raw UDP receive-only MAVLink
observer because protocol v1 does not expose sensor values. The `--transport` argument and the
`NOMAD_TRANSPORT` environment variable were removed in the same cutover, so an
invocation that names a transport fails closed with usage instead of silently
selecting something else.

Phases A-D landed as gated steps: opt-in subbuild, telemetry smoke consumer,
deterministic ArduPilot-like UDP fixture, pure qualification tests, Linux/Windows
CI jobs, ROS image compile wiring, dependency inventory, root NOTICE and a pinned
project MAVSDK fork. Command, link, velocity and fence/parameter parity are
proven by `scripts/dev/mavsdk_connection_fixture.py` and the core test targets.
ArduPlane QuadPlanes advertise a fixed-wing heartbeat in the pinned 4.7.1 SITL
profile, so the MAVSDK transport reads ArduPilot's `Q_ENABLE` parameter. Values
one and two promote an ArduPilot fixed-wing identity to QuadPlane, zero keeps it
Plane, and a failed read or invalid value leaves it Unknown. A deterministic
wire fixture proves every branch, and the live QuadPlane observer independently
confirms the value-two profile used by the transition qualification.
The parent gitlink pins `900fb0fe7fec74608f1911331218557915cc501a`; read
`.gitmodules`, the gitlinks and the
[dependency inventory](mavsdk-dependencies.md) for provenance. The Phase A/B
names survive in task, CI-job and provenance-check names, not as a second build.
The fork is rebased onto upstream `d7043d3cafe8cd6250565fd211b966d8b455d561`.
It adds a generic one-shot velocity API, vehicle fence rejection and transfer
recovery evidence, exclusion polygons, controlled retry loss, repeated progress
acknowledgements and concurrent deadlines. Millisecond timer rounding no longer
causes retries immediately before an operation deadline.

Historical local transport qualification on 2026-09-19 passed 415 MAVSDK unit tests, 114 system tests,
33 compatibility tests against pinned ArduCopter 4.7.1, and 80 repeated Linux
deadline/retry checks. The pinned revision's hosted compatibility run also
passed all 33; its final change affects only archive workflow paths and docs.
NOMAD's core suite passed 9/9, its Phase A and Phase B wire fixtures passed, and
live zero-delivery and velocity-watchdog probes verified stop and disarm. Wider
SDK system IDs are checked before narrowing, including rejection of ID 257 for
expected ID 1. Complete Linux, Apple and Windows workflows pass on that revision;
see the [dated compatibility evidence](migration.md#mavsdk-compatibility-evidence-2026-09-19) for the final evidence and remaining gates.

The merged-main Copter matrix and later pinned QuadPlane operation chain are
recorded in the [current migration status](migration.md#current-qualification-status).
The detailed observer measurements and earlier failed attempts remain in the
dated migration evidence. These results do not close resource approval, the
broader supported-aircraft/QuadPlane matrix, ROS, packaging/install/rollback,
generic landing, or hardware gates.

The original Phase A graph passed recursive hosted qualification. Test
run `34535620056` completed the Python suite, C++ core, provenance checker,
deterministic peer fixture and optional MAVSDK build on both Ubuntu and Windows.
Selected ROS-image run `34538394497` built with NOMAD_ENABLE_MAVSDK=ON and passed
the ROS adapter integration suite. Mainline SITL run `34538903820` built the
ArduPilot Copter 4.7.1 image, started the development/SITL stack and passed the
MAVSDK Phase A connect/status smoke. Subsequent changes through the mainline
workflow cleanup did not alter the selected MAVSDK source graph. These runs close
the clean-checkout/hosted/live-Copter evidence portion of Phase A; they do not
qualify command parity, aircraft hardware or QuadPlane behavior.

The remaining Phase A release blocker is resource qualification: collect
repeatable build-tree, executable, memory, startup and CI-time measurements and
approve explicit budgets. Historical one-machine measurements below are retained
for comparison but are not themselves approved thresholds.

## Ownership and rationale

User-confirmed ownership split (2026-09-12): the pinned fork owns ArduPilot
command, mode and telemetry semantics — how a verb is encoded, which command and
frame it uses, what a result code means, and how vehicle modes and telemetry
fields are interpreted. NOMAD retains safety policy, command validation,
authoritative verification of outcomes, mission decisions, target identity,
application deadlines, audit and the client contract. Internal library threads do
not by themselves prove bounded watchdog behavior, and a library acknowledgement
never proves an aircraft outcome.

This supersedes the earlier small-patch-set framing, which left ArduPilot gaps in
NOMAD's transport adapter. NOMAD must not add further ArduPilot command
construction: the raw-frame paths in `MavsdkMavlinkConnection` are transitional
and must shrink as the fork supplies each missing semantic.

The adoption supersedes the 2026-09-05 own-codec decision before its original
revisit triggers (new vehicle ecosystem, direct serial/TCP need, upstream quirks)
were met. The recorded rationale was maintained transport ownership and upstream
contribution. The user now explicitly makes adoption a competition deliverable.

Keep every fork patch individually reviewable and upstreamable, pin reviewed
dependencies, record firmware/library pairs and rerun qualification on every pin
change. The old firmware-matched
dialect property must be replaced by tested compatibility with both Copter and
the chosen ArduPlane/QuadPlane firmware. The library API/quirks below must be
verified against the pinned source; they are not claims about latest upstream.

## ArduPilot gaps measured at the pinned revision

Read from `cpp/src/mavsdk` at pin `9884f1095` (2026-09-12). MAVSDK's ArduPilot
support is strong on the read side and thin on the command side, which is why the
fork owns the semantics above:

| Gap | Evidence at the pin |
|---|---|
| No way to set a mode | `Action` exposes no mode-setting call; only NOMAD's `DO_SET_MODE` path exists today |
| Guided goto is absolute-only | Generic Copter goto uses the fork's relative-altitude `Action::goto_location_relative`. The narrow fixed-wing QuadPlane route and explicit recovery point use MAVSDK's typed `MavlinkPassthrough::CommandInt`/`send_command_int` for `MAV_CMD_DO_REPOSITION`; NOMAD creates no raw MAVLink bytes |
| Missing output verbs | No `DO_MOTOR_TEST`; `Action::set_actuator` is command 187, not `DO_SET_SERVO`; no user-command passthrough; no `COMMAND_ACK` result-code exposure |
| Mode handling is read-only | `core/ardupilot_custom_mode.hpp` and `core/flight_mode.cpp` translate Copter/Plane/Rover modes for reading; nothing sets one |
| Documentation is PX4-centric | Public comments link to `docs.px4.io` even where ArduPilot behaves differently |
| Telemetry parity items | AGL, metre accuracy, per-field freshness, link quality and the official mode mapping are required by this document and are not all present |

Recheck each entry against the pin before patching (the Phase F rule): the
historical findings above were recorded against earlier revisions, and upstream
moves.

## Adapter ownership audit

The 2026-09-13 source audit classifies every current responsibility in
`MavsdkMavlinkConnection` and `mavsdk_fence.cpp`. This table is the deletion
checklist: a row moves only after the fork has its own executable regression.

| Adapter responsibility | Classification | Owner and disposition |
|---|---|---|
| Select one configured system ID and reject absent or ambiguous peers | NOMAD policy | Keep; this is vehicle identity policy |
| Mark heartbeat and telemetry fields fresh with steady-clock timestamps | NOMAD policy | Keep; freshness and fail-closed behavior are application safety rules |
| Translate MAVSDK telemetry types into `VehicleState` | Thin translation | Keep, with no ArduPilot protocol decisions |
| Translate MAVSDK result enums into NOMAD acknowledgements | Thin translation | Keep while the NOMAD interface exposes MAVLink result codes |
| Build `COMMAND_LONG` for general commands through `MavlinkPassthrough` | Thin translation | Keep for commands with no suitable high-level API; qualify transport and ACK mapping in the fork |
| Adapt relative-altitude navigation | Thin translation | Generic Copter goto uses the fork's `Action::goto_location_relative`. The fixed-wing QuadPlane route and recovery point build MAVSDK's typed `CommandInt` with `CHANGE_MODE` clear in already-confirmed GUIDED; a deterministic peer checks recovery wire fields and pinned SITL checks route-then-recovery arrival. NOMAD keeps capability, target validation, fence policy and authoritative completion; MAVSDK serializes the packet |
| Translate one-shot body-velocity setpoints | Thin translation | Use `Offboard::set_velocity_body_once`; MAVSDK owns frame, mask and encoding without storing or resending the command. NOMAD converts yaw radians to degrees and retains authorization, freshness, watchdogs and zero-on-stop policy. Fork wire tests cover stalled producers, zero, destruction, removed links, invalid input and refusal to mix automatic resends; pinned Copter SITL verifies expiry under its unchanged `GUID_TIMEOUT` |
| Upload/download fence plans through `Geofence` | Thin translation | Keep; protocol transfer already belongs to MAVSDK |
| Validate fence shape, enabled state, and exact readback | NOMAD policy | Keep; these are operating-area and authoritative-verification rules |
| Probe integer then float parameters | Thin translation | Keep value conversion and carry the remaining caller budget between typed reads |
| Bound command and parameter retries | Generic MAVSDK capability | Use `OperationOptions`; MAVSDK owns retry counts and the overall deadline |
| Expose and construct `REQUEST_DATA_STREAM` | Already unnecessary | Removed; no production caller existed and telemetry setup belongs to MAVSDK |
| Send zero velocity on disconnect/destruction | NOMAD policy | Keep; this is watchdog/shutdown policy even if the send API moves |

The executable disproof for each migration is two-layered: the fork's pinned
ArduPilot test must fail without the compatibility behavior, and NOMAD's focused
transport/safety tests must continue to prove identity, freshness, bounds, zero
delivery, and authoritative state change after the raw workaround is deleted.
The goto handoff additionally preserves double-precision coordinates through
NOMAD's transport interface instead of narrowing them to float command slots.

## Phase A — Build, dependencies and telemetry

Keep the opt-in build isolated while measuring footprint and reviewing notices.
The current setup selects static libraries, telemetry, no gRPC server, no MAVSDK
tests, and BUILD_WITHOUT_CURL=ON. Revisit options only for a demonstrated required
plugin.

The smoke contract accepts only explicit UDP input/output endpoints and one
expected autopilot system ID. Status succeeds only while that system remains
connected and supplies valid mode, 3D-or-better GPS, battery, and at least three
position updates spanning one second with a latest-sample age at most 1.5
seconds. It fails closed for absent, wrong, ambiguous, invalid, or stale peers.

Exit: reproducible Linux/Windows/selected ROS builds; live connect/status with
correct ownship, mode and scaled telemetry; no-peer and wrong-peer failures;
dependency notice audit and approved build/runtime resource budgets.

Historical Windows measurements (2026-09-06, MSVC 19.44, Release, pinned v3
checkout) are retained as comparison data, not acceptance thresholds:

| Measurement | Recorded value |
|---|---|
| Clean configure including dependency builds | 3m14.806s |
| Following clean target build | 3m8.493s |
| Warm optional build | 2m57.515s |
| Legacy CLI executable | 148,480 bytes |
| Optional MAVSDK smoke executable | 2,020,352 bytes |
| Selected static archives total | 118,453,210 bytes |
| Optional build tree / legacy tree | 446,770,697 / 13,890,039 bytes |

The executable comparison is not a like-for-like product size comparison.
Approve actual latency/memory/binary/CI budgets before deciding Phase A passes.
After the hardened smoke rebuild on 2026-09-09, the Windows Release executable
was 2,032,128 bytes and the warm working build tree was 379,886,610 bytes. These
are diagnostic measurements from one machine, not approved budgets or clean
build benchmarks.
After the dependency-input rebuild on 2026-09-10, the executable remained
2,032,128 bytes and the warm build tree measured 379,308,757 bytes. Configure,
target build, the three-case peer fixture, provenance checker, 12 focused Python
tests and 10 CTests passed. Those local measurements predate the final published
fork compatibility changes and remain historical only.

Current accepted technical evidence is the recursive hosted matrix and live SITL
runs recorded above. Run `pixi run verify-mavsdk-provenance` after any source or
dependency change, and run `pixi run build-mavsdk-connectivity-smoke` followed by
`pixi run test-mavsdk-connectivity` for the deterministic peer contract. The
reduced mainline SITL job is the live connectivity-smoke regression gate; the
larger SITL suite stays nightly/on-demand because it exercises later safety and parity behavior. The
on-demand full suite passed on merged-main run `35489428247` above; it remains
an on-demand gate by workflow design.

The connectivity-smoke build task measures configure time, target-build time, build-tree
bytes, smoke-executable bytes and selected static-archive bytes in one record.
Hosted Linux and Windows jobs upload that JSON record as a required artifact.
This makes samples reviewable but does not make a warm build a clean benchmark or
approve a budget. The live smoke job records connect/status elapsed time and
per-process-tree peak RSS on Linux and Windows, and retains its output artifact.
An unavailable RSS sample is explicit rather than silently omitted.

Two local warm Windows runs on 2026-09-10 against the pre-existing dirty vendor
checkout `34b417d4` reproduced a 445,271,541-byte tree, 2,033,664-byte executable,
six selected archives totalling 12,073,548 bytes, with configure/target-build
times of 42.074/175.365 s and 29.701/169.367 s. The provenance check failed
because that checkout's MAVLink patch lacks the reviewed pinned-generator marker.
These measurements validate the collector only; they are not accepted dependency
or release evidence and do not supersede the published `9884f109` graph.

A separate recursive Windows checkout at NOMAD `610215f` and MAVSDK
`9884f109533f564bc6250e5471e6301d3a62f4a7` passed the provenance audit,
28 focused Phase A tests, all three deterministic peer cases, 10 CTests and the
full Python suite (345 passed, 3 environment skips) on 2026-09-10. Its clean
first build completed, but its requested evidence file used an unwritable output
location, so no first-build duration is claimed. A subsequent measured warm run
recorded a 422,952,646-byte build tree, 2,032,640-byte executable, six selected
archives totalling 12,070,896 bytes, 32.695 s configure and 165.748 s target
build. This is clean-source local Windows evidence, not a hosted sample, CI-time
measurement, runtime/SITL result, approved budget or flight qualification.

Hosted run `34550522657` against commit `219133e` on 2026-09-11 retained the
following clean recursive-checkout samples and passed provenance plus all three
deterministic peer cases on both runners:

| Runner | Tree bytes | Executable bytes | Archives | Archive bytes | Configure | Target build |
|---|---:|---:|---:|---:|---:|---:|
| `ubuntu-latest` | 213,615,930 | 4,719,032 | 6 | 1,552,468 | 43.020 s | 172.291 s |
| `windows-latest` | 430,393,315 | 2,015,744 | 6 | 12,051,864 | 187.318 s | 326.074 s |

Hosted run `34550529273` against the same commit connected to the configured
Copter 4.7.1 SITL system and retained `mavsdk-phase-a-runtime`: connect completed
in 0.731 s at 9,584,640-byte peak process-tree RSS; status completed in 2.534 s
at 9,940,992 bytes and returned six fresh samples for system 1. These are hosted
dependency and live-SITL integration samples, not approved resource budgets,
aircraft-specific evidence or flight qualification. The same full workflow
failed twice later in the legacy C++ zero-delivery scenario because its observer
captured no wire setpoints (`wire=[]`); subsequent full-suite scenarios were
skipped. The observer's `parse_char` misuse was repaired in `6a3e970`; hosted run
`34648914427` then observed the ordered nonzero/all-zero sequence and post-stop
hover. That run later failed the separate SR-LNK-04 heartbeat-cadence gate and is
not counted as a successful full-SITL run. The merged-main run `35489428247`
subsequently passed the heartbeat gate and the complete Copter matrix; the earlier
failure remains historical evidence only.

## Phase B — Vehicle and output parity

Implement the existing connection boundary using only required MAVSDK APIs.
Keep public safety/Vehicle semantics, CLI spelling, client errors and argument
validation stable unless a reviewed correction changes an unsafe contract.

Historical Phase B implementation record (2026-09-11, local only):
`MavsdkMavlinkConnection`
implements `MavlinkConnection` against pin `9884f109`, issuing commands as raw
COMMAND_LONG/COMMAND_INT (so the result-code and relative-altitude-frame
contracts are preserved) and mapping telemetry into `VehicleState` with the same
per-field validity flags and steady-clock timestamps, so the position-freshness
gate still applies. At that historical revision, the CLI took `--system-id` and
accepted `--transport mavsdk` only as a historical selector. `nomad_mavsdk_connection_tests` and
`scripts/dev/mavsdk_connection_fixture.py` (driving the deterministic vehicle
in `scripts/dev/mavsdk_peer.py`) cover accepted, denied, timeout,
no-peer, stale-telemetry, COMMAND_INT frame and wrong-identity cases, plus
per-command parity for mode, takeoff, QuadPlane VTOL takeoff, QuadPlane
transition-to-fixed-wing, goto, land, RTL, servo, relay, gimbal-config and
user-command. In those cases the peer applies the state change
each accepted command asks for and its initial state is observably different
from the required result, so they assert the core's state verification rather
than the acknowledgement alone. Live Copter SITL evidence (2026-09-11): with
`NOMAD_TRANSPORT=mavsdk` the `core-sitl-command-flow` and `core-sitl-payload`
scenarios pass against Copter 4.7.1 — GUIDED mode, 3D GPS fix, arm, takeoff to
5 m, guided goto sent as COMMAND_INT, RTL, land and disarm, every step verified
against reported state. Those selector forms describe historical evidence only:
the current CLI has no transport selector and rejects `--transport` with usage
because MAVSDK is the only production transport. This is not the end of Phase B:
motor-test is unproven
because `Vehicle::motor_test` sends command ID 139, which is not a `MAV_CMD`
entry in the pinned dialect (`MAV_CMD_DO_MOTOR_TEST` is 209); that is recorded
as C23 in the migration contradictions. Explicit vehicle-class identification
is landed for the pinned profiles, and the deterministic peer now covers the
QuadPlane startup and both transition directions. The back-transition peer
checks command 3000 as `COMMAND_LONG`, target system/component, `param1=3`
(`MAV_VTOL_STATE_MC`), accepted-state verification and denied ACK handling.
The transition uses MAVSDK's
existing typed `subscribe_vtol_state` path; no fork edit or hand-written
MAVLink encoder was needed. A two-point fixed-wing route and an explicit
recovery point now use the typed `COMMAND_INT` path. Generic navigation,
generic landing, RTL and full Task 1 coverage remain open; the pinned QLAND
operation is a separate QuadPlane-only capability. Runtime IPC v1 does not expose
the new transition operation. Everything up to and including the production
cutover has landed.
The recovery peer verifies command 192, relative-altitude frame, clear
`CHANGE_MODE`, 30 m loiter radius, target system/component and denied ACK
mapping. Pinned hosted [run 35897872732](https://github.com/YoussGm3o8/NOMAD/actions/runs/35897872732)
also observed the post-route recovery target approached from 242.3 m to a
42.3 m completion distance at 20 m requested relative-home altitude.

Cover arm/disarm, mode, takeoff, land/RTL, goto, servo, relay, motor-test,
gimbal-config. Unsupported verbs now belong in the fork: add the
missing semantic there with its own test rather than a new NOMAD adapter path, and
retire the transitional raw-frame route for each verb as it lands. Verify
COMMAND_INT location semantics and altitude datum rather than assuming a generic
goto call is equivalent.

Add explicit vehicle-class identification and reject incompatible operations.
Task 1 requires Plane/QuadPlane coverage, not a renamed Copter test. Pin mode
mapping, transition state and accepted command matrix.
Telemetry parity must also expose independently sourced AGL, metre accuracy,
per-field freshness, link-quality inputs and the official mode mapping required
by AE27-NET-002 through AE27-NET-006. The smoke's relative altitude and GPS fix
qualification are not those capabilities. Do not copy its 1.5-second age limit
into competition traffic policy or treat it as a CONOPS safety threshold.

Exit: fake-transport and codec-independent contract tests, CLI/plugin tests and
Copter command-flow/payload SITL pass; negative ACK, wrong target, delayed state,
timeout and cancellation results remain truthful. QuadPlane-specific mission/
transition tests qualify new operations before G2/G5, not merely discovery.

## Phase C — Velocity, watchdog, stop and heartbeat parity

Evaluate Offboard start/stop and any automatic background resend against
ArduPilot GUIDED semantics. A narrow raw-setpoint path is an option if needed,
not permission to bypass NOMAD gates. Stop must end stale resends; destruction,
reconnect and callback lifetime must not revive control.

The adapter uses `Offboard::set_velocity_body_once`, which queues a single
setpoint without starting Offboard mode or automatic resends. Success means
queued, not accepted by the vehicle. NOMAD's producer and watchdog own refresh
and stop; ArduPilot's existing Guided timeout handles a stalled producer without
requiring NOMAD's watchdog to run. No firmware failsafe is changed.

The wire fixture independently checks BODY_NED (8), mask `0x05C7`, all velocity
components and yaw rate. BODY_NED and the previous BODY_OFFSET_NED have the same
body-relative velocity meaning; their position difference is irrelevant because
position is ignored. The previous mask's force bit also applied only to ignored
acceleration fields. Zero delivery and no stale resends remain explicit checks.

Exit: each watchdog fault case; independent wire zero-delivery plus
core-sitl-zero-delivery; core-sitl-velocity-watchdog; GCS-heartbeat relay gate
with its dropped-announcement negative control. Verify identities, cadence,
thread contention and shutdown order. Fixed-wing safety is separately defined;
do not apply the Copter velocity scenario as a QuadPlane flight guarantee.

## Phase D — Fence, parameters and mission integration

Verify actual fence mission items, vertex semantics, readback and enable state
using Geofence/Param or narrowly required protocol access. Keep malformed input
and disabled/unreadable fence failure tests.

Exit: core-sitl-geofence and sitl-fence containment evidence; full relevant
parameter/fence state matches the reviewed plan. Add any mission upload/progress/
abort APIs required by Task 1 in a focused extension with independent QuadPlane
SITL evidence, not an untested side effect of transport replacement.
Resolve Q02 before proving aircraft-phase termination. Q01 is resolved: use
the hard polygon for termination and retain the plugin internal soft inset.
Prove non-convex hard containment, 100 m AGL, independent termination, C2 loss,
five-second activation and separate rotary/fixed-wing/transition outcomes.
Parameter parity alone cannot prove a minimum 2 m/s descent through touchdown.
Preserve ArduPilot failsafes and require independent physical evidence at G7.

## Phase E — Production cutover (implementation complete; release gates remain)

The production connection was switched after the A-D implementation work landed.
The obsolete codec, UDP/generation code and generated dialect headers were
removed after caller inventory, profile checks and safety traceability remapping.
Golden/wire semantic references remain in the peer and SITL fixtures. Packaging,
install/rollback, supported-firmware and full current-head integration evidence
remain release gates; a rollback package must start disarmed/inhibited and never
resume stale mission or payload actions.

Implementation exit: the default CLI/runtime, ROS 2 adapter and clients use
MAVSDK, and the old path is deleted rather than retained as a hidden fallback.
Release exit remains open until the full unit, adapter and SITL matrix for
supported firmware, dependency notices, packaging and install/rollback evidence
pass. Competition release G8 cannot pass with MAVSDK limited to a smoke
executable.

## Phase F — ArduPilot semantics in the fork, and upstreaming

Under the 2026-09-12 ownership split this is prerequisite work for Phases B-D,
not post-cutover maintenance. The fork gains every ArduPilot semantic NOMAD needs,
taken from the measured gap list above: mode setting, relative-altitude location
commands, the output verbs, and the telemetry fields this document requires.
Recheck each historical finding against pinned/upstream source before patching;
GUIDED mode interpretation, battery units and location command semantics are the
named candidates.

Required fork changes must have tests, a wire or SITL observation against the
selected firmware, an accountable maintainer and a provenance update even while
upstream review is pending. Each reviewable change should contain problem/
behavior, requirements, exact pins, unit/integration evidence and residual
limitations, and should be raised upstream when publication is authorized.

debt: the fork carries every ArduPilot semantic NOMAD needs, each patch with its
own test and upstream change; revisit on each upstream MAVSDK release or quarterly
maintenance review; then drop the patches upstream accepts and requalify the
firmware/library matrix. Numeric patch/footprint ceilings require D08/D10;
no unspecified threshold is treated as a passed gate.

## Risks to keep visible

- Library/firmware dialect drift: pinned pairs and independent wire/SITL tests.
- Mode and Offboard semantic mismatch: aircraft-specific validation and stop tests.
- Background resend/concurrency: prove no stale motion after cancel/shutdown.
- Plugin/API availability: inspect pinned source before choosing a wrapper.
- Build size and transitive notices: measurable Phase A acceptance, not optimism.
- Schedule: adoption is mandatory; prioritize it before integrated competition
  commands, while isolated perception/server test fixtures can progress.
