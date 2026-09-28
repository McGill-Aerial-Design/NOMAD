# Migration and release gates

Historical source review baseline: `fab9f46`, inspected 2026-09-10 with a clean working tree;
includes planning `d31b0aa`, runtime `3ef38e7`, profiles `51f309e`, setup `a9762b0`
and MAVSDK Phase A `6922371`. Earlier evidence below retains its original dates.
This document owns implementation status, the
cutover inventory and gate evidence. [PRD](prd.md) owns requirements and decisions;
[architecture](architecture.md) owns the target; TODO is the working ledger.

## Current qualification status

This is the current owner for qualification scope, provenance and unqualified
gates. Run [36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163)
was created 2026-09-26T02:04:54Z. Its pinned QuadPlane full chain and Copter
loop-closure check passed at implementation head
`dcdd1d121d51eefa451fa5d16ab121ef8793e427`; the separate
`mavsdk-phase-a-sitl` job was skipped. This is historical evidence, not a new
run. Code SHAs below identify the implementations tested, not this document's
current commit.

The pinned QuadPlane chain covers identity/telemetry → arm → VTOL takeoff →
VTOL-to-fixed-wing → two-point route → bounded recovery → fixed-wing-to-VTOL →
QLAND. Observed landing completion required `ON_GROUND`, disarm and stable final
samples.

| Operation | Aircraft / firmware SHA | Code SHA / SITL run and date | Command path | Deterministic evidence | Scope limits / next unqualified gate |
|---|---|---|---|---|---|
| Copter regression matrix and velocity loop closure | ArduCopter 4.7.1 image built from the `Copter-4.7.1` tag; the workflow does not record its source SHA | Full matrix: `26d7f9b101a029725d06aee2c6716da95e622417`, [35489428247](https://github.com/YoussGm3o8/NOMAD/actions/runs/35489428247), 2026-09-20. Later loop closure: shared run above, at `dcdd1d121d51eefa451fa5d16ab121ef8793e427`. | `.github/workflows/sitl.yml` `sitl` job; `pixi run sitl-scenario` runs `tests/sitl/velocity_loop_closure.py` | C++ fake-transport tests and the MAVSDK peer fixture cover command and failure contracts; SITL checks authoritative vehicle state. | These Copter scenarios do not qualify Plane or QuadPlane operations, hardware behavior or competition readiness. Next: record an immutable Copter firmware SHA; close Phase A resource approval, supported-firmware and ROS matrices, and packaging/install/rollback gates. |
| Pinned QuadPlane operation chain through QLAND | ArduPlane 4.7.1 `quadplane-tilttri`, `Q_ENABLE=2` and pinned parameters; firmware SHA `dbe792162d06cab66c3475fd5556bf7a120f119e` | Shared run above at `dcdd1d121d51eefa451fa5d16ab121ef8793e427`. | `.github/workflows/sitl.yml` `quadplane-observation` job; `pixi run core-sitl-quadplane-vtol-landing` runs the pinned chain. | C++ falsification tests and deterministic MAVSDK peer checks; an independent pymavlink observer verifies live post-command state. | Limited to these operations on this SITL profile. Arbitrary modes, generic takeoff/goto/land/RTL/QRTL, link-loss/manual takeover, full Task 1 execution and hardware remain unqualified. Next: qualify link-loss/manual takeover, the integrated Task 1 flight and hardware operation; complete G-M release and G7 safety gates. |

Repository checks are separate from aircraft qualification. On audited `main`
commit `bc7cf4e36690e0561a7af9370bdf46e4f9e91443` (2026-09-26), root
`CMakeLists.txt` registered 17 tests and CTest passed all 17 (17/17).
`pixi run test-python` completed with 618 passed and 3 skipped. File/function/line-length
improvement baseline entry counts are 5/1/0.
Strict docs build, lint and format checks passed on this baseline. These results
do not extend either aircraft row's scope.

These operation results do not close command-authority/runtime hardening,
competition-server integration or any unlisted G2-G8 gate.

### Disarmed native SITL RC-fault injector

`core-sitl-quadplane-rc-loss-probe` provides simulator-only fault-delivery
infrastructure, not aircraft loss-response qualification. It verifies fresh
telemetry from a disarmed, isolated pinned Plane/QuadPlane instance. It reads
effective parameters without changing failsafes, observes a healthy no-injection control, applies
`SIM_RC_FAIL=1`, then requires fresh unhealthy receiver evidence and confirmed
restoration to 0/healthy. Parameter readback alone cannot pass. A backend that
bypasses this injection fails the probe. Failure and restoration errors remain
separate, and its observer sends no commands or GCS heartbeats.

The nightly/manual QuadPlane job reuses its pinned image for two clean disarmed
attempts with retained JSON evidence before running the established flight
chain. An artifact's exact source SHA and `dirty_source` flag own its provenance;
development runs are not clean-head hosted qualification. The native profile's
effective `FLTMODE_CH=5`, `RC5_OPTION=0` does not model the team's CH5 arming
configuration. See [operation and scope](operations.md#simulation-and-test-operations).

This tool does not close G-M loss/manual takeover, D09 or Q02. ELRS/LTE/FPV
hardware independence, aircraft failsafe/termination response, manual arbitration
and explicit handback still require their reviewed contracts and live evidence.

### Reviewed baseline checks and D09 direction — 2026-09-27

At startup, fetched `main` was `55a2ac4826e485e92efb5d8f9297561a5498e32d`, the
PR #37 merge. At this code baseline `pixi run test-core` configured/built and
passed 21/21 registered CTests; `pixi run test-python` passed 652 tests with
3 skipped. The focused `test_sitl_harnesses.py` and `test_quadplane_sitl.py`
run passed 33 tests. These local checks used an existing checkout with
pre-existing untracked submodule state, not a clean-checkout provenance run.
The historical 17/17 entry above remains evidence for its original SHA.

Existing manually dispatched [run 36281636286](https://github.com/YoussGm3o8/NOMAD/actions/runs/36281636286)
passed the full current workflow on that exact baseline SHA. Job steps and the
QLAND observer trace were inspected: pinned firmware
`dbe792162d06cab66c3475fd5556bf7a120f119e`, observed QLAND/descent/ON_GROUND,
final disarm, altitude 0.29 m, groundspeed 0.02 m/s, climb 0.00 m/s and touchdown
at 42.72 s. This records an existing run, not a newly dispatched qualification;
its scope remains the Copter matrix and pinned QuadPlane chain through QLAND.
It does not prove manual takeover, ELRS/LTE loss or termination.

[New project direction](prd.md#c2-and-termination-direction-2026-09-27) resolves
the topology and human interfaces: primary ELRS RC/MAVLink, redundant LTE C2,
separate FPV, RC transmitter and intended ground joystick, Arduino HID red
button and independent transmitter chord for one latched termination intent.
Task 2 specifies Jetson Orin Nano/robotic-arm direction, not qualified hardware.
D09 remains open for arbitration/handback, exact channel/aircraft mechanisms,
automatic loss response, common-mode analysis and evidence. Q02 is unchanged.

The active qualification must separately cover LTE loss, ELRS loss with LTE
healthy, simultaneous loss (including FPV still available), ground software/
computer failure with RC healthy, FPV-only loss, both manual takeover paths,
stale-command/reconnect rejection, both termination activation paths and the
approved all-phase onboard response. Current client-shutdown and host-relay
tests do not establish these aircraft outcomes. The host relay leaves the
SITL/MAVProxy master connection intact; possible heartbeat masking needs
vehicle-facing capture, not inference credited as qualification.

The minimum authority prerequisite is explicit source arbitration and inhibition
across every enabled writer; a runtime mutex or connection session is insufficient.
Preserve QLAND physical completion and existing operation admission/freshness.
No new policy, RC mapping, termination dispatch or authority API is implemented
by this documentation record. The active G-M item remains open.

### Software authority foundation — 2026-09-28

The runtime starts without a software command owner. Explicit admission,
revocation and handback advance its authority generation; each typed mutation
is bound to a fresh runtime incarnation, aircraft session, source, generation,
monotonic sequence and five-second maximum expiry. A 256-response cache remains
only an optimization: its evicted requests cannot pass the generation's sequence
high-water mark. Restart changes incarnation; reconnect revokes and cannot
restore ownership. Outcome after a mid-operation revocation is interrupted,
not success merely because an ACK was returned.

Default Mission Planner mode is persistent IPC. Integrated deployment profiles
inhibit direct CLI actuation and plugin gimbal streaming; the ROS adapter is
observation only by default while explicit nonintegrated tests retain its direct
Vehicle behavior. The plugin direct mount-config fallback and `RCx_OPTION`
settings writer are removed. `LegacyOneShot` remains explicitly selectable
because GuidedGoto has no typed runtime replacement; this mode is excluded
from integrated operation. Native Mission Planner controls and ELRS RC remain
external aircraft inputs requiring independent arbitration.

The pinned MAVSDK command sender queues and retries `COMMAND_LONG` and
`COMMAND_INT` after NOMAD's transport-call admission check. It does not expose a
per-send cancellation hook. This is a blocking qualification gap: an SDK retry
after a physical takeover has not been excluded by the runtime fake tests.
debt: runtime transport-call fencing cannot stop an SDK-owned queued retry;
revisit before enabling integrated motion authority; then add a typed
per-transmission cancellation hook in the MAVSDK fork and prove the wire boundary.

Local validation: 21/21 CTests, 10/10 ROS integration tests, 667 Python tests
with four skipped, Mission Planner core/gimbal/build/lint checks, runtime IPC
smoke, strict documentation build and the pinned QuadPlane VTOL landing SITL
chain passed. The focused runtime suite covers competition, in-flight revoke,
old generations, restart, cache eviction, delayed requests, reconnect and
explicit handback. Fake transport checks prove no fake send after revocation;
they do not prove MAVSDK's later physical retries stop. Hosted CI has not run
for this unpushed working branch.

### Authority review and plugin termination removal — 2026-09-27

Review baseline: merged `main` `c3167422f18008f35910319535bcb0275b1111f3`
(PRs #38/#39 included). Six investigation roles were completed read-only by
three GPT-6 Luna MAX agents in two assignments each; the agent service rejected
creation of a fourth agent. Pre-existing `.gitignore` and untracked submodule
state were preserved. This is local development evidence, not clean-head CI.

The synthesized [authority contract](architecture.md#authority-contract-and-remaining-qualification)
requires one software owner and generation, revocation before sends, explicit
handback and an overriding aircraft-side termination latch. It is a target,
not implemented aircraft arbitration. No termination command, RC map, Lua
script, automatic loss-response recipe or Q02 policy is selected.

The plugin no longer implements `EmergencyLand`, reflective mode/parameter
helpers, descent-speed settings or the direct vehicle-fence upload/clear writer.
Its button and boundary callers use one unavailable request boundary and report
failure audibly; they cannot claim aircraft entry or completion. Review found
that even report-only fence maintenance lacked disarmed ownership and could leave
`FENCE_ENABLE=0` after failure. `MPFenceUploader` and its clear control are therefore
deleted; only visual Plan map export remains. No aircraft fence changes occur.
Misleading forced-descent countdown displays are removed. Stored obsolete speed
fields have no runtime reader or compatibility implementation.
The unused `CommLossAction=hover_and_wait` setting is deleted; it had no caller
and could misleadingly suggest a QuadPlane loss-response policy.

| Finding and source evidence | Disposition / next falsification |
|---|---|
| `FlightModeController`, joystick and boundary monitor treated LAND/parameter writes as termination | Removed with caller failure reporting, compiled request tests and source-boundary guards; no physical termination evidence claimed |
| `NOMADBoundaryView.MPFence` mapped kill labels to `FENCE_ACTION=2`; `MPFenceUploader` wrote LAND speed | Direct uploader/clear implementation and speed side effect deleted; visual map export only. Delayed/warn-only hard-boundary settings still conflict with immediate aircraft termination and stay GAP-05/06 blockers |
| `src/main.cpp` direct CLI, default MP `LegacyOneShot`, `ros2/nomad_ros/src/node.cpp` separate Vehicle, native MP flight controls and ELRS RC are independent writers | Active G-M prerequisite: migrate or exclude every software writer and prove FC input selection agrees with core inhibition; concurrent source test must show one effective owner |
| Plugin gimbal stream/fallback, payload `RCx_OPTION`, fence upload and motor-music `SCR_ENABLE`/MAVFTP writes bypass runtime | Fence writer deleted after review found unsafe disable/failure behavior. Other writers remain active G-M maintenance admission prerequisites: disarmed exclusive ownership or removal/migration |
| Runtime 256-response cache is evicted and cleared on restart; replay can execute a mutation again (`src/runtime/runtime.cpp`, `tests/runtime_ipc_test.cpp`) | Active G-M stale-request prerequisite: incarnation/session/generation plus bounded expiry and replay policy; test restart, eviction, queued intents and late requests without retransmission |
| NOMAD joystick handles gimbal/camera/switches, never starts MP flight RC override (`NomadJoystickService.cs`) | LTE flight joystick remains unimplemented/unqualified; prove real pilot input, takeover and handback instead of inferring them from LTE connectivity |
| Effective pinned profile has `FLTMODE_CH=5`, `RC5_OPTION=0`; no transmitter map/chord export | D09 production map blocked: distinguish transmitter output and FC input, read back mode/auxiliary assignments and prove distinct arming/mode/termination/payload controls |
| Pinned `docker/quadplane-tilttri.parm` has `Q_TRANS_FAIL=0` | Qualification-only transition-failure setting, not an approved operational failsafe baseline; D01/D09 must review it before flight qualification, without weakening failsafes to pass tests |
| G3 text still called Jetson undecided; SITL README omitted later qualified chain slices | Corrected current text; historical dated decisions and runs preserved |
| Existing `test-plugin-geometry` expects null/two-vertex polygons to count as inside (`GeoMath.IsInside`) | GAP-05 containment blocker: missing/invalid competition boundaries must report unavailable and fail preflight; advisory geometry tests do not qualify containment |

Native mechanisms remain the first design choice. The pinned
[ArduPlane AFS implementation](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/afs_plane.cpp)
uses termination outputs for action 42 and enters QLAND for action 43 whenever
QuadPlane is available, without a flight-phase check in `terminate_vehicle`.
This cannot approve fixed-wing/transition QLAND substitution. The
[AFS documentation](https://ardupilot.org/plane/docs/advanced-failsafe-configuration.html)
describes dual GPS/GCS loss, not dual ELRS/LTE loss. Ordinary RC/GCS failsafe
actions are not proof of the directed termination behavior. Dedicated ELRS
activation of the same aircraft-side function is still unresolved; motor-stop
auxiliary options are not a controlled descent substitute. Q02 remains blocked.

Next qualification must isolate ELRS/receiver loss, LTE/GCS loss, both lost
(including FPV still healthy), ground-computer loss with RC healthy and FPV-only
loss. A passive observer must capture source/fault boundaries, vehicle-facing
command traffic, mode, armed/VTOL state and motion. Prove each takeover direction,
cancelled follow-on sends, both explicit handbacks, delayed/replayed requests and
reconnect without reclaim. Both activation routes, termination override and
physical completion require the approved mechanism; complete loss requires the
approved onboard response. ACKs, client exits and host-relay pauses cannot pass
these gates; MAVProxy/other heartbeat producers must be independently accounted
for. No live fault or termination qualification is claimed by this cleanup.

Local validation: baseline `pixi run test-core` configured/built and passed
21/21 CTests (C++ source is unchanged by the cleanup). Focused termination source
guards passed 4/4; `test-plugin-core-client` compiled and passed the unavailable
request checks and existing client suite. `lint-plugin` rebuilt Release with
dead-code errors enabled and passed; `test-plugin-geometry` passed its existing
geometry expectations, including the explicitly blocked absent-boundary behavior.
Full `test-python` passed 681 tests with
4 skipped. Ruff lint/format, source complexity and strict docs build passed.
These checks do not demonstrate aircraft termination, flight authority or live
SITL response. Hosted CI and live qualification must be evaluated separately
from these local checks.

### Repository hygiene cleanup — 2026-09-27

PR #41 starts from `main` `6c4c78d98d8a7469bf3bccfb64273c06d547d312`;
its initial cleanup commit is `7d6076ea4535720c371315121bd856ae54faa241`.
This is separate from the authority review above. It deletes Motor Music,
including the Lua upload and `SCR_ENABLE` write, and removes its dedicated
`MAV_CMD_USER_1` operation from the Mission Planner client, C++ public API and
CLI. It also deletes migration-only deployment profiles, incomplete Gazebo
services and fail-only tasks, unused Python Edge dependencies, stale handoffs
and unreferenced scripts. The module SDK, disabled-by-default example, CPU ROS
integration and qualified Copter/QuadPlane SITL paths remain.

Local validation for the initial cleanup commit: `pixi run test-core` passed
21/21 CTests; `pixi run test` passed 666 tests with 4 skipped; ROS integration
passed 9/9 tests. Mission Planner build-only, dead-code lint and focused
core-client checks passed, as did Ruff lint/format, type checking, source
complexity, strict docs build, supported Compose configurations, three-profile
listing and an isolated Python wheel install/import. These checks do not qualify
live link loss, manual takeover or aircraft termination.

## Current implementation inventory

The [dated MAVSDK compatibility evidence](#mavsdk-compatibility-evidence-2026-09-19)
records the historical fork baseline and hosted qualification.

"Implemented" below means source and focused tests exist, not flight readiness.

| Area | Source and tests inspected | Actual scope and remaining limitation |
|---|---|---|
| C++ foundation | CMakeLists.txt; include/nomad; src; registered CTest coverage (current count above) | Library and CLI build against the mandatory MAVSDK transport; no Python or mavgen build dependency, no Python runtime dependency |
| MAVLink | src/mavlink (MAVSDK transport); core_test, mavsdk_connection_test, mavsdk_zero_delivery_test | MAVSDK owns framing/transport; NOMAD owns ACK classification, typed telemetry, heartbeat/relay handling, the zero-setpoint stop and fence/parameter traffic; native serial/TCP absent |
| Vehicle | src/vehicle/operation.cpp; src/vehicle/vehicle*.cpp; src/telemetry/identity.cpp; core_test.cpp; operation_capability_test.cpp; QuadPlane route/recovery/transition tests | Aircraft recognition is separate from a fail-closed per-operation capability policy; the pinned QuadPlane profile admits semantic GUIDED arm, VTOL takeoff, both explicit transitions, a two-point fixed-wing route and recovery; Plane, Unknown and adjacent QuadPlane operations remain rejected before transport |
| Missions | src/mission/executor.cpp; core_test.cpp | Synchronous small step executor; no integrated cancellation, persisted resume, survey or Task 2 workflow |
| Safety | src/safety; safety_test, fence_config_test, velocity_config_test, vio_source_test | Finite/range gates, VIO-conditioned velocity, watchdog, configured target fence, upload/readback and payload interlock |
| Stop delivery | tests/mavsdk_zero_delivery_test.cpp; scripts/dev/core_sitl_zero_delivery.py | Live peer-driven wire tests cover every stop path on the MAVSDK transport; whole-link outage cannot guarantee delivery; merged-main Copter SITL evidence is recorded below and must be rerun when the transport, fixture or firmware changes |
| Mission Planner | NomadCoreClient, OutputController, FlightModeController, GimbalController, BoundaryManager | LegacyOneShot retains goto/discrete outputs; PersistentRuntime covers only its protocol-v1 typed subset; native maintenance parameter/gimbal paths and UI-owned decisions remain; direct plugin fence and termination writers are removed |
| ROS 2 | ros2/nomad_ros/src/node.cpp, translation.cpp; tests/ros | Owns a Vehicle, telemetry topics, VIO health/source gate and Trigger services; blocking callbacks, no selected estimator or navigation fusion |
| Video | python/tools/simple_video_bridge.py, video_bridge_server.py; test_simple_video_bridge.py | ROS image to GStreamer/RTSP; control HTTP is loopback-only; no validated capture/CV/VIO product pipeline |
| Profiles | scripts/profile.py; three product profile files; test_deployment_profiles.py | Canonical endpoint and stale-setting checks exist; optional workloads and hardware remain unqualified |
| MAVSDK | CMake mandatory transport; `MavlinkConnection` (commands, telemetry, deterministic peer fixture); qualified telemetry smoke; provenance and CI gates | Phases A-E landed: the transport parity fixture passes (accepted/denied/timeout ACKs, COMMAND_LONG/COMMAND_INT, wrong identity, command parity for mode/takeoff/goto/land/RTL/servo/relay/gimbal-config, unlatched GCS-heartbeat announcement, coalesced-datagram handling, live-to-stale link observation, body-frame velocity parity with the zero setpoint on disconnect, fence upload/readback/enable-verification and parameter reads), the CLI, the ROS 2 adapter and the CTest targets build it, and the Phase E cutover deleted the hand-written codec; the unused legacy stream-request surface is removed, while focused Copter/Plane/QuadPlane identity classification is now present and full supported-aircraft coverage remains open |
| Competition | No dedicated implementation found in src/include/ROS/Python/plugin scans | Official telemetry, traffic model/deconfliction, herd survey, tracker/path and sampling workflows are open; events await contract |

The working tree removes the Edge Core source/service/API and many camera/
SLAM/terminal UI components. Their removal is already present, not performed by
this documentation change. Generic payload panels, RTSP playback, ROS adapter
and Python video tool remain. Do not restore deleted architecture to compensate
for missing workflows.

## Contradictions and concrete follow-up

| ID | Finding | Required resolution |
|---|---|---|
| C01 | PLAN/TODO previously repeated obsolete phases and incompatible pass counts | This document owns evidence; PRD decisions and TODO tasks link here |
| C02 | Deleted build/task/image references repaired in the current G1 slice; package now includes retained Python tools | Local checks recorded below; live image/SITL/ROS qualification remains open |
| C03 | Older docs said Edge Core still runs and only two deployment profiles exist | Three profiles are product scope; qualify actual entrypoints in G3 |
| C04 | CLI key check accepts any nonempty value; CLI logs admission before outcome; library has no inherited auth/audit | OS trust boundary now; authenticated client protocol and full lifecycle audit at G2 |
| C05 | Separate CLI invocations, ROS Vehicle and direct plugin writers can conflict | One integrated command owner, explicit manual handover and per-session cancellation |
| C06 | Removed plugin LAND-as-termination dispatch, descent tuning and LAND fence-action translation; activators report termination unavailable | Ground-side policy bypass removed; approved aircraft mechanism, activation mapping, authority latch and physical evidence remain G2/G7 blockers |
| C07 | Generic servo/relay paths bypass release_payload's consuming interlock; plugin has its own timers/confirmation | Reserve hazardous channels, unify authorization in core, provide independent output timeout/feedback |
| C08 | ROS callbacks wait synchronously; receipt timestamps and mixed odometry frames can misrepresent freshness/frame | Bounded worker operations; acquisition-time/frame contract and independent axis/time tests |
| C09 | VehicleState had validity flags but no per-field age, so connection freshness was not position freshness. Position freshness is now enforced by the core: `Vehicle::wait_for_location`/`wait_for_altitude` reject a position sample older than the configurable `position_freshness_timeout`, and battery/GPS/attitude timestamps are stamped but not yet gated | Add independent field timestamps; reject fresh-heartbeat/stale-position decisions |
| C10 | Product profiles previously autostarted unqualified optional workloads; VIO_SOURCE_REQUIRED does not select core policy | Optional autostarts now remain off; G3 must qualify providers and runtime readiness before enabling them |
| C11 | Profile sync wrote retired Jetson keys and allowed ambiguous/stale endpoints | Product profiles now validate canonical endpoints and clear retired/stale profile-owned settings |
| C12 | Deleted service owners remained in CLI/systemd/config; media HTTP bound broadly without auth | Effective service inventory is reconciled and unauthenticated media control is restricted to loopback |
| C13 | Copter GUIDED=4, LAND=9, RTL=6 and demo flows are treated as board-generic support | Detect/validate vehicle class; qualify ArduPlane/QuadPlane and Copter separately before Task 1 |
| C14 | Prior ZED/sim components were deleted; current sensor/workload path is unspecified | Preserve optional compute goals, choose supported capture/provider paths; no ZED prerequisite |
| C15 | Scan-based failsafe and traceability tests do not prove all command surfaces are safe | Semantic parameter/output authorization and actual fault tests; retain scans as structural checks |
| C16 | MAVSDK adoption was both "decided" and awaiting approval; Phase A was marked complete/in-progress in several places | User confirmed early release prerequisite; one transport record and G-M gates below |
| C17 | Preview plan asserted Task 2 battery swaps and event uploads, left all task details TBD | Replaced with AE27 inventory; swaps and event protocol remain Q03/Q04, not official facts |
| C18 | Generic survey/animal identity plan omitted specific deer clusters, tag codes and timed text/CSV submissions | G5/G6 now use exact task artifacts and scoring oracles |
| C19 | Advisory-first wording could imply display alone meets deconfliction; payload autonomy was wholly pending CONOPS | Actual cylinder avoidance required; manual operation permitted; Task 2 autonomy separately scored, D05/Q06 still needed |
| C20 | Generic link-loss recovery and fence upload could be mistaken for competition termination compliance | Independent all-mode termination, C2-path loss, 100 m AGL and verified hard-polygon containment are G7 gates; internal soft inset remains unchanged per U-FEN-01 |
| C21 | Profile README still claimed template credentials/endpoint defects; SITL README retained removed task; CONTRIBUTING described existing Edge Core | Local summaries reconciled to repaired source; no runtime code changed |
| C22 | GeofenceConfig.MaxAltitudeAglMeters and NOMADBoundaryView default to 122 m; preset display checks 122 m; optional soft-from-hard inset uses a project buffer | CONOPS ceiling is 100 m AGL; qualify core-owned limits and truthful UI, reject old competition presets, preserve the plugin internal inset unchanged per U-FEN-01; no second official polygon required (GAP-05/G7) |
| C23 | `Vehicle::motor_test` sent MAVLink command ID 139, which is not a `MAV_CMD` entry in the pinned dialect (`MAV_CMD_DO_MOTOR_TEST` is 209), so no autopilot handler matched it; `tests/output_command_test.cpp` pinned 139 and no SITL scenario exercised the verb, so nothing caught it. Now sends 209, with the parameter layout confirmed on live SITL (`MAV_RESULT_ACCEPTED`), the unit test repinned, `tests/test_command_ids.py` resolving every hand-typed command id against the pinned dialect definition, and a motor-test parity case in the MAVSDK fixture | Keep the dialect-id gate green when a verb is added or the pinned submodule moves; the acceptance is an acknowledgement plus status text, so obtain an independent observation of motor behaviour before relying on it for a flight clearance |
| C24 | The `core-sitl-*` scenarios verify against a host-side SITL copy fed by MAVProxy, but ArduPilot streams position, attitude and extended status only after a GCS requests the group while the C++ core deliberately never requests streams (measured: a passive vehicle link delivered ~1 Hz heartbeats and no position/attitude until `REQUEST_DATA_STREAM`), so with the Python edge service removed nothing asked for the streams the copy is read for and it carried heartbeats only. The SITL stack now applies `docker/sitl-streams.parm` (SERIAL0 group rates) at startup via `--add-param-file`, and the host copy carries heartbeat at 1.00 Hz plus position/GPS/attitude/status at ~4 Hz | Keep the SITL copy self-sufficient for the streams the core consumes when the stack or its service inventory changes, and re-run the scenarios when either moves |

## Official CONOPS gap analysis

Requirement IDs resolve in the [PRD source inventory](conops-requirements.md).
Every row is open. Existing primitives or mocked tests are partial implementation
coverage, never end-to-end competition acceptance. Dependencies are prerequisites,
not authorization to fly. Major gate evidence is expanded below this table.

| Gap / requirements | Required behavior and current source coverage | Owner | Dependencies | Safety risk | Objective test / required evidence | Gate | Blocked decision |
|---|---|---|---|---|---|---|---|
| GAP-01 / U-CORE-01, U-ADAPT-01 | Persistent C++ runtime and versioned local IPC now provide one NOMAD command owner for typed clients; ROS, native Mission Planner MAVLink, pilot/RC and maintenance writers remain independent | C++ runtime / client adapters | G-M, global handover contract | Conflicting actions and stale authorization | Concurrent client/pilot takeover/replay tests; one accepted writer, no automatic resume | G2 | D02/D10 |
| GAP-02 / AE27-NET-002 through AE27-NET-006 | Required telemetry fields; state.hpp lacks per-field age, position accuracy, link metrics and official mode model; protocol.cpp supplies MSL/home-relative altitude | C++ telemetry / link adapter | Firmware sources, time and datum model | False position/AGL/health | Independent fixtures for terrain change, fresh heartbeat with frozen position, unknown accuracy/battery/link and overlapping modes | G2/G4 | D08/Q04 |
| GAP-03 / AE27-NET-001, AE27-NET-009 through AE27-NET-014 | Authenticated armed 1 Hz upload and scoring receipt; no competition adapter exists; MAVLink heartbeat is unrelated | Competition adapter | GAP-02, official token/schema contract | Hidden outage or blocked control | Independent receiver clocks, 300 s armed run, 30 s+ outages, cadence boundaries, expired auth, startup armed; official server acceptance | G4 | Q04/D11 |
| GAP-04 / AE27-NET-007/008, AE27-INT-002 | Receive 1 Hz traffic and avoid cylinders; no traffic model or advisories | Adapter / C++ safety / operator | GAP-02/03, approved cylinder semantics and response budget | Collision / stale feed interpreted clear | Crossing/head-on/overtaking, vertical separation, exact boundary, duplicate IDs, stale/reordered feed; demonstrate actual operator avoidance within budget | G4/G7 | Q04/D05/D08 |
| GAP-05 / AE27-OPS-005/006/020/037/038 | Continuous all-mode concave/altitude containment; geofence.cpp checks position targets only, unset fence allows targets; fence upload not automatic enforcement proof | ArduPilot / C++ safety | Verified hard polygon, internal inset distance, AGL source, firmware | Flyaway / wrong termination trigger | Reject absent/invalid competition fence; readback action/altitude; trajectory crossing despite inside endpoint; breach tests all modes | G2/G7 | U-FEN-01 resolved; hard-fence/AGL evidence still required |
| GAP-06 / AE27-OPS-015 through AE27-OPS-019/035 | Aircraft-specific termination always available, five-second entry and C2-loss self-termination; watchdog zero and plugin LAND dispatch insufficient | ArduPilot / safety hardware; C++ verifies readiness | Qualified termination path, phase/type detection | Catastrophic uncontrolled aircraft | Props-off judge demonstration plus isolated SITL fault matrix; independently measure fixed-wing outputs, rotary descent/touchdown, C2/core/power faults and transition phases | G7 | Q02/D01/D09 |
| GAP-07 / AE27-T1-001/002/008/012 | Full 3–5 km prescribed lap route and single-battery reserve remain open; individual pinned QuadPlane slices through QLAND landing are qualified in SITL, while executor.cpp still has synchronous steps and Copter mode constants | C++ mission / ArduPilot | G-M QuadPlane parity, GAP-05/06 | Invalid transition, exhaustion, false landing success | Continuous QuadPlane task lifecycle/course/laps/abort SITL, then single-battery field rehearsal with independent position/landed evidence | G5/G7 | D01/D08; actual course |
| GAP-08 / AE27-T1-003 through AE27-T1-007/010/011 | Deer totals/clusters/codes/anomalies and timed TXT; video bridge moves images, no task perception/export | Python perception / C++ mission / ground evidence | Capture hardware, calibrated imagery, clock/attempt model | Wrong target or misleading count | Held-out tagged decoys/distractors/occlusion, clustering oracle, timed upload name/content checks and operator audit | G5 | Q05/D01/D08/D11 |
| GAP-09 / AE27-T2-001 through AE27-T2-006/017 | Custom compliant tracker and single attachment; no tracker interface, generic outputs only | Tracker hardware / payload / C++ mission | Chosen device/mechanism and safe feedback | Duplicate release, wrong target, contact harm | Complete tracker mass/axis measurement, placement witness, duplicate/reboot/no-contact/jam tests; 100 m exclusion through sampling/return | G6/G7 | D04 and official site geometry |
| GAP-10 / AE27-T2-007 through AE27-T2-010/018 | Five-minute path/ISO8601 CSV on time; no tracker ingestion/export | Tracker adapter / evidence tools | Device, time synchronization, GAP-09 | Lost person offset / falsely precise path | Independent 5 m/s trajectory with burst/gap/reboot faults; linear interpolation 5/15 m error scoring and deadline upload | G6 | Q07/D04/D08 |
| GAP-11 / AE27-T2-011 through AE27-T2-015/019 | Intact egg, 1-4 cm green core, four spheres on pad; release_payload interlock not collection verification; raw output bypass exists | Payload hardware / C++ safety | Mechanism/feedback, reserved channels, authority | Unintended actuation, damaged sample | Independently inspect quantity/integrity on pad; power/link/jam/unknown feedback, no repeated irreversible action | G6/G7 | D04/Q07 |
| GAP-12 / AE27-T2-020/021; AE27-INT-001 | Optional no-intervention bonus; conditional swap recovery; no autonomous task/persisted resume | C++ mission / operator | GAP-09/11, explicit strategy | Unsafe autonomous interaction / replay | Continuous no-intervention success record if bonus selected; swap/resume permissions expire and records persist if swaps permitted | G6 | D05/Q03/Q06 |
| GAP-13 / U-PROF-01 through U-PROF-03; AE27-OPS-004/027/029 | Core viable in all profiles with truthful missing features and live map; templates tested, capability service/capture/resource evidence absent | Integration / Mission Planner | Qualified camera/radio/compute, single owner | Frozen video/map or overload starves safety | Clean boots without GPU/ROS/camera; source loss and saturated video/LTE/RTK; position-age display and measured deadlines | G3/G7 | D01/D02/D06/D08/D09 |
| GAP-14 / AE27-OPS-023 through AE27-OPS-036 | Physical aircraft, mass, electric power, RF, prop inhibit and FRR | Airframe / safety / flight leads | Final hardware and approved procedures | Unsafe/ineligible aircraft | Per-aircraft weigh/BOM/licence/prop-inhibit inspection, full proof-flight video, weather/energy envelope and approved FRR | G7/G8 | Q03/Q08/D01/D10 |
| GAP-15 / AE27-ADM-001 through AE27-ADM-035; AE27-OPS-022 | Deadline, eligibility, publication, preparation and attempt evidence; no competition deliverable workflow | Competition lead / ground evidence | Roster, owners, secure storage and reviewed rubric | Lost eligibility/evidence, mixed attempts | Timed isolated-crew rehearsals, attempt reset, file/heading/page/rubric and private receipt checks | G8 | D10/D11/Q08/Q09 |
| GAP-16 / U project MAVSDK decision | Closed for transport. Vehicle classification and narrow pinned QuadPlane slices are SITL-qualified, including arm/takeoff, both transitions, route/recovery and QLAND landing; full QuadPlane/Task 1 parity remains open | Transport lead | Phase A pins/licences/CI/SITL/budgets then parity | Mistaking telemetry smoke for safe control | Phases A-E evidence, Copter and QuadPlane, watchdog/stop/heartbeat/fence/parameters, production provenance and rollback | G-M | D08/D10; pinned landing run 36210548163 |

Telemetry frame review must include the ROS odometry NED label with up-positive
position and body-frame metadata, and velocity sign conversion through both
adapter and wire encoder. Do not declare an axis correct from a test mirroring
the same conversion.

## Evidence contract

For each gate retain a sanitized manifest: requirement IDs, exact repository
artifact/diff identity, firmware and dependency pins, profile/config hash, test
command, environment class, dates, independent witness, measured values versus
approved thresholds, pass/fail/skips, artifact reference and accountable reviewer.
Store recordings/logs privately where appropriate; never commit raw transcripts,
secrets, machine-specific endpoints, wildlife coordinates or build output.

A mock proves the adapter contract against the mock; official-server acceptance
is separate. A unit test proves its scenario; SITL does not prove electrical
de-energization, RF independence or aerodynamic feasibility. A configured CI job
is not a passed run. A skipped required scenario leaves its gate open.

Historical notes (2026-09-03–06) reported Copter 4.7.0/4.7.1 SITL success for
status, command flow, mission, velocity watchdog, fence upload/readback, payload,
link loss/recovery and Linux GCS-heartbeat with negative control. Those reports
were not independently re-established on the pre-merge working tree; the
merged-main qualification below now supersedes them for the pinned Copter path.
Historical MAVSDK build/footprint measurements remain in its decision record.

Historical live containment and first Linux zero-delivery runs were explicitly
open despite stronger wording elsewhere. Hosted run `34648914427` at NOMAD
`6a3e970` on 2026-09-11 resolved the zero-delivery discrepancy: an independent
pymavlink observer recorded the ordered nonzero then all-zero setpoints, and the
vehicle held hover afterward. The full run later stopped at SR-LNK-04 because the
GCS-heartbeat harness measured 2.5 Hz against its documented 1 Hz ceiling. That
historical run did not close the remaining scenarios or containment gate.
Merged-main run `35489428247` now passes the complete Copter matrix; no hardware
evidence was supplied for this review, and aircraft-class, all-mode and hardware
availability beyond the user's stated selections is not inferred.

## Gate sequence and accountable roles

Roles below need named owners (D10). Dependencies order implementation, not
permission to operate hardware. No gate is closed merely by this planning change.

### G0 — Planning baseline (lead + competition lead)

Reconcile these docs, preserve stable safety IDs, resolve links and traceability.
The full v1.0 CONOPS is now read and inventoried under AE27 IDs in the PRD
source appendix. Preserve the retired P-ID crosswalk; retain resolved Q01/U-FEN-01, resolve Q02-Q09 and assign
named owners without pretending this review closes organizer questions.
Exit: strict docs build and traceability test pass; no unsupported requirement
claims; assigned decision owners and resolution of blocking interpretations
remain necessary. This pass establishes provenance, not flight readiness.

### G1 — Executable baseline (build/integration lead; depends on G0 planning)

Repair C02/C10-C12 deployment/task leftovers without reintroducing Edge Core.
Retain video and both optional compute placements. Demonstrate fresh checkout
builds and selected simulation startup with no deleted-path dependencies.
Exit: test-core, test-python, lint, format-check, complexity-check, docs-build,
plugin compile/helper checks and supported ROS build/tests. Check command paths
against actual files; current runtime repairs require their own focused change.

#### C02 repair evidence - 2026-09-09

Baseline commit: `d31b0aa`; subsequent runtime-repair topic work is recorded in
repository history. The repair removes deleted Edge Core services, image jobs,
console entrypoint and missing API/gimbal test tasks. `dev`/`dev-build` build the
core; `sitl` builds it before simulator startup. Compose forwards host traffic
independently of ROS, with the ROS destination enabled by `sim-ros-up`. The SITL
CI default no longer duplicates the private relay output. ROS image/module paths
package the retained video tool. Incomplete perception/Gazebo startup tasks exit
unavailable before starting containers; optional images and media/compute source
remain.

Local evidence: CTest 9/9; `pixi run test` 258 passed, 3 environment skips;
retained Python coverage 96.65% against the unchanged 85% floor. Seven runtime
wiring cases cover packaging metadata/imports, workflow task references, Compose
paths and absent-provider failure. A setuptools wheel was built, extracted
outside the checkout, and imported under isolated Python; video `--help` passed.
Daemon-free Compose resolution passed for SITL, ROS and Gazebo scaffolding, with
expected unique SITL/ROS output destinations. The unavailable perception task
returned exit 1 with its stated reason. Lint, format, type-check, complexity,
strict docs and all changed-file pre-commit hooks passed. Whole-tree pre-commit
remains blocked by the pre-existing missing SPDX header in
scripts/hardware/servo_test.c; that
unrelated hardware file is unchanged.

Docker daemon unavailable: no image build, live SITL, ROS integration, sensor
stream or GPU/hardware qualification was run for this repair. G1 is not closed.
Python dependency pruning is deferred; the distribution
name `nomad-edge` remains for compatibility although its deleted CLI is removed.

#### C10-C12 repair evidence - 2026-09-09

The three product profiles now use explicit C++ UDP listener endpoints, contain
no development key, and leave optional ROS/video/container workloads disabled
until their providers are qualified. Loading and Mission Planner synchronization
validate profile identity, endpoint scheme, host and port before mutation.
Switching profiles preserves unrelated plugin settings, clears empty owned
settings and removes retired Jetson fields. The shell profile command delegates
to the validated Python implementation.

The deployment CLI and systemd inventory no longer reference the deleted Edge
Core service. The retained video service directly owns its in-container Python
process, and the Jetson image includes that module. Its unauthenticated control
HTTP endpoint accepts loopback binds only; wildcard and remote binds fail before
the media pipeline starts.

No image, service or hardware runtime was started. Optional compute, camera,
estimator, RTSP output and clean-boot systemd behavior remain unqualified.

Local evidence: 297 Python tests passed with 3 skipped and 96.34% coverage;
all 9 CTest targets passed; strict documentation, lint, formatting, type checks,
complexity limits, lock validation, changed-file pre-commit hooks and edited
shell syntax checks passed.

#### Setup and provisioning repair evidence - 2026-09-09

Development bootstrap now builds the C++ core and checks the product profile
manager. Local and remote Jetson setup load or validate a supported product
profile, build the core, install the retained service inventory and reject
unqualified optional autostarts. Tailscale and Jetson firewall setup no longer
opens the deleted HTTP API or grants blanket access on the Tailscale interface.
Remote setup uses the existing SSH known-hosts trust store and rejects unknown
host keys. No remote command, package installation, firewall mutation, service
start or hardware check was run; these paths have structural and mocked-command
evidence only. Local evidence after integration: 313 Python tests passed with
3 skipped and 96.34% retained-module coverage; all 9 CTest targets passed; lint,
formatting, type checks, strict documentation, complexity, lock, shell syntax,
PowerShell parsing and changed-file pre-commit checks passed.

### G-M — MAVSDK adoption (transport lead; early prerequisite after G1)

The Phases A-E implementation is landed: the mandatory MAVSDK transport and
parity paths are in source, focused tests cover them, the default runtime and
ROS 2 adapter use MAVSDK, and the hand-written codec is deleted. Preserve unit
fault coverage, independent wire observations and CLI/plugin compatibility as
the current-head integration and release gates are completed. Test both Copter
and the chosen QuadPlane firmware as support is introduced; existing Copter
tests alone cannot qualify Task 1.

Current-head hosted Copter evidence is now recorded in run
[35489428247](https://github.com/YoussGm3o8/NOMAD/actions/runs/35489428247) at
merged `main` commit `26d7f9b101a029725d06aee2c6716da95e622417`. The full
telemetry, command, mission, velocity, payload, link, heartbeat, loop-closure
and geofence sequence passed. ROS, supported-aircraft/QuadPlane, resource-budget,
packaging and install/rollback gates remain open.

The first supported-aircraft implementation slice now carries the ArduPilot
autopilot and vehicle type from heartbeat discovery into `VehicleState`. It
classifies known Copter, Plane and QuadPlane identities, selects their guided
and return-to-launch modes, rejects unknown identities, and refuses non-Copter
body-frame velocity. Only Copter landing is supported by this slice. Plane and
QuadPlane landing are rejected before transmission because their direct
COMMAND_LONG NAV_LAND behavior has not been qualified. Focused tests exercise
this policy. ArduPlane's fixed-wing heartbeat is classified as
QuadPlane for `Q_ENABLE=1` or `2`, Plane for zero, and Unknown when the parameter
is unavailable or invalid.
The pinned observation harness described below now provides QuadPlane SITL
identity, telemetry, baseline-mode evidence, NOMAD arm + VTOL takeoff
qualification and one live NOMAD VTOL-to-fixed-wing transition qualification.
At that stage this did not close the aircraft gate: the full Task 1 course/lap
route, fixed-wing to VTOL transition and landing still needed separate
evidence. The fixed-wing-to-VTOL transition is now qualified for the pinned
SITL profile below. Full Task 1 execution, landing, hosted fault evidence and
the supported-aircraft ROS/SITL and release matrix remain open.

Create focused, reviewable implementation changes with unit tests, integration
evidence, requirement mapping and limitations. Under the clarified ownership
boundary, MAVSDK and the pinned fork own transport/framing, command encoding and
low-level ArduPilot protocol behavior. NOMAD may derive safety-relevant aircraft
identity and operation capabilities from reported heartbeats and parameters when
needed for admission and authoritative verification, and must fail closed when
that evidence is missing or ambiguous. New raw wire or command behavior belongs
in the fork and requires reproducible tests plus separately tracked upstream work
when publication is authorized.

Implementation exit: phases A-E in [MAVSDK adoption](mavsdk-adoption.md) are
landed; default production runtime demonstrably uses MAVSDK and legacy deletion
is complete. Release exit remains open until the supported-firmware matrix,
current-head ROS/SITL matrix, transitive notices and packaging/install/rollback
evidence pass. Phase F upstream acceptance may lag, but required patches must be
maintained and reviewed. G-M is mandatory for G8 and precedes dependent
integrated competition command work; isolated server/CV prototypes may proceed
without waiting.

Cutover status (2026-09-24): phases A-E have landed. The default runtime is the
MAVSDK transport, the ROS 2 adapter builds the same transport, and the legacy
codec plus its generated dialect headers are deleted, so the transport exit items
are met at the code and local-evidence level. G-M itself stays open: the pinned
QuadPlane profile now has observation, arm/takeoff, forward transition,
two-point route, fixed-wing recovery and fixed-wing-to-VTOL transition
qualification. Implementation-head
[workflow 35980310680](https://github.com/YoussGm3o8/NOMAD/actions/runs/35980310680)
passed transition-back and full Copter SITL regression. Generic RTL/QRTL, VTOL
landing, link-loss/manual takeover, full Task 1 execution, the supported-firmware
matrix and install/rollback evidence remain open. A completed transport cutover
is not a competition release.

### QuadPlane 4.7.1 observation and startup qualification profile - 2026-09-21

Requirement: G-M aircraft-class qualification. The reference is ArduPlane
`Plane-4.7.1`, exact commit `dbe792162d06cab66c3475fd5556bf7a120f119e`, built
by `docker/Dockerfile.sitl-plane` with frame `quadplane-tilttri` and
`docker/quadplane-tilttri.parm`. The NOMAD baseline remains commit
`844cfcff3c24a765e8b8688b5332bbd2983237cf`; MAVSDK remains pinned at
`3f85f6f808b617c736316d7da5f51f3d3eba1737`.

Independent local SITL observation found `MAV_AUTOPILOT_ARDUPILOTMEGA` (3),
`MAV_TYPE_FIXED_WING` (1) and the explicit reference-profile value `Q_ENABLE=2`;
it did not report a VTOL MAV type.
NOMAD therefore classifies this exact combination as QuadPlane. It also treats
`Q_ENABLE=2` as QuadPlane and zero as Plane; a missing or invalid value leaves
the fixed-wing identity unresolved as Unknown. The live observer
also required disarmed state; fresh heartbeat, position, GPS and attitude; and
aircraft-reported GUIDED=15, QLOITER=19, QRTL=21 and RTL=11 after each command.
The deterministic peer proves values zero, one and two plus unavailable and
invalid parameter cases, and proves generic-autopilot and unknown-vehicle
identities transmit no requested mode command. The stale-position falsification
test exceeds the 1500 ms bound and is rejected.

The first selected QuadPlane flight mechanism is ArduPlane's direct GUIDED
`MAV_CMD_NAV_TAKEOFF` dispatch (command 22), which enters GUIDED, arms through
the qualified NOMAD arm path, and treats its altitude parameter as a climb
delta from the final pre-command relative altitude. NOMAD captures and validates
fresh heartbeat, position, 3D GPS, armed state and GUIDED mode after preparation,
then verifies the derived target within a fixed 0.5 m completion margin. It is a
QuadPlane-specific semantic operation, not reuse of generic Copter `takeoff`.
The focused fake-transport tests reject a 4 m partial climb or disarm after the
ACK, while the hosted pinned profile proves the live command sequence and full
target climb. Mission semantics such as `NAV_VTOL_TAKEOFF`,
reviewed fixed-wing waypoint navigation and `NAV_VTOL_LAND` are separate from
this direct takeoff qualification. This takeoff slice did not qualify disarm,
arbitrary modes, generic takeoff/goto, cruise, return, transition, landing, link
loss or manual takeover. Later sections record the separately qualified pinned
forward transition, route/recovery, fixed-wing-to-VTOL transition and narrow
QLAND operation. Link-loss/manual takeover remains unqualified; body-frame
velocity and direct `NAV_LAND` remain unsupported for QuadPlane.

### QuadPlane VTOL-to-fixed-wing transition qualification - 2026-09-22

Requirement: G-M QuadPlane transition qualification. The selected mechanism is
ArduPlane's `MAV_CMD_DO_VTOL_TRANSITION` (command 3000), sent with
`param1=MAV_VTOL_STATE_FW` (4). The pinned source dispatches this command from
`ArduPlane/GCS_MAVLink_Plane.cpp` to
`QuadPlane::handle_do_vtol_transition`; `ArduPlane/quadplane.cpp` accepts it
only in `AUTO` and accepts only `MAV_VTOL_STATE_MC` and `MAV_VTOL_STATE_FW`.
The fixed-wing case clears `auto_state.vtol_mode`. This is the reviewed
ArduPilot mechanism, not a generic mode change and not a Copter takeoff
interpretation.

The exact reference remains ArduPlane 4.7.1 at
`dbe792162d06cab66c3475fd5556bf7a120f119e`, frame `quadplane-tilttri`, with
`MAV_TYPE_FIXED_WING`. The qualification profile explicitly sets
`Q_ENABLE=2` (ArduPlane's “Enable VTOL AUTO”), `Q_FRAME_CLASS=7`,
`Q_TILT_ENABLE=1`, `Q_TILT_MASK=3`, `Q_TILT_TYPE=0`, `Q_ASSIST_SPEED=6`,
`Q_TRANSITION_MS=5000`, `Q_TRANS_FAIL=0` and `Q_TRANS_FAIL_ACT=0`.
`Q_ENABLE=2` is required because the pinned `ModeAuto::_enter()` starts AUTO
in VTOL AUTO only for value 2. `Q_ASSIST_SPEED=6` is explicit because the
inherited 18 m/s value left this SITL tilt-tri in `AIRSPEED_WAIT`; the selected
forward waypoint reaches the pinned transition's airspeed condition at the
lower profile value. `Q_TRANS_FAIL=0` leaves ArduPilot's optional transition
failure action disabled; NOMAD owns the bounded verification deadline.

The authoritative completion signal is `EXTENDED_SYS_STATE.vtol_state`. The
pinned `GCS_MAVLink_Plane.cpp` implementation returns
`QuadPlane::transition->get_mav_vtol_state()`, and the pinned
`SLT_Transition::get_mav_vtol_state()` maps `MC` to 3,
`TRANSITION_TO_FW` to 1 and `DONE` to `FW` (4). NOMAD maps MAVSDK's existing
typed `subscribe_vtol_state` callback into a typed, timestamped `VehicleState`.
It admits the command only for QuadPlane with fresh heartbeat, matching system
ID, armed state, `AUTO` mode and fresh `MC` state. After an accepted ACK it
requires a newer VTOL-state observation, keeps `MC` and `TRANSITION_TO_FW` as
incomplete, rejects undefined/back-transition states, fails stale or interrupted
telemetry closed, and times out after the 90-second vehicle deadline. An ACK,
mode name, elapsed time, pitch or airspeed alone cannot complete the operation.

The live positive sequence is driven by
`scripts/dev/core_sitl_quadplane_transition.py`: it starts a fresh profile,
observes identity and telemetry, records the initial state, performs the
already-qualified NOMAD VTOL takeoff, establishes a long forward AUTO waypoint
as safe transition setup, observes `MC`, issues the single NOMAD transition
command, records the independent `MC -> TRANSITION_TO_FW -> FW` sequence and
reports success only after fresh `FW`. The forward waypoint is test setup for
this flight primitive; it is not fixed-wing route qualification.

Authority boundary: this primitive requires ArduPlane `AUTO`, but NOMAD still
rejects arbitrary QuadPlane `set_mode`. NOMAD therefore cannot yet autonomously
perform the complete `GUIDED` VTOL-takeoff -> `AUTO` -> fixed-wing-transition
sequence. The live harness uses an independent pymavlink operator/test driver
to establish `AUTO`; that driver is qualification setup, not a NOMAD capability
or a command-authority handover. The new capability admits only the already-
established, narrowly validated transition operation.

Focused falsification covers QuadPlane capability admission and zero
transmission for Copter, Plane and Unknown; command 3000 construction with
parameter 4; rejected ACKs; ACK-without-completion; an intermediate-state
timeout; stale/missing authoritative state; a stopped link during verification;
and fixed-wing completion only after a newer state sample. The deterministic
MAVSDK peer also reports an intermediate state before `FW`, so the success path
does not depend on the ACK alone. No VTOL-back operation is implemented or
claimed.

### QuadPlane fixed-wing route/navigation qualification - 2026-09-23

Requirement: G-M fixed-wing route slice. The selected mechanism is ArduPlane's
standard `MAV_CMD_DO_REPOSITION` (192) through MAVSDK's typed `COMMAND_INT`
transport. Pinned ArduPlane source review selected this over uploading an AUTO
mission: `ArduPlane/GCS_MAVLink_Plane.cpp::handle_command_int_do_reposition`
accepts the location in GUIDED without its `CHANGE_MODE` flag, converts the
relative altitude, sets the guided waypoint and applies a positive loiter
radius; `ArduPlane/commands.cpp::Plane::set_guided_WP` installs the target, and
`ArduPlane/mode_guided.cpp::ModeGuided::navigate` flies the target using loiter
navigation. AUTO mission exhaustion can enter RTL in
`ArduPlane/commands_logic.cpp::exit_mission_callback`, so mission upload would
add mission-state and return semantics outside this slice.

NOMAD sends two sequential `COMMAND_INT` packets in
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`: `param1=0` retains default speed,
`param2=0` keeps `CHANGE_MODE` clear, `param3=30 m` selects clockwise loiter,
`param4=NaN` leaves heading unset, scaled integer `x/y` carry latitude and
longitude, and `z` carries relative-home altitude. Each packet is sent only
after NOMAD's existing semantic GUIDED mode operation succeeds. This does not
admit generic `goto_location`, unrestricted mode setting or Plane navigation.

The public input contains exactly two typed waypoints. NOMAD rejects empty or
other-sized routes, non-finite/out-of-range coordinates, relative altitudes
outside 2–100 m, targets outside the configured global fence, and waypoint
spacing below 100 m. Before setup it requires current QuadPlane identity, a
connected heartbeat younger than 3 s, a nonzero session, position and 3D GPS
younger than 2 s, armed state, authoritative `EXTENDED_SYS_STATE=FW` younger
than 3 s, and AUTO mode. After the existing semantic GUIDED-mode setup, it
repeats the complete readiness check for the same session and requires GUIDED
immediately before each waypoint command. The MAVSDK transport checks the
expected nonzero session and freshly computed heartbeat again at its dispatch
boundary. MAVSDK advances the session generation on connection loss; stale
heartbeat or state also stops the operation.

An accepted `COMMAND_ACK` means only that ArduPlane accepted the request. For
each waypoint NOMAD immediately captures the latest fresh aircraft position as
the ACK-boundary progress baseline, then records a monotonic boundary. It
requires a newer `GLOBAL_POSITION_INT`-derived position sample at least 10 m
closer than that baseline, at most 2 s old, still within the same
QuadPlane/armed/GUIDED/fixed-wing session and backed by fresh GPS. That sample
must be within 45 m horizontally and within 5 m altitude. The pre-command
position must initially be more than 55 m away. A target reached while the ACK
is still pending cannot pass on repeated telemetry at the same position; it
needs additional post-boundary progress. NOMAD sends waypoint 2 only after
waypoint 1 passes that test and reports route success only after waypoint 2
passes. A prior location, intermediate ACK or first-point progress cannot
complete the final point. The overall route deadline is 180 s and each command
ACK wait is capped at 3 s.

`scripts/dev/mavsdk_connection_fixture.py` exercises the production MAVSDK
transport against a deterministic peer, advances each target gradually after
its ACK, and asserts both typed packet frames, mode request, command ID, flags,
radius, unset heading, coordinates and altitudes. `tests/vehicle/quadplane/quadplane_route_test.cpp` falsifies unsupported-class
transmission, malformed route input, stale telemetry, wrong mode/state,
post-setup changes, ACK/upload-only completion, pre-ACK arrival without later
progress, intermediate progress, prior location, timeout and link/session
interruption. The pinned hosted positive
flight and its independent position observer run through
`scripts/dev/core_sitl_quadplane_route.py`. The observer excludes pre-route
position samples and requires the post-command armed GUIDED heartbeat before
counting ordered waypoint proximity.

Hosted evidence: [workflow run 35818612311](https://github.com/YoussGm3o8/NOMAD/actions/runs/35818612311)
passed on implementation head `7f6206cbad51aade79ae86b20983d4e1fb818901`.
It observed `AUTO` and fixed-wing before the route, `GUIDED` during route,
progress `[1, 2]`, both 8 m relative-altitude targets at 42.3932800,-71.1475927
and 42.3932761,-71.1456430, arrival distances 43.1 m and 44.7 m, and
NOMAD completion after 14.7 s. The same run passed all 13 steps of the full
Copter SITL regression. The route was rerun as part of implementation-head
transition-back workflow 35980310680. That run again observed ordered progress
`[1, 2]`, arrival distances 43.0 m and 44.9 m, and completion at 14.7 s. This
route result alone does not qualify route planning, return/recovery,
fixed-wing link-loss response, transition-back, landing or the complete Task 1
flight.

### QuadPlane fixed-wing recovery qualification - 2026-09-23

The selected return mechanism is one explicit `RecoveryPoint` in GUIDED mode,
sent as `MAV_CMD_DO_REPOSITION` through the same typed `COMMAND_INT` transport as
the qualified route. The target is a caller-supplied coordinate and
relative-home altitude; it is not inferred from ArduPlane home or mission state.
The positive hosted scenario uses the independently observed start of
fixed-wing route qualification as its recovery point, after both route points
have passed, at no less than 20 m relative-home altitude. The operation does
not grant generic goto or arbitrary mode control.

Pinned ArduPlane source supports this choice:
`ArduPlane/GCS_MAVLink_Plane.cpp::handle_command_int_do_reposition` accepts an
already-GUIDED target with `CHANGE_MODE=0`, converts the relative-altitude
frame, installs the point and applies a positive loiter radius.
`ArduPlane/commands.cpp::Plane::set_guided_WP` clears VTOL loiter and guided
takeoff, while `ArduPlane/mode_guided.cpp::ModeGuided::navigate` flies the
fixed-wing loiter target. `ArduPlane/quadplane.cpp::guided_mode_enabled` permits
VTOL control in GUIDED when `Q_GUIDED_MODE` is nonzero, so NOMAD requires an
authoritative `Q_GUIDED_MODE=0` parameter readback before transmission and
rechecks the same session and full readiness state afterward. The pinned source
default is zero; this PR does not alter the aircraft profile.

RTL and QRTL are rejected for this slice. `ArduPlane/mode_rtl.cpp::ModeRTL::_enter`
and `navigate` may switch to QRTL or an autoland sequence depending on RTL
configuration; `ArduPlane/mode_qrtl.cpp::ModeQRTL::run` enters VTOL position
control and landing. `ArduPlane/commands_logic.cpp::exit_mission_callback`
can enter RTL at AUTO mission exhaustion. Those paths would make fixed-wing
arrival or post-return state ambiguous. Neither a mission end nor an automatic
failsafe is part of NOMAD's recovery proof.

The readiness gate requires current QuadPlane/ArduPilot/fixed-wing identity,
same nonzero session, live heartbeat, fresh finite position and 3D GPS, armed
state, fresh authoritative `EXTENDED_SYS_STATE=FW`, GUIDED mode, a valid target
and relative altitude from 2 to 100 m, and NOMAD fence inclusion when configured.
The starting distance must exceed 55 m. NOMAD sends exactly one relative-altitude
`COMMAND_INT` reposition packet with a 30 m loiter radius, unset heading and
`CHANGE_MODE=0`. An accepted ACK only starts verification: the position must
advance beyond the maximum of ACK receipt and the last position timestamp,
move at least 10 m closer than the ACK-boundary position, enter 45 m
horizontally, and be within 5 m of the requested relative-home altitude.
Every sample must retain the same session, fresh heartbeat/position/GPS/VTOL,
armed GUIDED mode and fixed-wing VTOL state. Loss, rejection or no arrival
fails; command/ACK and overall recovery deadlines are 3 s and 180 s.
The 45 m region follows the observed fixed-wing route tolerance, and is a
qualification tolerance rather than an obstacle-clearance or landing guarantee.
Completion leaves the aircraft armed in fixed-wing GUIDED loiter. That recovery
result alone does not qualify transition-back; the separate transition-back
slice later in this document qualifies the handoff from this state.

The independent observer in pinned hosted
[workflow run 35980310680](https://github.com/YoussGm3o8/NOMAD/actions/runs/35980310680)
passed after the two-point route on implementation head
`0231481e0fd20ccf5138938276f3bf1f575991c9`. It saw the explicit target
`42.3913000,-71.1476000` at 20.0 m relative-home altitude, 235.2 m initial
distance, decreasing position samples through 75.4 m, 39.2 m minimum distance,
44.3 m completion distance, 3.6 m altitude error and NOMAD completion in
11.5 s. The 45 m bound admits the observed fixed-wing turn and remains smaller
than the initial 235.2 m separation; it is a qualification tolerance only.

The pinned failsafe paths remain independent: `ArduPlane/events.cpp` can change
GUIDED mode on RC/GCS failsafe, `ArduPlane/fence.cpp` can redirect or enter RTL
on a breach, and the configured `Q_TRANS_FAIL=0` disables the transition-failure
timer in `ArduPlane/quadplane.cpp`. NOMAD neither changes those failsafes nor
counts their side effects as recovery; a mode/state change or lost telemetry
fails recovery verification. QuadPlane fixed-wing link-loss response remains
unqualified. Runtime IPC v1 does not expose this operation; the core and direct
CLI own it.

### QuadPlane fixed-wing-to-VTOL transition qualification - 2026-09-24

The selected command is `MAV_CMD_DO_VTOL_TRANSITION` (command 3000), sent as
`COMMAND_LONG` with `param1=MAV_VTOL_STATE_MC` (3) and all remaining parameters
zero. In pinned ArduPlane
[`GCS_MAVLink_Plane::handle_command_DO_VTOL_TRANSITION`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/GCS_MAVLink_Plane.cpp#L923-L929)
passes `param1` into
[`QuadPlane::handle_do_vtol_transition`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/quadplane.cpp#L1890-L1923).
That handler rejects any current mode except AUTO, sets `auto_state.vtol_mode`
for the requested multicopter state, clears forward-throttle demand and adjusts
the pitch limit. The command therefore cannot run directly from recovery's
armed GUIDED state. The qualification authority installs a single
`NAV_LOITER_UNLIM` mission item at the explicitly reviewed recovery coordinates
and sets AUTO before NOMAD acts. The loiter item keeps the explicit
recovery-point altitude. Both the requested loiter altitude and measured
aircraft altitude must independently be inside 15–25 m above home before
command transmission. The core then requires a five-sample, two-second readiness
window with no more than 1 m
of altitude variation; it does not require instantaneous altitude to match the
loiter target. Generic QuadPlane `set_mode` remains blocked.

The direct transition path is distinct from ArduPlane's VTOL landing approach.
[`QuadPlane::get_mav_vtol_state`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/quadplane.cpp#L4286-L4309)
reports multicopter once `in_vtol_mode()` is true. Its
`TransitionToMulticopter` report is tied to QRTL/landing approach stages, not a
guaranteed intermediate for this direct AUTO command. NOMAD accepts either a
direct `FixedWing` to `Multicopter` report or the optional
`TransitionToMulticopter` intermediate and requires the final authoritative
multicopter state. `Q_GUIDED_MODE=0` does not block this path: the handler is
called in AUTO, whereas `Q_GUIDED_MODE` controls whether QuadPlane's GUIDED
mode uses VTOL position control. The handler does not select QRTL, QLAND, a
landing command, disarm or a new mode; the qualification verifies that AUTO
and armed state are retained.

The pinned profile also sets `Q_ENABLE=2`. `ModeAuto::_enter()` therefore sets
`auto_state.vtol_mode=true` when an external authority changes the recovered
aircraft from GUIDED to AUTO; authoritative VTOL state becomes multicopter at
that setup boundary. The qualification installs an explicit loiter mission,
enters AUTO, confirms that state, then uses NOMAD's already-qualified
`transition-to-fixed-wing` operation (target state 4) to restore fixed-wing
flight in AUTO. The new transition-ready gate begins only after that operation
has completed. This narrow setup is required by pinned mode-entry behavior; it
is not generic mode authority or a second transition-to-VTOL qualification.
The exact pinned functions are
[`ModeAuto::_enter`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/mode_auto.cpp#L595-L637),
[`QuadPlane::update`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/quadplane.cpp#L1582-L1659)
and [`Tiltrotor::continuous_update`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/tiltrotor.cpp#L2233-L2430).
The pinned default tilt rate is 40 degrees/s and `Q_TILT_MAX` is 45 degrees;
`QuadPlane::get_mav_vtol_state()` can report multicopter as soon as AUTO's VTOL
mode flag is set, before tilt motion settles. NOMAD therefore requires two
seconds of stable post-ACK multicopter telemetry and position/velocity after
the fresh state reports.

Recovery completion is not transition readiness. The preceding recovery can
complete at 45 m horizontal and 5 m altitude error on its first qualifying
post-ACK sample. This operation requires a separate fixed-wing transition-ready
state: same nonzero session and system, pinned ArduPilot QuadPlane identity,
fresh heartbeat, position, velocity, 3D GPS and authoritative fixed-wing VTOL
state, armed AUTO mode, a valid relative-home target from 15 to 25 m, and current
position within 55 m of the explicit recovery coordinates. Before command
transmission, both the requested loiter altitude and measured aircraft altitude
must independently be inside 15–25 m above home. The measured altitude need
not match the loiter target: five distinct fresh position samples must remain
within a 1 m altitude range
for at least 2 s, independently proving the aircraft has settled. This separates
recovery's broad arrival tolerance from the narrower transition-ready band.
The pinned transition parameters are re-read as `Q_ENABLE=2`,
`Q_FRAME_CLASS=7`, `Q_TILT_ENABLE=1`, `Q_TILT_MASK=3`, `Q_TILT_TYPE=0`,
`Q_TILT_RATE_UP=40` and `Q_TILT_MAX=45`, then session, identity and fixed-wing
state are checked again. Groundspeed must not exceed 28 m/s, groundspeed
variation must not exceed 3 m/s, and absolute climb rate must not exceed 1 m/s.
Five distinct fresh position samples must span at least 2 s; over those samples
altitude may vary by at most 1 m and distance from the transition point by at
most 8 m. The live harness reports per-limit sample counts, peak speed/climb
and the final observed distance/altitude if the independent readiness proof
times out. Thus the core independently stabilizes the boundary instead of
reusing the recovery arrival sample.

The first exact-head attempt showed that the initial 40 m/20 m/s gate could
never become ready in this pinned loiter setup: the independent observer paired
180 position/velocity samples, found only 7 inside 40 m and none at or below
20 m/s, with a 26.8 m/s maximum. The recovery observer measured a 43.2 m
completion distance and the transition-ready observer last measured 41.9 m.
The revised 55 m region adds 11.8 m beyond the observed recovery completion
distance. The first 55 m exact-head attempt showed why the explicit recovery-
point altitude must remain the transition target: the harness invoked NOMAD
with a captured 24.102 m target near the 25 m ceiling, and the core rejected an
altitude outside its reviewed gate before sending command 3000. That failure
did not clearly distinguish the absolute 15–25 m band from the target-match
check. The core was updated to report those conditions separately.
The qualification keeps the original 20 m recovery target and requires the AUTO
loiter state to settle within the 15–25 m band before transmission. The band is
a pre-transition admission envelope; after command acceptance the aircraft must
stay at or above 15 m, but the 25 m ceiling does not constrain climb during
transition.
The next exact-head attempt, `35962664640` at `8b77a4052504c550e356c1d706151b4748decadf`,
completed route and recovery, and the independent observer established the
stable readiness window. NOMAD sent command 3000 and entered post-ACK
verification; ArduPlane logged `Entered VTOL mode`. NOMAD then failed the
transition because measured altitude was 22.303 m against the explicit 20.0 m
recovery-point target, a 2.303 m difference, while still inside the 15–25 m
band. That run did not prove stable final multicopter state. The extra ±2 m
target-match rule added no terrain/ground separation evidence beyond the
absolute altitude band and the 1 m stable-window limit, so it is removed while
both target and measured altitude remain band-limited before command
transmission. This run is diagnostic, not transition qualification evidence.

The next hosted exact-head attempt
[35969810201](https://github.com/YoussGm3o8/NOMAD/actions/runs/35969810201) at
`20a3c520f9323ce7955bfc4d77bf224f7f9aa5f1` repeated the route and recovery.
Recovery completed at 40.4 m horizontal distance with 3.2 m altitude error.
After the readiness window, ArduPlane accepted command 3000 and logged
`Entered VTOL mode`; post-ACK telemetry then reached 25.197 m and the core
failed because the 25 m pre-transition ceiling was still applied during
verification. That red step did not prove stable final multicopter state. The
validator now applies 15–25 m only before transmission. After ACK, it retains
fresh telemetry, armed AUTO mode, the 15 m minimum altitude floor, authoritative
newer multicopter reports and the two-second stable-position requirement. The full Copter
regression in the same workflow passed at this code head (job
`107536628135`); that regression result alone does not qualify transition-back.
The failed transition attempt is diagnostic only.

The implementation-head
[workflow 35980310680](https://github.com/YoussGm3o8/NOMAD/actions/runs/35980310680)
passed both transition-back qualification and full Copter regression on
`0231481e0fd20ccf5138938276f3bf1f575991c9`. In the repeated route/recovery
setup, the recovery portion completed 44.3 m from its target with 3.6 m altitude
error in 11.5 s. The independent transition observer then recorded
`recovery_mode=GUIDED`, `transition_mode=AUTO`, pre-state `fixed_wing`, recovery
distance 34.4 m, altitude error 4.1 m, transition altitude 20.0 m, maximum
ready distance 50.4 m, and 21.1 s stabilization. Maximum groundspeed was
25.4 m/s and maximum absolute climb was 0.5 m/s. It sent command 3000 targeting
`MAV_VTOL_STATE_MC`; observed states were `[fixed_wing, multicopter]`, with no
intermediate state emitted. Stable post-ACK completion took 45.2 s; final mode
was 10 (AUTO) and the aircraft remained armed. Exact-head ordinary CI also
passed. This qualifies transition to armed multicopter flight for the pinned
SITL profile only; it does not qualify VTOL landing, autonomous GUIDED-to-AUTO
setup, manual takeover or hardware flight.
The independent observer reports the maximum distance of its stable readiness
window. The 28 m/s ceiling allows 1.2 m/s above the previously measured peak.
Both are still bounded by the 2 s dwell, 8 m radial variation, 3 m/s speed
variation, 1 m/s climb and altitude constraints; they are specific to the pinned
SITL qualification profile and are not hardware flight approval.

The core then sends exactly one command 3000 request with target state 3. The
accepted ACK establishes only a verification boundary: the maximum of ACK
receipt time and the latest VTOL-state timestamp captured at that point. A
pre-ACK multicopter report cannot pass. Completion requires two fresh
post-boundary multicopter state reports and three fresh position samples
spanning at least 2 s, all inside the 55 m region, with groundspeed at most
3 m/s, absolute climb at most 0.5 m/s, position spread at most 4 m and altitude
variation at most 1 m. A final readback rechecks the same session and system,
fresh heartbeat/position/GPS/velocity/VTOL telemetry, armed state, AUTO mode and
post-ACK multicopter state. The overall deadline is 90 s and ACK wait is capped
at 3 s. Rejection, missing ACK, lost or stale telemetry, disarm, session/system
change, unexpected mode/state and timeout fail truthfully. No VTOL landing is
part of the operation, and Runtime IPC v1 is unchanged.

Deterministic C++ falsification covers unsupported aircraft classes, every
pre-admission gate with zero transmission, outside-envelope and undwelled
states, command id/target params, missing/denied ACK, ACK-only, pre-ACK state,
intermediate-only state, session/heartbeat/disarm/mode interruptions and stable
post-ACK completion, including a measured 25.197 m transition climb and a
post-command 15 m floor. The MAVSDK peer checks COMMAND_LONG, command 3000, the
target system/component, param1=3 and denied ACK handling.

### QuadPlane VTOL landing qualification - 2026-09-25

This adds one `quadplane_vtol_land` operation for the pinned ArduPlane 4.7.1
QuadPlane profile (`dbe792162d06cab66c3475fd5556bf7a120f119e`,
`quadplane-tilttri`). It does not add generic landing, arbitrary mode setting,
RTL/QRTL, or mission execution. The selected mechanism is ArduPlane QLAND custom
mode 20 requested with `MAV_CMD_DO_SET_MODE` (176), `param1=1`, `param2=20`,
and remaining parameters zero. QLAND takes the current position as its landing
target and descends in place. The command path does not call NAV_VTOL_LAND,
execute a mission, request RTL/QRTL, or authorize horizontal navigation.

The choice follows the pinned source: `ModeQLand::_enter` initializes QLOITER,
captures the current position and enters `QPOS_LAND_DESCEND`; `ModeQLand::run`
delegates to `ModeQLoiter::run`, which advances through
`QPOS_LAND_DESCEND` → `QPOS_LAND_FINAL` → `QPOS_LAND_COMPLETE` and calls
`QuadPlane::check_land_complete`. `QuadPlane::check_land_final` and
`land_detector` control the final-stage transition and touchdown detector.
`QuadPlane::check_land_complete` disarms because the selected mode is QLAND,
not AUTO. `GCS_MAVLINK_Plane::landed_state` reports ON_GROUND when ArduPlane's
`is_flying()` returns false; that is estimator/controller evidence, not a
contact switch. `GCS_MAVLINK::handle_command_do_set_mode` handles the selected
request. The relevant pinned code is [mode_qland.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/mode_qland.cpp),
[mode_qloiter.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/mode_qloiter.cpp),
[quadplane.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/quadplane.cpp),
[GCS_MAVLink_Plane.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/GCS_MAVLink_Plane.cpp),
[commands_logic.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/commands_logic.cpp),
and [GCS_Common.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp).
The alternative mission path is implemented in `Plane::start_command` and
`Plane::verify_command`: `NAV_VTOL_LAND` can call
`do_landing_vtol_approach`/`verify_landing_vtol_approach` when the fixed-wing
spiral approach is enabled, or `QuadPlane::do_vtol_land`/
`QuadPlane::verify_vtol_land` otherwise. NOMAD does not send a mission command,
so this path cannot insert a horizontal approach, execute the mission, or
select RTL/QRTL.
The pinned mode-selection path accepts QLAND from any mode in which ArduPlane
allows that mode (with QuadPlane enabled and without a GCS-mode block); AUTO is
not a firmware-only prerequisite. NOMAD restricts admission to the already-
qualified armed AUTO multicopter state. The API accepts
an explicit landing point for admission/final-envelope checks; QLAND itself
holds the current point.

The operation checks the same nonzero session and system/component identities,
the exact QuadPlane identity and firmware version/hash, 12 pinned parameter
readbacks, connected/fresh heartbeat, armed AUTO mode, fresh multicopter VTOL
state and `EXTENDED_SYS_STATE=IN_AIR`, plus fresh valid position, velocity and
3D GPS. The entry gate requires relative-home altitude 15–25 m, within 5 m of
the specified point, groundspeed at most 1 m/s, absolute climb rate at most
0.25 m/s, and five distinct fresh samples spanning two seconds with position
spread no greater than 1.5 m and altitude spread no greater than 1 m. These
are bounds for this SITL profile qualification, not general flight limits.
Pinned settings read back are `Q_ENABLE=2`, `Q_FRAME_CLASS=7`,
`Q_TILT_ENABLE=1`, `Q_TILT_MASK=3`, `Q_TILT_TYPE=0`, `Q_TILT_RATE_UP=40`,
`Q_TILT_MAX=45`, `Q_ASSIST_SPEED=6`, `Q_OPTIONS=0`, `Q_LAND_FINAL_SPD=0.5`,
`Q_LAND_FINAL_ALT=6`, and `Q_LAND_ALTCHG=0.2`.

Admission, ACK, progression and touchdown are separate. A matching accepted ACK
only establishes the post-command boundary. The 90 s operation deadline and
3 s ACK limit are finite. Completion requires newer same-session telemetry,
QLAND mode, at least 5 m descent from the entry altitude, fresh
`EXTENDED_SYS_STATE=ON_GROUND`, disarmed state, final altitude from −1 to 1.5 m,
groundspeed at most 0.5 m/s, absolute climb at most 0.2 m/s, remaining within
the 5 m point radius, and five stable final samples spanning two seconds.
ArduPlane's landed-state indication is based on its flight-state detector, not
a physical contact switch; it is stronger than ACK or near-zero altitude alone,
but does not claim contact-sensor proof. The pinned QLAND detector requires the
motors to remain at their lower limit for five seconds and the vertical estimate
to change by no more than `Q_LAND_ALTCHG=0.2 m` during the four-second detection
window. `Q_LAND_FINAL_ALT=6 m` selects the slower final descent at
`Q_LAND_FINAL_SPD=0.5 m/s`. QLAND automatically disarms once the detector
completes outside AUTO mission continuation. NOMAD treats missing, stale,
pre-ACK or contradictory landing evidence, link/session/mode changes and timeout
as failure.

The C++ falsification suite covers wrong class/profile/version/parameter,
admission faults, unstable readiness, command wire values, ACK failures,
no-descent, descent without touchdown, stale/pre-ACK landed indications,
session/link/mode/VTOL changes and unstable final conditions. The deterministic
MAVSDK peer checks the `COMMAND_LONG` mode request and the `EXTENDED_SYS_STATE`
mapping. Hosted full-chain pinned SITL run
[36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163)
passed at implementation head `dcdd1d121d51eefa451fa5d16ab121ef8793e427`.
Its independent observer recorded 20.01 m entry altitude, 0.04 m distance to
the landing point, 2.10 s readiness dwell, QLAND observed, descent beginning
9.43 s after the command, and `ON_GROUND` 42.63 s after the command. It observed
the landed state progress from `IN_AIR` to repeated `ON_GROUND` reports. Final
state was mode 20 (QLAND), disarmed, multicopter, `ON_GROUND`, at 0.14 m altitude,
0.02 m/s groundspeed and 0.00 m/s climb. Older full-chain attempts
[36093379773](https://github.com/YoussGm3o8/NOMAD/actions/runs/36093379773) and
[36207510531](https://github.com/YoussGm3o8/NOMAD/actions/runs/36207510531)
failed closed before the QLAND request because their observer did not retain a
two-second readiness window; the latter revealed five 4 Hz samples spanned only
one second. The sliding-window fix was exercised in the successful run.

The test authority prepares AUTO as needed outside NOMAD, then invokes the
already-qualified fixed-wing-to-VTOL transition. It does not send the landing
command or simulate touchdown; the NOMAD operation initiates QLAND and an
independent pymavlink observer verifies descent and final landing state.

### Aircraft operation capability boundary - 2026-09-23

`VehicleOperation` and `supports_operation` now make command admission an
explicit policy rather than a consequence of recognizing an aircraft class.
The full before/after audit is in [Aircraft operation capabilities](architecture.md#aircraft-operation-capabilities).
The policy now admits only the deliberately narrow QuadPlane operation subset:
`arm`, semantic
`set_guided_mode`, dedicated `vtol_takeoff`, dedicated
`transition_to_fixed_wing`, `fixed_wing_route`, `fixed_wing_recovery`,
`transition_to_vtol`, and the narrowly pinned `quadplane_vtol_land`. The route operation remains exactly two validated
waypoints; recovery is one explicit point through the ArduPlane GUIDED
reposition path; transition-back is admitted only from the separately
stabilized fixed-wing AUTO envelope described above. Generic
`takeoff`, arbitrary mode setting, generic `goto_location`, generic landing, RTL/QRTL,
body velocity, payload/output and fence transport remain rejected before
transmission. Plane and Unknown remain fail-closed for every aircraft-dependent
command. Focused tests inspect policy cells, fake transport histories/counters,
route parameters and route failure states. Local telemetry waits, VIO input,
payload-interlock arming and status accessors do not transmit and remain
class-neutral.

The independent Python driver establishes setup modes without granting
`Vehicle::set_mode` capability. Pinned harnesses separately exercise the NOMAD
GUIDED arm + direct NAV_TAKEOFF climb, both explicit transitions, fixed-wing
route, recovery, the tighter transition-ready handoff and the landing operation;
the full sequence through QLAND is independently observed in hosted pinned
SITL run [36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163).
The route, transition-back and landing harnesses use a test authority to establish AUTO; for the
transition-back run, the already-qualified fixed-wing transition restores the
fixed-wing state that `Q_ENABLE=2` AUTO entry resets. That setup does not
qualify NOMAD's arbitrary mode operation. Disarm, arbitrary modes,
generic takeoff/goto/land/RTL, fixed-wing QuadPlane link-loss response and
complete Task 1 remain separate gates. The ground-router slice is
independent and does not carry flight command authority.

Packaging/install slice (2026-09-20): the Release CMake configuration installs
only the NOMAD CLI, public headers, configuration template and reviewed license
notices; MAVSDK's development install rules are kept outside the selected
runtime component because its static libraries are linked into the executable.
`pixi run package-core` produces ZIP and TGZ archives in the build tree, while
`pixi run verify-core-install` validates a clean staged prefix and
`pixi run verify-core-package` validates both archives.
The verifier checks required files, dependency notices, absence of live
`config/nomad.env`, safe archive paths and the offline CLI usage path. This is
reproducible packaging/install evidence only; versioned activation, upgrade and
rollback to a prior qualified build remain open release gates.

### G2 — One authority and aircraft semantics (core + safety leads; G1/G-M)

#### Persistent runtime IPC foundation - 2026-09-23

At baseline `3cd11aee48b9e844e75829a9ef2d65bc1ecfa1f3`, every Mission Planner
core request launched a fresh CLI process and the CLI constructed a new MAVSDK
connection/`Vehicle`; ROS independently owned another connection and `Vehicle`.
The runtime IPC slice adds `nomad-runtime`, which owns one long-lived MAVSDK
connection and one `Vehicle`, plus JSON Lines protocol v1 on IPv4 loopback. HELLO,
PING, STATUS and typed requests for existing Vehicle methods are covered by
deterministic fake-connection tests. Mission Planner can opt into persistent
mode; explicit `LegacyOneShot` remains available. `nomad --runtime` is a typed
client path, while bare verbs and `--direct` retain the standalone path.

The runtime serializes mutating operations with a try-lock; a second concurrent
mutation receives `busy`. It keeps 256 completed request responses in memory,
keyed by client/request ID, and reports unknown outcome after a post-send
disconnect. The cache clears on process restart and operations do not resume.
This establishes one NOMAD command owner only for clients connected to this
runtime. Protocol v1 does not expose navigation requests while the separate
QuadPlane route slice is in progress. Native Mission Planner controls, RC/pilot,
ArduPilot, ROS and direct maintenance clients remain independent authorities.
The API key remains a nonempty local actuation gate, not authentication. No aircraft capability,
route, transition, takeoff, safety admission or completion behavior changed;
QuadPlane fixed-wing route qualification remains a separate aircraft slice.

The runtime/IPC foundation closes only the persistent ownership and local
transport portion of G2. Explicit global authority/handover, all-client
migration, complete audit and persisted mission/restart policy remain open.

Continue G2 by unifying mission/payload/
traffic state, stop/cancel/restart semantics and audit. Correct C04-C09/C13/C15.
Implement vehicle-class capability checks before interpreting mode numbers.
Plan Task 1 around ArduPlane/QuadPlane mission execution and Task 2 around Copter;
prefer autopilot execution of reviewed survey missions, with C++ planning,
validation, supervision and outcome ownership. Mission upload/progress/abort is
new work; current synchronous demo is not a survey implementation.

Exit: two conflicting clients cannot both command; manual takeover inhibits
automatic resume; replay/restart cannot refire payload; stale position with
fresh heartbeat is refused; blocked service cannot starve safety deadlines.
Unit and SITL tests verify class mismatch rejection, Copter behavior, QuadPlane
takeoff/transition/cruise/return/VTOL landing and aborted/failed transition.
Use vehicle-appropriate holds: fixed-wing flight cannot be stopped by zero
velocity. Critical operation completion requires authoritative state.

### G3 — Profile qualification (integration + perception leads; G2)

For each profile test clean boot, core/ROS/video process absence, missing GPU/
camera, wrong endpoint, restart, link loss and capability transitions.
Task 1 primary: groundstation_gpu with aircraft FPV capture and optional Pi Zero
LTE bridge. Task 2 direction: onboard_companion with Jetson Orin Nano and a robotic arm;
integration, authority and hardware capability remain unqualified.
groundstation_minimal remains supported.

Exit: minimal C++ telemetry/eligible mission/output safety works without ROS/
camera/GPU; both compute profiles run selected workloads with measured CPU/GPU,
thermal, bandwidth, frame age and end-to-end latency under load. Raw/synchronized
VIO feeds are separately qualified if needed. Profile strings are insufficient.

### G4 — Server and traffic (integration + core leads; G2, D07/D08)

Implement the confirmed field model and an independent fixture, then the
official schema adapter after Q04. Send 1 Hz telemetry whenever armed in Task 1
and consume 1 Hz traffic; event uploads and Task 2 applicability are unresolved. Define data age, retry, queue, clock and outage
policies with approved thresholds. Advisory-first is the user's initial scope;
manual operation is permitted, but actual exclusion-zone avoidance must be
demonstrated. Automatic maneuver design is not mandated by this document.

Exit: record send and independent receipt timestamps, missed cycles and jitter;
startup/armed and clock/rate penalties are independently exercised; malformed/auth-expired/
rate-limited/offline server cannot block command handling. Inject crossing,
head-on, overtaking, diverging, stationary, duplicate-ID and stale traffic with
ground truth; measure false/missed advisories and warning time against approved
thresholds. Missing feed shows unknown. Measure 300 seconds of armed receipt, 30-second
and longer outages, >1.1-second and <0.4-second intervals, invalid GPS and startup
armed against clarified scoring semantics. Test cylinder radial/vertical boundaries
and operator response under load. Official-server acceptance and actual simulated
traffic avoidance are separate runs, not mock-only closure.

### G5 — Task 1 mission (mission + perception + flight leads; G2-G4)

Cover area planning, CV evidence, count/identity review, simulated cooperation,
traffic advisories and safe return/landing on the VTOL. Characterize video
capture access from Walksnail equipment before assuming frame availability.
GNSS/Here4 RTK is the intended navigation baseline; VIO is not a Task 1 dependency.

Exit: independent known animal/marker counts and identities, held-out imagery,
coverage and geolocation errors, duplicate/occlusion cases and review audit.
QuadPlane SITL plus authorized field rehearsal verifies complete task with
single-battery reserve, transitions and landing. Use the prescribed 3-5 km total lap course, antler/no-antler decoys, capitalized
two-character codes, 10 m clusters (Q05 oracle), anomaly descriptions and correctly
named TXT export. Test submission exactly at five minutes before and 15 minutes
after cutoff and adjacent timestamps. Record actual Drive receipt. Runtime
accuracy/reserve/latency budgets still need D08; source scoring does not set them.

### G6 — Task 2 payload mission (payload + mission + safety leads; G2-G4, D04)

Custom tracker technology (possibly ESP32), attachment and sample mechanism remain
undecided. Implement tracker identity/position ingestion, association and mapped
path with uncertainty/gaps. Explicit action authorization is initial scope;
autonomous sampling is an optional scored strategy requiring D05/Q06 and
no-intervention evidence for the complete pickup-to-pad sequence.

Exit: known ground-truth trajectory and measured error/age; wrong tracker,
packet loss, replay, reboot, RF interference and depleted tracker battery cases.
Observe tag attachment and simulated sample collection with independent feedback,
measure quantity/containment, and test jam/no-contact/relay-off failure/abort.
Measure tracker under 250 g and at most 8 cm per axis, exactly one placement,
100 m withdrawal and continued horizontal offset, five-minute path up to 5 m/s,
chronological ISO8601 CSV before cutoff and independent interpolated 5/15 m
accuracy scoring. Inspect intact egg, 1-4 cm cylindrical core at least 75% green,
and four intact spheres on the 32-inch pad by cutoff. Require observed safe
landing and field clearance except the attached tracker. Q07 governs unspecified
measurement details. If Q03 permits battery swaps, prove disarm, payload safety,
persisted records, expired permissions and explicit resume without duplicate action.

### G7 — Hardware and operational safety (hardware + flight/safety leads)

Prerequisites: relevant software and SITL gates, selected hardware and separately
authorized test procedure. Cube Orange and a custom ArduPilot controller need
separate pin/power/UART/PWM/timing verification even with the same protocol.

Exit: calibrated all-up mass below 15 kg for each configuration (the "15 kg class"
quad must still be below the limit); measured CG, power, thermal and endurance
reserve including payload, compute, radios and transition energy. Verify Here4/
RTK base correction path, fix degradation and navigation fallback without
assuming RTK availability or accuracy from the product name.

Bench evidence covers props-safe setup, sensor calibration/time, camera access,
actual output de-energization on link/power/process faults, independent manual
control and common-mode link failures. A disconnected link cannot deliver stop;
verify the independent all-mode termination mechanism, including loss of C2,
within-five-second activation entry, fixed-wing motor/surface state, rotary
minimum-2-m/s descent through touchdown and QuadPlane transition cases. Settle
Q02 before aircraft testing; fence/altitude/termination readback alone does
not prove behavior. Complete every AE27-OPS FRR, licence, propeller-inhibit,
weather and proof-flight requirement, with judge acceptance before competition.
Authorized field tests cover aircraft-specific abort, transition failure, landing, traffic procedures and safe payload.
Re-read safe configuration and disarmed state after each session; record failures.

### G8 — Competition release (release + safety + competition leads; all gates)

Exit: MAVSDK is the tested production transport; both task rehearsals run on
versioned artifacts/configurations; required profiles and firmware combinations
pass their suites; official server contract is tested; no unresolved safety-
critical defect or acceptance threshold. Optional unavailable features are
explicit and do not conceal a failed required capability.

Package core, plugin, selected adapters, dependency notices, configuration
templates and operator procedures reproducibly. Test install/upgrade/rollback
and startup on clean qualified hosts; rollback to a prior qualified build never
auto-resumes motion. Exercise expired credentials, replay, disk-full, process
crash and server/video overload with complete audit. Review proposal/demo/report
evidence, assign operator roles and obtain separate operational authorization.

## Deletion and cutover rules

Before any further removal: inventory callers, assign replacement or explicit
feature unavailability, retain requirement/fault tests, record SITL/hardware
evidence appropriate to the behavior, pass each required profile path, and
remap safety traceability. Do not delete optional compute because its former
control UI was REST-based. Retire aliases/settings only after verified migration.

The two codecs coexisted temporarily for comparison; only one ever owned commands
in a test or deployment, and golden wire references were kept until equivalent
semantic and wire evidence survived the switch. The codec deletion itself ran on
2026-09-12 with explicit user authorization, after that checklist was met:
caller inventory (CLI, ROS 2 adapter, SMP tests) moved to
`make_mavsdk_connection`, the replaced transport tests were ported to the MAVSDK
contract first, the golden wire assertions became the peer fixture's decode
checks, and the SR rows were remapped in the same change. These rules stay in
force for the next removal; they are not retrospective permission for anything
else.

## Checks run for this documentation review

### Current-head hosted Copter SITL qualification - 2026-09-20

Requirement: G-M current-head SITL evidence plus the SR-LNK-03, SR-LNK-04 and
GAP-05 evidence paths. Falsification: any required workflow step fails, the run
uses a different commit or pin, or a required scenario is skipped.

Workflow-dispatch run
[35489428247](https://github.com/YoussGm3o8/NOMAD/actions/runs/35489428247)
ran the full `sitl` job on merged `main` commit
`26d7f9b101a029725d06aee2c6716da95e622417` with the pinned Copter 4.7.1 image.
That commit resolves the `YoussGm3o8/MAVSDK:nomad/ardupilot` gitlink at
`3f85f6f808b617c736316d7da5f51f3d3eba1737`.
The job passed telemetry smoke, MAVSDK Phase A connect/status, command flow,
mission execution, velocity watchdog, payload relay, link loss, zero delivery,
link recovery, GCS-heartbeat relay gating, velocity loop closure, geofence
containment and geofence upload/readback, followed by teardown. The automatic
post-merge push run `35489179313` was reduced to Phase A smoke by workflow design
and is not used as the full-matrix evidence.

This closes the current-head Copter SITL evidence for the pin. It does not close
Phase A resource-budget approval, ROS qualification, Plane/QuadPlane support,
packaging/install/rollback, hardware or flight-release gates.

### MAVSDK Phase A dependency hardening - 2026-09-10

Requirement: project MAVSDK decision / GAP-16. Falsification: change the
PicoSHA2 commit, either archive SHA-256, extraction timestamp option or a bundled
licence text and require the provenance tests to fail; rebuild and run the peer
fixture to reject wrong/absent systems.

The project MAVSDK fork is published and NOMAD pins
`9884f109533f564bc6250e5471e6301d3a62f4a7`. The selected graph pins PicoSHA2 to
`1bf940d8a03bb752604fbb366d47b97b50b9e6ce`, verifies nlohmann JSON and XZ
archives with SHA-256, handles archive extraction on both legacy and newer CMake,
and uses the pinned nested pymavlink generator source rather than build-time
network package resolution. `licenses/mavsdk-phase-a/` carries the selected-build
licence texts; the provenance checker verifies reviewed revisions, hashes, patch
invariants, NOTICE coverage and licence content.

Recursive hosted test run `34535620056` passed the Python suite, C++ core,
optional MAVSDK build, provenance checker and deterministic expected/wrong/absent
peer qualification on both Ubuntu and Windows. Selected ROS-image run
`34538394497` built with NOMAD_ENABLE_MAVSDK=ON and passed the ROS adapter tests.
Mainline SITL run `34538903820` built ArduPilot Copter 4.7.1, brought up the
development/SITL stack and passed the live MAVSDK Phase A connect/status smoke.
The larger SITL loop-closure job is intentionally nightly/on-demand; the main
push ran only the reduced Phase A live regression gate.

Historical Windows measurements remain diagnostic: the warm build tree measured
379,308,757 bytes and the smoke executable 2,032,128 bytes. They are not approved
budgets. Repeatable current build-tree, executable, runtime memory, startup and
CI-time measurements plus explicit budget approval remain the sole Phase A gate
items. This historical entry predates the Phase E cutover; production now uses
MAVSDK exclusively, and command/telemetry parity plus cutover are recorded in the
later G-M entries.

### MAVSDK ArduPilot ownership - 2026-09-12

Requirement: project MAVSDK decision / GAP-16 and G-M. Falsification: introduce a
new low-level ArduPilot wire or command-encoding path inside NOMAD that the pinned
fork should own, silently derive an admitted capability from missing or ambiguous
reported state, or change a fork patch without a provenance and requalification
update.

Decision (user-confirmed): the pinned fork owns ArduPilot transport/framing,
command encoding and low-level protocol integrations; NOMAD keeps safety policy,
validation, safety-relevant interpretation of reported state for admission,
authoritative verification, deadlines, audit and the client contract. This
clarifies the earlier shorthand that assigned all command/mode/telemetry
"semantics" to the fork: NOMAD may classify aircraft or capabilities from
reported heartbeat/parameter state when that interpretation is required to fail
closed, while new raw wire behavior remains a fork responsibility. The
`Q_ENABLE`-based fixed-wing/QuadPlane classification is one such admission rule.

The fork scope recorded in [MAVSDK adoption](mavsdk-adoption.md) still covers
reusable ArduPilot protocol behavior and dependency patches. Raw-frame paths in
`MavsdkMavlinkConnection` remain transitional where equivalent fork APIs do not
yet exist.

Sequencing effect: the ArduPilot completeness work in fork Phase F is prerequisite
work for Phases B-D, not post-cutover maintenance. Unknown work remains large —
mode setting, relative-altitude location commands, the output verbs and the
telemetry parity fields are all unstarted in the fork, and no fork change has been
made beyond dependency pinning.

Evidence effect: fork code becomes safety-relevant code outside this repository.
Each patch needs its own test, an independent wire or SITL observation against the
selected firmware, provenance pinning and an upstream change when publication is
authorized. Existing NOMAD verification evidence is unaffected because outcome
authority does not move.

### MAVSDK Phase B transport parity - 2026-09-11

Requirement: project MAVSDK decision / GAP-16 (command parity). Falsification:
run `pixi run test-mavsdk-phase-b`; a denied ACK, a missing ACK or a wrong
autopilot system ID must make the CLI or probe report failure (never success),
and `goto` must leave the host as COMMAND_INT with the GLOBAL_RELATIVE_ALT_INT
frame.

As recorded on 2026-09-11, the then-opt-in MAVSDK transport implemented
`MavlinkConnection` against the reviewed pin
`9884f109533f564bc6250e5471e6301d3a62f4a7` (the submodule was reconciled from a
dirty newer checkout, and provenance passes again). Commands are issued as raw
COMMAND_LONG/COMMAND_INT so NOMAD keeps its MAVLink result-code contract and
relative-altitude command frame. The CLI gained `--transport udp|mavsdk`
(default udp) and `--system-id`; at that point the legacy codec was still the
default, and the Phase E cutover below has since removed it.

Local evidence (2026-09-11, MSVC Release, Copter-oriented peer fixture):
`nomad_mavsdk_connection_tests` passes its no-peer configuration and
fail-closed cases; `scripts/dev/mavsdk_connection_fixture.py` passes status
telemetry, accepted, denied, timeout, COMMAND_INT frame 6, COMMAND_LONG and
wrong-identity cases, and per-command parity for mode, takeoff, goto, land, RTL,
servo, relay, gimbal-config and user-command. The command cases drive the real
CLI against a peer whose initial state is observably different from each
command's required result, so a pass means the core's state verification
observed the change rather than trusting the acknowledgement. motor-test parity
was added once C23 was fixed, and asserts the corrected id on the wire.
mission parity remains open, so this is transport parity evidence and not a
closed Phase E gate; the live SITL runs below complete Phase B's stated exit for
Copter without closing G-M.

Link parity was added on 2026-09-12 by porting the legacy transport's link-level
cases onto the same peer fixture:

- `case_gcs_heartbeat_announces_to_a_silent_peer` points NOMAD at a bound,
silent peer with `udpout:` and requires the announcements to arrive while no
vehicle is latched: four GCS heartbeats (`MAV_TYPE_GCS`,
`MAV_AUTOPILOT_INVALID`, component 190, system 245) in a 6 s discovery window
that discovers nothing. This is the pre-latch announcement a heartbeat-gated
relay needs, and the behavior `safety/gcs-heartbeat-cadence` protects. A
`udpin:` link has no pre-latch destination at all, because MAVSDK only learns
one from traffic it receives, so a relay setup must name the relay with
`udpout:` — the successor to the legacy `NOMAD_RELAY_ADDRESS` override, which
the MAVSDK transport does not implement separately.
- `case_coalesced_telemetry_is_verified` drives the CLI against a peer that
joins every telemetry frame and the command acknowledgement into one datagram,
the shape MAVProxy and router links produce, and still requires the arm to be
verified from reported state.
- `case_link_loss_is_observed_as_stale` streams telemetry, stops it while the
socket stays open, and requires `heartbeat_fresh` and `connected` to become
false through the probe's `--staleness` mode. That is the link-observation half
of SR-LNK-01.

Phase C (velocity) landed in the same pass. `send_velocity` queues
SET_POSITION_TARGET_LOCAL_NED through MAVSDK's passthrough using
`queue_message()` (so MAVSDK owns sequence numbering) with the legacy codec's
body-frame mask 0x07C7 and frame 9, refuses a non-finite rate, requires a live
latched peer for a non-zero setpoint, and zeros the vehicle before tearing the
link down. `case_velocity_setpoint_reaches_the_wire` proves all of that on the
wire rather than from the transport's own report: the peer decodes the frames it
received and the case requires the requested rates with that mask and frame and
the right target system, requires that no non-finite frame ever left, and
requires the all-zero setpoint after the stream when the transport disconnects.
Both guards were confirmed load-bearing by removing them and watching the case
fail, and the non-finite probe runs on a live link so a check that only held on
a dead link could not pass for an enforced one.

The core-level half of SR-LNK-03 followed in the same pass.
`tests/mavsdk_zero_delivery_test.cpp` is the MAVSDK successor to the legacy
`tests/zero_delivery_test.cpp`: it builds a real `Vehicle` on the MAVSDK
transport, feeds VIO, arms and selects GUIDED, then streams the setpoint. The
fixture drives five scenarios against the same peer — the watchdog stopping a
dead command stream, `Vehicle::stop_velocity()`, vehicle destruction
mid-stream, heartbeat loss, and VIO-feed loss — and each must report its own
watchdog reason while the peer observes the all-zero setpoint as the newest
setpoint on the wire. The binary also covers the no-link failure path under
CTest without a peer, so a declined command cannot masquerade as a stopped
stream. `zero_delivery_test.cpp` was therefore ported rather than rewritten under
pressure at cutover, and it and the other legacy transport tests were deleted with
the codec in the Phase E change below; the SITL scenario
`core-sitl-velocity-watchdog` covers the same behavior end to end.

Phase D (fence/params) landed on 2026-09-12 and closes the last functional gap
before the cutover decision. MAVSDK's `Geofence` and `Param` plugins implement
the transport's fence and parameter surface, and the two plugins are enabled in
the CMake MAVSDK options (`ENABLED_PLUGINS telemetry geofence param`) — without
that the plugin headers compile but the library link fails. Single-vertex fence
transfer (`send_fence_point`, `request_fence_point`, `wait_for_fence_point`)
stays fail-closed, because MAVSDK moves whole polygons and the core only ever
uploads a plan and then reads the autopilot's own copy back, so a partial
transfer would have no caller to serve.

`case_fence_upload_and_readback` and `case_disabled_fence_does_not_verify` are
the SR-FEN-01 evidence on this transport. They drive a real `Vehicle` through
`upload_fence` and `verify_fence_uploaded` with the probe's `--fence` mode
against a peer that stores the fence mission items it is sent, so the polygon
the peer decoded is the proof the upload happened and the readback cannot come
from the transport's own bookkeeping. The enabled case requires the verified
result plus the refusal paths — a boundary below the three-vertex minimum, a
non-finite vertex, and an expected boundary the autopilot does not hold. The
disabled case requires the upload to succeed while verification fails, because a
plan on the autopilot is not an enforced fence: `verify_fence_uploaded` reads
`FENCE_ENABLE` back and fails closed when it is not 1. Both guards were confirmed
load-bearing by breaking them and watching the case fail — a fabricated
`FENCE_ENABLE` made the disabled case verify wrongly, and a no-op upload left
the peer holding no polygon.

One fabricated success was found and fixed while reviewing the cutover surface.
Historically, `MavsdkMavlinkConnection::request_data_stream` returned
`is_connected()` and reported success for a `REQUEST_DATA_STREAM` it never put
on the wire. A deterministic peer test first proved the repaired frame delivery.
The unused operation and its protocol fixture were then removed on 2026-09-13:
telemetry initialization is a MAVSDK responsibility and NOMAD has no production
reason to expose the legacy MAVLink request. The core reads the telemetry
MAVSDK's subscriptions deliver, while the current SITL stack supplies rates
through `docker/sitl-streams.parm` (C24) until the fork proves clean-session
initialization and fallback behavior independently.

`MavsdkMavlinkConnection::read_param` and `send_command` now pass each caller's
positive overall budget through MAVSDK's `OperationOptions`. MAVSDK owns the
queue wait, retry schedule, and remaining per-attempt timeout; NOMAD no longer
mutates global SDK timeout state or depends on internal retry counts. Parameter
reads carry the unused portion of one NOMAD deadline from the integer probe to
the REAL32 probe. Non-positive budgets fail closed before any request is sent,
and independent operations may proceed in parallel while a shared lifetime lock
prevents plugin teardown during a call. Focused peer cases exercise 100 ms
command and parameter budgets against silent responses. Parameter reads accept
both ArduPilot integer parameters (such as `FENCE_ENABLE`) and REAL32 values,
returning the numeric value only after a matching response. No parameter write
path exists because the core only reads.
Phase E (production cutover) was still open when this historical evidence was
written; it landed the same day and is recorded below.

One candidate fix was measured and withdrawn rather than shipped: setting
`Mavsdk::Configuration::set_always_send_heartbeats(true)`, which MAVSDK
documents as sending heartbeats without a connected system, changed nothing (four
heartbeats either way), so the transport keeps MAVSDK's `GroundStation`
default and the fixture pins the observable behavior instead.

SITL exit criterion met later the same day, after fixing the feed rather than
the transport. Two captures on passive vehicle links inside the container (no
host relay involved) delivered ~1 Hz heartbeats and no position/attitude stream
until a GCS sent `REQUEST_DATA_STREAM`, after which ATTITUDE/
GLOBAL_POSITION_INT/GPS_RAW_INT/SYS_STATUS arrived at roughly 5 Hz. ArduPilot
behaves that way by design and the core deliberately never requests streams, so
with the Python edge service deleted nothing requested them for the host-side
copy the scenarios read; see C24. `docker/sitl-streams.parm` now sets
`SR0_POSITION`, `SR0_EXT_STAT`, `SR0_EXTRA1` and `SR0_EXTRA2` at SITL startup,
and a 20 s capture of the host copy then delivered HEARTBEAT at 1.00 Hz with
GLOBAL_POSITION_INT, GPS_RAW_INT, ATTITUDE and SYS_STATUS each at ~4 Hz.

With that feed: the legacy baseline passes `pixi run core-sitl-status`,
`core-sitl-command-flow` and `core-sitl-payload`, and the MAVSDK transport
passes the command flow (GUIDED mode, 3D GPS fix, arm, takeoff to 5 m, guided
goto sent as COMMAND_INT, RTL, land, disarm, every step verified against
reported state) and the payload acceptance run with `NOMAD_TRANSPORT=mavsdk`.
One earlier observation is withdrawn: this document first recorded the
container-to-host UDP relay as losing roughly three quarters of datagrams. That
reading came from the pre-fix stack, whose container had been running six days
with a stale environment and was still initialising when captured; the
recreated stack delivers heartbeats at exactly 1.00 Hz, so no relay loss is
claimed and the cause of that earlier rate is left unexplained rather than
guessed. Remaining Phase B gaps are unchanged: vehicle-class identification and
QuadPlane coverage. Phases C and D were open when this was written and have since
landed, followed by the Phase E cutover below. C23 (the undefined motor-test
command id) is fixed and carries
live evidence: on 2026-09-12 against Copter 4.7.1, `nomad motor-test 1 0 0.1`
sent `COMMAND_LONG cmd=139` and ArduPilot answered `result=3
(MAV_RESULT_UNSUPPORTED)`, and after the id was corrected the same run sent
`cmd=209` and received `result=0 (MAV_RESULT_ACCEPTED)` with the vehicle's own
"starting motor test" / "finished motor test" status texts, which also confirms
the parameter layout (`param2` is the throttle type). That acceptance is a
command acknowledgement and a status text rather than an independent state
measurement, so the verb stays out of the state-verified evidence set.

Full legacy sweep on that feed (2026-09-12, Copter 4.7.1): `core-sitl-status`,
`core-sitl-command-flow`, `core-sitl-mission`, `core-sitl-velocity-watchdog`,
`core-sitl-geofence`, `core-sitl-payload`, `core-sitl-link-loss`,
`core-sitl-link-recovery`, `core-sitl-zero-delivery`, `sitl-scenario` and
`sitl-fence` pass. Two harness defects surfaced while running them, neither in
the core. `sitl-fence` could never pass: it was the only scenario task without
the development API key, and every step is an actuation command the CLI refuses
without one (`audit command=... result=refused auth=none
reason=missing_api_key`); the task now sets it like its siblings. Separately,
the geofence scenario leaves its uploaded polygon fence active and ArduPilot
then refuses guided targets outside that polygon (observed: the same reposition
accepted with `FENCE_ENABLE=0`, rejected with the polygon loaded), so
scenario ordering now matters and is recorded in `tests/sitl/README.md`.

`core-sitl-gcs-heartbeat` passed once the transport kept announcing for the whole
wait: its negative control requires at least two captured announcements, but the
UDP transport announced once and then blocked in a receive until the deadline, so
a closed gate yielded exactly one. The receive is now capped at a short slice
(measured 2026-09-12: the closed gate captured three announcements at ~1.0 Hz
before status failed closed, and the open gate relayed the stream). `SR-LNK-04`
still asked for MAVSDK requalification; the current-head hosted requalification
is recorded above.

### MAVSDK Phase E cutover - 2026-09-12

Requirement: project MAVSDK decision / GAP-16 and G-M, with explicit user
authorization to delete the replaced codec. Falsification: configure the tree
without the MAVSDK submodule and require failure rather than a transport-less
binary; name the removed transport on the CLI and require usage instead of a
silent selection; run the peer fixture and require every command, link, velocity,
fence and parameter case to keep passing on the transport that remains.

What changed. `src/mavlink/{protocol,udp_connection,udp_commands,fence,params}.cpp`,
their headers, the build-time generated dialect headers and the legacy transport
tests (`codec_golden_test.cpp`, `udp_connection_test.cpp`,
`zero_delivery_test.cpp`) are deleted. `CMakeLists.txt` has no MAVSDK on/off
option any more: a missing `third_party/MAVSDK` checkout is a `FATAL_ERROR`,
which removes the last way to build a NOMAD binary with no transport. The ROS 2
adapter built `UdpMavlinkConnection` directly and now builds
`make_mavsdk_connection` with a declared `system_id` parameter, so no adapter
depends on a transport-free core. `--transport` and `NOMAD_TRANSPORT` are removed
with the selector they fed, so naming a transport (including the removed codec)
is an unknown-argument usage failure rather than a silent fallback. The dead
build-time header generator (`scripts/dev/generate_mavlink.py`
and its Pixi task) is deleted with the CMake rule that invoked it, and the three
ROS images drop it from their build context and ship the MAVSDK submodule
instead (`Dockerfile.sim-ros`, `Dockerfile.jetson`, `Dockerfile.sim-isaac`; the
latter two also clean the MAVSDK patch line endings the way the sim-ros image
already did).

Keeping the client diagnostics. MAVSDK's `connect()` folds opening the endpoint
and discovering the peer into one call, which collapsed the two client-facing
failure messages into one. `MavlinkConnection::connect()` now reports which half
failed through `ConnectFailure`, so an endpoint that never opened still reports
`could not connect to <endpoint>` while an open link that no expected autopilot
answered on reports `timed out waiting for ArduPilot heartbeat` - the message the
`core-sitl-link-recovery` and `core-sitl-gcs-heartbeat` scenarios and the client
contract tests key on. `tests/mavsdk_connection_test.cpp`'s
`test_unusable_endpoint_is_reported_as_a_link_failure` and
`test_absent_peer_fails_closed` pin both reasons at the transport, and
`tests/test_core_client_contract.py`'s
`test_occupied_endpoint_reports_connect_failure` requires the heartbeat
diagnostic to be absent when the endpoint never opened, so neither message can
be replaced by the other again. The ROS 2 adapter's throttled warning names the
same two cases (`ros2/nomad_ros/src/node.cpp:report_connect_failure`).

Evidence on this tree (Windows, MSVC Release, 2026-09-12): `pixi run
test-mavsdk-phase-b` configures and builds clean, and `ctest` in that tree reports
9/9 including `nomad_mavsdk_connection_tests` and
`nomad_mavsdk_zero_delivery_tests`; `scripts/dev/mavsdk_connection_fixture.py`
passes 73/73 peer checks against the deterministic vehicle, so command, link,
zero-delivery, fence and parameter behavior all survive the deletion; the ROS 2
adapter compiles against the transport in the sim-ros image's colcon workspace.
The no-transport falsification was executed: configuring with
`-DNOMAD_MAVSDK_SOURCE_DIR` pointing at an empty directory stops at
`CMakeLists.txt:37` with the submodule instruction and exit 1, so no
MAVSDK-less binary can be configured. Collapsing the two connect diagnostics
back into one reproduces the client contract failures, so the preserved messages
are checked rather than incidental.
The current-head live SITL matrix over the MAVSDK transport is recorded in the
hosted run above. The ROS adapter integration suite on a rebuilt image, hardware
and QuadPlane behavior remain outstanding G-M evidence. The default runtime is
MAVSDK on every path now; this is not a fallback comparison.

debt: remaining generic commands and heartbeat observation use MAVSDK passthrough
APIs marked deprecated in the pin (`send_command_long`, `subscribe_message`);
revisit when the fork provides equivalent command and observation APIs; then use
the peer fixture as the wire check. Relative goto uses
`Action::goto_location_relative`; velocity uses `Offboard::set_velocity_body_once`
without automatic resends. NOMAD retains freshness, authorization, watchdogs and
authoritative verification. The adoption record names the independent fault tests.

### Boundary clarification - 2026-09-10

The project owner resolved Q01: hard-boundary violation means termination;
soft boundary is an internal configurable inward distance from the same hard
polygon, e.g. 5 m. The existing plugin implementation/default inset is retained
unchanged, including its configuration controls. No second official soft polygon
or organizer ruling is required for this design. Appendix C's inconsistent labels
remain a source note. Hard-containment/termination tests and the separate 100 m
AGL/default-display discrepancy remain open; no runtime behavior changed.

### CONOPS reconciliation - 2026-09-10

Read all 36 pages before editing; inventoried 125 statements/conditions with
source pages, owner, safety relevance, provenance status and evidence. Visually
checked the contradictory Appendix C table and task scoring/sample tables.
Mapped implementation into 16 open gaps and retained stable SR and preview IDs.
The new structural test does not prove semantic completeness or flight compliance.

Checks recorded when the reconciliation itself completed, before the later
Phase A source hardening above:

- `pixi run test-core`: 10/10 CTests passed, Windows Debug; compiler emitted
  existing C4530 exception-unwind warnings. No C++ source/build options changed.
- `pixi run test`: 328 passed, 3 skipped; retained Python coverage 96.34%.
  This runs the full Python suite, including the four new traceability cases.
- Focused CONOPS and safety traceability suite: 74 passed.
- `pixi run lint`, `format-check`, `typecheck`, `complexity-check`, `docs-build`
  and `check-mavsdk-phase-a`: passed. Type check covers its configured seven
  source files; it is not a C++ or full adapter proof.
- `pixi run lint-plugin`: non-deploying Release compile/dead-code check passed.
- All changed-file pre-commit hooks passed. Read-only whole-tree SPDX hook
  still fails only on the pre-existing scripts/hardware/servo_test.c header;
  that file is unchanged. A whole-tree all-hook pass is not claimed.
- Local Markdown link check: 67 links in changed documents resolved before
  this evidence note; every changed file below 100 KB. Final diff review includes
  both new files; no runtime/configuration mutation, secret, real deployment
  address, generated artifact or unrelated source change is included.

Changed document set: PLAN.md, TODO.md, README.md, CONTRIBUTING.md; docs/prd.md,
docs/conops-requirements.md (new subordinate appendix), docs/architecture.md,
docs/migration.md, docs/operations.md, docs/safety.md, docs/development.md,
docs/index.md, docs/mavsdk-adoption.md, docs/mavsdk-dependencies.md;
config/profiles/README.md, mission_planner/README.md and tests/sitl/README.md.
Supporting changes: properdocs.yml navigation and tests/test_conops_traceability.py.

The CONOPS reconciliation was prepared from clean main at `fab9f46`; those local
checks predate the later published implementation commits. Main now contains the
reconciliation plus subsequent MAVSDK Phase A hardening and qualification. Do
not reinterpret the earlier local pass counts as current hosted or live-SITL
evidence; the current Phase A evidence is recorded separately above.

No hardware or official-server acceptance was performed by the reconciliation.
Q04 remains open. MAVSDK production adoption is implemented, but release
qualification remains incomplete; no release gate closes solely because
documentation, hosted Phase A smoke, or local regressions pass.

### Earlier planning review - 2026-09-08

- 2026-09-08: pixi run test-core — 9/9 CTest targets passed (Windows Debug).
- 2026-09-08: pixi run test-python — 251 passed, 3 skipped.
- 2026-09-08: strict docs build, Ruff lint/format-check, complexity-check and
  non-deploying plugin lint/compile passed. Size checks retain existing baseline
  debt; a passing policy check is not a claim that every file meets target size.
- 2026-09-08: safety traceability passed after documentation changes; plugin
  core-client, payload-interlock and dual-link tests passed.
- 2026-09-08: all 72 checked local Markdown file links resolved; git diff
  whitespace check passed. Existing staged content and non-document files were
  compared against the pre-edit snapshot to verify preservation.
- No SITL startup, hardware actuation, deployment, hosted CI or MAVSDK rebuild
  was performed in this review. Earlier pass counts are historical only.

## Ground router extraction

The transport slice based on PR #21's merged `main`
(`ca9f4ee89418e481f37d2faa846c7beec0a869cb`) replaces the hard-coded two-link router
with owned per-link UDP/TCP/COM state and arbitrary stable IDs. Legacy settings
translate to two configured entries. The shared MP-independent implementation,
standalone host, collection-based plugin status and two-consumer socket topology
are described in the [router README](https://github.com/YoussGm3o8/NOMAD/blob/main/infra/transport/ground_router/README.md).

`pixi run test-ground-router` builds the standalone library/host and runs legacy
router regressions, three-link handover dedup, single-recipient outbound commands,
parameter pinning, invalid configuration/override checks, per-source parser and
sequence isolation, TCP reconnect, consumer isolation, the versioned loopback
status/control protocol and standalone-client stale/reconnect tests, plus a
standalone loopback process smoke with failover and immediate port reuse.

The plugin remains an optional embedded host, or a non-owning standalone management
client selected explicitly by `RouterMode`. The local protocol is limited to
status, events, and safe link selection; it does not implement persistent C++ IPC,
remove one-shot CLI clients, arbitrate global command authority, qualify an
aircraft operation or establish independent physical redundancy. Mission/fence/FTP
transaction pinning is not implemented. QuadPlane forward transition and the
narrow two-point fixed-wing route, explicit fixed-wing recovery point and
fixed-wing-to-VTOL transition are qualified for their stated pinned profile.
The narrow QLAND operation is qualified for the pinned SITL profile; generic
landing and RTL/QRTL, fixed-wing link-loss/manual takeover, integrated complete
Task 1 qualification, command-authority/runtime hardening, competition-server
integration and hardware qualification remain open.

## MAVSDK compatibility evidence — 2026-09-19

Historical record consolidated from the 2026-09-19 closeout and the
2026-09-20 post-PR #13 snapshot (NOMAD main
`26d7f9b101a029725d06aee2c6716da95e622417`). Results and upstream PR states
below are observations at those dates, not current release approval. Current
scope and remaining gates are owned by the qualification status matrix above.

## Rebased fork

`YoussGm3o8/MAVSDK`, branch `nomad/ardupilot`, publishes qualified revision
`3f85f6f808b617c736316d7da5f51f3d3eba1737`, rebased onto upstream main
`d7043d3cafe8cd6250565fd211b966d8b455d561`. The previous consumer pin was
`e0dada26a606ffa4f48e72841efa231177733d05`, retained on the published
`ardupilot-before-rebase-20260919` branch for existing checkouts and rollback.
The fork's independent build,
pinned ArduCopter 4.7.1 inputs and exact executable compatibility matrix live in
`docs/en/cpp/guide/ardupilot_compatibility.md` in that repository.

## Completed implementation

- Debian 11's live security indexes referenced unavailable OpenSSL and ICU
  packages even after refresh. The retained packaging job now uses signed dated
  Debian snapshots and requires index refresh to succeed. This is a reproducible
  build environment, not a claim of continuing Debian 11 security support.
- iOS applied an overlapping MAVLink patch after the common patch. Every platform
  now applies the pinned-generator patch once. Device, simulator and XCFramework
  jobs remain enabled. NOMAD's ROS, Jetson and Isaac Dockerfiles normalize only
  the retained patch; local MAVSDK build caches are excluded from image inputs.
- Windows combined ZIP uploads used repository-root paths even though ZIPs were
  created under `cpp/`. Upload and release paths now name the actual directory,
  and a missing combined archive fails the workflow instead of only warning.
- `ArduPilotFenceFaults.ReportsVehicleRejectedUploadAndRecovers` independently
  observes `MAV_MISSION_NO_SPACE` and the SDK rejection result before recovery.
  `UploadTimesOutDuringResponseLossAndRecovers` and
  `DownloadTimesOutDuringResponseLossAndRecovers` drop actual vehicle replies,
  count the loss, bound completion, restore the link path and verify recovery.
  `RoundTripsExclusionPolygon` extends the existing inclusion-fence evidence.
- `Offboard::set_velocity_body_once` queues one body-velocity frame without
  storing a setpoint, changing mode or starting automatic resends. Five
  `OffboardOneShot` wire tests cover stalled producers, invalid inputs, zero and
  destruction, removed connections and refusal to mix automatic resend modes.
  `OneShotVelocityExpiresWhenProducerStalls` verifies movement followed by expiry
  under the vehicle's unchanged `GUID_TIMEOUT`.
- NOMAD now calls that API and converts yaw rate from radians to degrees. MAVSDK
  owns the frame, mask and packing. NOMAD retains authorization, freshness,
  watchdogs, velocity limits, cancellation, final zeros and authoritative checks.
  Independent consumer wire tests and live stop probes qualify the transition.
  Both hosted Linux/Windows consumer jobs now run the full Phase B fixture.
- Four `OperationDeadlineWire` tests cover lost command/parameter attempts
  followed by success, repeated progress acknowledgements and concurrent
  independent deadlines. Upward rounding to timer precision prevents spurious
  retries just before expiry; `OperationTimeout.FinalAttemptNeverRoundsBeforeTheDeadline`
  covers the boundary. Existing overloads retain their SDK-wide timeout behavior.
- Rebased SDK system IDs are checked as 32-bit values before narrowing. NOMAD's
  identity regression rejects 257 when the expected MAVLink ID is 1.
- Three isolated upstream contributions are open:
  [pinned generator and shared iOS patch](https://github.com/mavlink/MAVSDK/pull/3102),
  [geofence input validation](https://github.com/mavlink/MAVSDK/pull/3103),
  and [Windows archive paths](https://github.com/mavlink/MAVSDK/pull/3104).
  The first two focused regressions pass and fail against the original implementations.
  All three Windows archives are independently verified as published and nonempty;
  the downloaded x64 ZIP contains both configurations, headers and CMake metadata.
  Submission is not upstream acceptance; retain patches until merged and requalified.

## Exact-revision evidence

| Check | Result and evidence |
|---|---|
| Standalone ArduCopter compatibility | Passed, all 33 tests: [run 35481870540](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870540) |
| Fork Linux matrix | Passed, including Debian 11 build, unit/system tests and installation: [run 35481870573](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870573) |
| Fork Apple matrix | Passed: all four macOS builds, three iOS builds and XCFramework: [run 35481870483](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870483) |
| Fork Windows matrix | Passed, all 14 jobs and three combined archives: [run 35481870500](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870500) |
| Local fork suites (unchanged SDK source) | Passed: 415 unit tests, 114 system tests, 33 fresh pinned-Copter compatibility tests |
| Linux deadline stability | Passed: 20 repetitions of all four wire tests, 80 checks |
| NOMAD core and wire fixtures | Passed: 9 CTests; Phase A valid/wrong/absent peer; Phase B command, deadline, link, fence and all five zero-delivery cases |
| NOMAD live stop behavior | Passed: `core_sitl_zero_delivery.py` observed the final zero, hover and disarm; `core_sitl_velocity_watchdog.py` observed `command_timeout` and disarm |
| NOMAD ROS adapter | Passed: fresh `Dockerfile.sim-ros` image build and all 9 ROS integration tests |
| Consumer pin/provenance and Phase A | Passed: reviewed gitlinks, dependency/license checks, 16 tests and valid/wrong/absent peer fixture |
| Python and quality checks | Passed: 514 Python tests, 3 environment skips; lint, format check and strict documentation build |

The complete fork platform workflows passed before publication of this consumer
update. The gitlink and provenance records adopt that qualified revision together.
The dedicated Copter result qualifies the named surface, not every plugin,
platform or vehicle.
Linux completed 30 jobs successfully; its conditional `Update Docker Images`
job was skipped. Its style and API regeneration check passed. The Apple
XCFramework/framework artifacts and all three combined Windows ZIPs
are published and nonempty. Tag-only Debian package creation and release-upload
steps were not executed; no tagged release is qualified by these branch runs.

The preceding revision `f1381975b403f497ca855187d9e8c9d19a22fb99` passed
Linux, Apple, all 33 hosted compatibility tests and Windows on retry. Its Windows
upload warnings exposed the archive-path fix above; SDK source and dependency
pins are identical between that revision and the final pin.

`SystemTest.RequestMessageRapid` and its request implementation are unchanged
from upstream. The failing Windows test passed 20 consecutive local repetitions
and another 100 with command diagnostics enabled. Its hosted rerun passed;
the final pin's complete Windows matrix passed on its first attempt.
This does not establish that
the intermittent timeout's cause has been fixed.

debt: one hosted rapid-request timeout has not reproduced in 120 local runs;
the fork maintainer revisits on another hosted occurrence, then captures the
request/ACK lifecycle and adds a deterministic regression before changing it.

## Upstream MAVSDK pull requests

Three isolated contributions were open against `mavlink/MAVSDK` on 2026-09-20:

1. `#3102` — **build: use the pinned MAVLink generator on desktop and iOS**. It removes build-time pip upgrades, uses the pinned pymavlink submodule and replaces the overlapping desktop/iOS patch sequence with one reproducible patch. The change is focused and has a no-pip generator regression. Fork workflow statuses required repository-side approval/action at the time, so they were not evidence of a source regression.
2. `#3103` — **geofence: validate geometry before starting mission transfer**. It rejects short polygons, invalid coordinates, invalid circles and wire-count overflow before emitting `MISSION_COUNT`, with hardware-free system tests. The 2026-09-20 review found no blocker.
3. `#3104` — **ci: upload Windows archives from their actual directory**. It fixes the demonstrated `cpp/` path mismatch for artifacts/releases and makes missing combined archives fatal. The downstream full Windows run proves the corrected paths produce non-empty archives. Tag-triggered release publication itself has not been exercised.
