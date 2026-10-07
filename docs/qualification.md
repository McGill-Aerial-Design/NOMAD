# Safety and qualification status

Runtime lifecycle supervision has software-only checks for clean/crash process
replacement over persistent protected credentials/audit history, new incarnation
journals, stale-context rejection and no implicit ownership/replay. The peer
also covers absent/returning aircraft/router traffic, permanent startup errors
and bounded fixture retries. See the [lifecycle fixture](../scripts/dev/runtime_lifecycle_qualification.py).
Systemd syntax/path and Windows SCM control/provisioning tests need no privileged
registration. They validate adapters and process invariants, not an installed
host's service account, boot recovery or physical flight safety. Accept privileged
registration, permissions, upgrade and rollback on the deployment host;
see [Operations](operations.md).

This is the current summary of what the repository proves and what remains
unqualified. The source-arbitration qualification slice started from `main` at
`34d93335c41a000d78a323436e9027704bffc160` after PR #52. It adds a disarmed
production-runtime SITL scenario and a physical qualification procedure without
changing the production command surface or aircraft parameters. The executable
mutation scenario uses Copter; current runtime v1 mutations are unsupported for
QuadPlane, and that capability gate remains intact.
A test, passing workflow or simulator result proves only the boundary and
revision it actually exercised. This page is not flight authorization.

## Evidence levels

| Evidence | What it establishes | What it does not establish |
|---|---|---|
| Unit and software integration tests | C++ policy against fake transports; request validation and lifecycle; client translation; profile/template handling; ROS and Mission Planner software boundaries | MAVLink delivery to a real controller, radio behavior, physical actuation or flight safety |
| Deterministic MAVLink peer | MAVSDK wire encoding, selected retry/admission behavior and independently counted UDP frames against a local peer | Flight-controller acceptance, aircraft outcome, RF behavior or pilot takeover |
| ArduPilot SITL | Simulated state transitions and observed outcomes for the exact simulator profile, source revision and scenario | Hardware behavior, radio independence, physical takeover, payload mechanics or competition readiness |
| Hardware or flight qualification | Only the aircraft, firmware, equipment, procedure and outcomes observed in a reviewed record | Any untested profile, phase, fault or configuration |

Do not promote evidence from one row to the next. The [safety case](safety.md)
contains stable requirement mappings and fault-path detail. Dated logs,
historical task names and run-specific reports remain in the
[migration evidence archive](migration.md).

## Proven software boundaries

The [source-arbitration model](source-arbitration.md) lists every intended
source, the software/SITL/bench/flight distinctions, controller mechanisms,
unresolved production RC inputs and the later hardware procedure. `handback`
explicitly returns software authority to NOMAD after `revoke`; it does not
establish pilot control. External mode changes do not revoke runtime ownership.

- The production runtime starts without an admitted software command owner.
  Typed mutations require explicit authority admission and carry the runtime
  incarnation, vehicle session, owner, generation, sequence and expiry. Revoke
  and explicit handback advance the generation. Reconnect, runtime restart and
  response-cache eviction do not restore authority or make an old request
  executable. See [runtime IPC](runtime-ipc.md) and
  [runtime authority tests](../tests/runtime_ipc_test.cpp).
- The pinned MAVSDK fork carries that admission through `COMMAND_LONG` and
  `COMMAND_INT` retries to final UDP delivery. The independent peer fixtures
  observe suppressed frames after revocation, including retry and queued-send
  cases. This guarantee is scoped to those command encodings and the tested UDP
  path; it is not a general fence for every MAVLink message or transport. See
  [`test-mavsdk-authority-wire`](development.md#mavsdk-transport-and-authority-checks),
  [wire fixtures](../scripts/dev/mavsdk_authority_wire_fixture.py), and the
  [MAVSDK provenance record](mavsdk-dependencies.md).
- Product profiles set `NOMAD_INTEGRATED_FLIGHT=1`, which inhibits direct
  actuation by the separately built `nomad-qualification` test tool. That tool
  is non-installed and exists for SITL and transport qualification. Template
  checks are in [test_deployment_profiles.py](../tests/test_deployment_profiles.py).
- The installed `nomad` CLI uses runtime IPC only. Mission Planner's supported
  plugin requests also use runtime IPC. Unsupported v1 requests, including
  GuidedGoto, return unavailable without a direct-vehicle fallback. See the
  [runtime client regression](../tests/test_runtime_wiring.py) and
  [Mission Planner client tests](../mission_planner/tests/coreclient/NomadCoreClientTests.cs).
- The standalone ground router rejects command egress from its
  `mission_planner` consumer. It still transports traffic; it does not authorize
  flight actions, authenticate consumer names or block unrelated external
  sources. See [router review tests](../mission_planner/tests/duallink/RouterReviewTests.cs).
  The aircraft-side `mavlink-router` is a different process.
- ROS 2 uses a receive-only MAVLink observer and publishes validated GPS and
  battery samples. It exposes no vehicle command or VIO submission interface.
  Its direct observer socket is separate from runtime IPC because protocol v1
  does not yet supply the source measurements ROS needs. See the
  [ROS integration suite](../tests/ros/test_nomad_ros_services.py) and
  [observer contract tests](../tests/test_ros_observation_boundary.py).
- Core and adapter tests cover decision-specific telemetry freshness, invalid
  values, generic actuator authorization, sequencing and recovery failure handling. The pinned QuadPlane
  landing operation requires post-command descent, landed-state telemetry,
  disarm and a stable final envelope; an ACK alone is not reported as touchdown.
  These are software and simulator proofs, not physical payload or aircraft
  outcomes. See [core safety tests](../tests/safety_test.cpp) and
  [QuadPlane landing tests](../tests/vehicle/quadplane/quadplane_vtol_landing_test.cpp).

## Connection lifetime concurrency

The [connection lifetime fixture](../scripts/dev/mavsdk_lifetime_fixture.py)
exercises concurrent readers through unpublished candidates, subscription setup,
publication, retirement, failed setup and repeated connection generations. The
[runtime lifetime fixture](../scripts/dev/runtime_connection_lifetime_fixture.py)
keeps four HELLO/STATUS clients active through absent-peer startup, two real
link-loss/return cycles and shutdown. It checks fresh sessions, no restored
owner and zero command delivery from retired authority context. These fixtures
run with the existing Linux/Windows transport and lifecycle qualification jobs;
they require neither hardware nor SITL.

Deterministic interleavings and source lock reasoning are the evidence for the
NOMAD resource contract; passing ordinary CI alone is not race-detector proof.
A local GCC ThreadSanitizer capability probe on WSL failed before application
execution with `FATAL: ThreadSanitizer: unexpected memory mapping`. The lifetime
fixture therefore remains dynamically unqualified by TSAN on that platform.
No suppressions were added. Pinned MAVSDK internal synchronization and callback
queues are outside the scope of this NOMAD publication fix.

## SITL and ROS readiness

| Workflow | Trigger and scope | Evidence limit |
|---|---|---|
| [test.yml](../.github/workflows/test.yml) | Pull requests and pushes: Python, C++ core, and deterministic MAVSDK qualification | No live flight-controller or hardware evidence |
| [lint.yml](../.github/workflows/lint.yml) | Pull requests and pushes: changed-source checks, C++ tests, Ruff, ShellCheck, and the strict docs build | Does not qualify vehicle behavior |
| [ros-sim.yml](../.github/workflows/ros-sim.yml) | Pull requests and relevant pushes: CPU ROS Humble image plus the real observer node and an in-process MAVLink responder | Tests translation, freshness and receive-only behavior; no live SITL, camera, GPU or flight |
| [sitl.yml](../.github/workflows/sitl.yml) main push | Path-triggered reduced Copter connectivity smoke | The successful exact-base run [36659200606](https://github.com/YoussGm3o8/NOMAD/actions/runs/36659200606) at `042c980` covered MAVSDK connect/status only; full SITL jobs were skipped for the push event |
| [sitl.yml](../.github/workflows/sitl.yml) schedule / manual dispatch | Full Copter scenarios, disarmed simulator RC-fault delivery probe and pinned QuadPlane operation chain | The latest completed full run before `042c980` was [36576342974](https://github.com/YoussGm3o8/NOMAD/actions/runs/36576342974) at `cd9eb4e`; it is evidence for that SHA, not a full run at `042c980` |
| [docker.yml](../.github/workflows/docker.yml) manual dispatch | Jetson ARM64 image and Isaac ROS GPU image on self-hosted runners | Jetson requires an ARM64 runner; Isaac ROS requires its base image and an NVIDIA GPU. Standard hosted CI has neither |

The pinned QuadPlane run exercises ArduPlane 4.7.1 commit
`dbe792162d06cab66c3475fd5556bf7a120f119e` with the `quadplane-tilttri`
profile and `Q_ENABLE=2`: identity and telemetry, arm and VTOL takeoff,
VTOL-to-fixed-wing transition, a two-point fixed-wing route, recovery,
fixed-wing-to-VTOL transition and QLAND landing. The independent qualification
driver/operator establishes some setup states, including AUTO; this is not an
autonomous end-to-end Task 1 flight. See the
[SITL workflow](../.github/workflows/sitl.yml),
[SITL task list](../pixi.toml), and [scenario notes](../tests/sitl/README.md).

The exact-base CPU ROS integration run
[36659200578](https://github.com/YoussGm3o8/NOMAD/actions/runs/36659200578)
passed at `042c980`. It uses the CPU-only `nomad-sim-ros` image and an
in-process MAVLink responder. The node is an observer; this run is not a real
ArduPilot SITL flight or a perception/VIO qualification.

Local Copter SITL requires Docker and the `nomad-sitl:copter-4.7.1` image;
the Compose file documents how to build it. The pinned QuadPlane image is built
from `docker/Dockerfile.sitl-plane`. CPU ROS tests require Docker to build
`nomad-sim-ros:latest`, but do not require the SITL stack. GPU/Jetson workflows
need their compatible hardware and base images. See the exact local commands in
[development](development.md#sitl-and-live-mavsdk-smoke).

## Runtime source qualification evidence

The new [runtime authority SITL scenario](../scripts/dev/core_sitl_authority.py)
and its [run procedure](../tests/sitl/README.md#runtime-authority-and-independent-source)
are wired into scheduled/manual Copter CI with a lighter PR guard/observer
gate. The local disarmed Copter run on 2026-09-30 passed at implementation SHA
`aff6d152f3f73e0ef45dd5ae8eefa9ee5e23863b`, with firmware
`dbe792162d06cab66c3475fd5556bf7a120f119e`. Its source-dirty flag records
preserved pre-existing workspace changes; this is not clean-checkout evidence.
The same scenario passed from a clean hosted checkout in
[manual SITL run 36672382568](https://github.com/YoussGm3o8/NOMAD/actions/runs/36672382568)
at that exact implementation SHA. The `runtime-authority-sitl` artifact records
all nine checks, actual parameter readbacks and source IDs; its scope is
`disarmed_pinned_copter_runtime_software_authority_boundary_only`.
Fresh FC output observations matched runtime requests, source 250 changed FC
modes while NOMAD was admitted and revoked. Two wire attempts (initial send plus
one retry) were observed, with two FC ACKs dropped; no further qualified command
frame was observed after revocation. Link recovery changed the session without
restoring an owner; runtime restart rejected old context and required new
admission. The initial simulator output was restored and observed after runtime
shutdown. These results complete the tested Copter/software slice only.

The earlier hosted run above failed the heartbeat cadence assertion and skipped
later velocity/geofence steps; retain that failed record as historical evidence.
PR53's final-head [manual SITL run 36744321862](https://github.com/YoussGm3o8/NOMAD/actions/runs/36744321862)
at `1c8baf76de29b1db5134db11dbac979d1fc2c73d` passed the heartbeat relay-gate
scenario and all downstream velocity loop-closure, geofence containment and
geofence upload/readback steps. Both full Copter and QuadPlane jobs passed.
GitHub job/step conclusions were verified before updating this ledger. This
closes the stale hosted-heartbeat TODO without claiming physical qualification.

## Authenticated client and durable audit slice

The software-only client trust boundary adds distinct configured shared-secret
credentials, HMAC request/runtime proofs and a runtime-owned JSONL command journal. Its
[protocol, threat model and failure policy](runtime-ipc.md#authenticated-local-clients)
retain the PR53 generation/session/expiry/sequence and MAVSDK final-send fences.
No physical hardware is needed, and no PR54 physical-arbitration work is included.
Software command results and journal records are not physical outcome evidence.
Local full CTest suites passed all 21 tests on Windows/MSVC and native Linux/GCC
(WSL Ubuntu). The real runtime/installed CLI deterministic-peer smoke passed on
both platforms. Authentication, admission, replay, expiry, interrupted authority,
durable intent ordering, native protected-file failure/locking and corrupt-history
recovery are software-only checks. Validation results for this slice are reported
against its PR head; the prior SITL run above is baseline evidence only.

## Mutation outcome correctness slice (H2)

This slice starts from PR59/main
`11dec219583d30b63f4e5efe744ad6213d49a268` and excludes PR54.
The base runtime already journaled interrupted/internal-error outcomes more
accurately than its error JSON, while Mission Planner collapsed definite failure
and most runtime errors into rejection. The
[protocol outcome contract](runtime-ipc.md#vehicle-mutation-outcomes) now carries
the evidence through runtime JSON, C# client diagnostics and operator display.

| Evidence path | Deterministic check | Required classification |
|---|---|---|
| Authentication, authority, expiry, replay, malformed request | Runtime CTest and real runtime peer smoke | `rejected`, no eligible send |
| Final admission denied before eligibility | Runtime fake admission and existing independent authority-wire probes | `rejected`; admitted preflight without delivery certainty remains uncertain |
| Negative FC ACK | Real deterministic MAVSDK peer and C# envelope harness | `failed`, acknowledged, never rejected |
| Authority change after actual command delivery | Runtime completion gate and real peer withholding ACK until revoke | `authority_interrupted` / `interrupted`, journal agreement, no replay |
| Internal exception after possible execution | Runtime fake transport fault | `unknown`, journal agreement |
| Durable outcome write failure | Existing injected journal failure with cache regression | `unknown` after possible send, health latch retained; incomplete intent stays unknown |
| Connection loss before/after request write | C# loopback harness | `FailedBeforeSend` / `UnknownOutcome`, no automatic retry |
| All five supported successful mutations | Runtime fake matrix, real peer and C# harness | `success`, ACK preserved independently |
| Payload release/retract | Mission Planner output/payload harness | Commanded state changes only on software success; physical state unverified |

The real peer counts MAVLink deliveries independently of runtime responses and
compares response outcomes/ACK evidence with committed journal records. Cached
unknown responses remain identical and cause no new delivery. After authority
changes, the old context is rejected before cache lookup without re-execution.
These are software-only checks; they do not prove physical release, servo travel
or gimbal arrival. This section describes the checks, not a claim that every
hosted run has passed; final PR validation records the exact head and run links.

Local Windows/MSVC validation passed all 22 CTest executables, the real-runtime
H2 peer matrix, clean/crash and session/reconnect lifecycle qualification, the
expanded Mission Planner core-client/output/payload/reel harness, gimbal and
payload interlock checks, plugin build and dead-code lint. Lint, format,
changed-file complexity, strict docs and changed-file pre-commit passed.
Hosted Linux, Windows and C# conclusions are recorded against the PR head
in its validation report, rather than inferred from this local evidence.

The broader audit M2 immutable result and async/concurrency cleanup, audit M1
sequence resume, physical arbitration and aircraft-operation changes remain
outside this slice.

## Software resource budgets

The software-only resource slice starts from merged PR56/main
`ded3f5a5f6287c2b1af68eddeeba959e0d4f37a8`; PR54 is excluded. Baselines come from
[hosted run 36952301883](https://github.com/YoussGm3o8/NOMAD/actions/runs/36952301883),
whose checkout was PR head `ddf24ca060f4e9c405ba9bc6609705a9a9a27fd8` merged by
GitHub with that base, measured as `a89b65b44ea766e18d3e9fcafa540ec1ab4bab94`.
Both platform measurement steps passed; their verifier steps deliberately
failed because no budgets had yet been approved. This is baseline evidence,
not a passing final policy run. Subsequent resource jobs check out the exact PR
head and retain its SHA in each report. MAVSDK remains pinned at
`900fb0fe7fec74608f1911331218557915cc501a` with the unchanged six-plugin static
[composition](mavsdk-dependencies.md#production-resource-composition).
Those initial artifacts predate the mandatory `source_clean` field; their clean
provenance comes from the hosted checkout. Current collector/verifier reports
require the explicit attestation and reject its omission. Reviewed numeric
baseline definitions preserve that historical source/run provenance.

Both hosted trees were fresh Release builds, with Pixi caching enabled but hit
status and dependency download-cache state unknown. Linux used GCC 13.3.0,
CMake 3.31.6 and unstripped ELF; Windows used MSVC 19.51.36260.0, CMake 4.4.3,
Visual Studio 2026 and PE without PDBs. These representations are distinct.
Values below are bytes; each cell is **baseline / hard limit**.

| Metric | Hosted Linux | Hosted Windows |
|---|---:|---:|
| `nomad-runtime` | 5,760,592 / 7,208,960 | 2,526,208 / 3,211,264 |
| `nomad` CLI | 217,312 / 393,216 | 157,696 / 327,680 |
| Complete core stage | 6,173,748 / 7,733,248 | 2,879,748 / 3,604,480 |
| TGZ | 1,753,079 / 2,228,224 | 948,790 / 1,245,184 |
| ZIP | 1,780,817 / 2,228,224 | 973,452 / 1,245,184 |
| Peak runtime resident memory | 15,613,952 / 32,505,856 | 14,393,344 / 30,408,704 |

Memory state-window maxima across five processes were idle 11,505,664 /
11,128,832; session 12,488,704 / 12,943,360; authenticated admission 14,983,168 /
13,090,816; and admitted-after-command 14,987,264 / 13,135,872 bytes
(Linux / Windows). Linux RSS and Windows working set cover the runtime PID,
not the peer, child process tree, private allocations or deployed service stack.
Windows private-memory fields are retained separately when native APIs provide
them. The 20 ms sampled peak can miss shorter transients.

Hosted Linux gained 86,016 resident bytes between cycles 256 and 300; hosted
Windows gained 61,440 bytes and local Windows gained zero. These small slopes
were investigated before approval. Source review found the 256-response cache,
32-active-client limit/reaping and completed MAVSDK command queue bounded; audit
records append to disk rather than accumulating in memory. Two additional
1,200-cycle WSL/GCC Release diagnostics at PR head `ddf24ca` checked native
`/proc` RSS, anonymous/file RSS, PSS, private-dirty pages and thread count, and
validated all 1,200 peer commands plus durable intent/outcome/handback/revoke
records. Their different kernel, CMake and `BUILD_TESTING=OFF` configuration
exclude them from hosted absolute-memory comparisons.

The default allocator run lasted 111.73 seconds. RSS stayed at 14,938,112 bytes
from cycles 1,000 through 1,200, with five settled threads and constant file
RSS; private-dirty pages increased another 81,920 bytes in that interval.
An explicitly labeled `MALLOC_ARENA_MAX=1` control lasted 106.77 seconds:
RSS 13,869,056, anonymous RSS 3,276,800 and private-dirty 3,387,392 bytes all
stayed flat from cycles 850 through 1,200, again with five threads. This supports
allocator/page retention as an explanation rather than proving leak freedom.
The allocator control is not a production setting or budget baseline. Growth
remains advisory; sustained slopes across longer comparable runs still require
investigation before raising ceilings.

Startup/restart distributions have five samples, in seconds. Empirical p95 is
the observed maximum. Vehicle readiness requires a deterministic peer session
and fresh heartbeat; IPC readiness requires a valid HELLO without a peer.

| Measurement | Linux min / median / max | Windows min / median / max | Hard limits Linux / Windows |
|---|---|---|---|
| Launch to IPC | 0.0524 / 0.0526 / 0.0541 | 0.5056 / 0.5116 / 1.0564 | 5.1 / 6.4 |
| Launch to vehicle | 0.4243 / 0.4246 / 0.4275 | 0.5494 / 0.5505 / 0.5614 | 5.5 / 5.6 |
| Clean restart to IPC | 0.0525 / 0.0526 / 0.0526 | 0.5061 / 0.5084 / 0.5287 | 5.1 / 5.6 |
| Clean restart to vehicle | 0.3725 / 0.3727 / 0.3728 | 0.5381 / 0.5619 / 0.5768 | 5.4 / 5.6 |

Fresh hosted phase baselines are advisory, not deterministic wall-time gates:

| Phase, seconds | Linux | Windows |
|---|---:|---:|
| Root configure, including dependency downloads/configure/build | 45.33 | 190.53 |
| MAVSDK target build | 131.61 | 343.10 |
| Core configure/build remainder (build only) | 40.16 | 87.60 |
| Full C++ suite | 63.68 | 66.86 |
| Connectivity / transport | 11.49 / 64.91 | 11.57 / 101.31 |
| Authenticated IPC / lifecycle | 1.17 / 17.45 | 2.00 / 14.25 |
| Package / package verification | 2.93 / 0.16 | 1.25 / 3.70 |
| Core stage / stage verification | 0.03 / 0.06 | 0.09 / 1.68 |

The superbuild runs synchronously inside root configure, so its work cannot be
presented as a separate exclusive configure phase without instrumenting the
dependency. All remaining named phases are retained too. Timing limits are
twice each baseline plus 30 seconds; a 45-minute job ceiling catches catastrophic
regressions. Incremental timings are labeled and never compared with cold-tree
phase thresholds. No CMake build-tree cache is restored in hosted CI.

Advisory footprint references are MAVSDK workspace 219,062,155 / 434,217,493,
SDK stage 20,367,778 / 122,412,915, dependency stage 20,477,282 / 29,070,297,
and linked static archive inputs 12,152,456 / 124,504,890 bytes (Linux / Windows).
Intermediate SDK archives/workspaces may contain toolchain metadata; they do
not enter release payload gates. Both reports list seven SDK-side archive
inputs, with no built SDK-side archive absent from the runtime link line.
Archive-input size does not establish embedded object-code cost; a link-map or
shared-link counterfactual remains unmeasured.

Local Windows/MSVC 19.44.35228.0, Visual Studio 2022, CMake 4.2.0-rc1 has a
separate incremental profile. At the same PR head it measured runtime 2,561,024,
CLI 157,696, core stage 2,914,564, TGZ 964,145, ZIP 989,596 and peak resident
14,741,504 bytes. Its hard stage ceiling is 3,670,016 and peak ceiling 31,457,280
bytes; other release ceilings match hosted Windows. This is not pooled with
the newer hosted compiler. Full Release CTest, IPC/auth/audit, lifecycle,
connectivity, transport, authority-wire, final-send and package/stage checks
passed locally and in both hosted measurement jobs.

The [versioned policy](../config/core-resource-budgets.json) records every
baseline, headroom, rationale and exact comparable metadata. Release footprint
headroom is 25% with small absolute floors; peak memory gets 50% plus 8 MiB;
startup limits are `max(baseline + 5 seconds, 6 * baseline)`. These allow
engineering growth and normal native variance while detecting meaningful
payload or gross runtime regressions. State medians, bounded growth, dependency
footprints and compile/test phases remain advisory. No functionality or safety
check was removed to meet a threshold.

Per-run JSON and phase evidence are retained for 30 days as separate
`core-resource-metrics-ubuntu-latest` and `core-resource-metrics-windows-latest`
artifacts, with measured-versus-budget CI summaries. The checked-in baseline
definitions remain after artifact expiration. Reproduction, error exit codes
and review procedure are in
[development](development.md#software-resource-qualification). These software
budgets do not prove physical-aircraft startup latency or flight performance.

## Versioned release lifecycle

The release model was audited at `1efaa335833038419b5bdb4f9566bce8ea1b14d8`
before implementation. The audit and exact operator procedures are in
[operations](operations.md#versioned-release-deployment). One complete manifest
binds four independent component packages to a source revision, pinned MAVSDK,
product version, platform, content digests and protocol expectations. Checksums
establish integrity and correlation; publisher signing is not implemented.

Local Windows and Linux software qualification exercises distinct compiled core
A and B through stage A, activate A, stage B with A still admitted/running,
activate B, and rollback to the exact retained A package. Each transition waits
for the previous process to exit. B and rollback A report new runtime
incarnations, no owner, and reject the old authority context; fresh client
proof and explicit admission are required. The fixture retains external
configuration, credentials and durable command audit across every transition.
Failed candidate startup and failed health automatically restore verified A;
a changed package is rejected before the active process or record changes.

On Windows, compiled standalone router A and B exercise loopback management
hello/status, expected versions, independent process shutdown and A → B → A.
The production pointer/supervisor adapter also exercises failed health and
startup-exit recovery using a test launcher instead of Task Scheduler.
External topology remains byte-identical. Fake Mission Planner directories
exercise DLL A → B → A, loaded-file refusal, wrong target/version rejection,
closed-application enforcement, interrupted recovery and installer argument
quoting through paths containing spaces. Simulated settings, credentials,
Mission Planner files and another plugin remain unchanged. Actual Mission
Planner GUI startup is not part of these tests.

The platform filesystem/transaction tests cover corrupt and incomplete
manifests, platform/protocol mismatch, unsafe archive paths/links, duplicate
identities, staging faults, same-version different bytes, active deployment
drift, stop refusal, commit-record write failure, missing/corrupt rollback
payload, interrupted recovery and protected cleanup. Linux tests use actual
atomic symlinks with a modeled systemd runner; Windows tests model SCM calls
and exercise native DLL sharing locks and deployment ACL validation. They do
not register services or scheduled tasks on the test host.

Reproduce with `pixi run test-release-lifecycle`,
`pixi run python scripts/dev/build_release_core_fixtures.py`, then
`pixi run python scripts/dev/release_process_qualification.py --core-a
build/release-fixtures/core-A --core-b build/release-fixtures/core-B`.
For Windows router qualification, first run
`pwsh -File scripts/build/build_release_router_fixtures.ps1`, then the same
qualifier with `--router-a build/release-fixtures/router-A --router-b
build/release-fixtures/router-B`. The core fixtures cannot be installed or
packaged as production releases. CI runs these checks on their supported
platforms and aggregates all required release packages before publication.

Privileged real-host acceptance must still verify systemd unit ownership and
boot/recovery, native SCM registration/account/recovery and path updates,
Task Scheduler independent router supervision, protected groundstation plugin
permissions, Mission Planner 1.3.83 loading, and restart after power loss.
A supervisor crash with an unconfirmed router child requires an operator to
verify child exit before recovery; the tool fails closed instead of claiming
rollback. Filesystem pointer/record replacements are atomic individually;
process activation and multi-host deployment are not atomic. This software
qualification establishes binary rollback, not aircraft readiness or physical
flight safety. PR54 remains separate and is not a dependency.

## Not yet equivalent to qualification

Native GCS
source-250 mode acceptance in a disarmed simulator is simulated external-source
evidence, not physical pilot takeover. Copter output evidence does not qualify
a QuadPlane runtime mutation; protocol v1 currently has none that can execute.
The standalone production router and
aircraft-side router are absent from this scenario's direct simulator topology.

- Runtime authority is one admitted **software source** for typed requests to
  that runtime. It is not whole-aircraft authority. Native Mission Planner
  controls, pilot/RC input, ArduPilot behavior, maintenance tools and external
  MAVLink sources remain outside that guarantee.
- The retry gate does not prove RC/ELRS priority, physical pilot takeover or
  handback. It covers `COMMAND_LONG`/`COMMAND_INT` retries through tested final
  UDP delivery. Offboard setpoints and fence transfers are not runtime IPC v1
  commands and do not share that final-send guarantee.
- The final production RC channel map, physical RC/ELRS arbitration, complete
  C2-loss policy and production termination behavior remain unqualified. The
  plugin's termination requests report unavailable and send no substitute
  command. The disarmed `SIM_RC_FAIL` probe proves simulator fault delivery and
  restoration only.
- The QuadPlane operation chain is pinned SITL evidence. It does not qualify
  every transition/recovery scenario on real aircraft, transition-phase
  termination, QuadPlane link-loss response, generic mode/takeoff/goto/land or
  the complete Task 1 flight.
- No hardware or flight qualification is recorded for the product profiles,
  manual takeover, termination, physical payload release, gimbal motion,
  aircraft endurance or competition readiness. Profile tests validate
  configuration files; they do not prove a deployed system works.

Track unresolved work in [TODO](../TODO.md). Keep test, peer, SITL and hardware
records tied to their exact revision and profile.
