# Development

The current tree builds a C++20 library and CLI with optional ROS 2 and Mission
Planner adapters. Edge Core source is deleted and active build, deployment and
setup entrypoints target the C++ core. Current qualification scope and baseline
counts belong in [migration status](migration.md#current-qualification-status).

## Prerequisites and verified local checks

Use Git, Pixi, CMake (3.22.1 or newer) and a C++20 compiler, and initialize the
submodules: the transport is the pinned `third_party/MAVSDK` checkout, and CMake
fails configuration when it is missing. The MAVSDK build fetches its own pinned
dependencies, so a first configure needs network access. C++ runtime use does not
require Python, ROS or a GPU.

~~~sh
pixi run build-core
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run complexity-check
pixi run docs-build
pixi run build-core-release
pixi run package-core
pixi run verify-core-staged-install
pixi run verify-core-package
~~~

`build-core` builds only the production `nomad` client and `nomad-runtime` in the
build tree. `test-core` builds and runs the CTest suite. Use
`build-qualification-cli` for the non-installed direct MAVSDK driver, or
`build-sitl-tools` for the production executables plus that driver.
format-check is read-only with respect to source; format rewrites source and is
not appropriate for a documentation-only review of unrelated migration work.
docs-build is the strict ProperDocs site check.
Build, package, stage and install are separate operations. `build-core-release`
creates the Release CLI/runtime in the build tree. `package-core` creates CPack
ZIP/TGZ artifacts from that build.
`verify-core-staged-install` installs into `build/package/stage` and verifies
that isolated tree; `verify-core-package` checks both archives without opening a
vehicle connection. `install-core` is the explicit prefix-install task and
requires a destination argument. Build, test and package tasks do not write to a
user or system installation.

complexity-check applies the source-size rules to new and modified files: 500
lines per source file, 40 per Python function, and 120 columns per C/C++ line
(ruff already enforces the Python limit through E501). It also rejects a baseline
entry that no longer points at an oversized file, function or tolerated over-long
line, so the baselines cannot silently widen the gate.
The three improvement baseline files record the outstanding offenders; see the
[current status](migration.md#current-qualification-status) for their entry
counts. Add an entry only for a genuine offender — the freshness check makes
removing it when the file is split, the function is shortened, lines are wrapped
or the file is deleted non-optional. The C/C++ column limit is enforced through
this reporter rather than clang-format because the formatter is not a pinned
repository dependency. Shared helper code lives in exactly one place: C++ tests use
`tests/support/test_harness.hpp` (assert/report/pass/fail plus the console-safe `main`)
and `tests/loopback_socket.hpp`; `nomad::util` owns argv and environment number
parsing; `infra/tailscale/shell.py` owns the command probe both monitors call. Do
not copy one of those helpers back into a caller.

The full `line-report` census reads tracked source paths only. Changed-only
checks also include non-ignored untracked first-party files; both scopes exclude
private `local/`, generated/build output and vendored paths. Numerical ceilings
for the five existing oversized files and one Python function live in
`config/source_size_caps.json`, measured at its recorded baseline commit. These
caps reject growth; the stale-baseline check separately requires removing an
exemption once its file or function reaches the 500/40-line threshold.

`pixi run cpp-complexity-report` prints a human summary;
`pixi run python -m scripts.dev.cpp_complexity_report --output report.json`
writes the versioned JSON artifact. CI uploads that report by head SHA as
advisory evidence; no C++ complexity threshold blocks the build.
`nomad-cpp-lexical-v1` counts physical
function/lambda spans, decision tokens and lexical brace depth. It does not
expand macros or evaluate conditional branches; brace depth is not control-flow
nesting, nested-lambda decisions also appear in their enclosing function, and
constructors with braced member initializers are omitted and listed. Operator
definitions, function-pointer declarators and unusual C++ syntax may also be
missed, so review coverage and representative safety functions before using a
numeric C++ cap.

debt: C++ complexity remains report-only; revisit when a clean-base artifact's
tracked-file and function/lambda counts, macro/conditional totals, known omitted
constructor count, and representative safety-function rows are reviewed; then
ratchet only a reviewed subset or replace the lexical analyzer.

`build-core` is the production build entrypoint. `pixi run test` measures
coverage of retained Python tools and Tailscale helpers. The deleted API server,
API smoke task and gimbal SITL task are removed.

`dev-up` starts isolated ArduPilot SITL plus the passive Mission Planner bridge;
it requires Docker and the `nomad-sitl:copter-4.7.1` image (build instructions are
in docker/docker-compose.dev.yml). `sitl` builds the core first, then starts that
stack and runs its scenario. `sim-ros-up` additionally forwards a receive-only
MAVLink telemetry stream to the ROS observer. Direct Compose ROS use must set
`NOMAD_SITL_ROS_OUTPUT` to
`--out udp:nomad_vehicle_node:14552`. The simulator always emits the normal host
stream on 14570 and a private relay copy on 14572; scenario tasks choose which
stream to read through `NOMAD_CORE_SITL_PORT`.

The stack seeds the vehicle from two parameter files: `docker/sitl-fence.parm`
for the polygon fence and `docker/sitl-streams.parm` for the SERIAL0 MAVLink
stream rates. ArduPilot streams position, attitude and extended status only
after a GCS requests the group and the core deliberately never requests
streams, so those rates are what make a host-side copy readable; a stack that
drops them leaves the scenarios watching heartbeats. See C24 in
[migration](migration.md).
These paths have local configuration checks, but current live image/SITL/ROS
qualification remains open at G1.

CPU ROS integration and the optional GPU adapter image remain available.
No simulation sensor provider is included; qualify a selected source before
claiming perception or VIO availability. The Python video bridge can consume
an explicitly supplied ROS image stream.

On a configured companion host, `nomad start video_bridge` directly manages the
retained bridge inside the adapter container; it no longer calls a vehicle REST
service. The control endpoint is loopback-only. Product profiles keep this and
other optional compute services disabled until G3 qualification.

## Test layers

| Layer | Current checks | Required expansion |
|---|---|---|
| C++ | Current registrations and qualification scope: see [migration status](migration.md#current-qualification-status) | Authority, per-field freshness, cancellation, MAVSDK and vehicle-class coverage |
| Python | pytest includes client contracts, traceability, harnesses, profiles and video tools | Mock competition server/traffic, perception replay and tracker fixtures |
| ROS | ros2/nomad_ros telemetry translation plus tests/ros integration | Runtime-owned telemetry values, source acquisition metadata, and a defined odometry frame contract |
| Mission Planner | lint-plugin and test-plugin-* helper scripts | Ownership, capabilities, stale displays, action lifecycle and replay |
| SITL | core-sitl-* and sitl-fence | Current artifacts, QuadPlane transitions, competition scenarios and independent faults |
| Hardware | No evidence collected in this review | Selected board/sensor/radio/payload/endurance and task gates |

Tests/ros and live SITL checks are environment-gated; report skips explicitly.
The checked cpp_traceability block only proves references exist. It does not
prove every safety requirement is covered or satisfied.
The CONOPS traceability test checks inventory fields, source-page bounds, unique
IDs, canonical references, unresolved-question links and the single active ledger
item. Run `pixi run python -m pytest tests/test_conops_traceability.py -q` after
requirements edits. A passing structural check does not prove extraction
completeness, interpretation accuracy or implemented flight compliance.

`pixi run test-runtime-ipc` builds the core and exercises `nomad-runtime` with a
local fake MAVLink peer and the installed `nomad` client, including typed
requests, client reconnect and runtime restart. It does not require aircraft
hardware or Docker.

## SITL discipline

Use an isolated identified simulator with known state. Serialize scenarios that
share vehicle state. Every important test establishes state, performs one action,
observes authoritative outcome, asserts independent conditions and restores
state when required. A relay ACK is not physical sample collection, and LAND
mode is not a completed landing.

Existing tasks: core-sitl-status, core-sitl-command-flow, core-sitl-mission,
core-sitl-velocity-watchdog, core-sitl-geofence, core-sitl-payload,
core-sitl-link-loss, core-sitl-link-recovery, core-sitl-zero-delivery,
core-sitl-gcs-heartbeat, core-sitl-quadplane-observe,
core-sitl-quadplane-vtol-takeoff, core-sitl-quadplane-transition,
core-sitl-quadplane-route, core-sitl-quadplane-transition-back,
core-sitl-quadplane-vtol-landing and sitl-fence. Use them
against a configured isolated endpoint with no hardware path attached; a live
passing run is still required before G1 closes.

These scenarios and `tests/sitl/velocity_loop_closure.py` use the explicitly
built `nomad-qualification` driver because they exercise direct vehicle methods
that protocol v1 does not expose. The target is excluded from default builds and
packages. Production commands use the installed `nomad` IPC client.

The Task 1 observation reference is ArduPlane 4.7.1 at exact commit
`dbe792162d06cab66c3475fd5556bf7a120f119e`, using the
`quadplane-tilttri` frame and `docker/quadplane-tilttri.parm`. Build and start it
with `pixi run quadplane-sitl-up`, observe it with
`pixi run core-sitl-quadplane-observe`, and stop it with
`pixi run quadplane-sitl-down`. The observer requires the real heartbeat,
`Q_ENABLE=2`, fresh position/GPS/attitude, disarmed state, and aircraft-reported
GUIDED=15, QLOITER=19, QRTL=21 and RTL=11 modes. ArduPlane reports this profile
as `MAV_TYPE_FIXED_WING` (1), not a VTOL MAV type. NOMAD combines that heartbeat
with `Q_ENABLE=1` or `2` to identify `QuadPlane`; zero identifies Plane, while a
failed read or another value leaves the class unresolved as `Unknown`.

This harness qualifies discovery, telemetry and baseline mode semantics, then
qualifies one explicit NOMAD startup path: QuadPlane GUIDED, authoritative arm,
and ArduPlane's direct GUIDED `MAV_CMD_NAV_TAKEOFF` climb by the requested
delta from the final pre-command relative altitude. NOMAD revalidates heartbeat,
fresh position, 3D GPS, armed state and GUIDED mode immediately before sending
the command, then requires the derived target within a fixed 0.5 m margin. The
Python mode driver requests the observed modes independently; it does not use
or qualify the production `Vehicle::set_mode` path for QuadPlane.
The separate transition harness qualifies only the dedicated
`Vehicle::transition_to_fixed_wing` operation after this startup sequence. The
route harness then uses the independent test operator to establish AUTO for the
qualified transition, and asks NOMAD to fly two fixed-wing GUIDED reposition
targets. The operator's AUTO request remains qualification setup; arbitrary
QuadPlane `Vehicle::set_mode` is rejected. NOMAD verifies each target from a
fresh post-ACK position sample at least 10 m closer than the captured
ACK-boundary position and within 45 m horizontally and 5 m vertically; route
success requires the second target. The harness independently observes the
ordered aircraft position trace. The recovery harness then repeats that
sequence and asks NOMAD to reposition to an explicit point near its starting
route location. Independent post-command position, altitude, mode and VTOL
telemetry must show the bounded recovery region reached while fixed wing.
The transition-back harness repeats the pinned identity/takeoff/forward
transition/route/recovery sequence. It then gives AUTO one explicit
`NAV_LOITER_UNLIM` mission item at the explicit recovery coordinates through the
independent test authority because the pinned transition handler requires AUTO.
It verifies the first fresh recovered relative altitude is inside the reviewed
15–25 m band, then keeps the explicit recovery-point altitude as the AUTO loiter
and transition target. The reference profile enters AUTO in VTOL state. The
already-qualified `transition-to-fixed-wing` operation restores fixed-wing
state before the transition-ready dwell starts. NOMAD verifies the pinned
frame and tilt parameters before it requires the measured altitude and requested
loiter altitude to be within 15–25 m relative to home before command
transmission, and the aircraft within 55 m of the horizontal point. Post-ACK
verification retains a 15 m minimum altitude floor but has no 25 m upper ceiling during
transition climb. The measured altitude need not exactly
match the mission target; its five fresh samples must vary by no more than 1 m.
It must remain at no more than 28 m/s
groundspeed with at most 3 m/s variation and at most 1 m/s climb rate. Five
fresh position samples must span 2 s, with bounded altitude and radial variation.
The historical transition-back results and their implementation provenance are
recorded in [migration status](migration.md#current-qualification-status). The
speed cap remains specific to this pinned SITL profile.
Completion requires fresh post-ACK `Multicopter` state reports, retained armed
AUTO mode and two seconds of stable position/velocity telemetry. Run `pixi run
core-sitl-quadplane-transition-back` for the pinned live sequence. The new
`core-sitl-quadplane-vtol-landing` task repeats the full starting chain and then
calls NOMAD's dedicated `quadplane-vtol-land` CLI verb. It verifies pinned
ArduPlane 4.7.1 / `quadplane-tilttri` readback, armed AUTO multicopter state,
fresh position/velocity/GPS/VTOL/landed telemetry, relative-home altitude
15–25 m, ≤1 m/s groundspeed, ≤0.25 m/s absolute climb and five fresh samples
over two seconds within 5 m of the explicit landing point. The historical full-chain
run provenance is in the [current qualification status](migration.md#current-qualification-status);
its detailed observer measurements remain in the dated migration evidence. This
is existing evidence, not a newly run check. The core sends only fixed
`DO_SET_MODE` custom
mode 20 (QLAND); the point is an admission/final-state reference, and QLAND
holds current position rather than navigating to it. Acknowledgement is not
completion: post-command proof requires QLAND, at least 5 m descent, fresh
`ON_GROUND`, autopilot disarm and five stable final samples within 5 m, with
altitude -1 to 1.5 m, speed ≤0.5 m/s and absolute climb ≤0.2 m/s. The
independent pymavlink observer checks that trace separately from the CLI result.
These are pinned SITL qualification bounds, not general flight limits. These
slices do not qualify disarm as a public operation, generic takeoff/goto, route
planning, generic land, arbitrary RTL/QRTL, QuadPlane link-loss response or the
complete Task 1 flight. Copter mode numbers and velocity-stop behavior cannot stand
in for those tests. GPU adapter images are optional and are not prerequisites
for basic unit or server-contract tests. The independent pymavlink mode
driver establishes `AUTO` because NOMAD deliberately rejects arbitrary QuadPlane `set_mode`; this
does not qualify an autonomous GUIDED -> AUTO -> transition sequence or transfer
command authority to the test driver.

Historical transition-back and Copter regression provenance is in the [migration
status](migration.md#current-qualification-status). The transition-back result
alone establishes that transition only; it does not establish landing or hardware
qualification.

## Adapter and optional build checks

`build-mavsdk-connectivity-smoke` builds the non-installed connect/status
executable and records configure/build timing and footprint metrics.
`run-mavsdk-sitl-smoke` runs that check against an already-running SITL vehicle.
`test-mavsdk-connectivity` runs the equivalent deterministic UDP peer cases plus
provenance tests; `verify-mavsdk-provenance` checks the pinned source and
dependency inventory. Follow [MAVSDK parity gates](mavsdk-adoption.md).

`nomad` is an IPC-only client and has no aircraft transport selector or endpoint
configuration. Run `pixi run test-mavsdk-transport-qualification` for the direct
qualification check: it builds `nomad-qualification`,
`nomad_mavsdk_connection_tests` and `nomad_mavsdk_zero_delivery_tests`, then runs
`scripts/dev/mavsdk_connection_fixture.py`, which asserts accepted, denied,
timeout, wire-form, stale-telemetry and wrong-identity behaviour plus
per-command mode/takeoff/goto/land/RTL/servo/relay/gimbal-config
parity against the deterministic vehicle in `scripts/dev/mavsdk_peer.py`; those
command cases run the test-only qualification driver and assert its verified output against a peer
whose starting state differs from the required result. The same task covers the
link and zero-delivery cases in `scripts/dev/mavsdk_link_fixture.py`: the
pre-latch GCS-heartbeat announcement, coalesced datagrams, live-to-stale link
observation, the body-frame velocity setpoint with its zero on disconnect, and
the SR-LNK-03 stop paths (watchdog, caller stop, destruction, link loss, VIO
loss). `scripts/dev/mavsdk_fixture_harness.py` holds the shared binary, peer and
assertion primitives; add a case to the module that owns its behaviour rather
than to the entry point. Passing it is transport
parity evidence, not a closed Phase B-E or G-M gate. The hosted matrix also writes and
retains a JSON artifact; local output from a dirty vendor checkout is diagnostic,
not clean-checkout qualification. Live smoke output includes per-process-tree
peak RSS and elapsed connect/status time and is retained by the SITL workflow.

After the transport qualification has built its probes,
`pixi run test-mavsdk-authority-wire` runs the final-send cancellation checks at
the MAVLink peer and the command probe.

ROS builds use the separate ament package and supported image; test-ros-integration
runs its real adapter tests. Current source still has blocking callbacks; passing
existing tests does not prove the target callback deadline contract.

Mission Planner uses Windows/.NET Framework 4.8 and its reference assemblies.
Use `pixi run build-plugin-only` to compile the plugin to
`mission_planner/src/bin/Release/NOMADPlugin.dll` without changing the Mission
Planner installation. `pixi run test-plugin-build-only` checks this dispatch
with a temporary deny-write installation and the C# compiler bundled with
Visual Studio MSBuild. Install reviewed artifacts separately using the
[packaging guide](https://github.com/YoussGm3o8/NOMAD/blob/main/mission_planner/packaging/README.md).
Use `lint-plugin` and relevant `test-plugin-*` tasks for other non-deploying
checks. Do not confuse a pure helper test with full plugin integration.

## Contribution and evidence workflow

1. Read AGENTS and deeper guidance; trace owning symbols, callers and tests.
2. Define a concrete falsification test and keep one active work item.
3. Make one focused change; preserve unrelated staged/unstaged work.
4. Run focused tests, then full relevant checks; report every skip/failure.
5. Review the final diff and record requirement, artifact/config identity,
   independent observations, measured values, thresholds and limitations.
6. Update the canonical subject owner; avoid copying status into several plans.

Use a focused branch for implementation and the repository commit prefix style.
Do not stage, commit, push or deploy without an explicit request. MAVSDK
implementation needs a focused merge request with unit and integration evidence;
this documentation review only plans that work.

Size/complexity rules and technical-debt format remain in AGENTS. Keep C++ public
headers in include/nomad, implementation in src, and ROS/Python/vendor types out
of the core API. Retained Python is for tools, perception and tests, never a
parallel vehicle state machine.

## Architecture maintenance

Delete superseded implementations once their replacement is proven. Git is the
source archive; do not create archive/legacy directories for normal source history.
Add subsystem code when implementation starts rather than retaining unfinished
scaffolding indefinitely. Compatibility paths need a concrete current consumer.

Reusable vehicle control, safety, telemetry, transport, operator interfaces,
deployment and infrastructure belong in base NOMAD. Competition-, event- and
mission-specific behavior normally belongs in opt-in modules. Modules must use
the C++ core safety and authority boundaries and must not own parallel vehicle
commands. Keep examples that teach supported extension patterns, disabled as
production features by default.

## Python tooling and component versions

The `nomad-tools` package contains retained media adapters and infrastructure
helpers, with NumPy as its runtime dependency. The `dev` extra supplies MAVLink
fixtures, process resource measurement and repository checks. The `hardware`
extra supplies the serial/gamepad joystick bridge dependencies. GStreamer and
ROS are supplied by their selected runtime images, not by pip. Python does not
own vehicle behavior; CV/ML dependencies should be added with real consumers.

Components intentionally version independently: the C++ core/package version is
in CMakeLists.txt, the Mission Planner plugin version in NOMADPlugin.cs, the
Python tooling version in pyproject.toml, and the ROS adapter version in
ros2/nomad_ros/package.xml. The Pixi workspace version follows the Python
tooling environment. There is no unified product release yet; component
versions are not protocol compatibility guarantees. Qualify the exact component
revisions and runtime protocol version together before deployment.
