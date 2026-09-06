# Migration plan

**Goal:** replace the Python-centered vehicle system with a small C++ core without
creating a permanent dual-runtime architecture.

## Rules

- Freeze feature growth in the current Python vehicle path.
- Port behavior before deleting its implementation.
- Keep safety requirements and tests alive until C++ replacements pass.
- Do not introduce a compatibility framework, broker, or generic module system.
- Delete legacy code as soon as its replacement owns the behavior.
- Mark unfinished work `Open`; do not document aspirations as current behavior.

## Current inventory

| Current area | Main responsibility | Target | Final action |
|---|---|---|---|
| `edge_core/services/mavlink/` | Python transport, packets, telemetry | C++ `mavlink` | Port, test, delete |
| `edge_core/safety/` | Velocity, gates, geofence, payload decisions | C++ validation/safety code | Port each requirement |
| ~~`edge_core/ros_http_bridge/`~~ | ROS to HTTP and direct velocity path | `ros2/nomad_ros` | **Deleted 2026-09-05** — see the Phase 7 ledger |
| `edge_core/api_routes/` | REST endpoints | CLI/client operations | Delete without a real use case |
| `edge_core/core/` | Dynamic modules and lifecycle | None | Delete registry |
| `edge_core/services/video*` | Video bridge control | Python/deployment adapter | Keep only if required |
| `edge_core/services/health*` | Hardware health | Telemetry/client adapter | Move ownership |
| `scripts/nomad` | Service dispatcher | C++ `nomad` CLI | Replace |
| `infra/systemd/` | Many service units | One core plus adapters | Consolidate |
| Mission Planner controls | UI and some direct commands | Client UI | Remove duplicated decisions |
| Removed Python module example | Dynamic module example | None | Do not recreate |

## Phase 0: guidance and enforcement

- Rewrite `AGENTS.md`, README, contributor guidance, and canonical docs.
- Remove conflicting module/setup documents.
- Record current size debt.
- Add changed-file enforcement for the 500-line rule.
- Freeze new architecture in `edge_core`.

**Exit:** docs build, links resolve, current transition tests are understood, and
new contributors see one target architecture.

## Phase 1: C++ foundation

- Add root CMake project.
- Add one `nomad` library and one CLI target.
- Add C++20 value types and minimal logging.
- Add CTest tests without ROS, Python, or hardware.

**Exit:** clean configure, build, and test on a development machine.

## Phase 2: MAVLink and telemetry

- Hide generated MAVLink headers and packet conversion in `mavlink`.
- Implement UDP against SITL first.
- Add heartbeat filtering, connection state, ACKs, telemetry conversion, and
  disconnect detection.
- Add a fake transport for deterministic tests.

**Status:** Implemented in the C++ MVP. MAVLink v1 command frames and MAVLink v1/v2
telemetry frames are CRC-validated, non-GCS heartbeats latch the vehicle, typed
position/battery/GPS/attitude state is exposed through `Vehicle`, and the Docker
SITL stack emits a host UDP copy for `pixi run core-sitl-status`.

**Exit:** C++ connects to SITL and receives verified telemetry without Edge Core.
The automated workflow now runs the smoke command; local completion still
requires a host with Docker and the ArduPilot SITL image.

## Phase 3: vehicle API

**Status:** The MVP command surface is implemented for arm, disarm, mode, takeoff,
land, and RTL with fake-transport ACK and state-verification tests. The automated
`core-sitl-command-flow` task now runs the same CLI through a fresh SITL vehicle;
local or CI execution supplies the real evidence.

Implement and test:

```cpp
vehicle.arm();
vehicle.disarm();
vehicle.set_mode("GUIDED");
vehicle.takeoff(10.0);
vehicle.goto_location(target);
vehicle.land();
vehicle.return_to_launch();
```

Every critical operation sends, waits, verifies, times out, and returns a clear
result.

**Exit:** fake-transport tests and SITL tests cover the complete MVP command set.

## Phase 4: CLI and SITL

**Status:** The thin CLI, `connect`, `status`, and command-flow scenario are
implemented. The usage line documents all twelve supported verbs (`connect`
included). `tests/test_core_client_contract.py` pins the CLI boundary without a
vehicle: the verb surface, fast parse rejections for malformed values, and the
two deterministic no-vehicle failure modes (occupied endpoint and silent
endpoint heartbeat timeout). SR-SEC-02/03 moved onto the boundary: actuation
verbs require `NOMAD_API_KEY` before any socket work, accepted and refused
attempts emit a machine-readable `audit command=... result=... auth=...` line
on stderr, and telemetry verbs keep the no-key local fallback (2026-09-03).
The pixi `core-sitl-*` tasks export the documented development-simulation key
from `config/nomad.env.example`; deployed machines must set a real key. The
deletion gate for the transitional Python command path remains open until
client callers have moved to the C++ boundary.

- Keep argument parsing separate from `Vehicle`.
- Support `connect`, `status`, `arm`, `takeoff`, `land`, and `rtl`.
- Return non-zero status for rejected commands, missing ACKs, and timeouts.
- Make the C++ CLI the primary sim quickstart.

**Exit:** a new developer can build, start SITL, and run the core command flow.

## Phase 5: missions

**Status:** Implemented as a small synchronous C++ executor. It supports takeoff,
navigation, wait, named GUIDED/arm/disarm/land/RTL actions, explicit RTL, and
explicit land. Unknown actions, invalid coordinates, and invalid wait durations
fail closed. The `mission-demo` CLI and `core-sitl-mission` runner now exercise
this executor against a fresh SITL vehicle. The `core-sitl-velocity-watchdog`
runner proves the active setpoint timeout against the same live transport. Cancellation, progress callbacks,
and mission file parsing remain open until a client needs them.

- Add ordinary C++ mission data.
- Support takeoff, waypoint, wait, action, RTL, and land.
- Add cancellation and progress only when execution needs them.
- Do not add a mission language or generic task engine.

**Exit:** deterministic fake-transport mission passes. The dedicated SITL mission
scenario is wired in CI; local evidence remains environment-dependent until run.

## Phase 6: adapters

### ROS 2

**Status:** `ros2/nomad_ros` exists as a separate ament package. It links
`nomad_core` as a subproject, publishes typed telemetry (`NavSatFix`,
`BatteryState`, `Odometry`, `connected`), translates `TwistStamped` into core
FRD velocity commands (FLU-to-FRD documented in the package README), feeds VIO
health/confidence topics into `Vehicle::update_vio`, and exposes thin
`std_srvs/Trigger` services for arm/disarm/land/RTL. All callbacks are short;
no vehicle decisions are duplicated. YAML parameters + launch file included.
The package compiles and its 12 pure translation unit tests pass inside aROS 2 Humble image (2026-09-04; the shipped `nomad-sim-ros` image lacked the
C++/colcon toolchain until `docker/Dockerfile.sim-ros` grew a build layer that
compiles `nomad_core` + `nomad_ros` into a colcon workspace; the core's CTest
targets are gated on the `tests/` tree being present so the subproject builds
cleanly without it).

Live-feed evidence (2026-09-04): the node connected to the Copter 4.7.0 SITL
vehicle over UDP, published typed telemetry, and — with a synthetic healthy
VIO feed on `/nomad/vio_health` + `/nomad/vio_confidence` — drove the vehicle
with `/nomad/cmd_vel` (an ~11 m displacement at ~1.1 m/s against a 1.5 m/s
command). Two real bugs were found and fixed by that proof: the node never set
`vio_sample_.updated`, so every velocity command was refused with "without a
VIO feed" (fixed in `node.cpp`); and the core UDP transport starved heartbeat
consumption under a slow sampler (`wait_for_state` stopped at the first
telemetry frame and only drained the socket when its pending queue was empty,
so a 5 Hz caller fell behind the link and on-demand freshness checks reported
a healthy link as stale — fixed in `src/mavlink/udp_connection.cpp`, with a
flood regression test added to `tests/udp_connection_test.cpp`).

CI switch (2026-09-04): `docker/Dockerfile.sim-ros` now builds the adapter
into the image, and `tests/ros/test_nomad_ros_integration.py` replaces the old
Python-bridge integration test: it runs the real `nomad_vehicle_node` against
an in-process pymavlink MAVLink responder and asserts (1) typed telemetry is
published, (2) the fail-closed VIO gate refuses commands with no VIO feed, and
(3) a healthy VIO feed + `/nomad/cmd_vel` delivers a FLU-to-FRD
`SET_POSITION_TARGET_LOCAL_NED` setpoint (vx ~= 1.0) to the vehicle. Three
integration tests pass inside the image (2026-09-04); `ros-sim.yml` and the
`pixi run test-ros-integration` task now exercise this suite.

Command services under concurrency (2026-09-04): five more integration tests
exercise `/nomad/arm`, `/nomad/disarm`, `/nomad/land` and `/nomad/rtl` against
the stateful responder — each command is sent through the core, its ACK is
observed, and the resulting authoritative state (armed bit / custom mode from
heartbeats) is verified; a `MAV_RESULT_FAILED` ACK surfaces as a failed
service response (8/8 tests pass in the image). Writing these tests exposed a
real transport bug: the 10 Hz telemetry pump could consume a command
acknowledgement before `send_command`'s waiter saw it. The UDP transport is
now thread-safe (`send_command` holds the receive mutex for the whole
exchange; `receive_message`/`wait_for_state`/`get_state` gained locked
variants used by the fence helpers), closing the single-consumer design
ceiling that the CLI's single-threaded loop had never exercised.

The Python `edge_core/ros_http_bridge` package was **deleted on 2026-09-05**
with its host unit tests and the compose bridge services; the deployment tail
was switched to the C++ `nomad_vehicle_node` (systemd unit
`nomad-ros-vehicle.service`, `scripts/services/nomad_ros_vehicle.sh`,
`scripts/nomad` service name `ros_vehicle`, mavlink-router `nav_bridge` leg,
Isaac routes). The only remaining evidence gate is the Jetson/sim-isaac
image rebuild (aarch64/GPU) and a live node run on the device —
environment-gated, see the Phase 7 ledger.

Keep callbacks short, use standard messages, use YAML parameters and launch
files, and call the C++ core instead of packing MAVLink. Use a single-threaded
executor unless measured requirements say otherwise.

SR-SEC-02/03 adapter note: the ROS 2 service/topic surface is trusted to the
DDS domain; production deployments must enable SROS2 DDS security, and the
node's commands inherit the core's audit trail through the core API.

### Mission Planner

Reduce the plugin to presentation, configuration, telemetry, and core-client
calls. Keep GCS-native features only where NOMAD does not own the behavior.

The vehicle-command call sites to move behind the core boundary are enumerated:
`Control/FlightModeController.cs`, `Control/OutputController.cs`,
`Control/GimbalCommand.cs`, `Control/GimbalController.cs`,
`Input/NomadJoystickService.cs`, `MotorMusic/MotorMusicCommand.cs`,
`Geofence/MPFenceUploader.cs`, and `Connectivity/DualLinkSender.cs`.

The Mission Planner-free pure checks (geometry, payload interlock, gimbal
command wire format, altitude callouts, log analysis, dual-link router) and the
dead-code Release compile all pass on the Windows development host (2026-09-03).
`pixi run build-plugin` also passes and deploys NOMADPlugin.dll (about 800 KB)
into the installed Mission Planner `plugins` directory (authorized on
2026-09-03); re-running it replaces the installed plugin.

The first code slice is done (2026-09-04): the core CLI has a `goto <lat>
<lon> <alt>` verb with contract tests and a live SITL phase in
`core-sitl-command-flow` that flies a guided reposition and verifies arrival.
`GuidedGoto` (BoundaryManager soft-boundary return) has moved behind the core
boundary as the first migrated call site through the Mission Planner-free
`NomadCoreClient`, which spawns the C++ CLI with the pinned argument vector and
fails closed on invalid input or an unavailable core. `FlightModeController`
now builds that client from plugin config (`CoreExePath` / `CoreMavlinkEndpoint`
/ `CoreApiKey`); its dead reflection tail, `SetMember`, and caller-less
`SetGuidedMode` were deleted. `pixi run test-plugin-core-client` pins the
boundary (argument vector, invariant formatting, validation gates, and the
live CLI auth gate when the C++ binary is built) and is a new CI step in
`csharp.yml`. The plugin compiles clean under the dead-code gate
(`lint-plugin`).

Output-command slice (2026-09-04): the core boundary grew five verbs —
`servo <channel> <pwm_us>` (DO_SET_SERVO), `relay <number> <0|1>`
(DO_SET_RELAY), `motor-test <instance> <pwm_us> <timeout_s>` (DO_MOTOR_TEST,
timeout clamped 0.05..3.0 s), `gimbal-config <mount_mode>`
(DO_MOUNT_CONFIGURE, MAV_MOUNT_MODE 0..4), and `user-command <7 values>`
(MAV_CMD_USER_1, the motor-music Lua opcode protocol). Each validates input
and verifies the ACK in the core (`src/vehicle/output.cpp`, split from
vehicle.cpp by responsibility); the CLI parses positionally with bounds and
rejects extra values, pinned by `test_core_client_contract.py` (usage surface,
malformed cases, and an actuation-without-key refusal case per verb).
`NomadCoreClient` gained `Servo` / `SetRelay` / `MotorTest` /
`GimbalConfigure` / `SendUserCommand` with fail-closed gates, pinned by
`NomadCoreClientTests` (argument vectors + validation).

Migrated call sites: `OutputController` (discrete servo, relay toggle, and
relay pulse) and `MotorMusicCommand.SendAsync` now route through the core
client first, with the direct MAVLink path kept as a transitional fallback;
the high-rate drag streams (joystick tilt, panel sliders) and the gimbal stick
stream stay on direct MAVLink by design — spawning the CLI per frame would not
scale — with a `debt:` entry to move them behind a core streaming verb later.
The plugin's edge_core REST vehicle-control fallbacks were **removed
(2026-09-05, deletion-gate step 2)**: `OutputController` no longer calls
`/api/servo/channel/{channel}/pwm`, `/api/servo/shooter/arm`, or
`/api/servo/shooter/trigger` — the last C# consumers of the `payload_module`
relay routes and the `calibration.py` direct-servo route. Servo and payload
release now fail closed (false) when neither the core client nor a live
MAVLink link is available, instead of deferring to the transitional Python
vehicle path. `SendServoPwmAsync` is now a plain `Task<bool>` wrapper over the
synchronous core→MAVLink attempt. Remaining C# REST calls on the Jetson
(`/api/servo/camera/*`, `/api/spray/calibration`, video, Isaac, ZED
calibration) are device/camera-side functions of the transitional Jetson
stack, tracked separately below. The plugin's dead `SendMotorTestPwm*`/`MotorTestCommand`
code (no callers) was deleted; the capability is replaced by the core
`motor-test` verb. `GimbalController.SetMode` (mount configure) routes through
the core client. Remaining Phase 6 call sites: joystick mode switches already
reuse `FlightModeController`/`OutputController`, `Geofence/MPFenceUploader`
(stays on MP's native Fence class — its own client code — until the core's
fence verbs gain FENCE_* parameter control), and `Connectivity/DualLinkSender`
(a transport routing concern, not a vehicle command). Live evidence for the
goto wire path is documented in `PLAN.md` (four Copter 4.7.0 wire bugs found
and fixed: COMMAND_INT crc_extra, REPOSITION frame constant, relative-altitude
verification, and the ardupilotmega telemetry layouts).

Quality gates on this slice: `src/vehicle/output.cpp` and
`tests/output_command_test.cpp` split out of files that had crossed 500 lines;
`src/main.cpp` and `src/mavlink/udp_connection.cpp` were compacted back under
(duplicated heartbeat-apply extracted to `apply_heartbeat_locked`; platform
send/recv branches unified where the casts are portable); the ROS integration
test split its wire decoding into `tests/ros/mavlink_wire.py` and its
fixture into named helpers (all functions <= 40 lines, all files <= 500).

### Python

Move CV, ML, simulation, analysis, and utilities under `python/`. Delete the
FastAPI service and Python vehicle-control path after parity is proven.

**Exit:** all clients use one core behavior and no adapter duplicates commands.
The ROS 2 slice is implemented and compile/test-validated; Mission Planner and
the remaining Python callers still sit outside the core boundary.

## Phase 7: cutover and deletion gates

Delete each legacy area only after:

- the C++ or adapter replacement exists;
- unit tests cover normal, invalid, boundary, and failure behavior;
- SITL covers the real MAVLink path where relevant;
- client contract tests use the replacement boundary;
- documentation names the replacement as current;
- no remaining caller depends on the old implementation.

Delete the Python FastAPI app, module registry, REST-only tests, Python MAVLink
path, ROS HTTP bridge, obsolete service units, redundant profiles, module examples,
and conflicting documentation. Do not keep dead compatibility code.

### Cutover inventory (2026-09-03)

The Python vehicle-control surface and its consumers, with the replacement and
the gate that still pins each one:

| Python surface | Owned behavior | C++ replacement | Remaining caller holding the gate |
|---|---|---|---|
| `edge_core/services/mavlink/` (`connection.py`, `commands.py`, module) | Transport, packets, arm/mode/takeoff/land/RTL commands | `src/mavlink/*`, CLI verbs | ~~C# plugin REST calls~~ **gate closed 2026-09-05** (see `src/main.cpp` CLI; remaining Python route tests only) |
| `edge_core/services/payload_module.py` | Relay/servo actuation routes | `src/safety/payload.cpp`, `payload-demo` CLI | ~~C# plugin REST calls~~ **gate closed 2026-09-05** (remaining Python route tests only) |
| ~~`edge_core/ros_http_bridge/mavlink_velocity.py`~~ | Direct GUIDED velocity | `Vehicle::set_velocity` + watchdog, `ros2/nomad_ros` cmd_vel | ~~`ros-sim.yml` rclpy bridge integration test~~ **gate closed 2026-09-05** — the bridge is deleted; `tests/ros/test_nomad_ros_integration.py` + `core-sitl-velocity-watchdog` + `tests/sitl/velocity_loop_closure.py` (C++ CLI-driven) pin the C++ path |
| `edge_core/api_routes/calibration.py` direct-servo route | Direct servo PWM override | `src/vehicle/output.cpp`, `servo` CLI verb, `NomadCoreClient.Servo` | ~~C# plugin REST calls~~ **gate closed 2026-09-05** (remaining Python route tests only) |
| `edge_core/api_routes/calibration.py` (camera tilt + spray) | Camera-servo tilt + spray relay config on the Jetson camera stack | CV-side aim stays Python; device-side stack decision | C# plugin REST calls (`/api/servo/camera/*`, `/api/spray/calibration`) — the bridge's tilt polls died with the bridge (2026-09-05) |
| `edge_core/safety/` decision code | Velocity/fence/payload decisions | `src/safety/*` (ported) | `cov-safety` 100% branch CI gate, `tests/test_safety_*.py` |
| `tests/sitl/velocity_loop_closure.py` | Drive-to-boundary loop closure evidence | C++ watchdog stop scenario | **gate closed 2026-09-05** — the scenario now drives the C++ CLI `velocity` verb (motion, watchdog stop, mode gate); Python observer link only |
| `tests/sitl/geofence_containment.py` | SR-FEN-02 containment evidence | C++ projected gate + upload/readback | No C++ drive-to-boundary SITL yet (test only) |
| `tests/sitl/gimbal_mount_control.py` | Gimbal mount command evidence | None needed: ArduPilot owns gimbal; C# `GimbalCommand` is the GCS client | Test harness only |

Ordered deletion plan (each step only after its gates close):

1. Retire the ROS HTTP bridge once `ros2/nomad_ros` runs against the live
   feed and `ros-sim.yml` switches to it; this removes the direct Python
   velocity path and the bridge's REST route calls. Both the live-feed proof
   (2026-09-04, node drove the Copter 4.7.0 SITL vehicle via `/nomad/cmd_vel`)
   and the CI switch (2026-09-04, `Dockerfile.sim-ros` builds the adapter;
   `tests/ros/test_nomad_ros_integration.py` + `ros-sim.yml` exercise the real
   node against a MAVLink responder) are closed. **Closed 2026-09-05**: the
   Python bridge package, its host unit tests, and the compose bridge services
   are deleted; `Dockerfile.jetson`/`Dockerfile.sim-isaac` now colcon-build
   `nomad_core` + `nomad_ros`, the systemd unit is `nomad-ros-vehicle.service`
   (script `scripts/services/nomad_ros_vehicle.sh`), the Isaac routes and
   `scripts/nomad` manage the node, and the loop-closure scenario drives the
   C++ CLI. Remaining evidence gate: rebuild the Jetson/sim-isaac images and
   run the node on a device (environment-gated).
2. Move the C# plugin's vehicle-command and payload calls behind the core
   client boundary (see Phase 6 Mission Planner); this removes the last
   consumers of the `mavlink` and `payload` REST modules. **Closed for the C#
   side 2026-09-05**: `OutputController` no longer calls the edge_core
   vehicle REST routes (servo PWM, shooter arm/trigger); remaining consumers
   of those Python routes are the routes' own tests (`test_api_routes_*`,
   `test_auth_middleware.py`), which are deleted with the modules.
3. Delete `edge_core/services/mavlink/`, `payload_module.py`, the REST-only
   route modules, the module registry, and `edge_core/safety/` after their
   ports pass, dropping `cov-safety` and the `test_mavlink_*`/`test_api_*`/
   `test_client_contract.py` gates with them. The camera-tilt/spray routes
   stay tied to the Jetson camera-stack decision (their row above).
4. Keep the three SITL Python scenarios as test-only evidence while no C++
   scenario covers drive-to-boundary containment; retire them only when a
   C++ scenario (or the C# gimbal client + C++ watchdog) proves the same
   behavior.

Nothing may be deleted while a row above still lists a live consumer or CI
gate. This inventory is the deletion ledger; update it whenever a gate closes.

## Phase 8: hardening

**Current slice:** C++ continuous velocity watchdog, stale-VIO/link/mode/disarm
fault injection, projected geofence validation, payload range/interlock checks,
C++ command-surface scanning, and C++ traceability are implemented and covered
by deterministic tests. The telemetry, command-flow, mission, velocity-watchdog,
geofence, payload, link-loss, and link-recovery SITL scenarios run against
the ArduPilot SITL image (Copter 4.7.1, rebuilt 2026-09-04) and pass locally
(previously verified on Copter 4.7.0, 2026-09-03). The MAVLink codec is now
generated (2026-09-05) from `third_party/ardupilot-mavlink` at the exact
commit Copter 4.7.1 compiles against — see the codec upgrade note below.

When bumping the ArduPilot version (SITL image or firmware target):

1. record the new `Copter-X.Y.Z` tag in `docker/docker-compose.dev.yml` and
   `.github/workflows/sitl.yml`;
2. update the submodule pin to the tagged release's `mavlink` commit:
   `git -C third_party/ardupilot-mavlink fetch && git checkout <commit>`
   (read the tag's `modules/mavlink` hash in the firmware checkout);
3. regenerate the codec: `pixi run generate-mavlink` (build-core does this
   automatically when the pin changes);
4. re-run `pixi run test-core` (golden wire frames must stay byte-identical)
   and the `core-sitl-*` battery; update any golden that the new dialect
   legitimately changes after review.

### Codec provenance

`src/mavlink/protocol.cpp` and `fence.cpp` compile against headers generated
by `scripts/dev/generate_mavlink.py`, which runs the pinned submodule's own
mavgen (`--lang C`, `--wire-protocol 2.0`, `ardupilotmega.xml`). Generated
headers live in `build/generated/mavlink` (gitignored) with a `provenance.txt`
recording the source commit. NOMAD still owns framing, CRC verification,
sequence numbers, and the payload round-trip (`src/mavlink/generated.hpp`);
the generated encoders trim trailing zero bytes, so the round-trip helper pads
the payload back to the full generated length before NOMAD frames it.

Remaining hardening:

- Run CTest, Python tests, ROS 2 tests, SITL, and plugin checks.
- Add C++ adapter feeds for VIO and payload I/O, then prove them in SITL/hardware.
- Add failure injection for relay-on/relay-off failures and link shutdown.

**Payload failure slice:** deterministic fake-transport tests now prove that a relay-on
failure still attempts de-energization and that a relay-off failure is returned as a
failed release. Hardware/SITL payload evidence remains open.

**Fence upload slice:** `Vehicle::upload_fence` sends a validated polygon through
the MAVLink mission protocol (fence mission type, vertex count in `param1`), and
`verify_fence_uploaded` downloads and compares every point and reads
`FENCE_ENABLE` back through the PARAM_REQUEST_READ/PARAM_VALUE conversation
(`src/mavlink/params.cpp`), failing closed when the parameter cannot be read
or is not 1 — a plan that is not enabled is no fence. The C++ codec's
dialect tables and per-message pack/decode are generated by mavgen from the
pinned `third_party/ardupilot-mavlink` submodule (the exact commit the
ArduPilot firmware compiles against), and the wire bytes are pinned by
pymavlink-golden unit tests. Live SITL evidence on Copter 4.7.1 (2026-09-05,
re-verified on the generated codec): ArduPilot answers `MISSION_ACK:
TYPE_FENCE: ACCEPTED`, the CLI readback reports `fence upload verified`,
and FENCE_ENABLE is read back as 1; the scenario restores the prior
FENCE_ENABLE value so a shared dev vehicle keeps its state.
- Use the C++ fence upload/status verification path, then record live ArduPilot readback before autonomous flight.
- Review every dependency and remove those that do not reduce complexity.
- Split remaining files and functions above the project targets.

## Safety porting checklist

For every current safety requirement:

1. copy the stable requirement ID;
2. identify the C++ owner;
3. add unit tests for invalid and boundary inputs;
4. add adapter transmission tests;
5. add SITL evidence where possible;
6. update `docs/safety.md`;
7. delete the Python implementation only after all gates pass.

## Completion criteria

The migration is complete when:

- the root C++ project builds with one command;
- `nomad status`, `arm`, `takeoff 10`, and `land` work against SITL;
- CLI, ROS 2, and Mission Planner use one core behavior;
- transports do not change the `Vehicle` API;
- Python is limited to tools and experimentation;
- no new source file reaches 500 lines without a temporary reviewed exception;
- no normal function exceeds about 40 logical lines;
- docs, tests, build files, and runtime layout describe the same product.
