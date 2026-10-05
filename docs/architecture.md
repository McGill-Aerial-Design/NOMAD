# Architecture

This document describes the system present at the current branch tip. NOMAD's
production command path is a persistent C++ runtime with local typed clients.
ArduPilot remains responsible for stabilization, EKF, navigation execution and
its own failsafes. The [qualification status](qualification.md) separates tested
software and SITL behavior from aircraft-wide guarantees.

Frontends own presentation and input translation only. All behavior required to
safely and correctly execute a NOMAD operation lives behind the runtime API, so
another frontend can provide equivalent control without reimplementing aircraft,
actuator, safety, recovery, authority, or command-sequencing logic.

Mission Planner reads ordinary USB HID joystick and keyboard input directly. It
discovers configured actuator names, labels, actions and software state through
authenticated runtime IPC and sends semantic operations. Channel/PWM mapping,
confirmation policy, pulse sequencing and safe recovery belong to the runtime.

## Current command and observation paths

```mermaid
flowchart LR
    MP["Mission Planner plugin<br/>UI, management, status"] -->|"supported typed requests"| IPC["Loopback runtime IPC"]
    CLI["Installed nomad CLI"] -->|"typed requests"| IPC
    IPC --> RT["nomad-runtime<br/>authority and request lifecycle"]
    RT --> VP["Vehicle and safety policy"]
    VP --> MAV["One MAVSDK command transport"]
    MAV <-->|"MAVLink over UDP"| GR["Standalone ground router"]
    GR <-->|"selected physical link"| AR["Aircraft-side mavlink-router<br/>(when deployed)"]
    AR <-->|"MAVLink"| FC["ArduPilot"]

    GR -->|"receive-only consumer"| MP
    RO["ROS 2 observer"] -->|"validated GPS and battery"| ROS["ROS topics"]
    GR -->|"separate receive-only feed"| RO
    Q["nomad-qualification<br/>(non-installed test tool)"] -. "direct test transport" .-> MAV
    RC["Pilot / RC / ELRS"] --> FC
    NATIVE["Native GCS or maintenance source"] --> FC
```

Mission Planner's plugin connects to runtime IPC for the supported requests. Its
native telemetry connection can use the router's receive-only
`mission_planner` consumer; the plugin also observes the router through its
separate loopback management endpoint. The installed CLI has no aircraft
endpoint or direct-transport mode. Both clients fail closed when the runtime or
authority is unavailable. Unsupported runtime-v1 requests, including
GuidedGoto, report unavailable without a direct MAVLink fallback.

The diagram shows the ground-station route. The standalone ground router owns
ground physical links and forwards traffic; it does not approve a flight action.
The aircraft-side `mavlink-router` is a separate service that can forward
flight-controller serial/IP traffic on an onboard host. One is not a replacement
for the other. The ground router's `mission_planner` consumer is receive-only,
but its local consumer ID is not authenticated identity and does not arbitrate
other network sources.

The ground router and Mission Planner management client use process-local
`Stopwatch` time for elapsed durations: physical reconnect/opening deadlines,
packet and heartbeat freshness, heartbeat jitter, health, stats/rate windows,
failover cooldown and preferred-link recovery dwell, duplicate/echo expiry,
parameter pinning, learned consumer endpoints, and management polling/freshness.
UTC remains the source for public packet/heartbeat observations, failover events,
management protocol timestamps and operator-visible timestamp projections. Those
UTC fields are presentation/record data and do not drive local timeout decisions.
The clock is injectable internally for deterministic jump and boundary tests;
its monotonic values never cross the management protocol. Existing worker ticks,
socket timeouts and OS scheduling still bound when a deadline is observed.

ROS 2 is an observation-only adapter. It receives a separate MAVLink telemetry
feed because runtime IPC v1 does not expose the source measurements needed for
its GPS and battery messages. It has no command topics or services, no VIO
submission path and no direct `Vehicle` connection. A future ROS request surface
would require a deliberately added typed runtime request and its own authority,
validation and qualification; it is not part of the current architecture.

`nomad-qualification` is built explicitly for deterministic transport tests and
SITL. It is excluded from the installed package and default production build.
Integrated product profiles inhibit this direct test tool. Do not use it as an
operator CLI.

## Runtime and authority boundary

The private `Runtime::Implementation` declaration and owned state live in
`src/runtime/runtime_implementation.hpp`, next to its lock/ownership table.
Member definitions use ordinary source files:

| Source in `src/runtime/` | Responsibility |
|---|---|
| `runtime.cpp` | Lifecycle, callback wiring, IPC dispatch and public forwarding |
| `runtime_authority.cpp` | Authority admission/revocation and vehicle-session fencing |
| `runtime_mutation.cpp` | Mutation execution, deduplication and response caching |
| `runtime_audit.cpp` | Authentication, audit records and persisted outcomes |
| `runtime_status.cpp` | Read requests and status projection |
| `runtime_detail.cpp` | Protocol parsing and shared helper definitions |

The authority gate protects its session, generation, owner and sequence. Vehicle
mutations use the command mutex; response deduplication uses the cache mutex and
releases it before command execution. Nested locks run from command or cache to
authority to the journal's internal mutex. Final-send admission holds authority
and journal locks through the send. Callback captures retain a weak authority
gate and shared journal, while the request context has one thread-local
definition shared by execution and admission. Shutdown fences authority, drains
IPC, joins the connection worker, disconnects, then closes the journal.

debt: two session-loss record constructions and existing function spans up to
53 lines remain; revisit when the session-loss schema changes or request dispatch
grows; then consolidate record formatting and extract focused helpers while
preserving lock scopes and callback captures.

OS service managers own process lifecycle: foreground systemd on Linux and native
SCM on Windows, with console mode retained. Supervisors load protected external
configuration and apply bounded crash recovery; they never admit clients or
restore vehicle ownership. Runtime lifecycle projects existing IPC/audit/session/
heartbeat state. Router ownership remains independent, with no dependency that
restarts it alongside runtime. Mission Planner's loopback IPC requires a Windows
runtime on the same host; onboard Linux placement supports local clients. See
the [deployment matrix](operations.md#supported-deployment-matrix).

`nomad-runtime` owns one long-lived MAVSDK connection and one `Vehicle`. It
serves versioned JSON Lines IPC on IPv4 loopback, `127.0.0.1:14611` by default.
The installed `nomad` CLI and the Mission Planner plugin send typed requests to
that process; each accepted request calls an existing core operation. Runtime
v1 does not expose generic MAVLink commands, navigation requests, mission
execution, velocity, fence transfer or payload release through IPC.

The runtime starts with no admitted software source. Admission binds a client to
the current runtime incarnation and vehicle session. Mutations also carry the
authority generation, source, increasing sequence and short expiry. Revoke and
explicit handback advance the generation; reconnect, cache eviction or runtime
restart cannot restore an old owner or replay an old mutation. Mutating requests
are serialized and a concurrent request can return `busy`. See the full
[runtime IPC contract](runtime-ipc.md).

The local `NOMAD_API_KEY` requirement is a nonempty actuation gate. Per-client
shared-secret credentials authenticate local IPC identity separately; authentication
does not grant software authority. A runtime-owned durable journal records
intent before vehicle execution and observed outcomes. Audit failure inhibits
mutations. See [runtime IPC](runtime-ipc.md#authenticated-local-clients).
IPC is loopback-only; credential confidentiality relies on host access control. One
admitted source is a software boundary for typed requests to this runtime, not a
whole-aircraft single-writer guarantee. Native Mission Planner controls, pilot
and RC/ELRS input, ArduPilot behavior, maintenance tools and unrelated MAVLink
sources remain independent. The router cannot resolve those competing
authorities. Revoke NOMAD and use the approved external procedure before
takeover; reconnection alone never performs handback.

The MAVSDK fork carries the operation admission check through `COMMAND_LONG` and
`COMMAND_INT` retry queues to the final UDP delivery gate shared with revoke,
handback and session rollover. Deterministic peer tests observe the resulting
wire frames. This fence is specific to the tested encodings and UDP path; it
does not prove aircraft response or fence every message and transport. Offboard
setpoints and fence transfers are not runtime-v1 requests and are not covered by
the final-send guarantee. TCP/serial transport delivery is not a qualified
integrated command path.

## MAVSDK connection resource lifetime

The persistent connection owns one published bundle: the selected `System`,
Action, Telemetry, MAVLink Passthrough, Geofence, Param and Offboard plugins,
and their nine persistent subscription handles. Candidate construction and
subscription run before publication, outside the resource lifetime lock.
An exclusive section publishes the complete bundle and establishes its
vehicle session. Status and command methods hold the shared lifetime lock while
using resources; command methods retain it through completion as before.
The immutable configured system ID and autopilot component identify every bundle.

Connect/disconnect writers serialize on a separate lifecycle mutex. Discovery
and identity waits can delay another lifecycle writer, but never hold the
exclusive resource lifetime lock or prevent startup IPC from answering HELLO/STATUS.
Retirement takes the exclusive lifetime lock after existing command users finish,
revokes the bundle's callback gate, detaches the bundle, rolls the session and
clears observations. Unsubscription, plugin destruction and endpoint removal
then run outside the lifetime lock. No new reader or command can obtain the
retired bundle after detachment.

MAVSDK unsubscription does not drain callbacks already copied to its user queue.
Each persistent callback captures a shared gate, acquires its mutex and checks
the owner before accessing connection state. Retirement clears that owner under
the same mutex, waiting for entered callbacks. Late callbacks remain inert even
after a replacement bundle or connection destruction. Candidate callbacks are
inactive until publication. Temporary version/fence callbacks already own their
local result state and do not capture the connection.

Nested locks follow lifecycle -> resource lifetime -> callback gate -> observation
-> runtime authority -> audit journal. Callbacks never acquire lifecycle/resource
locks. Runtime copies transport state before acquiring authority locks. Existing
final-send admission, retry cancellation and session fencing remain in place.
IPC availability, vehicle readiness and software authority remain distinct
through discovery, reconnect and shutdown. Publication/retirement retain the
existing synchronous session notification and audit ordering; callback drainage
and durable audit latency can delay readers, without discovery or MAVSDK waits
under the exclusive lifetime lock.

## Component ownership

| Component | Owns | Does not own |
|---|---|---|
| C++ core (`include/nomad/`, `src/`) | Reusable vehicle state, command validation, aircraft-class and operation policy, safety checks and observed software outcomes | UI rendering, ROS types, packet packing outside the MAVLink implementation |
| Runtime (`tools/runtime/`, `src/runtime/`) | Long-lived `Vehicle`/MAVSDK composition, local IPC, request admission, authority lifecycle, generic actuator behavior/authorization/recovery and client outcomes | Pilot/native-GCS arbitration, remote authentication, persistent mission execution |
| MAVSDK boundary and pinned fork (`src/mavlink/`, `third_party/MAVSDK/`) | One production MAVLink transport, MAVSDK calls and reviewed ArduPilot command semantics | Mission choices, NOMAD policy or proof of physical outcomes |
| Mission Planner (`mission_planner/src/`) | Operator UI, status, configuration, router management and supported typed runtime requests | Parallel policy or a fallback vehicle-command path for those requests |
| ROS (`ros2/nomad_ros/`) | Validated receive-only GPS and battery observations | Vehicle commands, VIO submission, mission decisions or actuation |
| Ground router (`infra/transport/ground_router/`, distributed Windows host) | Physical ground links, routing, link health and safe link selection | Flight authorization, command validation or aircraft-wide arbitration |
| Aircraft-side router (`infra/transport/mavlink_router/`) | Serial/IP forwarding on the aircraft-side host | The standalone ground router's multi-link selection or NOMAD policy |
| Qualification tooling (`tests/`, `scripts/dev/`) | Fake peers, deterministic wire fixtures and isolated SITL scenarios | Installed production operation or evidence beyond each test's declared scope |

## Mission Planner video lifetime

Each embedded player and the plugin's HUD player owns one `VideoSession`. The
session owns the worker task, cancellation source, generation and pending frame.
Its states are stopped, starting, streaming, stopping and terminal disposal.
Duplicate starts are rejected while active or stopping. Stop invalidates the
generation, cancels and joins the worker; restart first completes that cleanup.
The worker exclusively owns its disposable native GStreamer pipeline, appsink,
bus and borrowed sample/map lifetimes, including partial startup failures.
NOMAD does not start Mission Planner's shared static GStreamer worker.

Frames are copied before native samples are released. A UI timer takes the
latest owned frame; workers never queue callbacks into controls or wait for the
UI thread. Old generations cannot publish into a new session. The UI owns its
display image, timer, fullscreen controls and any separately launched VLC
process/temporary SDP file. Stop and disposal release those resources. Plugin
exit cancels video views and disposes the HUD owner/subscriptions before other
plugin cleanup. Reinitialization uses a fresh shutdown token; queued load work
checks the token it captured before starting video.

Video control operations run on the creating UI thread. Cancellation can arrive
from another thread: pipeline/process cleanup joins immediately and control
disposal runs on the UI thread. The bounded native sample read observes
cancellation; an in-progress native initialization/state-change call must return
before its worker can finish cleanup. Software tests do not qualify native
decoder performance, a real stream or an OS/native library that never returns.

## Where a change belongs

- Put reusable aircraft behavior, validation and safety policy in the C++ core;
  add focused fake-transport tests in `tests/`.
- Put new client-visible commands behind an explicit, typed runtime IPC request.
  Update the runtime contract, CLI or Mission Planner translation, admission
  tests and failure behavior together. Do not add a client-side MAVLink
  fallback.
- Put Mission Planner rendering and operator interaction in
  `mission_planner/src/`; keep vehicle decisions in the core/runtime.
- Put ROS telemetry translation in `ros2/nomad_ros/`. Do not add command
  interfaces without a reviewed runtime request and explicit qualification.
- Put link parsing, transport selection and failover in the standalone ground
  router. Do not put flight-action authorization there.
- Keep MAVLink/ArduPilot wire semantics in the MAVSDK implementation or its
  pinned fork. Preserve the dependency pin, license notices and tests described
  in [MAVSDK dependencies](mavsdk-dependencies.md).
- Competition-specific server, traffic and scoring behavior belongs in a
  future opt-in application adapter, not the reusable vehicle core. Its current
  external contract and open work are documented in
  [AEAC 2027 integration](aeac-2027.md).

## Profiles and further detail

The repository provides `onboard_companion`, `groundstation_gpu` and
`groundstation_minimal` configuration templates. A profile describes compute
placement and endpoint settings; passing profile tests does not qualify its
hardware, sensor chain or flight behavior. Optional ROS, video and GPU services
remain disabled in the templates until their providers are selected and
qualified. See [operations](operations.md) for current process/configuration
ownership and [development](development.md) for build and test workflows.
