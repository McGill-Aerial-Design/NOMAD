# Persistent C++ runtime and local IPC

This document records the source ownership at the PR #23 main baseline
(`3cd11aee48b9e844e75829a9ef2d65bc1ecfa1f3`) and the runtime IPC foundation
added from that baseline, followed by the current software-authority foundation.
It does not qualify aircraft-wide manual takeover.

## Ownership before the runtime

At the baseline, `src/main.cpp` created a MAVSDK-backed `MavlinkConnection`
through `make_mavsdk_connection` for each `nomad` invocation. `run_command`
constructed a stack `Vehicle` for a command. The separate `status` path in
`src/cli_commands.cpp` also constructed a stack `Vehicle`; `connect` did not.
The process then exited, destroying the transport and telemetry subscriptions.
The `Vehicle` held a reference to the connection; it did not own it.

`MavsdkMavlinkConnection` owned the MAVSDK instance, connection handle, selected
system, plugins, telemetry subscriptions and mutex-protected latest
`VehicleState`. The `Vehicle` owned its current safety policy objects and
watchdog state. The `MissionExecutor` was a small synchronous helper used by a
CLI demo, not a persistent mission session.

Mission Planner's `NomadCoreClient` created a new OS process for each `goto`,
servo, relay, motor-test or gimbal-config request. Its API key was copied to that
child as `NOMAD_API_KEY`; the C++ CLI only checked that the value was non-empty
before an actuation verb and wrote an audit line. It did not compare a client
credential with a runtime secret or authenticate a user.

The ROS node has a separate ownership path: `nomad_ros` creates its own
`MavlinkConnection` and, after heartbeat discovery, its own `Vehicle` in
`ensure_connected`. ROS is not migrated by this change. Python vehicle-facing
code found in this source review is limited to test/SITL peers, passive
observers and maintenance utilities; there is no Python `Vehicle` runtime.
Mission Planner's native MAVLink functions, pilot/RC and ArduPilot remain
independent command sources.

The standalone ground router's current topology reserves router consumer UDP
`127.0.0.1:14602` and MAVSDK client UDP `127.0.0.1:14601`. Mission Planner uses
router UDP `127.0.0.1:14600`. Router management is TCP `127.0.0.1:14610`.
Runtime IPC uses a separate TCP port, `127.0.0.1:14611` by default.

## Runtime ownership now

`nomad-runtime` is the long-lived composition root. It creates one
`MavlinkConnection`, constructs one `Vehicle` that references it, and keeps both
alive while the process runs. A connection worker retries the same connection
object when startup discovery fails or MAVSDK reports that its system is no
longer connected. Client connect/disconnect does not create or destroy the
vehicle connection. The runtime starts its IPC listener even while aircraft
identity is unresolved, so `STATUS` can report partial startup state.

The runtime loads the existing fence and velocity policies from environment
configuration and passes them to the existing `Vehicle`. Typed command handlers
call `Vehicle::set_servo`, `Vehicle::set_relay`, `Vehicle::motor_test`,
`Vehicle::configure_gimbal` or `Vehicle::set_gimbal_target` directly. The IPC layer
does not pack MAVLink or copy Vehicle capability checks. No Vehicle API,
capability table, transition behavior, route behavior or completion rule is
changed here.

The C++ library, `nomad` one-shot executable, `nomad-runtime` executable and
Mission Planner client now have these modes:

| Client | Mode | Behavior |
|---|---|---|
| Mission Planner | runtime IPC only | Connects to the configured loopback port; never launches the CLI or falls back to native MAVLink/direct vehicle writes |
| C++ CLI | bare verb or `--direct` | Existing one-shot connection and `Vehicle` lifetime |
| C++ CLI | `--runtime` | Sends supported typed requests to the runtime |
| ROS 2 | integrated default | Uses its independent connection for observation; actuation is inhibited |
| ROS 2 | explicit nonintegrated test mode | Owns its independent connection and `Vehicle` |

Mission Planner and the C++ CLI runtime mode support the typed requests listed
below. The protocol does not expose every CLI verb. In particular, there is no
generic command ID, raw MAVLink or shell command. Mission Planner has no
one-shot compatibility mode. GuidedGoto remains unavailable because protocol v1
does not expose a typed navigation request; the plugin reports this and sends no
vehicle command.

## Protocol v1

The endpoint is IPv4 loopback only (`127.0.0.1:<port>`); the default port is
`14611`. The runtime has no configurable external bind address. A newline ends
one UTF-8 JSON object, with a maximum JSON payload of 65,536 bytes. Clients send
one request and wait for its response before sending the next request on that
connection. The server applies a bounded read timeout and a bounded number of
client workers. The local OS account is the trust boundary; loopback TCP is not
multi-user authentication.

Every request carries:

```json
{
  "protocol": "nomad-core",
  "version": 1,
  "client_id": "stable-client-id",
  "id": "stable-request-id",
  "type": "status"
}
```

Responses echo `protocol`, `version` and `id`, and include `ok` plus a response
`type`. Protocol failures include a structured `error` with `code` and
`message`. Unknown request types, malformed JSON, wrong protocol names and
incompatible versions are rejected without dispatch. Unknown additive JSON
fields are ignored. Clients negotiate with `hello` before sending a command.

| Request type | Result |
|---|---|
| `hello` | Protocol/version, runtime version and supported capabilities |
| `ping` | `pong` health response |
| `status` | Runtime readiness; MAVSDK connection open and vehicle transport connected; vehicle session and heartbeat state; identity/class; armed state; current custom mode; valid telemetry fields and their observed ages |
| `admit_authority` | Explicit first admission of one local software source into a new generation |
| `revoke_authority` | Invalidate the current generation and inhibit mutations |
| `handback_authority` | Explicit admission after revocation into another new generation |
| `set_servo` | Calls `Vehicle::set_servo` with channel and PWM microseconds |
| `set_relay` | Calls `Vehicle::set_relay` with relay number and boolean state |
| `motor_test` | Calls `Vehicle::motor_test` with instance, PWM microseconds and timeout seconds |
| `configure_gimbal` | Calls `Vehicle::configure_gimbal` with mount mode |
| `set_gimbal_target` | Calls `Vehicle::set_gimbal_target` with finite `pitch_deg` and `roll_deg`; pitch is limited to -90..90 degrees and roll to -30..30 degrees |

Vehicle navigation requests are intentionally absent from protocol v1. The
two-point QuadPlane fixed-wing route is qualified in the core, but it is not
exposed through runtime IPC v1. Mission Planner's legacy mode retains its
existing goto CLI path.

`STATUS` reports runtime IPC readiness, MAVSDK connection open, vehicle
transport connected, vehicle heartbeat/session, identity resolution and
telemetry validity/age separately. It reports the autopilot's numeric custom
mode; the IPC layer does not infer a safety state or flight-mode name.

Mutating requests use a try-lock command policy: one mutating `Vehicle` call at
a time. A concurrent mutation receives `busy`; it is not queued. Read-only
requests use independent client workers and do not wait on the command lock.
The server limits active IPC clients and per-message size. A slow or disconnected
client does not hold the telemetry callback or vehicle command lock while the
runtime writes a response.

The server records up to 256 completed mutating responses in memory, keyed by
`client_id` and request `id`. Repeating the same ID and payload returns the
recorded response; reusing an ID with different data is rejected. A repeated
request still in progress is rejected as `request_in_progress`. The cache is
bounded and is cleared by runtime restart. The authority generation, runtime
incarnation and monotonic sequence high-water mark now reject stale mutations
even after cache eviction or restart; the cache remains a response optimization,
not durable exactly-once execution. An exact cached response can still be read
after its request expiry while its authority generation remains current; this
does not dispatch another vehicle command.

Disconnecting a client does not cancel an already-dispatched `Vehicle` call.
The runtime completes that call and retains its response when possible. If the
client loses its socket after sending but before receiving the response, its
outcome is unknown. Mission Planner does not retry the request or use a CLI,
native MAVLink or direct-write fallback after that failure. The next user
operation opens a new TCP connection and negotiates again. A runtime restart recreates the MAVSDK
connection and `Vehicle`, clears the in-memory request cache, and does not
resume an in-progress operation.

Mission Planner's gimbal window, arrow keys and physical joystick send angle
targets only as `set_gimbal_target` requests. The runtime/core constructs the
fixed `DO_MOUNT_CONTROL` command after angle validation. Mission Planner keeps
its in-flight drop behavior; runtime `busy`, missing authority and unavailable
runtime outcomes do not trigger a direct MAVLink fallback or replay.

`NOMAD_API_KEY` retains its current limited meaning. The Mission Planner client
requires its configured local gate to be non-empty, and the runtime requires its own
`NOMAD_API_KEY` environment value to be non-empty for mutation. The value is
not sent in IPC and the two values are not compared. This is an actuation gate,
not client authentication; local machine access remains trusted.

## Software authority foundation

The runtime starts with generation zero and no admitted owner. `hello` and
`status` expose its random incarnation, current vehicle session and authority
generation. A trusted local client explicitly calls `admit_authority` with the
current context and its `client_id` as `command_source`. Only one source can win.
`revoke_authority` advances the generation and removes the owner. After a prior
admission, `handback_authority` explicitly admits a source into another new
generation; reconnect alone never does. Loss of the observed aircraft session
also revokes the owner. A session mismatch is checked and revoked during
`hello`, `status`, mutation and transport admission, without waiting for the monitor loop.
The MAVSDK connection also invalidates the shared gate when it changes vehicle
session, before a later queued command can use the new session.
No old mission is restored.

Each typed mutation echoes the incarnation, vehicle session, generation and
source, supplies a positive monotonically increasing `sequence`, and an absolute
`expires_at_ms` no more than five seconds ahead. The runtime reserves each
sequence before dispatch and keeps the high-water mark after response eviction.
`hello` includes the next sequence for the one-shot CLI; Mission Planner uses
a process-lifetime counter and stable logical source across its client instances.
Duplicate requests can retrieve a cached response only while the same authority
is still current. An evicted replay is rejected. In-flight operations return
`authority_interrupted` if authority changes before completion. The runtime
captures the request context for each SDK command operation. `nomad --runtime admit`,
`revoke` and `handback` are explicit local operator controls for the CLI source;
Mission Planner exposes the same deliberate controls on its Core settings tab.
Neither client admits itself on reconnect.

This is local-account trust, not authenticated user identity. A local process
can claim another client ID. The pinned MAVSDK fork checks the captured context
before the first send and each retry of `COMMAND_LONG` and `COMMAND_INT`. Its
posted UDP delivery also runs under the same authority gate used by revoke and
handback. Denied passthrough work returns a distinct admission-cancelled result
to the transport; an interrupted runtime request reports `authority_interrupted` and
never treats an uncertain aircraft outcome as success. Fence transfer and
one-shot Offboard setpoints are outside runtime IPC v1 and need their own
per-frame admission before integrated authority can expose them.

## Remaining ownership limits

This establishes one admitted software source for typed clients connected to this runtime.
It does not establish one writer for the aircraft. Native Mission Planner
MAVLink controls, RC/pilot input, ArduPilot behavior, explicit nonintegrated ROS and
maintenance/test tools remain independent authorities. Integrated profiles
inhibit direct CLI actuation and default ROS actuation. Profile sync writes
`NOMAD_INTEGRATED_FLIGHT` to Mission Planner's `IntegratedFlightMode`; the
embedded router then makes Mission Planner's consumer receive-only. The
standalone router example has the same policy, but other standalone router
configurations, direct Mission Planner links and RC/ELRS are not inhibited.
Aircraft input selection still needs
independent proof. The runtime also does not own a
persistent mission executor or migrate all Mission Planner, ROS or Python
surfaces. The QuadPlane fixed-wing route is qualified in the core and remains
outside runtime IPC v1.
