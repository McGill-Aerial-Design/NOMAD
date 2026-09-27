# Persistent C++ runtime and local IPC

This document records the source ownership at the PR #23 main baseline
(`3cd11aee48b9e844e75829a9ef2d65bc1ecfa1f3`) and the runtime IPC foundation
added from that baseline. It does not change vehicle qualification or command
admission.

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
call `Vehicle::set_servo`, `Vehicle::set_relay`, `Vehicle::motor_test` or
`Vehicle::configure_gimbal` directly. The IPC layer
does not pack MAVLink or copy Vehicle capability checks. No Vehicle API,
capability table, transition behavior, route behavior or completion rule is
changed here.

The C++ library, `nomad` one-shot executable, `nomad-runtime` executable and
Mission Planner client now have these modes:

| Client | Mode | Behavior |
|---|---|---|
| Mission Planner | `LegacyOneShot` | Compatibility default; starts `nomad` for each supported operation |
| Mission Planner | `PersistentRuntime` | Connects to configured loopback IPC port; does not spawn `nomad` |
| C++ CLI | bare verb or `--direct` | Existing one-shot connection and `Vehicle` lifetime |
| C++ CLI | `--runtime` | Sends supported typed requests to the runtime |
| ROS 2 | current adapter | Continues to own its independent connection and `Vehicle` |

Mission Planner persistent mode and the C++ CLI runtime mode support the typed
requests listed below. The protocol does not expose every CLI verb. In
particular, there is no generic command ID, raw MAVLink or shell command.

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
| `set_servo` | Calls `Vehicle::set_servo` with channel and PWM microseconds |
| `set_relay` | Calls `Vehicle::set_relay` with relay number and boolean state |
| `motor_test` | Calls `Vehicle::motor_test` with instance, PWM microseconds and timeout seconds |
| `configure_gimbal` | Calls `Vehicle::configure_gimbal` with mount mode |

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
bounded and is cleared by runtime restart; it is not durable exactly-once
execution.

Disconnecting a client does not cancel an already-dispatched `Vehicle` call.
The runtime completes that call and retains its response when possible. If the
client loses its socket after sending but before receiving the response, its
outcome is unknown. Mission Planner does not retry the request or fall back to
one-shot mode after that failure. The next user operation opens a new TCP
connection and negotiates again. A runtime restart recreates the MAVSDK
connection and `Vehicle`, clears the in-memory request cache, and does not
resume an in-progress operation.

`NOMAD_API_KEY` retains its current limited meaning. The persistent client
requires its configured value to be non-empty, and the runtime requires its own
`NOMAD_API_KEY` environment value to be non-empty for mutation. The value is
not sent in IPC and the two values are not compared. This is an actuation gate,
not client authentication; local machine access remains trusted.

## Remaining ownership limits

This establishes one command owner for typed clients connected to this runtime.
It does not establish one writer for the aircraft. Native Mission Planner
MAVLink controls, RC/pilot input, ArduPilot behavior, the ROS adapter and
maintenance/test tools remain independent authorities. Global handover and
inhibition require a later reviewed slice. The runtime also does not own a
persistent mission executor or migrate all Mission Planner, ROS or Python
surfaces. The QuadPlane fixed-wing route is qualified in the core and remains
outside runtime IPC v1.
