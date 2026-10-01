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

At the starting baseline, `nomad_ros` created its own command-capable
`MavlinkConnection` and `Vehicle`. The ROS architecture slice removes that
path. The shipped node now uses a separate raw UDP `MavlinkObservation` because
protocol v1 status does not include sensor values. Its API and socket expose no
MAVLink send path.
It exposes no flight command topics or services. Python vehicle-facing code
found in this source review is limited to test/SITL peers, passive observers and
maintenance utilities; there is no Python `Vehicle` runtime.
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

The C++ library, installed `nomad` executable, `nomad-runtime` executable and
Mission Planner client now have these roles:

| Client | Mode | Behavior |
|---|---|---|
| Mission Planner | runtime IPC only | Connects to the configured loopback port; never launches the CLI or falls back to native MAVLink/direct vehicle writes |
| Installed C++ CLI | bare verb | Sends typed requests to runtime IPC; has no MAVSDK connection or direct fallback |
| `nomad-qualification` | build-tree test target | Direct MAVSDK/`Vehicle` driver for SITL; excluded from install/default build |
| ROS 2 | telemetry observer | Uses the receive-only `nomad_mavlink_observation` target; publishes validated GPS and battery samples only |

Mission Planner and the installed C++ CLI support the typed requests listed
below. Recognized CLI verbs without a typed v1 request return
`unsupported_request` and do not contact a vehicle. In particular, there is no
generic command ID, raw MAVLink or shell command. Mission Planner has no
one-shot compatibility mode. GuidedGoto remains unavailable because protocol v1
does not expose a typed navigation request; the plugin reports this and sends no
vehicle command.

The direct test driver accepts aircraft endpoint and system-ID options solely
for qualification. It is not installed or used by production clients.

## Protocol v1

The endpoint is IPv4 loopback only (`127.0.0.1:<port>`); the default port is
`14611`. The runtime has no configurable external bind address. A newline ends
one UTF-8 JSON object, with a maximum JSON payload of 65,536 bytes. Clients send
one request and wait for its response before sending the next request on that
connection. The server applies a bounded read timeout and a bounded number of
client workers. Loopback limits exposure to local processes. Privileged requests additionally
require a shared-secret credential bound to one configured identity.

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
exposed through runtime IPC v1. The installed `nomad goto` command therefore
reports unavailable; the non-installed qualification driver retains direct
navigation for its SITL evidence.

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

`NOMAD_API_KEY` remains a nonempty runtime deployment/actuation enable gate.
It does not authenticate identity. Mission Planner's old `CoreApiKey` setting
is retired and ignored; it is never migrated into an authentication credential.

## Authenticated local clients

The pre-slice implementation checked only self-declared `client_id` and
`command_source`. The CLI asserted `nomad-cli`; Mission Planner asserted a
process-generated `mission-planner:<guid>`. Any local process could read status,
copy the owner/context and impersonate that owner. No installed client had a
stronger identity check. The nonempty runtime key and plugin setting were gates.

Protocol v1 is retained with an explicit `hmac-sha256-v1` authenticated-client
extension. A local port-squatting process must not be able to harvest a reusable
bearer credential. `hello` carries a CSPRNG-generated 64-hex `auth_nonce`.
The response advertises `client_authentication: hmac-sha256-v1` and a hex
`server_proof`: HMAC-SHA256 keyed by the client's configured secret over UTF-8
`nomad-core:server:v1:<client_id>:<nonce>:<runtime_incarnation>`. Updated installed
clients verify this proof before sending privileged requests; an old or rogue
runtime is refused. An old client may read hello/status/ping, but cannot mutate.

Each privileged request adds `auth_payload`, the exact serialized unsigned
request JSON string, and `auth_proof`, HMAC-SHA256 of UTF-8
`nomad-core:request:v1:` followed by those exact bytes. Secrets are UTF-8
64-hex strings used as the HMAC key (not hex-decoded). The runtime checks that
parsing `auth_payload` equals the outer request with the two authentication
fields removed. It then verifies the proof against configured secrets and
derives the identity from the matching entry. No raw credential is transmitted;
a legacy `credential` field is rejected. Request-bound proofs cannot authorize
changed requests; existing incarnation/session/generation/expiry/sequence/cache
fences reject replay. The 65,536-byte envelope limit still applies; authenticated
inner payloads are limited to 32,768 bytes and the same depth bound.

Malformed, missing, unknown or mismatched proofs fail closed. `client_id`,
required `command_source`, and optional `source` must equal the identity resolved
from the proof. Auth fields are discarded before cache/dispatch/audit. Updated
clients require the marker and runtime proof; there is no legacy mutation
fallback. Unknown unrelated additive fields remain ignored.

The runtime loads a protected JSON object mapping stable identities to distinct
random tokens at startup. It requires 1..32 identities, unique keys and tokens,
IDs of 1..64 ASCII letters/digits or `-_.:`, and exactly 64 lowercase hexadecimal
characters per token. It computes proofs across all configured secrets and compares 64-hex proofs
with fixed-length XOR accumulation without an early byte exit; this is a practical timing
mitigation, not a formal compiler/microarchitecture guarantee. Configuration
is immutable during runtime; rotation requires restart and fresh admission.

The installed CLI reads `NOMAD_CLIENT_CREDENTIAL` and optional `NOMAD_CLIENT_ID`
(default `nomad-cli`). Mission Planner uses stable identity `mission-planner`
and separately provisioned `CoreClientCredential` (empty by default), entered
in its masked settings field. Profile synchronization removes `CoreApiKey` and
preserves the separately provisioned credential; it no longer copies
`NOMAD_API_KEY` into plugin credentials. Profiles must never save credentials.
Future approved clients may receive their own configured identity/token.

Any configured authenticated client may explicitly revoke software authority,
including another owner's authority. Authentication alone grants no authority:
admission/handback still require fresh aircraft state, matching incarnation,
session, generation and expiry. Mutations still require the authenticated
owner, generation, sequence, expiry and final-send MAVSDK admission.
Read-only hello/status/ping remain unauthenticated on loopback and do not
create ordinary telemetry journal entries. Context fields are not secrets.

The threat model is an ordinary local process that can reach IPC but cannot
read provisioned client credentials. Protect the runtime credential map and
each client's environment/configuration with OS account separation and file
permissions. Shared-secret credentials identify a configured client, not a process binary
or human operator. Processes sharing credentials are the same identity.
There is no claim against root/administrator, kernel compromise, malware that
can read another process's credentials, or physical host compromise. There is
no remote IPC, TLS, OAuth, user account or general permission framework.

## Durable runtime command evidence

The runtime requires `NOMAD_AUDIT_DIRECTORY`, creates a missing final directory
under an existing parent, and exclusively locks it for its lifetime. Each
incarnation exclusively creates `<incarnation>.jsonl`; previous files remain
unchanged. Every JSON line has schema 1, a wall-clock `time_ms`, monotonic
per-incarnation `ordinal`, event and runtime incarnation. Request records
include authenticated client, request ID, requested incarnation/session/
generation, sequence/expiry, operation and normalized typed numeric/boolean
parameters, result and final-send eligibility. Outcome records also contain
observed session/generation. No raw JSON, credentials, signing keys, phrases,
user tokens or telemetry payloads are journaled.

Ordering is authentication, request/authority/cache/replay validation, sequence
consumption, command serialization and revalidation, durable mutation intent,
Vehicle invocation with the existing final-send admission, then durable outcome
before response caching/return. Intent failure prevents vehicle invocation.
Exact cached retries produce no new intent or vehicle call. Evicted replay and
context/authority/expiry rejection are separate rejection records. Cache and
in-flight keys use authenticated identity plus request ID; cache fingerprints
exclude the credential. Replay tracking and this journal have separate roles.

Native writes handle partial writes and synchronize every complete line with
POSIX `fsync` or Windows `FlushFileBuffers`. POSIX creation uses mode 0600,
directory mode 0700, owner-only checks, no symlink/hardlink files and directory
fsync at creation. Windows creation uses an explicit protected current-user,
SYSTEM/administrator DACL and rejects reparse points. These are OS/filesystem
flush semantics, not a promise of power-loss atomicity or hardware persistence;
Windows has no portable directory-fsync guarantee here.

Outcome write failure latches audit health false, returns `audit_failure` with
`outcome: unknown` when a send may have been eligible, and inhibits subsequent mutations
and authority changes until restart. `status.audit_healthy` exposes the latch.
Final-send admission serializes against journal health transitions. An unhealthy
sink cannot reliably journal its own failure; the runtime emits a secret-free
structured `audit_failure` to stderr and never pretends that marker is durable.
Safety-driven session loss and shutdown still revoke authority if auditing fails.

Startup refuses a missing/invalid credential configuration, unavailable audit
file/lock, empty prior journal, malformed complete history, or an unterminated final record. It
preserves damaged evidence for operator investigation; there is no automatic
truncation, replay or repair. This includes a crash between file creation and the first
start record. An operator must preserve and move damaged files out of the active
directory before restarting. A new incarnation always requires fresh admission.
An intent without a matching outcome is interpreted as unknown/possibly sent;
absence of orderly shutdown is an unclean previous boundary. A restart begins
a new file/start record and never replays old work. History validation currently
has a 64 MiB per-file ceiling; archive validated closed files outside the active
directory before that limit, preserving them as evidence.

Events cover runtime start/shutdown, vehicle sessions and session authority loss,
authority intent/admission/revoke/handback, authenticated request rejection,
authentication rejection, mutation intent/outcome and interrupted authority.
`send_eligible` is tri-state: false when no admission check passed; `unknown`
when a check passed but no matching ACK was observed; true when NOMAD observed
a matching ACK. The pinned MAVSDK guard invokes indistinguishable preflight and
send callbacks, so callback admission alone cannot prove actual delivery.
`admission_checked`, `acknowledged` and `observed_command_success` retain that
separate evidence, including interrupted outcomes. None proves physical action. A positive
Vehicle result is only the software observation. Failed/interrupted/unknown
outcomes must not be reported as proof of a physical outcome.

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
`hello` includes the next sequence for each short-lived `nomad` invocation;
Mission Planner uses a process-lifetime counter and stable logical source across
its client instances.
Duplicate requests can retrieve a cached response only while the same authority
is still current. An evicted replay is rejected. In-flight operations return
`authority_interrupted` if authority changes before completion. The runtime
captures the request context for each SDK command operation. `nomad admit`,
`revoke` and `handback` are explicit local operator controls for the CLI source;
Mission Planner exposes the same deliberate controls on its Core settings tab.
Neither client admits itself on reconnect.

The HMAC credential authenticates a configured client identity; it does not identify
a human operator or protect a compromised host. A process without the client's
secret cannot use its authority by claiming its ID. The pinned MAVSDK fork checks the captured context
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
MAVLink controls, RC/pilot input, ArduPilot behavior and maintenance/test tools
remain independent authorities. The ROS observer has no flight command path;
its temporary MAVLink connection receives telemetry only. Integrated profiles
set `NOMAD_INTEGRATED_FLIGHT` to inhibit direct actuation by the non-installed
qualification tool. Profile sync removes the obsolete Mission Planner
`IntegratedFlightMode` field and does not copy the qualification gate there. The
separately supervised ground router enforces its `mission_planner` consumer as
receive-only and keeps `nomad_core` command-capable. An explicit Mission Planner
entry with `AllowOutbound` omitted (default true) or set true is rejected at
startup. Direct Mission Planner links and RC/ELRS are not inhibited.
Aircraft input selection still needs independent proof. The runtime does not
own a persistent mission executor. Mission Planner's supported typed requests
and the installed CLI use this IPC; ROS is observation-only, and Python vehicle
access is limited to test, SITL and maintenance tools. The QuadPlane fixed-wing
route is qualified in the core and remains outside runtime IPC v1. Mission,
navigation, geofence and payload requests are not exposed through the installed
client protocol.
