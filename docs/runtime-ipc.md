# Runtime IPC contract

The persistent `nomad-runtime` owns all production vehicle writes.
[Architecture](architecture.md) defines ownership; this document defines protocol v1,
outcomes, actuator requests, authentication and durable evidence.

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
| `land` | Bounded Copter LAND engagement; accepted ACK plus a fresh later LAND heartbeat, not touchdown or termination |
| `set_servo` | Calls `Vehicle::set_servo` with channel and PWM microseconds |
| `set_relay` | Calls `Vehicle::set_relay` with relay number and boolean state |
| `motor_test` | Calls `Vehicle::motor_test` with instance, PWM microseconds and timeout seconds |
| `configure_gimbal` | Calls `Vehicle::configure_gimbal` with mount mode |
| `set_gimbal_target` | Calls `Vehicle::set_gimbal_target` with finite `pitch_deg` and `roll_deg`; pitch is limited to -90..90 degrees and roll to -30..30 degrees |
| `configure_gimbal_target` | Calls `Vehicle::configure_gimbal_and_set_target` with `mount_mode`, `pitch_deg` and `roll_deg`. It validates all fields before writing, configures the mount first, and sends the target only after the mode command succeeds. The pair is one authenticated, deduplicated request; a failed or unknown result is never replayed by the client. |

Other vehicle navigation requests are absent from protocol v1. The
two-point QuadPlane fixed-wing route is qualified in the core, but it is not
exposed through runtime IPC v1. The installed CLI rejects `goto` during argument parsing; the non-installed
qualification driver retains direct
navigation for its SITL evidence.

### Copter LAND engagement

`land` accepts no operation parameters and inherits the authenticated mutation contract.
Clients require `land` in a fresh authenticated HELLO; there is no direct fallback.
Unknown additive metadata stays ignored; known operation parameter fields are rejected.
The runtime rechecks the original Copter identity/session, heartbeat freshness and
deadline at every covered send/retry through existing authority/audit admission.

One three-second monotonic budget covers ACK and observation. Success requires an
accepted ACK followed by a newer same-aircraft/session LAND heartbeat before expiry:
**LAND mode observed; touchdown not verified**. It proves neither continued LAND,
exclusive control, descent, touchdown nor termination. Negative ACK is `failed`;
missing ACK, unverified engagement or admission expiry with intact context is `unknown`.
Known authority/identity/session/lifecycle loss is `interrupted`; pre-dispatch refusal
is `rejected`. ACK evidence stays separate. No timeout/disconnect/revoke causes replay
or an undo command. Safe actuator requests retain their original expiry after waiting
behind LAND. Scheduling, IPC and audit I/O overhead are measured separately; this is
not a hard real-time response guarantee.

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
is unsupported and rejected by current settings validation.

## Vehicle mutation outcomes

Vehicle mutations use the existing additive protocol v1 `outcome`
field on both `command_response` and error envelopes. `ok` describes the
response envelope, not vehicle success. `error.code` explains why normal
completion was unavailable; it does not classify transmission. Authority
admission, revocation and handback are runtime state transitions and retain
their separate `authority_response` contract.

Protocol v1 already carried `outcome` on vehicle command responses; extending
it to error envelopes and preserving it in updated clients is additive. Classification remains explicit in protocol v1. Missing classification
from an older runtime remains unknown to an updated mutation client.

| `outcome` | What NOMAD knows | Transmission guarantee |
|---|---|---|
| `success` | The operation's existing software success criterion was satisfied | A valid command result was obtained; physical effect is not guaranteed |
| `rejected` | The request never became eligible for vehicle transmission | Definitely no eligible vehicle send for this attempt |
| `failed` | A legitimate attempt produced a definite unsuccessful result | The command may have been sent; a negative FC ACK proves it was received |
| `interrupted` | Authority, session or lifecycle changed after possible execution | Specialized unknown: final vehicle effect cannot be asserted |
| `unknown` | The final result cannot be established | Vehicle transmission is possible; missing evidence is not a rejection |

Current servo, relay, motor-test, gimbal configuration and gimbal target
operations succeed on command-protocol acceptance. They do not observe servo
travel, payload release, motor motion or gimbal arrival. `command_result.success`
retains the Vehicle result and `command_result.acknowledged` independently
records whether the FC responded, including a negative ACK. Error envelopes
after execution retain available command evidence as well as `outcome`.
An interrupted response can therefore retain a successful/acknowledged command
result while its authoritative request outcome remains `interrupted`.

For example, authority loss after possible transmission returns:

```json
{
  "protocol": "nomad-core",
  "version": 1,
  "id": "release-request",
  "ok": false,
  "error": {
    "code": "authority_interrupted",
    "message": "authority changed during vehicle operation"
  },
  "outcome": "interrupted"
}
```

The durable `mutation_outcome.result` and response `outcome` use the same
classification. Pre-execution rejection records also expose `result: rejected`
while retaining the specific rejection reason. An exact request ID still in
progress reports `request_in_progress` / `unknown`: this duplicate does not
execute again, but the original operation may already have been transmitted.
Its journal record retains that uncertainty. If an outcome cannot be durably
written after a possible send, the response is `audit_failure` / `unknown`;
the remaining intent is incomplete evidence, interpreted as unknown, rather
than a fabricated durable outcome. If no send was eligible, audit failure is
`rejected`. The audit health latch and durable-intent-before-execution rule
remain in force.

The pinned MAVSDK admission callback runs at preflight as well as send time.
Cancellation before any admitted callback or ACK is `rejected`. Cancellation
after admitted preflight alone cannot establish zero transmission and remains
`unknown`, or `interrupted` if authority changed. Independent zero-wire tests
do not manufacture stronger evidence inside the runtime.

Mission Planner preserves the five outcomes, plus `NotAttempted` for local
validation and `FailedBeforeSend` for failure before the mutation request write
begins. A socket failure after write begins, mismatched response, or absent or
unrecognized mutation outcome is `UnknownOutcome`; no error-code fallback
guesses rejection. A nullable acknowledgement property preserves unavailable
versus observed ACK evidence. Authority responses are interpreted separately.

Cached responses retain their original outcome and command evidence while the
request context remains current, including after audit health latches false.
Authority validation still precedes cache retrieval. An interrupted request
whose generation changed is consequently rejected on a later stale request,
without executing again. Eviction never clears the sequence high-water mark.
No mutation outcome authorizes automatic retry without explicit higher-level
reasoning. In particular:

- success != guaranteed physical effect;
- acknowledged != physical completion;
- unknown != failed;
- unknown != rejected;
- interrupted != safe to retry automatically.

Mission Planner's asynchronous request API returns an immutable
`NomadCoreRequestResult` containing `Outcome`, `ErrorCode`, `Message` and nullable
`Acknowledged`; `Succeeded` means only software success. Production callers use
the result of their own request, including when responses finish out of order.
The synchronous bool API and mutable `Last*` properties remain compatibility
wrappers only; asynchronous requests do not update those properties.

Runtime networking uses .NET Framework 4.8 asynchronous TCP connect, stream
write and stream read operations. Cancellation closes the request's socket.
Connect, write and read-idle deadlines remain 1500 ms, 3000 ms and 120000 ms.
Buffered reads retain unconsumed bytes between frames and use one cancellation
source per response, resetting its idle deadline for each network read.
Cancellation or failure before mutation write begins is `FailedBeforeSend`;
once write begins it is `UnknownOutcome`. Neither uncertain nor stale mutations
are retried automatically. UI callers await on the WinForms context, and gimbal
requests retain their existing nonwaiting gate. Configured actuators use semantic
requests; the backend owns concurrency, authorization, output values, sequencing
and recovery. Physical HID input validity is checked before transmission; a stale
unsent observation is `NotAttempted`. Input changes do not cancel an already-started
mutation. Presentation uses backend revisions and retires old runtime incarnations.

## Semantic actuator requests

The additive v1 capabilities are `get_actuators`, `configure_actuators` and
`actuator_action`. Discovery is authenticated; configuration and actions retain the
ordinary admitted owner/session/generation, sequence, expiry, cache and durable-audit
boundary. Raw servo/relay requests reject configured logical outputs.

`get_actuators` returns `actuators_response`, the runtime incarnation, configuration
recovery flag and entries with stable IDs, names, backend-built action labels/control
types, configuration data and software state. No frontend needs to know channels/PWM
to operate an entry. Position actions also provide `continuous_axis_allowed` and
`continuous_axis_blocked_reason`; missing metadata is not permission to stream.
Every full catalog includes `actuator_configuration_revision`, captured with its
array under the backend state lock and incremented on definition replacement, including
an empty replacement. Frontend catalog ordering uses this server revision within a
runtime incarnation, not the client request sequence; missing revisions fail closed.
The runtime rejects HID position requests for hazardous or confirmation-required
definitions independently of frontend controls. UI position requests retain discrete
confirmation support. `configure_actuators` carries a typed `actuator_configs` array;
the schema is the complete set in [ActuatorDefinition](../include/nomad/runtime/actuator.hpp).
The protected file wrapper contains only that array. Unknown/missing/wrong-typed fields,
duplicate IDs/outputs and invalid bounds fail visibly; no automatic remapping occurs.

`actuator_action` carries `actuator_id`, `operation`, `input_source` (`ui`/`hid`),
HID `input_slot` (0–31), and `value` only for normalized position. Operations are
activate/toggle/position/positive/negative/safe/stop, actual HID neutral and backend-
provided `release_input`. Neutral never sends a vehicle command. Directional release
preserves partial confirmation while idle; during pending/active/uncertain motion it
performs an ordinary authorized stop. It never manufactures neutral readiness.

`actuator_response` separates `request_result.success` (semantic request result),
`execution_attempted`, `command_result.success`/`acknowledged` (observed software
command facts), and `actuator_state`. Accepted confirmations send no vehicle command.
State includes confirmation remaining, recovery/pending, commanded normalized position,
activation-commanded, software command success, pulse-ON success, exact activation/safe
outcomes and a monotonic revision. None proves physical actuator state. A staged action
with successful ON and rejected OFF has a failed composite disposition, not a claim
that no vehicle mutation occurred. Every unsuccessful required OFF latches recovery;
only an explicit successful safe action clears it. Initial unknown/interrupted ON
also requires recovery, without automatically issuing another mutation.

Semantic clients use the existing maximum five-second request validity. Pulse/directional
software waits are 50–1500 ms, admitted only with a 3000 ms ON ACK budget plus wait and
250 ms margin. Both edges retain the original final-send token; expiry/revocation is
never bypassed for OFF. Explicit safe requests can interrupt the wait and retain their
own normal expiry/authority checks. Configuration changes report disk uncertainty
separately, without inventing vehicle admission/ACK. See [operations](operations.md#current-configuration)
and [backend fault tests](../tests/actuator_test.cpp).

debt: at most eight actuators and the existing 32 IPC workers bound explicit safe
waiters; revisit if measured concurrent safe requests exhaust workers or delay
authority/status; then permit one pending explicit safe request per logical output.
Longer actuator runs require a separately reviewed operation/authority lifetime.

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
in its masked settings field. Current settings reject `CoreApiKey`; portable
exports omit the separately provisioned client credential. The actuation-enable
gate must never be used as a client credential.
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
SYSTEM/administrator DACL and rejects reparse points.
Windows creation explicitly sets the current process user as owner, including
elevated processes whose OS default owner may otherwise be Administrators.
Existing objects must have that same owner and only owner/SYSTEM/administrator
allowed ACEs. This remains outside an administrator-compromise defense. These are OS/filesystem
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



## Authority request context

The runtime starts at generation zero with no owner. Each typed mutation includes
`runtime_incarnation`, `vehicle_session`, `authority_generation`, `command_source`,
a positive monotonically increasing `sequence` and absolute `expires_at_ms` no more
than five seconds ahead. The authenticated `hello.authority.next_sequence` is the
minimum next sequence. Reservation precedes dispatch; response eviction does not
clear the high-water mark. Duplicates retrieve cached responses only under the same
current authority; evicted replay is rejected.

Initial startup requires explicit `admit_authority`. After prior admission/session
loss or revoke, explicit `handback_authority` admits a new generation. Reconnect never
admits itself. Session mismatch revokes during hello/status/mutation/admission, without
waiting for the monitor loop. Runtime replacement creates a new incarnation and ownerless
state; no prior operation is restored.

Mission Planner coordinates sequence allocation and a nonwaiting mutation gate by
endpoint/client identity across instances in one process. Authority controls bypass
the vehicle-mutation gate so revoke can interrupt an executing operation. Separate
processes sharing an identity are unsupported. Authentication does not identify a human.

The pinned MAVSDK hook checks captured authority before initial COMMAND_LONG/COMMAND_INT
send, every retry and posted UDP delivery. Revocation shares this send fence. Fence
transfer and Offboard setpoints are outside production IPC and require their own
per-frame admission before future exposure.

## Remaining ownership limits

One admitted software source does not establish one writer for the aircraft. Native
GCS, RC/pilot and maintenance tools remain independent. The ground router rejects outbound
traffic from its receive-only Mission Planner consumer; consumer IDs do not authenticate
external sources. Direct qualification is non-installed and inhibited by an occupied
runtime port or `NOMAD_INTEGRATED_FLIGHT`. Apart from Copter LAND engagement, runtime IPC exposes no mission/navigation,
QuadPlane, authoritative geofence or VIO request. Configured actuator operations and
primitive outputs remain as specified above. Physical pilot arbitration and total C2-loss
behavior require the separate [safety procedure](safety.md#controller-bench-and-aircraft-procedure).
