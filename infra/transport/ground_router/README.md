# Standalone Ground Multi-Link Router

This Windows/.NET Framework process is the sole NOMAD ground-routing lifecycle
owner. Mission Planner uses its loopback management API and does not compile or
start the router data plane. There are no Mission Planner assembly dependencies
in this directory. Product ownership is defined in
[architecture](../../../docs/architecture.md); deployment policy is in
[operations](../../../docs/operations.md).

## Build and run

```powershell
pixi run build-ground-router
build/ground-router/nomad-link-router.exe build/ground-router/router.example.json
pixi run test-ground-router
```

Build requires Visual Studio MSBuild/Roslyn and the .NET Framework targeting pack.
The output includes `Nomad.LinkRouter.dll`, `router.example.json`, and this
README; keep them beside the executable when distributing the host.
No install, service registration, deployment or aircraft connection is performed
by the build or tests. The smoke test uses only loopback peers.

The host accepts `status`, `select <id>`, `auto`, and `stop` on standard input;
Ctrl+C also stops it. An invalid selection prints `REJECTED`. Closing standard
input does not stop an independently supervised host. SIGKILL/process termination
relies on OS socket cleanup; `stop` exercises orderly worker shutdown. While the
host is running it also exposes the loopback-only versioned management endpoint
described below.

## Topology and socket ownership

Historically, plugin-owned `GroundLinkRouter` bound LTE UDP `14560`,
RadioMaster UDP `14550` (or opened a TCP/COM connection), and loopback UDP `14600`.
Mission Planner used UDPCl to `14600` from an ephemeral source port; the router
remembered only the most recent sender. `NomadCoreClient` separately launched
one-shot MAVSDK CLI processes whose default listener was also `14550`, conflicting
with RadioMaster when run on the same computer. The aircraft-side
`infra/transport/mavlink_router` configuration belongs to a different host.

The [example](example.json) has the following ownership on the ground computer:

| Endpoint | Binder | Direction |
|---|---|---|
| `0.0.0.0:14560` | Router, physical `lte` | Aircraft traffic in; selected outbound replies to learned peer |
| `0.0.0.0:14550` | Router, physical `radiomaster` | Same, independently monitored |
| `0.0.0.0:14570` | Router, physical `wifi` | Same, independently monitored |
| `127.0.0.1:14600` | Router, consumer `mission_planner` | MP UDPCl receives telemetry only; unsafe outbound configuration is rejected |
| Ephemeral MP port | Mission Planner | Receives telemetry and sends native GCS MAVLink |
| `127.0.0.1:14602` | Router, consumer `nomad_core` | Sends downlink to `14601`; accepts outbound only from `14601` |
| `127.0.0.1:14601` | One persistent C++ runtime or one exclusive direct CLI process | `udpin:127.0.0.1:14601`; MAVSDK learns router peer `14602` |
| `127.0.0.1:14610` | Standalone router management server | JSON Lines status, events, and safe link-selection controls |

Run one standalone host with the configured physical links and consumers, then
connect native Mission Planner using UDPCl to `14600`. The plugin always observes
the host through the management endpoint and reports unavailable/stale status if
that endpoint cannot be reached. Closing Mission Planner does not stop the host.
There is no embedded mode or plugin-owned routing lifecycle.

## Local management protocol

The standalone host binds TCP `127.0.0.1:14610` by default. The bind address is
validated as IPv4 loopback; non-loopback management binds are rejected. The
protocol is UTF-8 JSON Lines with one request or response per newline and a
maximum encoded message size of 64 KiB. Every request identifies
`protocol = "nomad-link-router"` and `version = 1`; incompatible requests receive
a structured error and do not reach the router worker.

The version-1 operations are `hello`, `get_status`, `get_links`, `select_link`,
`set_auto`, `subscribe`, and `ping`. Requests carry an optional `id`, plus a
`type`; `select_link` carries a stable `link` ID. Responses carry `type`, `ok`,
and the request `id` when supplied. Status responses contain router state,
configured/connected counts, active and manual IDs, automatic-failover and
preferred-link state, a UTC timestamp, and an entry for every configured link.
Each link reports its stable ID, display name, transport/endpoint, enabled/open/
connected state, health, packet and heartbeat ages, loss estimate, data rate,
heartbeat count and jitter, RSSI values, and received/forwarded/duplicate frame
counters.

`subscribe` enables bounded event delivery for `link_health_changed`, `failover`,
`active_link_changed`, and `router_stopping`. Health events are coalesced per
client; slow or disconnected management clients cannot block the MAVLink data
plane. The only live mutations are selecting an enabled stable link and releasing
that selection with `set_auto`. The API has no raw MAVLink operation, command
admission, mission control, parameter policy, or aircraft-control authority.

When enabled, the plugin connects through this API. It reconnects in the background,
marks data stale after a bounded silence, and leaves the router process running if
Mission Planner closes. Structural settings such as link endpoints, consumers,
deduplication, preferred-link policy, and the management port require a host
restart. Loopback is a trust boundary, not authentication; the process should be
run only by the intended local user.

## Configuration

The standalone host's JSON is authoritative for physical links, local consumers
and routing policy. Mission Planner separately stores only its client endpoints
and UI preferences. `Links` contains between 1 and 64 entries, at least one
enabled. Each link has:

- `Id`: unique, nonempty, case-sensitive stable string; no enum extension required.
- `Name`: optional display name; `Enabled`: default true.
- `Transport`: `UDP`, `TCP`, or `COM`.
- `Port`: UDP listener port or TCP server port; `BindAddress`: IPv4 UDP bind address.
- `RemoteHost`: optional UDP hostname/address, or required TCP hostname/address.
  UDP hostname lookup is asynchronous and must resolve to IPv4.
  A configured UDP peer also requires `RemotePort`; otherwise replies use the
  latest UDP sender. Configure a fixed peer when source restriction is required.
- `Device` and `BaudRate`: serial port configuration for `COM` transport.
- `Priority`: higher is preferred; `ReconnectSeconds`: positive retry interval.

`PreferredLink` optionally ranks one enabled ID ahead of configured priorities.
Set it to `""` when selection should use priorities alone. Historical names such
as `LTE` and `RadioMaster` are ordinary stable IDs, not a transport model.
`Links` is required. Legacy `LteBindPort`, `RadioBindPort`, COM/TCP, `LocalPort`,
and top-level bind-address settings are rejected; move physical links into
`Links` and local endpoints into `Consumers`.

`Consumers` contains 1–32 entries with unique `Id`, `RouterPort`, and optional
`ClientPort`. `AllowOutbound` defaults to true; false lets a consumer receive
telemetry while preventing its local MAVLink frames from reaching physical
links. The exact ID `mission_planner` is reserved as receive-only: an explicit
entry with `AllowOutbound` true, including an omitted field that defaults true,
is rejected before sockets open. `Consumers` is required. `nomad_core` remains a
separate, command-capable consumer. Every router consumer socket binds IPv4
loopback. The management listener also binds IPv4 loopback unconditionally;
only its port is configurable. `ClientPort = 0`
learns one loopback peer; another peer can replace it only after three seconds
without traffic from the old peer, resetting parser state. A fixed client port
accepts only that endpoint and receives downlink without first sending anything.
Each consumer owns its own parser. Local commands are never copied to other local
consumers. Exact downlink reflections within the 750 ms dedup window are dropped.
Do not add an external relay that loops traffic back after that window.

Configuration rejects overlapping physical UDP/router/client ports and physical
UDP destinations that point into the local topology through loopback or any local
IPv4 interface address. The same check runs after hostname resolution and reads
the current interface addresses on each validation/reconnect. Port binds are exclusive.
A failed physical connection retries independently; a local bind failure aborts
startup and closes resources already opened. Missing consumers do not stop delivery.
Structural configuration changes require restart; per-link settings are copied
at construction. Local loopback access is a trust boundary, not authentication.

The standalone host JSON is authoritative for `Links`, `Consumers`, and the
preferred ID. Mission Planner's `RouterLocalPort` and management port configure
its local client endpoints only; changing the MP UDP port also requires changing
the matching host consumer port. Old plugin settings are not copied into the
standalone host. The default C++ runtime listener is `udpin:127.0.0.1:14601`,
fed by the `nomad_core` consumer at `14602`. One-shot CLI processes must not run
concurrently on the same listener.

## Selection, delivery, and transactions

All enabled links are read by one owned worker with bounded polling batches.
TCP connect attempts are asynchronous; per-link reopen state and parser resets
are independent. Complete frames establish freshness, not arbitrary incoming
bytes. UDP silence expires after `HeartbeatTimeoutSec` (default 3 seconds).
Sequence gaps are tracked per physical link and `(sysid, compid)`; duplicates
and backward/out-of-order sequence deltas are ignored. `LatencyMs` is retained
for compatibility and measures heartbeat interval deviation, **not RTT**.

Automatic selection ranks usable, non-critical links ahead of critical ones,
then preferred ID, priority, and ordinal ID. A stale active link is replaced
immediately by the best receiving link, regardless of cooldown. Healthy-link
recovery requires `PreferredLinkReconnectDelaySec` (default 10) continuously
healthy plus `FailoverCooldownSec` (default 2). `AutoReconnectPreferred = false`
retains a usable active link. Manual selection pins an enabled ID; unknown or
disabled IDs fail without changing state. A dead manual selection drops outbound
rather than silently using another link. `auto` releases the pin.

Only active-source telemetry is distributed, apart from pinned parameter replies.
A bounded cache records frames actually delivered, including their complete v1/v2
bytes, identity, sequence, payload, checksum and signature. Cross-link copies are
suppressed for 750 ms across handovers; standby-first copies cannot suppress an
active delivery. Same-link repeats retain upstream semantics. At 16,384 cache
entries new telemetry is dropped until entries expire. The framing parser does
not validate CRC/signatures; endpoint software remains responsible for validation.

debt: bounded polling and 16,384 recent frames; revisit when measured traffic
exceeds 20,000 frames/second or link count exceeds 64; then qualify queued I/O
and explicit per-consumer backpressure against the same no-fan-out tests.

**Outbound MAVLink is sent through one selected physical link and is not broadcast
across all healthy links.** A failed send is not retried on another transport:
partial TCP/serial delivery may already have happened. The caller decides retries.
Before the first telemetry frame, an open TCP/COM or configured UDP destination
can carry initial GCS announcements on one selected link. A previously receiving
but now stale link does not use this startup exception.

`PARAM_*` and `PARAM_EXT_*` requests/writes pin one physical link globally across
consumers. Parameter traffic stays pinned across automatic/manual changes.
Replies and successful outbound traffic refresh a four-second inactivity lease.
If the pinned link becomes stale, outgoing parameter frames are dropped and do
not extend the lease; after expiry a new request may select the current link.
Unsolicited parameter replies after expiry follow the current active source.
This is not request correlation or concurrent-client arbitration. Mission/fence
and FTP traffic use the active link per frame; no transaction pinning is claimed
for them. Their callers must detect failure and restart/verify exchanges.

## Limits and next step

The router owns transport selection, not command admission, payload policy,
aircraft qualification or mission sequencing. MP native controls, pilot/RC and
ArduPilot remain external authorities. C++ owns NOMAD validation, safety, deadlines
and verified outcomes. The persistent C++ runtime now offers versioned loopback
IPC for typed clients; the runtime IPC overview is in
[docs/runtime-ipc.md](../../../docs/runtime-ipc.md). There is no global
single-writer authority; integrated operation still requires handover and
inhibition.
Shared hardware/power/network paths do not provide independent redundancy.

The versioned local status/config control client is implemented for the
standalone host. The persistent C++ runtime now exposes typed loopback IPC as a
separate command path. Explicit authority handover remains separate work.
QuadPlane fixed-wing route qualification is handled independently from the
runtime and router transport changes.
