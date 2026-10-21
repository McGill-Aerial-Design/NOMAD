# Plugin-owned ground link router

Mission Planner's NOMAD plugin owns this C# engine; no standalone router process,
management TCP service or second backend consumer ships. Physical links and the
preferred ID are saved per aircraft profile. `MAVLinkConnectionManager` constructs
one bidirectional loopback consumer named `mission_planner`; use UDPCl from MP to
the profile's `RouterLocalPort`. An empty physical-link list opens no connections.

The engine retains UDP/TCP/serial transport, MAVLink frame parsing, link health,
active-link failover/recovery, duplicate suppression and parameter-transaction pinning.
Commands leave on one selected link. A failed send is reported without engine-level
frame retry; MP owns higher-level command retries. Link liveness is based on received
traffic and does not prove command authority or physical aircraft outcomes.

The manager disposes old sockets/workers before replacing routing settings. Unchanged
settings keep physical links running. Disconnect MP before changing the selected
aircraft or route settings; reconnect explicitly and verify the live aircraft. A
link that has no inbound UDP peer cannot deliver outbound data unless RemoteHost
and RemotePort identify the reviewed peer. TCP and COM require explicit endpoints.
Never configure a physical endpoint back into the router's own loopback consumer.

`pixi run test-ground-router-logic` compiles the engine and plugin owner and exercises
real loopback sockets: frame splits/batching, dedup, failover, parameter pinning,
ownership replacement, UTC jumps, reconnect and bounded load. These tests do not
qualify ELRS/LTE RF links, physical takeover or aircraft failsafes.

See [architecture](../../../docs/architecture.md) and
[operations](../../../docs/operations.md) for current product ownership.
The separate [aircraft serial router template](../mavlink_router/main.conf) remains
an aircraft deployment concern and is not installed by this plugin.
