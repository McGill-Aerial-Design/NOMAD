# Architecture

```text
Mission Planner / installed nomad CLI
          | authenticated typed loopback IPC
     nomad-runtime (persistent C++20 process)
          | authorization, sessions, policy, actuator sequencing, audit/recovery
     one MAVSDK command transport
          | standalone ground router: one selected physical uplink
     aircraft serial router (when deployed)
          | ArduPilot: stabilization, EKF, navigation and native failsafes
```

The installed CLI has no aircraft endpoint. Mission Planner translates HID/keyboard
input and presents immutable request outcomes. Reusable vehicle policy, output values,
confirmation interlocks, pulse sequencing, audit and recovery belong to the runtime.
No frontend has a direct-vehicle fallback. ROS/perception/competition adapters may be
added when a real consumer requires them; no such provider ships today.

## Runtime and authority boundary

One runtime owns its transport and local IPC listener. Per-client HMAC proofs bind
identity, incarnation and exact request bytes. Sequence, age, session and authority
generation checks prevent replay. Each mutation acquires command admission; revocation
cancels eligible work and waits for covered sends. An already accepted FC command
cannot be undone by fencing. Durable intent precedes execution; the observed result
is synchronized afterward. Audit faults inhibit further writes. Restart/reconnect
requires fresh authentication and explicit admission or handback.

Actuator definitions are protected runtime JSON, separate from UI input settings.
Definitions, revision, readiness, confirmation progress and software outcome are
queried through IPC. Configuration replacement invalidates old revision/input context.
Safe output establishment and explicit recovery gate activation. The five behaviors
are ServoToggle, ServoPosition, ServoBidirectional, RelayToggle and RelayPulse.
Raw primitives cannot bypass a configured output. [IPC](runtime-ipc.md) owns the wire
contract; [safety](safety.md) owns the obligations and limits.

## Transport and concurrency

MAVSDK owns packing, socket I/O and internal workers. NOMAD owns policy and verified
outcomes. The pinned fork supplies ArduPilot-specific command semantics and a final-send
admission hook. [Dependency provenance](mavsdk-dependencies.md) is mandatory.

Connection candidates are unpublished until subscriptions are ready. Readers hold
owned leases; retirement drains admitted users before subscriptions/plugins are
destroyed. Operations stay bound to aircraft identity and connection session.
These lifetime and authority mechanisms are intentionally retained.

The ground router handles physical links, failover, deduplication and consumer
permissions, not flight authorization. Its Mission Planner consumer is receive-only;
consumer names are not authenticated identity. The aircraft-side router fans serial
MAVLink onto configured network routes. These are distinct roles. Native GCS, RC and
other external writers remain outside NOMAD's software authority guarantee.
Elapsed-time decisions use monotonic clocks; UTC is presentation/evidence data.

## Component ownership

| Path | Responsibility |
|---|---|
| `src/runtime`, `tools/runtime` | IPC, process lifecycle, authorization, actuator storage/sequence, durable journal |
| `src/vehicle`, `src/safety`, `include/nomad` | Aircraft policy, validated operations and authoritative outcome checks |
| `src/mavlink` | Single MAVSDK command transport and qualification receive-only observer |
| `src/qualification` | Non-installed direct SITL/transport driver; no production command path |
| `mission_planner/src` | Runtime client, operator input, status, advisory map and live video |
| `infra/transport/ground_router` | Standalone physical-link router and local management protocol |
| `scripts/release`, `infra/runtime` | Verified versioned deployment, protected service configuration |
| `tests`, `mission_planner/tests`, `docker` | Hardware-free regressions and pinned Copter/QuadPlane qualification |

The direct driver is inhibited while the runtime port is occupied or integrated-flight
mode is enabled. Its QuadPlane navigation, fence and velocity paths remain because
they implement active product requirements with focused safety/SITL coverage; they
are not advertised as production IPC capabilities. There is no generic mission
interpreter or plugin module SDK.

Mission Planner video owns cancellation, frame buffers and decoder/worker lifetimes across
view replacement, disposal and plugin exit. HUD and embedded playback are retained
because both have real host integration and lifecycle tests. UI controls never infer
physical actuator completion from an ACK.
