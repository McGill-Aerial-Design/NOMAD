# Operations

This document covers the target operational model and the current transitional
workflow. Real hosts, credentials, and absolute paths belong in local ignored
configuration only.

## Target runtime

```text
nomad CLI or client
       |
NOMAD core process or library
       |
MAVLink over serial, UDP, or TCP
       |
ArduPilot
```

The MVP uses a one-command CLI. A persistent companion process and remote client
protocol are deferred until an actual client needs them.

## Local build and SITL

The target workflow is:

```bash
pixi run build-core
pixi run test-core
pixi run sitl
# With the SITL stack running:
pixi run core-sitl-status
pixi run core-sitl-command-flow
pixi run core-sitl-mission
pixi run core-sitl-velocity-watchdog
```

SITL should expose a local endpoint such as `udp:127.0.0.1:<port>` or a TCP
endpoint chosen by the simulator. The repository Docker stack emits a dedicated
host UDP telemetry copy on `NOMAD_CORE_SITL_PORT` (default `14570`) for the C++
status smoke test. Use placeholders in scripts and documentation; do not commit
a real address.

Until the C++ core is ready, the current transitional stack is available:

```bash
pixi run dev
pixi run dev-up
pixi run test-fast
pixi run dev-down
```

The transitional stack runs the Python Edge Core and ArduPilot SITL. It is a
migration tool, not the target product architecture.

## Connection types

The core API must not change when the transport changes. Active velocity control
also owns a fail-closed watchdog: missing commands, stale heartbeat, disarm, mode
loss, or stale/low-confidence VIO send a zero setpoint and stop the session.

The core API must not change when the transport changes:

```text
serial:<device>:<baud>
udpin:<host>:<port>
udpout:<host>:<port>
tcp:<host>:<port>
```

Use environment variables or local configuration for device paths, hosts, ports,
keys, and deployment-specific limits. Defaults are for local development only.

`NOMAD_RELAY_ADDRESS` (optional, `udp`/`udpin`/`udpout:host:port`) overrides where
the pre-latch GCS heartbeat is sent. UDP relays such as mavlink-router and
MAVProxy only stream a leg after the endpoint announces itself, so the core emits
a 1 Hz GCS heartbeat until the first vehicle datagram latches the peer. The
announcement target defaults to the configured endpoint (a wildcard `0.0.0.0`
bind address falls back to `127.0.0.1`), which works when the relay shares the
host loopback. Set `NOMAD_RELAY_ADDRESS` when the relay lives behind a separate
gateway IP (Docker Desktop's UDP proxy, LTE routers). A malformed value fails
`connect()` closed — a silent wrong target would strand the link behind a relay
that never opens. Once a vehicle datagram latches the peer, the override stops
applying; replies go to the latched address.

## Companion deployment

The eventual companion deployment is one core process plus only the adapters that
a deployment uses. Systemd may manage that process, but a separate unit is not
created for every internal responsibility.

Required deployment values belong in a local ignored file such as
`config/nomad.env`. The committed `config/nomad.env.example` contains placeholders
and safe development defaults only.

Do not put Tailscale, LTE, Docker, ZED, or ROS 2 assumptions in the C++ core. They
are deployment concerns.

## Deployment matrix

The same core serves every configuration; only the link endpoints and the set of
running services differ. Profiles live in `config/profiles/` and are loaded with
`nomad profile load <name>` (or `python scripts/profile.py load <name>`).

| Configuration | Core host | Drone side | Ground-side links | Profile |
|---|---|---|---|---|
| Jetson on drone | Drone Jetson | Jetson runs core + adapters | Tailscale/LTE UDP from drone | `drone` |
| Ground-station hosted | GCS computer/Jetson | RPi Zero + LTE + Tailscale (mavlink-router only) | LTE UDP + ELRS serial, aggregated | `groundstation` |
| ELRS-only degraded | GCS computer/Jetson | RPi optional (LTE absent) | ELRS serial only | `groundstation` |
| Local development | Host | SITL container | Docker UDP copy | `dev` |

### Drone-side LTE bridge (Raspberry Pi Zero or similar)

The bridge is configuration, not product code: install Tailscale, then run
mavlink-router with two endpoints — the FC serial port and a UDP endpoint
reachable by the ground station over Tailscale. Example
`mavlink-routerd` config (values are placeholders):

```ini
[General]
TcpServerPort=5760
ReportStats=false

[UdpEndpoint lte]
Address=0.0.0.0
Port=14550

[SerialEndpoint fc]
Device=/dev/ttyAMA0
Baud=921600
```

### Ground-side link aggregation

- Linux GCS: `mavlink-router` receives the Tailscale UDP endpoint and the ELRS
  serial device and forwards both into the aggregated UDP port the core and
  Mission Planner consume.
- Windows GCS: the plugin's `GroundLinkRouter` performs the same aggregation
  and failover natively.

The C++ core never switches transports itself. It binds one endpoint; the
aggregator decides which physical link carries the traffic. Both links down
means no heartbeats, and the core fails closed (SR-LNK-*) and resumes normal
operation as soon as a link returns.

### Degraded-link semantics

"The system still works for the available controls" means exactly this:

- one link up: commands and telemetry flow normally;
- both links down: the core refuses commands, stops active velocity with a
  zero setpoint, and reports disconnected;
- a link returns: heartbeats resume and the core operates normally again.

The `core-sitl-link-recovery` scenario proves the last two points against
SITL by dropping and restoring the UDP stream to a live client.

## Ground station

Mission Planner is one client. A deployment may use its plugin for status,
telemetry, missions, video, and operator controls. The plugin must connect through
the NOMAD client boundary and must not duplicate core vehicle decisions.

A future GCS can replace Mission Planner without changing the core.

## ROS 2 deployment

ROS 2 runs as a separate adapter package. Nodes use standard messages, YAML
parameters, and launch files. The adapter may depend on the core library; the core
must not depend on the ROS installation.

The adapter boundary is useful for ZED, Nav2, nvblox, and perception workloads,
but the core should remain usable without those systems.

## Networking

Remote operation may use a secure network such as Tailscale, LTE, or another VPN.
NOMAD does not hard-code one technology. Network configuration belongs outside
the core and must be authenticated and logged where commands cross a trust
boundary.

## Logging

Normal logs should explain connection changes, command results, mission state,
errors, and safety decisions. Do not log high-frequency telemetry at `INFO`.
Keep logs local to the deployment and rotate them.

## Ports

Ports are deployment values, not core constants. The current transitional setup
uses values such as:

| Purpose | Example |
|---|---|
| Transitional Edge Core API | `8000/TCP` |
| MAVLink ground link | `<configured UDP port>` |
| RTSP video | `<configured TCP port>` |
| SSH | `22/TCP` |

Use `config/nomad.env.example` and the active deployment configuration as the
source of truth.

## Safety before flight

Before using a real vehicle:

1. Run the unit tests.
2. Run the relevant SITL scenario.
3. Verify command acknowledgement and state-change handling.
4. Verify ArduPilot failsafes and the independent RC link.
5. Confirm boundaries, modes, limits, and payload interlocks.
6. Keep real credentials and host details out of source control.
