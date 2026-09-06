# Architecture

**Status:** Target architecture; the Python edge tree is transitional.

## Deployment configurations

One core binary serves both supported configurations; the transport endpoint is
configuration, not code. No deployment forks the product logic.

### A. Jetson on the drone (existing)

The companion computer on the drone runs the core and any adapters it needs.
The ground station connects over a secure network (Tailscale/LTE) and runs
Mission Planner as a client.

### B. Ground-station hosted (no Jetson on the drone)

Everything runs on the ground station computer (or a Jetson at the ground
station) for weight reduction. The drone carries only an LTE module with a
thin bridge (Raspberry Pi Zero or similar) running mavlink-router over
Tailscale — configuration, not product code. A local ELRS transmitter on the
ground station provides the second MAVLink path.

```text
Drone:  ArduPilot <-> (serial) RPi Zero mavlink-router <-> LTE/Tailscale
        ArduPilot <-> ELRS RX (RC + telemetry radio)

GCS:    Tailscale UDP <-> ground link aggregation <-> NOMAD core <-> Mission Planner
        ELRS TX serial ^
```

Both links converge on the ground station (mavlink-router on a Linux GCS, the
plugin's GroundLinkRouter on Windows) into the one stream the core and plugin
consume. The system keeps working for whichever link is available: commands
flow over the live link, and if both links are down the core fails closed
(SR-LNK-*) and resumes normal operation when a link returns.

### C. ELRS-only degraded

Same ground-station layout with no LTE path. Only the controls the ELRS link
supports are available; the core behavior is identical.

Configuration profiles: `config/profiles/drone.env` (A) and
`config/profiles/groundstation.env` (B/C). Drone-side bridge details live in
`docs/operations.md`; nothing in the core depends on LTE, Tailscale, or ELRS.

## System boundary

```text
┌──────────────────────────────────────────────────────────┐
│ Clients and adapters                                    │
│ CLI · Mission Planner · ROS 2 · Python tools            │
└────────────────────────┬─────────────────────────────────┘
                         │ calls core API
                         v
┌──────────────────────────────────────────────────────────┐
│ NOMAD C++ core                                           │
│ Vehicle · telemetry · missions · validation              │
└────────────────────────┬─────────────────────────────────┘
                         │ transport boundary
                         v
┌──────────────────────────────────────────────────────────┐
│ MAVLink transport                                        │
│ UDP · serial · TCP · heartbeat · acknowledgements        │
└────────────────────────┬─────────────────────────────────┘
                         │ MAVLink
                         v
                    ArduPilot
```

ArduPilot owns low-level flight control, stabilization, EKF, motor control, and
failsafes. NOMAD owns higher-level commands, telemetry models, mission behavior,
and verification of the commands it sends.

## Dependency direction

```text
UI / CLI / ROS 2 / Python tools
              |
              v
        NOMAD C++ core
              |
       MAVLink implementation
              |
           ArduPilot
```

Dependencies point inward. The core does not import UI frameworks, ROS 2, Python,
FastAPI, Mission Planner assemblies, cloud clients, or VPN libraries.

## Core ownership

| Area | Owns | Does not own |
|---|---|---|
| `mavlink` | Transport, packet conversion, heartbeat, ACKs | Missions or UI decisions |
| `vehicle` | Arm, mode, takeoff, navigation, land, RTL, state verification | Packet layout or rendering |
| `telemetry` | Stable state and value types | Transport sockets |
| `mission` | Small mission data and synchronous execution | UI workflows or scripting language |
| CLI | Argument parsing, output, exit status | Flight behavior or MAVLink packing |
| `src/mavlink` | MAVLink frame conversion and UDP peer handling | Vehicle policy |
| ROS 2 adapter | Message translation and node lifecycle | Core decisions |
| Mission Planner | Operator UI and client integration | Core mission or safety logic |
| Python tools | CV, ML, simulation, analysis | Vehicle ownership |
| ArduPilot | Stabilization, EKF, low-level control, failsafes | NOMAD application concerns |

## Target project layout

```text
NOMAD/
├── CMakeLists.txt
├── include/nomad/
│   ├── mavlink/
│   ├── vehicle/
│   ├── telemetry/
│   └── mission/
├── src/
│   ├── main.cpp
│   ├── mavlink/
│   ├── vehicle/
│   ├── telemetry/
│   └── mission/
├── tests/
├── examples/
├── ros2/
│   └── nomad_ros/
├── python/
│   ├── vision/
│   ├── ml/
│   ├── simulation/
│   ├── analysis/
│   └── tools/
├── mission_planner/
├── docs/
├── config/
├── docker/
└── infra/
```

The first C++ build contains one library, one CLI, and small CTest targets. It
intentionally has no daemon, plugin loader, service locator, message bus, generic
command framework, or discovery system.

## CLI flow

```text
nomad arm
  -> parse arguments
  -> open the configured connection
  -> construct Vehicle
  -> Vehicle::arm()
  -> send MAVLink command
  -> wait for ACK or armed state
  -> print result
  -> close connection
```

The CLI is a client of the core. A future long-running process may reuse the same
core API, but it is not required for local CLI operation.

## ROS 2 boundary

ROS 2 lives in `ros2/`, outside the core. A ROS node may subscribe to a standard
message, validate and translate it, then call the core. It must not pack MAVLink
messages or implement a second vehicle state machine.

Callbacks remain short and non-blocking. Timers or owned workers handle heavy
processing. Callback groups are explicit when concurrency is needed, and a
`SingleThreadedExecutor` is the default.

ROS parameters are declared in the node and loaded from YAML through launch files.
Standard messages are preferred. Custom interfaces, if required, live in a
separate interface package.

## Mission Planner boundary

The plugin is a ground-station client. It may render telemetry, expose commands,
manage configuration, and provide operator workflows. It must call a client
boundary rather than duplicate `Vehicle` or safety logic in event handlers.

The plugin is not the foundation of NOMAD. It can eventually be replaced by
another ground-control client without changing the core.

## Python boundary

Python remains useful for rapid iteration, computer vision, ML, simulation,
analysis, testing, and ground utilities. Python code must not become a second
source of truth for vehicle commands after the C++ cutover.

The current `edge_core/` service is transitional. Its REST API, module registry,
Python MAVLink path, and ROS HTTP bridge are migration targets rather than new
extension points.

## Current-to-target mapping

| Current area | Target home | Action |
|---|---|---|
| `edge_core/services/mavlink/` | `include/nomad/mavlink`, `src/mavlink` | Port transport, packets, telemetry, ACKs; then delete Python path |
| ~~`edge_core/ros_http_bridge/`~~ | `ros2/nomad_ros` (C++ adapter node) | **Deleted 2026-09-05** — the adapter node owns the MAVLink link and core velocity path; no HTTP hop |
| `edge_core/safety/` | C++ vehicle validation and safety modules | Re-establish each requirement and test |
| `edge_core/api_routes/` | CLI/client operations or delete | No REST route without a real client requirement |
| `edge_core/core/` | No replacement | Delete dynamic module registry |
| `edge_core/services/video*` | Python tools or deployment adapter | Keep only if a real product workflow needs it |
| `edge_core/services/health*` | Client/adapter telemetry | Keep behavior, move ownership |
| `scripts/nomad` | C++ `nomad` CLI | Replace service dispatcher with core client |
| `infra/systemd/` | One core unit plus required adapters | One unit per deployment host; ground-station profile disables drone-side services |
| Mission Planner controls | Plugin client | Remove duplicate core and safety decisions |
| Removed Python module example | None | Do not recreate the registry pattern |

## What is deliberately absent

The target does not contain:

- a generic module registry;
- a service locator;
- a route-driven vehicle API;
- a second Python MAVLink implementation;
- an HTTP hop for local core operations;
- a mission scripting language before a use case exists;
- a cloud or VPN dependency in the core;
- a broad fallback chain for transports or runtimes.

Link selection is not a fallback chain: the ground station aggregates the
available links into one stream, and the core simply uses that stream. It never
switches transports itself based on availability.

Each new abstraction must have more than one real caller and remove complexity.

## MAVLink library decision

Status: decided (2026-09-05, evidence below).

NOMAD frames MAVLink 2 itself and owns the verification semantics; the dialect
facts come from generated code, never hand-maintained tables. The C++ core
pins `third_party/ardupilot-mavlink` as a submodule at the exact commit the
ArduPilot firmware release compiles against, and
`scripts/dev/generate_mavlink.py` runs that submodule's own mavgen into the
gitignored build dir (`build/generated/mavlink`). `src/mavlink/protocol.cpp`
and `fence.cpp` use the generated message ids, crc_extras, lengths, and
per-message pack/decode functions; only the framing, CRC check, sequence, and
payload round-trip helpers are NOMAD's own code (~1,000 lines including tests).
Golden unit tests (generated with pymavlink) pin the wire bytes, and every
message NOMAD emits has been accepted live by ArduPilot Copter 4.7.1 (fence
upload, commands, reposition). Because the tables come from the pinned
submodule, future ArduPilot version bumps cannot silently drift dialect facts
(crc_extras, field layouts): updating the pin and regenerating is the whole
upgrade. The safety-critical part of the core is not the codec: it is the
verification semantics (send -> ack -> authoritative state check -> fail
closed), which remains NOMAD's code regardless of library.

A MAVSDK evaluation was carried out in parallel and recorded here for the
revisit triggers:

- MAVSDK is pinned as a submodule at `third_party/MAVSDK` (v3 main,
  commit `v3.15.0-441-g34b417d4` at evaluation time).
- It builds cleanly on Windows with CMake `-A x64` and
  `-DBUILD_WITHOUT_CURL=ON` (the default superbuild fails at the openssl
  step on Windows; curl is only needed by the camera/http_loader plugins).
- A smoke client using `add_any_connection("udpin://0.0.0.0:14570")`
  discovered the autopilot and streamed telemetry from the live SITL stack
  (ArduPilot Copter 4.7.0).
- Two ArduPilot quirks surfaced in the smoke test, both worth knowing before
  adopting MAVSDK: `Telemetry::FlightMode` reports `Offboard` for an
  ArduPilot `GUIDED` vehicle (PX4-oriented enum mapping; the
  `ardupilot_custom_mode` support in the core exists for this), and
  `Battery.remaining_percent` came back as `100.0` from ArduPilot's percent
  field (treated as a fraction), so ArduPilot clients must rescale.

Revisit MAVSDK when a measurable trigger fires, then re-evaluate adoption:

- a PX4 product appears; or
- the core must speak MAVLink directly over serial/TCP without
  mavlink-router as a sidecar; or
- the ArduPilot quirks above get fixed upstream and a release is cut.

If adoption happens, use the pinned fork-and-patch model (this submodule) and
contribute the ArduPilot fixes upstream. Contributing the wire-semantics
findings (golden frames, `MAV_CMD_DO_REPOSITION` command_long rejection,
fence import vertex-count quirk) to pymavlink/ArduPilot test suites is a
cheaper open-source contribution path that does not require adoption.
