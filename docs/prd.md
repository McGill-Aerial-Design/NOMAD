# NOMAD Product Requirements Document

**Status:** Target product during the C++ migration

## 1. Overview

NOMAD is a standalone software system for interacting with, monitoring, and
controlling ArduPilot-based vehicles. It provides a clean boundary between
applications and the flight controller so the same vehicle behavior can be used
from a CLI, ground-control application, Mission Planner integration, or onboard
hardware.

NOMAD prioritizes simplicity, modularity, reliability, understandability, low
coupling, easy testing, and ArduPilot/MAVLink compatibility.

The current repository contains a Python edge service and C# plugin. Those are
transitional implementations. This PRD defines the product they are being
replaced by.

## 2. Problem

Vehicle functionality becomes difficult to reuse when it is coupled to a ground
station, spread across scripts, or implemented separately by each UI. That causes:

- poor reuse outside Mission Planner;
- tests that require the full application or hardware;
- weak support for autonomous operation;
- tight coupling between UI and vehicle logic;
- difficulty replacing a ground-control application;
- growing complexity as capabilities are added.

NOMAD separates vehicle-control logic from user interfaces and deployment tools.

## 3. Goals

### Primary goals

1. Communicate with ArduPilot through MAVLink.
2. Provide a simple API for common vehicle operations.
3. Separate vehicle logic from UI and application code.
4. Support local and remote MAVLink transports.
5. Work without Mission Planner.
6. Be testable without physical hardware.
7. Extend only when a real requirement justifies it.
8. Support onboard and ground-side deployments over time.
9. Provide a foundation for autonomous missions.

### Secondary goals

- mission execution;
- telemetry monitoring;
- vehicle state management;
- parameter management;
- multiple vehicles when required;
- remote operation over a secure network;
- logging;
- hardware-independent testing;
- Mission Planner and other GCS integrations.

## 4. Non-goals

NOMAD does not replace ArduPilot. ArduPilot remains responsible for:

- low-level flight control;
- stabilization and motor control;
- sensor fusion and EKF;
- vehicle-specific navigation primitives;
- failsafes and safety mechanisms owned by the flight controller.

NOMAD is not initially a replacement for Mission Planner or QGroundControl. It
is a reusable vehicle-control core that clients can use.

NOMAD does not initially build a mission scripting language, distributed broker,
cloud service, plugin marketplace, or generic task framework.

## 5. Product architecture

```text
                  CLI
                   |
          Mission Planner / other GCS
                   |
        ROS 2 and Python adapters
                   |
                   v
             NOMAD C++ core
       Vehicle / telemetry / mission
                   |
             MAVLink transport
                   |
                ArduPilot
```

Dependencies point inward. Clients depend on NOMAD. NOMAD does not depend on a
client, UI, ROS 2, Python, cloud provider, VPN, or hardware platform.

NOMAD is not tied to a specific flight controller. It targets ArduPilot on any
supported board (Cube, Pixhawk, Holybro, CUAV, and other ArduPilot flight
controllers). All vehicle control, telemetry, and payload outputs use
standard board-independent MAVLink commands and ArduPilot servo/relay
channels, so the vehicle can be swapped to a different ArduPilot flight
controller without changing NOMAD.

## 6. Core technology

The core uses C++20, CMake, modern standard-library facilities, RAII, explicit
ownership, and minimal dependencies. The core is one library with a thin CLI in
the MVP. C++ code favors straightforward control flow over advanced templates,
complex inheritance, global state, or framework abstractions.

MAVLink generated C headers and the selected dialect are hidden inside the
MAVLink implementation. The rest of the core calls a small connection boundary
and high-level vehicle methods.

## 7. Core modules

The initial core contains only modules with demonstrated responsibilities:

```text
include/nomad/
├── mavlink/       transport, packets, heartbeats, acknowledgements
├── vehicle/       high-level commands and state access
├── telemetry/     value types and state conversion
├── safety/        pure command validation and limits
└── mission/       small mission representation and execution
```

A module must have a clear owner, a small public surface, and tests that can run
without hardware. No module registry is required.

## 8. MAVLink and transport

The MAVLink boundary owns:

- serial connections;
- UDP connections;
- TCP connections when needed;
- packet parsing and transmission;
- heartbeat detection;
- connection state;
- command acknowledgements;
- basic transport errors.

The first transport is UDP against ArduPilot SITL. Serial follows when the UDP
path is stable. TCP is added only for a concrete deployment requirement.

The rest of NOMAD must be able to call:

```cpp
connection.send(command);
```

It must not pack MAVLink messages throughout vehicle, UI, or ROS code.

## 9. Vehicle abstraction

The high-level API remains explicit:

```cpp
Vehicle vehicle(connection);
vehicle.arm();
vehicle.disarm();
vehicle.set_mode(mode_id);
vehicle.takeoff(10.0);
vehicle.goto_location(target);
vehicle.land();
vehicle.return_to_launch();
```

Initial operations:

- connect and disconnect;
- arm and disarm;
- change mode;
- takeoff;
- land;
- return to launch;
- read vehicle state;
- read position;
- read battery;
- read GPS state.

Critical commands are not considered successful merely because a packet was
transmitted. NOMAD sends, waits for an acknowledgement or state change, applies a
timeout, and returns a clear result.

## 10. Telemetry

NOMAD exposes simple, transport-independent values:

- position;
- altitude;
- velocity;
- attitude;
- battery;
- GPS state;
- flight mode;
- armed state;
- connection state.

Example:

```cpp
const auto state = vehicle.wait_for_state(std::chrono::seconds(3));
if (state.has_value() && state->armed) {
    // Use the state in an application-specific way.
}
```

Telemetry collection is independent of UI and logging.

## 11. Mission system

A mission is ordinary data and a small executor:

```text
TAKEOFF 10m
GO TO waypoint A
GO TO waypoint B
PERFORM action
RETURN TO LAUNCH
LAND
```

The initial mission model supports takeoff, navigation, wait, action, RTL, and
land. It supports cancellation and clear failure state only where execution
requires them. A mission scripting language is deferred.

## 12. Networking and remote operation

The connection abstraction must not assume a physical serial cable:

```text
Serial
UDP
TCP
```

Local operation is the first milestone. A future ground-side NOMAD client may
connect to a drone-side NOMAD process over a secure network:

```text
Ground client -> secure network -> NOMAD on companion -> MAVLink -> ArduPilot
```

The architecture does not hard-code Tailscale, LTE, VPN, or any cloud provider.
A remote application protocol is deferred until a real client requires it.

## 13. ROS 2 integration

ROS 2 is an adapter, not a core dependency. ROS 2 nodes depend on the core and
translate standard messages into core operations.

ROS 2 requirements:

- use components where composition is useful;
- keep callbacks short and non-blocking;
- use timers or owned workers for heavy work;
- define callback groups explicitly when concurrency is needed;
- prefer a `SingleThreadedExecutor`;
- use standard `geometry_msgs`, `nav_msgs`, and `sensor_msgs` first;
- place custom interfaces in a standalone interface package;
- load parameters from YAML through launch files.

## 14. Mission Planner and other clients

Mission Planner is an integration layer. It may provide:

- connection and status UI;
- mission controls;
- telemetry visualization;
- configuration;
- debugging tools.

It must not contain the core mission, vehicle, or safety decisions. A different
GCS should be replaceable without changing the core.

## 15. CLI

The CLI is the smallest client and the first acceptance surface:

```text
nomad connect --endpoint udpin:0.0.0.0:14550
nomad status --endpoint udpin:0.0.0.0:14550
nomad arm --endpoint udpin:0.0.0.0:14550
nomad takeoff 10 --endpoint udpin:0.0.0.0:14550
nomad land --endpoint udpin:0.0.0.0:14550
nomad rtl --endpoint udpin:0.0.0.0:14550
nomad mission-demo --endpoint udpin:0.0.0.0:14550
```

In the MVP, each invocation performs one operation and exits. The CLI parses
arguments, constructs the core objects, formats the result, and returns a useful
exit code. It does not implement MAVLink packing or vehicle decisions.

## 16. Simulation

NOMAD must work against ArduPilot SITL before hardware testing. SITL verifies:

- connection and heartbeat;
- telemetry;
- arm and mode changes;
- takeoff;
- land and RTL;
- command acknowledgements;
- disconnection and timeout behavior;
- basic mission execution through the C++ `MissionExecutor`.

Fake transports cover deterministic core logic. SITL covers the real MAVLink loop.

## 17. Testing

### Unit tests

Test MAVLink parsing, state conversion, command validation, vehicle operations,
mission logic, and connection state using fake transports.

### Integration tests

Test NOMAD against ArduPilot SITL for connection, telemetry, commands, missions,
and failure handling.

### Adapter tests

Test ROS 2 translation and Mission Planner client behavior without requiring the
core to know about those frameworks.

### Hardware tests

Run hardware tests only after the same behavior passes unit and SITL tests.

## 18. Safety

NOMAD must never assume that transmission means success:

```text
Send command -> wait -> verify acknowledgement/state -> report result
```

The core rejects invalid and non-finite inputs, applies explicit command limits,
requires appropriate vehicle state, handles stale telemetry and lost links, and
relinquishes control on uncertainty. ArduPilot remains the independent safety
backstop. See [the safety case](safety.md).

## 19. Logging

NOMAD provides useful, restrained logs at `DEBUG`, `INFO`, `WARNING`, and `ERROR`.
Logs explain connection failures, command failures, mission failures, network
problems, and important vehicle state changes. High-frequency telemetry is not
logged at normal levels.

## 20. Configuration

Start with command-line arguments and sensible defaults. Add a small external
configuration file only when a deployment has multiple values that should not be
embedded in commands. Configuration must never contain committed secrets, real
hosts, or absolute developer paths.

## 21. Dependencies

Every dependency must answer:

> Does this significantly reduce complexity or provide functionality unreasonable
to implement ourselves?

If not, do not add it. The core does not depend on Mission Planner, QGroundControl,
ROS 2, Python, cloud providers, VPN products, or hardware SDKs.

## 22. MVP

The first product release contains:

- C++20 project;
- CMake build;
- MAVLink connection;
- ArduPilot SITL connection;
- heartbeat detection;
- vehicle state;
- basic telemetry;
- arm and disarm;
- mode changes;
- takeoff;
- land;
- RTL;
- basic logging;
- unit tests;
- SITL integration tests;
- simple CLI.

It does not contain a Mission Planner plugin, Tailscale integration, LTE support,
computer vision, AI, multi-vehicle support, web interface, or distributed
architecture as an MVP requirement.

## 23. Development phases

1. **Foundation:** CMake, C++20, layout, logging, testing.
2. **MAVLink:** connection, heartbeat, parsing, telemetry.
3. **Vehicle:** arm, disarm, modes, takeoff, land, RTL.
4. **SITL:** automated connection and integration tests.
5. **Missions:** small representation and executor.
6. **CLI:** status and core vehicle commands.
7. **Ground integrations:** Mission Planner and other GCS clients.
8. **Networking:** remote operation when required.
9. **Advanced autonomy:** perception, ML, and custom behaviors in adapters/tools.

The executable migration gates are maintained in [migration.md](migration.md).

## 24. Success criteria

NOMAD succeeds when a developer can clone the repository, build the C++ core with
one command, start ArduPilot SITL, and run:

```text
nomad status
nomad arm
nomad takeoff 10
nomad land
```

They must understand the relevant code without understanding the whole system,
replace the CLI without changing vehicle behavior, and replace the local MAVLink
transport without changing the `Vehicle` API.

The architectural test is:

> Can we change how NOMAD is used without changing what NOMAD does?

If yes, the architecture is working.
