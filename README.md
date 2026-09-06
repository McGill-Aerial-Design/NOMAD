# NOMAD

NOMAD is a standalone system for interacting with, monitoring, and controlling
ArduPilot-based vehicles. The long-term product is a small C++20 core that owns
vehicle behavior and MAVLink communication while clients use it through simple
interfaces.

```text
CLI / Mission Planner / ROS 2 / Python tools
                    |
              NOMAD C++ core
                    |
              MAVLink transport
                    |
                 ArduPilot
```

ArduPilot remains responsible for stabilization, motor control, sensor fusion,
EKF, low-level navigation, and failsafes. NOMAD operates at the higher level:
connect, inspect state, issue a verified command, and report the result.

## Current status

The repository is in the C++ migration phase. The existing Python FastAPI edge
service, ROS HTTP bridge, and Mission Planner plugin are transitional. They remain
available for current simulation and hardware workflows while the C++ core is
built, but they are not the target architecture and should not receive new
frameworks or duplicated vehicle logic.

See [the migration plan](docs/migration.md) for the phase gates and deletion rules.

## Target API

The intended core API stays deliberately boring:

```cpp
Vehicle vehicle(connection);
vehicle.arm();
vehicle.takeoff(10.0);
vehicle.goto_location(target);
vehicle.set_velocity({1.0F, 0.0F, 0.0F, 0.0F});
vehicle.stop_velocity();
vehicle.land();
```

The CLI is a thin client. For example, `nomad arm` parses its arguments, creates a
connection, calls `Vehicle::arm()`, waits for acknowledgement or a state change,
prints the result, and exits. It does not contain MAVLink packet logic.

## Repository layout

```text
NOMAD/
├── include/nomad/       # C++ public headers
├── src/                 # C++ core, transport, and CLI
├── tests/               # Unit, integration, and SITL tests
├── ros2/                # ROS 2 adapters, outside the core
├── python/              # CV, ML, simulation, analysis, and utilities
├── mission_planner/     # Ground-station integration client
├── docs/                # Product and engineering documents
├── docker/              # Reproducible development and SITL images
└── infra/               # Deployment and network support
```

## Documentation

- [Product requirements](docs/prd.md)
- [Architecture](docs/architecture.md)
- [Development workflow](docs/development.md)
- [Operations](docs/operations.md)
- [Migration plan](docs/migration.md)
- [Safety case](docs/safety.md)

## Transitional development

Until the C++ MVP is accepted, the existing hardware-free Python environment can
still be used:

```bash
pixi run dev
pixi run test-fast
pixi run lint
```

The C++ target workflow will become:

```bash
pixi run build-core
pixi run test-core
pixi run sitl
# With the SITL stack running:
pixi run core-sitl-status
pixi run core-sitl-command-flow
pixi run core-sitl-mission
```

Use [operations](docs/operations.md) for deployment placeholders and [development](docs/development.md)
for the full verification workflow. Never commit `config/nomad.env` or real
connection details.

## License

Apache 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
