# NOMAD

NOMAD is a C++20 system for monitoring and controlling ArduPilot vehicles. The
production NOMAD command path is the installed `nomad` CLI or supported Mission
Planner requests over local IPC to one long-running `nomad-runtime`; it applies
vehicle and safety policy, then sends through one MAVSDK transport. ArduPilot
continues to own stabilization, navigation execution and its failsafes.

Mission Planner provides operator UI, management and status. Its supported
plugin actions use runtime IPC; its router consumer is receive-only. ROS 2 is an
optional telemetry observer with no vehicle command interface. The standalone
ground router routes MAVLink but does not authorize flight actions. The separate
`nomad-qualification` executable is non-installed test tooling. These software
boundaries do not arbitrate pilot/RC input or native Mission Planner controls;
see the [qualification status](docs/qualification.md).

## First check

Prerequisites are Git with the MAVSDK submodule, Pixi, CMake 3.22.1+ and a C++20
compiler. The first C++ configure/build may need network access for pinned
dependencies. No aircraft, Docker, ROS or GPU is needed for these checks.

```sh
git submodule update --init --recursive
pixi run test-python
pixi run build-core
pixi run test-core
pixi run test-runtime-ipc
```

`test-python` is the quickest hardware-free regression check. `test-runtime-ipc`
builds and exercises the persistent runtime against a local fake MAVLink peer.
Builds and tests do not install or deploy executables.

## Guides

- [Architecture](docs/architecture.md) — current components, command path and
  where changes belong.
- [Development](docs/development.md) — build, test, SITL, ROS and packaging.
- [Contributing](CONTRIBUTING.md) — repository workflow and code standards.
- [Operations](docs/operations.md) — runtime, router, profiles and installation.
- [Safety case](docs/safety.md) and [qualification status](docs/qualification.md)
  — implemented boundaries, evidence levels and unqualified behavior.
- [Runtime IPC](docs/runtime-ipc.md) — local protocol and authority lifecycle.
- [ROS 2 adapter](ros2/nomad_ros/README.md) and [Mission Planner client](mission_planner/README.md).
- [Migration evidence archive](docs/migration.md) — dated implementation and
  qualification records; use the current guides above for branch-tip behavior.
- [Remaining work](TODO.md) — the actionable branch-tip ledger.

## Repository map

`include/nomad/` and `src/` contain the reusable C++ core;
`tools/runtime/` composes the long-running runtime; `tests/` contains C++ and
Python checks; `scripts/dev/` contains qualification runners; `ros2/nomad_ros/`
is the ROS observer; `mission_planner/src/` is the Mission Planner client; and
`python/` contains retained ground-side utilities.

NOMAD is licensed under Apache 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
