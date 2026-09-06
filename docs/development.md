# Development

NOMAD is moving toward a C++20 core with independent ROS 2, Python, and Mission
Planner clients. The current Python edge service remains a transitional test
surface until its replacement passes the migration gates.

## Prerequisites

- Git;
- Pixi for the repository development environment;
- CMake and a C++20 compiler for the core;
- Docker for reproducible SITL and ROS 2 tests;
- Mission Planner and Visual Studio only for plugin work.

## Target workflow

```bash
pixi run build-core
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run docs-build
```

Use `pixi run dev` and `pixi run test-fast` while the transitional Python path is
still required. Do not add new product behavior to that path.

## C++ layout

```text
include/nomad/       Public headers
src/                 Implementation and thin CLI
tests/               CTest and integration tests
examples/            Runnable examples
```

Keep each file focused. A public header should expose the smallest useful API.
The CLI should call named core operations rather than contain business logic.

## CMake

The core has one library target and one CLI target in the MVP:

```bash
cmake -S . -B build/core -DCMAKE_BUILD_TYPE=Debug
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

The local C++ status smoke test is:

```bash
pixi run core-sitl-status
```

The C++ core must configure without ROS 2, Python, Mission Planner, or hardware.
ROS 2 builds are separate adapter packages and depend on the core library.

## CLI authentication (SR-SEC-02/03)

Actuation verbs (`arm`, `disarm`, `mode`, `takeoff`, `land`, `rtl`, and the
`*-demo` commands) are refused before any socket work unless `NOMAD_API_KEY` is
set; `connect` and `status` keep a no-key local fallback. Accepted and refused
actuation attempts emit one stderr line each:

```text
audit command=arm result=refused auth=none reason=missing_api_key
audit command=arm result=accepted auth=api-key
```

The `core-sitl-*` pixi tasks export the development-simulation key from
`config/nomad.env.example`; deployed machines set a real generated key there
(`config/nomad.env`).

## Tests

### C++ unit tests

Use fake transports to test:

- MAVLink packet conversion;
- heartbeat and connection state;
- telemetry conversion;
- command validation;
- acknowledgement handling;
- vehicle operations;
- mission execution;
- timeout and shutdown behavior.

### SITL integration tests

Use ArduPilot SITL to prove the real loop:

```bash
pixi run sitl
```

The SITL suite must cover heartbeat, telemetry, arm, mode changes, takeoff, land,
RTL, command failure, and link loss. The C++ telemetry smoke test is run with:

```bash
pixi run core-sitl-status
pixi run core-sitl-command-flow
pixi run core-sitl-mission
pixi run core-sitl-velocity-watchdog
```

The command-flow task runs the C++ CLI through mode, arm, takeoff, RTL, land,
and disarm, polling `status` after each transition. The mission task runs the
C++ `MissionExecutor` through GUIDED, arm, takeoff, RTL, land, and disarm. The
velocity watchdog task proves an active setpoint stops after command timeout.
Both expect a fresh SITL vehicle in the required starting state.

It expects the ArduPilot Docker service to emit a host UDP copy on
`NOMAD_CORE_SITL_PORT` (default `14570`). Run safety scenarios only after unit
tests cover invalid and boundary inputs.

To watch any SITL run live, connect Mission Planner as a passive observer to
the dev stack's operator link at **TCP `127.0.0.1:5762`** (CONNECT → TCP →
127.0.0.1:5762). The link accepts multiple clients, so it never disturbs the
scenario or the core CLI; the `core-sitl-*` tasks print this hint on start.
Details: `tests/sitl/README.md`.

### ROS 2 tests

ROS 2 tests belong in the adapter package. Test message translation, callback
behavior, parameter loading, and shutdown without putting ROS dependencies in the
core.

### Python tests

Retained Python tests cover CV, ML, simulation, log analysis, and transitional
compatibility until each legacy subsystem is deleted. Avoid expanding tests for
architecture that the migration plan removes.

### Mission Planner tests

Run the framework-free C# tests for pure client and safety helpers on Windows.
The full plugin build requires Mission Planner reference assemblies.

## Code quality

The project uses language-native tools:

- C++: `clang-format`, `clang-tidy`, CMake, and CTest;
- Python: Ruff, mypy where configured, and pytest;
- shell: ShellCheck;
- C#: the existing compiler and focused test scripts;
- docs: ProperDocs strict build.

The shared rules are simple: functions target 40 logical lines, files above 500
lines are refactoring debt, imports are ordered, and comments explain only
non-obvious decisions.

## Size checks

The size report excludes generated, build, lock, and vendored files. Existing
files above 500 lines are migration debt. New or modified files above 500 lines
fail unless a temporary reviewed exception names an owner and removal issue.

Run:

```bash
pixi run line-report
pixi run complexity-check
```

## Change workflow

1. Identify the owning layer before editing.
2. Trace callers and current data flow.
3. Delete unused code before adding abstractions.
4. Keep core decisions independent of adapters.
5. Add the smallest test that fails when the behavior breaks.
6. Update the canonical document if ownership or status changed.
7. Run focused checks, then the full relevant suite.

## Hardware boundary

Hardware is tested last. A feature is not complete because it works on a connected
drone. It must first work with a fake transport, then against SITL, then on the
actual vehicle with explicit operational safeguards.
