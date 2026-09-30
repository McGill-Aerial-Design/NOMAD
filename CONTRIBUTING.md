# Contributing to NOMAD

NOMAD's production command path is a persistent C++20 `nomad-runtime` with one
MAVSDK transport. The installed CLI and supported Mission Planner requests use
local runtime IPC; ROS 2 is observation-only. Keep changes within these current
ownership boundaries and make core behavior testable without an aircraft.

## Start here

1. Start with the root [README](README.md) and hardware-free first check.
2. Read [architecture](docs/architecture.md), [development](docs/development.md),
   and [qualification status](docs/qualification.md) before changing ownership
   boundaries.
3. Install Pixi and run the focused checks for your change.

```bash
pixi run test-python
pixi run build-core
pixi run test-core
pixi run test-runtime-ipc
```

The [development guide](docs/development.md) owns quality, ROS, SITL, Mission
Planner, router, package and install commands. [Qualification status](docs/qualification.md)
describes what each check proves; the [migration archive](docs/migration.md) is
historical evidence, not the current architecture guide.

## Target layout

```text
include/nomad/       C++ public headers
src/                 C++ core implementation
tools/runtime/       Long-running production runtime
tests/               CTest, unit, integration and qualification tests
ros2/                Receive-only ROS 2 observer
python/              CV, ML, simulation, analysis and ground tools
mission_planner/     UI, status and typed runtime client
```

The `edge_core/` tree has been removed. Do not recreate dynamic modules,
service registries, REST layers or Python vehicle-control paths. Put reusable
vehicle behavior in the C++ core/runtime; add new client requests through typed
runtime IPC. Competition-specific application behavior belongs in a future
adapter after its requirements and safety boundaries are defined.

## Code standard

Write for a first-year engineering student.

- Keep functions to about 40 logical lines or fewer.
- Treat 500 source lines as a refactoring signal.
- Use early returns and shallow indentation.
- Write explicit multi-line conditionals and loops.
- Give every function one responsibility and a precise verb name.
- Keep imports at the top and order them consistently.
- Keep comments to one line unless a longer explanation is genuinely necessary.
- Make top-level functions read as named steps.
- Prefer the standard library and existing project code.
- Do not add speculative abstractions, factories, registries, or fallback chains.
- Keep UI, ROS, transport, and core decisions separate.
- Never commit real hosts, secrets, emails, or absolute paths.

C++ code uses C++20, CMake, RAII, explicit ownership, small headers, and ordinary
standard-library types. MAVLink packet details stay inside the MAVLink boundary.
ROS 2 dependencies stay outside the core.

## Safety changes

Read [the safety case](docs/safety.md) before changing a command or actuation path.
Every safety change needs:

- a stable requirement ID;
- a test for normal, boundary, invalid, and failure inputs;
- acknowledgement or state-change verification for critical commands;
- SITL evidence when the flight behavior is affected;
- an updated requirement-to-code-to-test mapping.

NOMAD must never weaken or disable ArduPilot failsafes.

## Tests and checks

Run the smallest relevant check during development, then the full checks before
review:

```bash
pixi run build-core
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run docs-build
```

The current Python test suite remains a transition gate. C++ tests use CTest and
fake transports. ROS 2 integration tests run in the dedicated container. Plugin
checks remain Windows-only where Mission Planner assemblies are required.

## Size policy

| Item | Rule |
|---|---|
| Function | About 40 logical lines; split by responsibility |
| Source file | 500 lines is a refactoring signal |
| Python line | 120 characters, enforced by Ruff |
| Generated/third-party files | Excluded from source-size checks |

Existing oversized files are tracked as migration debt. New or modified files
above 500 lines fail the changed-file check unless a temporary, reviewed exception
names an owner and removal issue.

## Review checklist

- [ ] The change belongs in this layer.
- [ ] Unused code was deleted before abstractions were added.
- [ ] Functions are small, explicit, and named by behavior.
- [ ] Core logic is testable without ROS, UI, or hardware.
- [ ] Inputs, errors, timeouts, and shutdown are handled.
- [ ] Documentation reflects current ownership and status.
- [ ] No new hardcoded host, secret, or absolute path exists.
- [ ] Relevant checks pass.

## Branches and commits

Use a focused branch in your fork. Keep one logical change per merge request and
use a subject no longer than 72 characters:

```text
[part,sub-part] Imperative description
```

Do not commit, push, deploy, or modify production infrastructure without an
explicit request.
