# NOMAD contributor guide

NOMAD is a C++20 system for monitoring and controlling ArduPilot vehicles.
The production command owner is the long-running `nomad-runtime`; installed
clients use its local IPC and it sends through one MAVSDK transport. Verify
source and current guidance before trusting historical migration notes.

## Agent operating rules

- Read this file and any deeper directory instructions before editing.
- Start from the concrete request, failure, owning symbol, or nearest test.
- Trace the controlling implementation, callers, and focused tests before changing code.
- State assumptions that materially affect correctness, safety, compatibility, or scope.
- Define an executable check that could disprove the proposed fix.
- Prefer purpose-built repository tools over shell commands when both are available.
- Parallelize independent reads only; serialize edits and commands that share mutable state.
- Keep at most one active work item in a long-running checklist and report material findings.
- Never expose or persist secrets, raw transcripts, temporary infrastructure state, or machine-specific identifiers.
- Do not push, publish, deploy, flash, erase, reset, or run destructive commands without explicit authorization.
- Review the final diff for unrelated changes and verify every claimed check was actually run.

## Product boundary

```text
Mission Planner / installed CLI / optional ROS observations
                    |
             local interfaces
                    |
              nomad-runtime
     authority, requests, vehicle/safety policy
                    |
       one MAVSDK command transport
                    |
          standalone ground router
                    |
                ArduPilot
```

Mission Planner is the operator UI, management and status client. Its supported
requests and the installed CLI use runtime IPC. ROS 2 is observation-only. The
standalone ground router routes traffic and does not grant flight authority.
ArduPilot owns stabilization, motor control, EKF, low-level navigation and
failsafes. Native GCS, RC/pilot and other external sources remain outside
NOMAD's software authority guarantee.

Python is for computer vision, machine learning, experiments, simulation,
analysis, tests and ground-side utilities. `nomad-qualification` is a
non-installed direct test driver for SITL/transport qualification.

## Repository layout

```text
NOMAD/
├── CMakeLists.txt
├── include/nomad/       # Public C++ headers
├── src/                 # C++ implementation and thin CLI
├── tests/               # CTest and integration tests
├── examples/            # Small runnable examples
├── ros2/                # ROS 2 adapter packages, outside the core
├── python/              # Vision, ML, simulation, analysis, utilities
├── mission_planner/     # Mission Planner client/integration
├── docs/                # Canonical product and engineering documents
├── config/              # Deployment templates, never real secrets
├── docker/              # Reproducible development and SITL images
└── infra/               # Deployment and network support
```

Put new vehicle behavior in the C++ core/runtime and preserve the single
production command path. Do not recreate `edge_core/`, service registries,
vehicle REST layers or parallel vehicle logic. Current workflows are in
`docs/development.md`; dated migration evidence is in `docs/migration.md`.

## Code rules

Write code for a first-year engineering student to read.

- Keep functions to about 40 logical lines or fewer.
- Treat 500 source lines as a refactoring signal; split files by responsibility.
- Use early returns and keep indentation shallow.
- Write explicit multi-line `if` and loop bodies.
- Give each function one job and name it with the exact action it performs.
- Prefer verbs such as `get`, `set`, `parse`, `send`, `wait`, `format`, and `validate`.
- Make main functions read as a sequence of named helper calls.
- Keep helpers small enough to understand and reuse.
- Keep imports at the top of each file and order them consistently.
- Use one-line comments only for non-obvious rationale.
- Prefer the standard library and existing local code before adding dependencies.
- Delete unused code before adding abstractions.
- Do not add an interface, factory, registry, service locator, event bus, or config
  object with one real implementation.
- Do not build fallback chains for speculative environments.
- Avoid global mutable state and hidden ownership.
- Never hardcode secrets, real hosts, emails, or absolute user paths.
- Keep source lines at 120 characters or less where the language tooling supports it.

A function should read like a short story. A caller should be able to understand
what happens without opening every implementation detail.

## C++ rules

Use C++20, CMake, RAII, explicit ownership, and ordinary standard-library types.
Keep public headers in `include/nomad/` and implementation in `src/`. Keep MAVLink
packing inside the MAVLink implementation. The rest of the core should call
boring methods such as:

```cpp
Vehicle vehicle(connection);
vehicle.arm();
vehicle.takeoff(10.0);
vehicle.land();
```

Do not use advanced templates, unnecessary inheritance, macros, or framework-like
abstractions. Every dependency must remove meaningful complexity.

## ROS 2 rules

ROS 2 is an adapter, not a core dependency.

- Build nodes as components when ROS 2 composition is useful.
- Keep callbacks short: validate, translate, enqueue, and return.
- Do not block or run heavy calculations in a subscription callback.
- Use timers or owned workers for deferred work.
- Define callback groups explicitly when concurrency exists.
- Prefer `SingleThreadedExecutor` unless measured requirements demand more.
- Reuse standard messages before creating custom interfaces.
- Put custom `.msg`, `.srv`, and `.action` definitions in a separate interface package.
- Declare parameters and load them from YAML through launch files.
- Keep ROS types and dependencies out of the C++ core headers.

## Safety and communication

Validate all external input. Critical commands must be sent, acknowledged or
verified through a state change, and reported as success or failure. Stale
telemetry, lost links, invalid values, and shutdown must lead to a safe result or
relinquish control to ArduPilot. NOMAD must never disable ArduPilot failsafes.

Use services only for short non-blocking operations. Use actions for long-running,
interruptible goals when an adapter actually needs them. Keep callbacks and route
handlers thin; decision logic belongs in testable core functions.

## Testing

Every important behavior must be testable without hardware.

- C++ unit tests use fake transports and CTest.
- SITL tests prove the MAVLink loop against ArduPilot.
- ROS 2 tests cover adapter translation and callback behavior.
- Python tests cover retained tools and experiments.
- Mission Planner tests cover client and UI behavior that remains its responsibility.
- Safety changes require invalid-input, boundary, and failure-path tests.

Run the smallest relevant test while working, then the full project checks before
handing off a change.

## Development commands

The exact task names are kept in `pixi.toml`; follow the current
[`docs/development.md`](docs/development.md) workflow. Core and Python checks
start with:

```bash
pixi run build-core
pixi run test-core
pixi run test-python
pixi run lint
pixi run format
pixi run docs-build
pixi run build-core-release
pixi run package-core
pixi run verify-core-staged-install
```

Use placeholders for deployment values and keep real configuration in the
ignored `config/nomad.env`.

`build-core` and `build-core-release` build production executables into the build
tree. They do not install them. `package-core` creates release archives, and
`verify-core-staged-install` installs only into `build/package/stage` for
verification. `install-core` is the explicit prefix-install task and requires a
destination argument. Mission Planner deployment uses its explicit packaging
installer; ordinary build and test tasks must not call installation or host
deployment scripts.

## Documentation

The canonical documents are:

| Topic | Document |
|---|---|
| Product requirements | `docs/prd.md` |
| Current architecture | `docs/architecture.md` |
| Development workflow | `docs/development.md` |
| Operations and deployment | `docs/operations.md` |
| Current qualification status | `docs/qualification.md` |
| Safety case | `docs/safety.md` |
| Historical migration evidence | `docs/migration.md` |

Component READMEs should point to these documents and describe only local details.
Do not create competing architecture or setup guides.

## Test readability

Tests are diagnostic documentation. Each important test should read as a deterministic
story: establish known state, perform one action, observe authoritative state, assert
independent conditions, and restore state when required. Prefer named helpers, explicit
polling deadlines, narrow fixtures, and assertion messages that identify the observed
value. Do not replace authoritative state checks with command acknowledgements.

## Technical debt

When deliberately deferring complexity, record the ceiling, measurable revisit trigger,
and upgrade path in a concise local comment or the relevant canonical document:

```text
debt: <current ceiling>; revisit when <measurable trigger>; then <upgrade path>
```

Do not use unexplained TODOs as a substitute for an owned decision.

## Contribution workflow

Use a focused branch in a fork and one logical change per merge request. Keep
commit subjects under 72 characters and use the repository prefix style:

```text
[part,sub-part] Imperative description
```

Before requesting review:

- run the relevant tests and quality checks;
- review the diff for unrelated changes;
- update the canonical document when behavior or ownership changes;
- include a requirement and fault-path test for safety-critical work;
- confirm no secrets, real addresses, generated artifacts, or oversized new files
  were added.

Do not commit, push, deploy, or change runtime infrastructure without an explicit
request.
