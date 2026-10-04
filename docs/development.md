# Development

This is the canonical developer workflow. It uses the task names in the current
[`pixi.toml`](../pixi.toml). The root [README](../README.md) has the shortest
hardware-free first check; [architecture](architecture.md) explains where code
belongs, and [qualification status](qualification.md) states what each check can
prove.

## Bootstrap and prerequisites

Install Git, Pixi, CMake 3.22.1 or newer and a C++20 compiler. Non-Windows
runtime/client builds also require OpenSSL Crypto development headers/libraries
(for example `libssl-dev` on Ubuntu); Windows uses the OS BCrypt library. Initialize the
submodules before configuring the C++ build:

```sh
git submodule update --init --recursive
```

The pinned MAVSDK source is required. Its first configure/build downloads pinned
dependencies, so allow network access. A C++ build does not need an aircraft,
ROS, Docker or a GPU. Docker is needed for the simulator and containerized ROS
image. Windows with Visual Studio/MSBuild and .NET Framework 4.8 is required for
the Mission Planner plugin and standalone router checks.

## Production build and hardware-free tests

```sh
pixi run test-python
pixi run build-core
pixi run test-core
pixi run test-runtime-ipc
pixi run test-runtime-lifecycle
```

`test-python` runs the retained Python tools and regression guards.
`build-core` builds the production `nomad` CLI and `nomad-runtime` into the build
tree. `test-core` configures the C++ test targets and runs CTest.
`test-runtime-ipc` builds the production executables and exercises the persistent
runtime against a local fake MAVLink peer. None of these commands contacts an
aircraft.

`test-runtime-lifecycle` reuses protected files and audit history across real
process clean/crash restarts against the fake UDP peer. It checks incarnation,
old-context rejection, fresh authentication/admission, no replay, duplicates,
permanent configuration errors and link absence/return. Its qualification-only
supervisor uses shortened bounded delays and is never installed. Artifact tests
validate actual systemd/SCM settings and PowerShell arguments; Windows CTest
exercises SCM controls without registering a privileged service. CI runs these
and archive/staged-install checks on Linux and Windows. Privileged registration
remains explicit deployment acceptance; ordinary checks never register services.

Use the repository quality checks before review:

```sh
pixi run lint
pixi run format-check
pixi run complexity-check
pixi run docs-build
pixi run precommit
```

`docs-build` is the strict ProperDocs build. `format-check` is read-only;
`format` rewrites Python files. `complexity-check` enforces the tracked
changed-file limits. The Python suite checks that documented `pixi run` tasks
exist and that internal Markdown links resolve.

Source-only ROS container builds omit Git metadata and report
`0.0.0-development`; they cannot install/package a production core release.
Release archives are generated from a provenance-bearing Git checkout.

## Software resource qualification

Run the production Release measurement and budget verifier without hardware:

```sh
pixi run measure-core-resources
pixi run verify-core-resource-budgets
```

The first task uses `build/resources`, builds the pinned MAVSDK and production
targets, runs the full C++ suite and deterministic connectivity, transport,
authority-wire, final-send, authenticated IPC and lifecycle qualifications,
then packages and verifies both archives and the core staged installation.
It stages MAVSDK separately for dependency-footprint evidence. These stages
remain inside the build tree; no OS service or production prefix is installed.
Repeat runs use the same tree and label build timings incremental. Use a new
`--build-dir` with `python scripts/dev/core_resources.py --qualify --build-dir
<new-build-tree> --output <metrics.json>` for a cold build-tree sample.

The versioned report is `build/resources/metrics.json`; phase diagnostics are
`build/resources/resource-phases.json`. Per-run outputs stay outside Git.
The schema and reviewed policy are
[`config/core-resource-metrics.schema.json`](../config/core-resource-metrics.schema.json)
and [`config/core-resource-budgets.json`](../config/core-resource-budgets.json).
The hosted `resources.yml` matrix retains separate Linux/Windows artifacts for
30 days and prints measured-versus-budget summaries. Download evidence before
retention expires when comparing release candidates.
The policy records measured baselines, exact comparable toolchains/configuration,
engineering headroom and rationale for every limit. Current profiles are hosted
Linux/GCC 13.3, hosted Windows/MSVC 19.51 and local Windows/MSVC 19.44. Toolchain
upgrades require a new reviewed baseline; they cannot silently inherit a profile.
Collection requires a clean source checkout and recursive submodules before
and after execution, and the repository's pinned MAVSDK source directory.
The existing-build marker binds both NOMAD and MAVSDK SHAs. Keep custom build
directories and observation outputs under ignored `build/` or outside the
checkout. A changed/dirty source snapshot requires a fresh full measurement;
resource qualification never falls back to Debug binaries.
This Release qualification supplements the existing Debug core job so timings
and peer observations apply to the measured production build; it adds CI work
deliberately and does not share or restore a CMake build-tree cache.

The verifier rejects unknown platforms/toolchains and incomparable dependency
pins, build types, CMake options, binary representations and workload protocols.
It validates the v1 report structure, units and finite nonnegative values.
Exit 1 means a hard regression; exit 2 means invalid/incomparable evidence.
Advisory overages are visible warnings with successful exit status. Compare
timing trends only within the same cache class and toolchain. A hosted fresh
CMake tree is cold even when Pixi's environment cache is enabled; dependency
download-cache hits are unknown. Shared-runner phase timings are advisory and
the 45-minute job timeout guards catastrophic regressions.

Release executables and archives get 25% or 128 KiB headroom, rounded up to
64 KiB; the core stage gets 25% or 512 KiB. Peak resident memory gets 50% plus
8 MiB, rounded up to a MiB, to catch gross growth while allowing native allocator
variation. Startup/restart limits are `max(baseline + 5 seconds, 6 * baseline)`,
rounded up to a tenth of a second; these are catastrophic guards. State medians,
growth, dependency/workspace sizes and timings are advisory. Phase limits are
twice the measured baseline plus 30 seconds, rounded up to a second. Exact
per-profile limits and remaining advisory headroom are in the policy, and
[qualification](qualification.md#software-resource-budgets) records the evidence.

Footprint gates measure the unstripped Linux ELF or Windows PE executable
without PDBs, the complete core stage, and ZIP/TGZ separately. Intermediate
MAVSDK workspace bytes, SDK/dependency stages and static archive input sizes
are retained separately; none describes bytes embedded by the linker.
Debug files in the core stage reject collection. Do not compare Linux ELF
bytes directly with PE, PDB or COFF archive bytes.
The SDK archive inventory follows the configured Release target artifact,
so other configuration outputs in a reused multi-config tree cannot inflate
the linked-library metric.

The fake peer emits telemetry every 200 ms. Five no-peer launches end at a
usable protocol-v1 HELLO; five independent launch/restart pairs against a
persistent peer end at an established session plus fresh heartbeat. Clean
process restarts reuse protected configuration/audit history and require a new
incarnation with no restored owner. Timing starts immediately before process
creation with a monotonic high-resolution clock; credential provisioning and
OS service-manager recovery delays are excluded. Five-sample empirical p95 is
the observed maximum, not a precise population-tail estimate.

Memory samples cover only the runtime PID (native Linux RSS / Windows working
set). Each state settles for one second, then reports the median of a separate
one-second window at a 20 ms polling cadence. States are idle without a peer,
fresh vehicle session, authenticated admission, and admitted-after-command.
A continuous sampler retains the peak across startup and the bounded workload.
The first process performs 300 status/authentication/command/revoke cycles;
four others perform 30 each. Explicit handback follows every revoke, peer
command counts and durable intent/outcome records are checked, and indexed
post-revocation windows at 256, 278 and 300 operations expose growth after
response-cache capacity. Total cache warm-up and post-capacity growth are
separate advisory signals. Small RSS changes or allocator reservation do not
establish a leak; investigate sustained growth before raising a ceiling.

For a longer Linux diagnostic, reuse the same fixture after the Release build
(the 180-second operation deadline still applies):

```sh
cd scripts/dev
python -c "from pathlib import Path; from runtime_resource_measurement import measure_sample; s=measure_sample(Path('../../build/resources/nomad-runtime').resolve(),1200); print(s['operations']['growth_checkpoints'])"
```

Inspect `/proc/<pid>/status` and `smaps_rollup` for anonymous/file RSS,
private-dirty pages and thread count when a resident slope persists. An optional
`MALLOC_ARENA_MAX=1` environment control can distinguish Linux allocator effects;
label it separately and never pool it with the default workload/budget profile.
The diagnostic output is a fixture observation, not a verified policy report.

On regression, reproduce the same profile, inspect the linked-input inventory
and state windows, then review a baseline/policy change with explicit headroom.
Never weaken authentication, durable audit or final-send fencing to meet a
resource budget. Software-peer results do not qualify aircraft startup or
flight performance.

## MAVSDK transport and authority checks

These checks use deterministic peers and the pinned fork. They do not need a
flight controller:

```sh
pixi run verify-mavsdk-provenance
pixi run test-mavsdk-connectivity
pixi run build-mavsdk-transport-qualification
pixi run test-mavsdk-transport-qualification
pixi run test-mavsdk-authority-wire
```

The connectivity task runs local UDP peer and provenance cases. The transport
qualification task builds the non-installed `nomad-qualification` driver and
checks MAVSDK command, telemetry, link, fence and velocity cases. The explicit
build task produces the driver and wire probe; the test task also ensures its
build is current. The final authority-wire task uses independent UDP peers to
check the supported `COMMAND_LONG` and `COMMAND_INT` retry/admission boundary.
The driver is test tooling, not an alternate production CLI. See
[MAVSDK adoption](mavsdk-adoption.md) and
[dependency provenance](mavsdk-dependencies.md).

## Standalone router and Mission Planner

On Windows, build and test the separate ground-router host with:

```powershell
pixi run build-ground-router
pixi run test-ground-router
```

The second task also runs loopback routing and socket checks. The ground router
build does not install or start a service; its process/configuration contract is
in the [router README](../infra/transport/ground_router/README.md).

The Mission Planner plugin targets Windows, Mission Planner 1.3.83 reference
assemblies and .NET Framework 4.8:

```powershell
pixi run build-plugin-only
pixi run test-plugin-build-only
pixi run lint-plugin
pixi run test-plugin-core-client
pixi run test-plugin-video
```

The build writes `mission_planner/src/bin/Release/NOMADPlugin.dll`; it does not
install it. `test-plugin-config-migration`, `test-plugin-interlock`,
`test-plugin-core-client`, and `test-plugin-duallink` cover focused helpers,
the runtime client and router behavior. Follow the [Mission Planner build and installation guide](../mission_planner/README.md)
and its [packaging guide](../mission_planner/packaging/README.md). Tagged
releases use `.github/workflows/release.yml` to assemble separate plugin and
router ZIP files; a manual workflow dispatch uploads artifacts without
publishing a release.

`test-plugin-video` requires the plugin build and runs fake pipeline lifecycle,
isolated WinForms view/HUD disposal, plugin exit, and owned child-process checks.
It needs no camera, GStreamer installation, hardware or network stream. The
hosted C# build job runs this harness against the built plugin and pinned
Mission Planner references.

## ROS integration

The ROS 2 Humble adapter is CPU-capable and observation-only. To build its image
and run the observer against the simulator:

```sh
pixi run sim-ros-build
pixi run sim-ros-up
pixi run sim-ros-logs
pixi run sim-ros-down
```

`sim-ros-up` starts the ROS observer and Copter SITL stack. To run the adapter
integration suite without SITL, build the image and then run:

```sh
pixi run test-ros-integration
```

That suite runs the real node against an in-process MAVLink responder. It checks
validated GPS/battery translation, freshness and the absence of command output;
it does not qualify ROS commands, VIO, camera input, GPU workloads or a flight.
The package-specific parameters and topics are in the
[ROS README](../ros2/nomad_ros/README.md).

The optional Isaac ROS/Jetson images are separate. They require the matching
NVIDIA/Isaac base image and hardware or a compatible self-hosted runner; the
normal CPU integration workflow does not build them. See
[`docker.yml`](../.github/workflows/docker.yml).

## SITL and live MAVSDK smoke

The standard Copter SITL stack requires Docker and the local
`nomad-sitl:copter-4.7.1` image. The Compose file documents how to build it from
the pinned ArduPilot SITL Docker source. Start the stack and run its loop-closure
scenario with:

```sh
pixi run dev-up
pixi run sitl-scenario
pixi run dev-down
```

`pixi run sitl` combines the core build, stack start and default scenario.
Current C++ scenario tasks start with `core-sitl-`; they build the
non-installed qualification driver and use the configured isolated simulator.
The pinned QuadPlane image and tasks are `quadplane-sitl-build`,
`quadplane-sitl-up`, `core-sitl-quadplane-observe`,
`core-sitl-quadplane-vtol-takeoff`, `core-sitl-quadplane-transition`,
`core-sitl-quadplane-route`, `core-sitl-quadplane-recovery`,
`core-sitl-quadplane-transition-back`, and
`core-sitl-quadplane-vtol-landing`. The simulator-only disarmed receiver-fault
probe is `core-sitl-quadplane-rc-loss-probe`.

`run-mavsdk-sitl-smoke` tests connect/status against a running Copter SITL.
Keep scenarios serial because they share vehicle state. See the
[scenario guide](../tests/sitl/README.md) and
[current evidence limits](qualification.md#sitl-and-ros-readiness) before
interpreting results. A configured workflow is not a passed run; live SITL is
not hardware qualification.

Hosted `test.yml`, `lint.yml`, `ros-sim.yml` and `csharp.yml` run pull-request
checks for their configured scopes. `sitl.yml` runs a path-triggered reduced
connectivity smoke on selected pushes to `main`; its full Copter and QuadPlane
jobs run nightly or by manual dispatch, not on every PR. `docker.yml` is manual
and targets self-hosted Jetson/GPU runners. The
[qualification page](qualification.md) records the exact current-base run and
its limits.

## Package, stage and install

Core build, package creation, staged verification and deployment are separate:

```sh
pixi run build-core-release
pixi run package-core
pixi run verify-core-package
pixi run verify-core-staged-install
pixi run install-core <prefix>
```

`build-core-release` builds the Release `nomad` and `nomad-runtime` executables.
`package-core` writes ZIP/TGZ archives. `verify-core-package` checks the
archives; `verify-core-staged-install` installs into `build/package/stage` and
checks that temporary tree. Only `install-core` writes to its required prefix.
The runtime package includes both production executables. It does not include
the direct qualification driver. No rollback/activation workflow is implied by
these build and staging tasks.

For Mission Planner, `build-plugin-only` compiles the DLL without touching the
Mission Planner installation. Installation uses the separately staged plugin
ZIP or `mission_planner/packaging/INSTALL.ps1` and changes the local Mission
Planner deployment. The standalone router is a separate process and package;
the plugin installer does not install or supervise it.

## Release identity audit (versioned deployment slice)

The pre-slice release workflow published only the Windows plugin and router,
while CPack produced core ZIP/TGZ archives through a separate local flow. It
could publish these two components without any core artifact. No manifest bound
the artifacts to the same source SHA, pinned MAVSDK SHA, platform or digest.
CMake/runtime used the fixed 0.1.0 project version, the plugin reported 0.2.0,
and router management reported nomad-link-router-1. A v* workflow trigger did
not validate a semantic release tag or reconcile these versions. Dispatch used
the branch name as an artifact version. Staged-install checks validated core
contents and offline behavior but established neither deployed identity nor
rollback. PR56 service lifecycle and PR57 resource/provenance checks remain
required inputs; they do not supply a release-set identity.

The executable disproof check for the new model is release manifest tests:
aggregation must reject absent components, changed bytes, mismatched embedded
source identity and unsupported metadata before any publication or activation.

A complete release uses four independently activated component payloads: Linux
x86-64 core TGZ, Windows x86-64 core ZIP, Windows x86-64 standalone router ZIP,
and Windows AnyCPU Mission Planner plugin ZIP targeting Mission Planner 1.3.83.
An additional deployment-tools ZIP contains the reviewed operator CLI and service
adapter support. The aggregate job requires all four payloads, their embedded
`package-identity.json`, their real required files and their common source identity
before writing `release-manifest.json` and `SHA256SUMS`. Only a tag push publishes;
a dispatch always remains an inspectable workflow artifact, including a dispatch
against a tag. Checksums correlate bytes; they provide no publisher authentication.

A clean exact `vX.Y.Z` tag supplies the numeric CMake project/CPack version and
component versions. Runtime, plugin and router management report that component
version. Otherwise the numeric CMake version is 0.0.0, the component identity is
`0.0.0-dev.<full source SHA>`, and the release-set identity is `dev-<full source SHA>`.
`source_dirty` records source modifications and nonignored untracked files in development builds; their
source SHA identifies the base revision, not uncommitted bytes. Package SHA-256
always identifies actual archive bytes. Official builds reject tracked source
modifications and nonignored files. A test-only `NOMAD_FIXTURE_VERSION` permits distinguishable runtime
A/B builds with `BUILD_TESTING=ON`; it cannot be enabled in production packages.
The aggregate job never rebuilds components or substitutes a different revision.
