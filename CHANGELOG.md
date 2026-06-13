<!-- SPDX-License-Identifier: Apache-2.0 -->
# Changelog

All notable changes to NOMAD are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `PayloadReleaseInterlock` (C# plugin, tier SC): the pure, UI-free arm→confirm
  state machine behind every multi-click-armed payload release on the ground
  station (drop servos + momentary relay/pump fire). N deliberate clicks within a
  rolling window authorize exactly one actuation, then it disarms; a click after
  the window lapses (or a backwards clock) restarts the count.
- `pixi run test-plugin-interlock` (+ `scripts/build/test_plugin_interlock.ps1`):
  a standalone Roslyn (`csc`) assertion harness covering the interlock
  arm/fire/rolling-window-expiry/backwards-clock/reset paths — the GCS
  counterpart of `tests/test_safety_payload.py`, in the same framework-free idiom
  as the existing `test-plugin-geometry`/`-duallink` tests. A new `plugin-tests`
  job in `.github/workflows/csharp.yml` runs the interlock and geometry harnesses
  on `windows-latest` (no .NET SDK, NuGet, or staged Mission Planner needed) —
  the first plugin unit tests to gate in CI.
- `StateManager` and `TimeSyncService` unit coverage: the 10 Hz model-rebuild
  batching contract (immediate vs. rate-limited snapshots, forced updates) and
  the NTP/GPS time-sync status machine (`timedatectl`/socket reachability,
  GPS-offset estimation, `force_sync_from_gps` success/fallback/error paths, the
  monitor loop, and the module lifecycle) — all with subprocess, socket, and the
  system clock mocked. Raises the `pixi run test` coverage floor 60% -> 65%
  (66% actual).
- `docker/Dockerfile.jetson` (+ `docker/ros_entrypoint.sh`): the on-board Jetson
  perception/odometry image, layered on the Isaac ROS dev base — ZED SDK 5.2.3,
  GStreamer, the ZED/nvblox/nav2 ROS2 runtime, `isaac_ros_nvblox_utils`, and the
  self-contained NOMAD ROS-HTTP bridge. Resolves the previously dangling
  `docker/Dockerfile.jetson` references in `.github/workflows/docker.yml` and
  `docs/deployment.md`; the Jetson deployment docs were corrected to describe the
  image accurately (perception container on the Isaac ROS base, not an
  all-in-one) and note the `BASE_IMAGE` prerequisite.

### Changed
- `PayloadControlPanel` (drop + momentary-relay rows) now delegates its
  arm/confirm decision to `PayloadReleaseInterlock`, keeping only the rendering
  and visual-revert timer. Closes the rearchitecture §3.3 GAP "payload release
  logic still mixed with panel chrome" (`docs/safety/partition.md`); behaviour is
  unchanged (3-click drop, 2-click relay fire, 3 s window). Also disposes the
  relay-fire reset timers on teardown (previously leaked).

## [0.2.1] - 2026-06-13

### Added
- ROS-HTTP bridge unit coverage: the `simple_video_bridge` GStreamer pipeline
  lifecycle and HTTP API routing, plus the `mavlink_velocity` connection,
  setpoint-TX, watchdog, and heartbeat-gating paths and the `mesh_packer`
  background sender loop.
- Edge Core API route coverage: the system/network endpoints (health, status,
  network, ping, `/ws/state`), the `services` status fan-out, the whitelisted
  `terminal` runner, and the `calibration`/`isaac` management modules — all
  driven through `TestClient` with subprocess and hardware probes mocked. The
  `pixi run test` coverage floor rises 45% -> 60%.
- CI now compiles the Mission Planner plugin on a `windows-latest` runner
  (`.github/workflows/csharp.yml`): it stages a version-pinned portable Mission
  Planner so the csproj's DLL HintPaths resolve, then builds with MSBuild — a
  real gate (no SDK needed) catching broken references/syntax that the
  whitespace-only `dotnet format` job could not. Triggered on `mission_planner/`
  changes; uploads `NOMADPlugin.dll` as a build artifact.
- Mission Planner plugin release packaging (`.github/workflows/release.yml`): on
  a `v*` tag, the plugin is built and published as a GitHub Release asset — a
  small zip (`NOMADPlugin.dll` + `INSTALL.ps1` + `README.md`) plus the bare DLL —
  so users can install it without cloning or building. `INSTALL.ps1` copies the
  DLL into the per-user `%LOCALAPPDATA%\Mission Planner\plugins` folder (no admin)
  and warns about a stale Program Files duplicate.

### Changed
- Type-checking the safety-critical core is now an enforced CI gate. `edge_core/safety`
  is checked under strict mypy settings (`disallow_untyped_defs` et al.) via the new
  `pixi run typecheck-safety` task, and the lint workflow fails on a type regression
  there. The whole-tree mypy run stays advisory — same asymmetric rigor as
  `cov-safety`/`lint-safety`.
- Mission Planner plugin C# sources are now globbed
  (`<Compile Include="**\*.cs">`) instead of being listed file-by-file, so new
  files and partial-class splits build without hand-editing the csproj. It stays
  a plain MSBuild project (no .NET SDK prerequisite, unlike SDK-style).
- All GitHub Actions workflows pinned to Node-24 action releases ahead of GitHub
  forcing Node 24 on 2026-06-16 and removing Node 20 on 2026-09-16: `checkout` v5,
  `setup-pixi` v0.9.6, `setup-buildx-action` v4, `build-push-action` v7,
  `upload-artifact` v6, `cache` v5, `setup-dotnet` v5, `login-action` v4,
  `metadata-action` v6, `action-gh-release` v3, `configure-pages` v6,
  `upload-pages-artifact` v4, `deploy-pages` v5, `setup-msbuild` v3.
- Safety case docs brought in line with verification reality. The geofence
  containment SITL scenario (`pixi run sitl-fence`) is recorded as run & passing
  (2026-06-11, commit f69a137) and now gating nightly, so SR-FEN-02's loop-closure
  evidence is ✅; SR-FEN-01 (FC-side fence upload) is marked met (verified
  manually before flight); and the SC/NC partition rule is documented as enforced
  by `tests/test_safety_partition.py` rather than a stated intention — leaving no
  open requirements (🟡 0, 🔴 0) in the Python SC tier.
- README quick-start fixed: the `pixi`-install lines sat outside the code fence
  and rendered as oversized headings; install and run steps are now two clean
  fenced blocks.

### Fixed
- [tests] [bug] SITL scenarios now land and disarm before takeoff, so the
  nightly `sitl-fence` job no longer times out waiting for a climb when it
  starts while the prior `sitl-scenario` flight is still returning (RTL). Each
  scenario establishes its own clean ground state, independent of run order.

## [0.2.0] - 2026-06-12

### Added
- `vio` route module (`/api/vio/*`): VIO status and trajectory endpoints fed
  by the ROS-HTTP bridge.
- Boot, VIO, auth-middleware, geospatial, coordinate-math, mesh-packer, and
  MAVLink-velocity unit tests; an api_reference ↔ OpenAPI drift guard.
- Contract gates now verify HTTP verbs for Mission Planner and ROS bridge
  callers, with `KNOWN_DRIFT` burned down to zero rows.
- `pytest-cov` coverage reporting in `pixi run test` with a ratcheting floor.
- `ModuleRegistry.wire_safe()` — the single, fault-isolated module-wiring path.
- `TailscaleManager.reconnect()` (fixes a latent `AttributeError` in the
  auto-reconnect loop); `infra/tailscale` is now type-checked by mypy.
- `video` and `network` modules now own the video-stream and Tailscale/network
  monitor lifecycles through the `nomad.modules` entry-point system.

### Changed
- Module lifecycle now runs through an ASGI `lifespan` context manager instead
  of the deprecated FastAPI `@app.on_event` hooks.
- Edge Core route parsing now uses Pydantic request models for VIO,
  calibration, and spray settings instead of ad hoc JSON reads.
- Camera-tilt control now round-trips a float angle via POST and GET, matching
  both Mission Planner and ROS bridge callers.
- Video and actuator routes use FastAPI error envelopes consistently instead of
  HTTP 200 responses with `{"success": false}`.
- Mission Planner HTTP sender plumbing, service command lookup, ground-link
  router state, and payload async command paths were consolidated and hardened.
- Edge Core fails fast on module-wiring errors outside sim mode (was: log and
  continue half-wired).
- `E501` (line length) is now enforced; `ruff`, pre-commit, and CI pinned to the
  same versions; `setup-pixi` unified across workflows.
- Docs dev server moved to `:8001` to avoid clashing with `pixi run dev`.
- VS Code Docker/Isaac-Sim tasks now call the `pixi run` tasks (one source of
  truth) and the default build task is the cross-platform lint.
- Competition-specific wording removed from code; one-off hardware bring-up
  scripts moved to `scripts/dev/archive/`.
- Oversized Mission Planner C# files split to stay under the 800-line cap
  (`MapOverlayManager`, `LinkHealthPanel`, `NotificationService`,
  `NOMADConfig`).

### Removed
- Verified-dead Python and C# control surfaces from the refactor audit,
  including the unused IPC/logging/operational-mode services, the old
  IsaacROSBridge class, stale VIO/SLAM/admin client calls, and vestigial
  `NOMAD_ENABLE_VISION`, `SERVO_MODE`, and `UseELRS` configuration.
- Isaac Sim (Omniverse) integration — the `Dockerfile.isaac_sim`,
  `docker-compose.sim.yml`, `docker/isaac_sim/` launch scripts, the `isaac_sim`
  route module, the pixi `sim` feature/environment + `sim-*` tasks, the `sim`
  config profile, and all `ISAAC_SIM_MODE` branches. It needs a high-end GPU we
  don't have yet and will return later as a remote simulation server. The
  hardware-free dev stack (Edge Core + ArduPilot SITL) and the on-Jetson Isaac
  ROS bridge are unaffected.
- `tests/test_p3_7_debounce.py` (competition-specific, permanently skipped).

## [0.1.0] - 2026-06-08

### Added
- Initial public baseline: reusable drone edge (FastAPI companion-computer
  service) + Mission Planner ground-control plugin, with the module SDK,
  MAVLink/ROS bridges, video streaming, health monitoring, and payload
  actuation.
