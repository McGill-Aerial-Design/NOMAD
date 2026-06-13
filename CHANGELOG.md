<!-- SPDX-License-Identifier: Apache-2.0 -->
# Changelog

All notable changes to NOMAD are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
