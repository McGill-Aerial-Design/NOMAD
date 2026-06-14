<!-- SPDX-License-Identifier: Apache-2.0 -->
# Changelog

All notable changes to NOMAD are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `GimbalCommand` (C# plugin): the pure, Mission-Planner-free core of the gimbal
  control — the `DO_MOUNT_CONTROL`/`DO_MOUNT_CONFIGURE` command frames sent to
  ArduPilot plus the stick-integration and angle-clamping math, extracted from
  `GimbalController` so it is unit-testable offline (the `PayloadReleaseInterlock`/
  `GeoMath` idiom). `GimbalController` now delegates to it and just maps the frame
  onto `MAVLink.MAV_CMD` for the send.
- `pixi run test-plugin-gimbal` (+ `scripts/build/test_plugin_gimbal.ps1`,
  `mission_planner/tests/gimbal/GimbalCommandTests.cs`): a `csc` assertion harness
  pinning the exact MAVLink the gimbal emits — command ids (205/204), the param
  layout (P1=pitch, P2=roll, P7=MAVLINK_TARGETING; P1=mode), `MAV_MOUNT_MODE`
  values, and the integrator/clamp math. Added as a step in the `plugin-tests`
  CI job.
- `pixi run sitl-gimbal` (+ `tests/sitl/gimbal_mount_control.py` and its skipped
  pytest wrapper): an end-to-end SITL scenario that configures a servo mount on a
  live ArduPilot, sends the same `DO_MOUNT_CONFIGURE`/`DO_MOUNT_CONTROL` commands,
  and asserts the mount's reported attitude (`GIMBAL_DEVICE_ATTITUDE_STATUS`)
  tracks the commanded pitch and reaches the configured limit. Wired into the
  nightly `sitl.yml` as the last scenario (it reboots the shared vehicle to apply
  `MNT1_TYPE`).
- Shared responsive-UI foundation for the Mission Planner plugin (`mission_planner/src/UI/`):
  typography + spacing constants and font helpers on `NOMADTheme`
  (`Font()`/`Mono()`, `SIZE_*`, `PAD`/`GAP`, named `CONTROL_BG`/`PANEL_ALT`
  surfaces), plus reflowing builders on `ControlFactory` (`Card`, `LabeledRow`,
  `ButtonRow`, `SectionTitle`, themed `CheckBox`/`Numeric`/`TabControl`/`ListBox`,
  and the `BoundaryGrid` skin). These replace the per-view ad-hoc fonts/colors and
  absolute positioning, so the subsequent view-layout work sizes dynamically to
  any aspect ratio without overlap.
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
- Simple video bridge fallback + entrypoint coverage (`tests/test_simple_video_bridge.py`):
  the appsrc GStreamer fallback (with bindings present and absent), the
  python-subprocess pipeline (success/failure), the kill-after-terminate-fails
  teardown branch, and `main()` (serve-then-shutdown and Ctrl-C) — taking
  `services/ros/simple_video_bridge.py` to 99% (only the `__main__` guard left).
  Raises the `pixi run test` coverage floor 84% -> 85% (85.40% actual).
- `MavlinkCommands` unit coverage (`tests/test_mavlink_commands.py`): the MAVLink
  command builders driven with a fake pymavlink link and the command-ack sender
  stubbed — the `_send_guarded` link guard (no-connection / None-result /
  truthy-falsy / exception), `arm_disarm`, `trigger_payload`, `set_relay`,
  `set_mode`/`land`, `takeoff`, `request_home_position`, velocity (default/custom
  frame, `stop_velocity`), `send_statustext` (truncation + severity), and the
  global/local position targets (link guard + yaw mask; fence gating stays in
  `test_mavlink_fence`) — taking `services/mavlink/commands.py` to **100%** (total
  84.70%). `cov-safety` unchanged (no SC source touched).
- Payload actuation I/O coverage (`tests/test_payload_servo_io.py`,
  `tests/test_payload_module.py`): the non-safety MAVLink-I/O surface of the servo
  adapter — `MavlinkServo` (channel validation, angle clamp/transmit, PWM validate/
  no-link/result, pulse mapping), `ServoController` (camera tilt config + last-angle
  preservation, status, lifecycle, defensive branches), the module-global
  controller helpers, and the `PayloadModule` arm/trigger command routes (200/400/
  409/503) — taking `modules/payload/servo.py` and `services/payload_module.py`
  both to **100%**. The SR-PAY-01/02/03 fault-injection guarantees stay in
  `tests/test_payload_servo.py`; `cov-safety` remains 100%. Raises the `pixi run
  test` coverage floor 82% -> 84% (84.24% actual).
- `TailscaleManager` unit coverage (`tests/test_tailscale_manager.py`): the
  Tailscale status poller exercised with the module-level `_run` wrapper stubbed —
  the `parse_status_json` state map + IPv4 extraction, `_check_status`
  (not-installed/error/parsed/bad-JSON), `_reconnect` (`tailscale up` ok/fail),
  the auto-reconnect monitor loop, and the thread lifecycle — taking
  `infra/tailscale/tailscale_manager.py` from 72% to **100%** (total 82.62%).
- `NetworkMonitor` unit coverage (`tests/test_network_monitor.py`): the 4G/LTE
  modem + connectivity monitor exercised with the module-level `_run` shell
  wrapper stubbed to a command-dispatching fake — the `nmcli`/`mmcli`/`ip`/`ping`
  parsers (RSRP→quality/percent, NM connection lookup + fuzzy LTE-profile
  heuristics, ModemManager modem read with signal/bearer fallbacks, interface
  guessing, ping RTT), the NM+MM merge in `_check_modem_status`, `check_connectivity`,
  and the thread lifecycle — taking `infra/tailscale/network_monitor.py` from 42%
  to **100%**. Raises the `pixi run test` coverage floor 78% -> 82% (82.06% actual).
- `VideoStreamManager` unit coverage (`tests/test_video_stream_manager.py`): the
  in-container video-bridge controller driven with `subprocess.run` (a
  docker-command-classifying stub) and `urlopen` (a URL->payload map) faked —
  container/relay status probes, every `start_with_reason` failure branch (stale
  adopt, container down, missing script, `docker cp`/`exec` errors and timeout,
  crash/no-frames/uncheckable diagnosis), the crash-recovery watchdog loop
  (dead-bridge restart, stalled-pipeline `/restart`, relaunch fallbacks), and the
  switch-topic / overlay / status HTTP API — taking
  `services/video_stream_manager.py` from 40% to **100%**. Raises the `pixi run
  test` coverage floor 75% -> 78% (78.52% actual).
- `JetsonHealthMonitor` unit coverage (`tests/test_health_monitor.py`): the Jetson
  sysfs/proc readers and `tailscale` status exercised against a fake file system
  (scoped `open`/`os.path.exists`, mocked `os.statvfs`/`subprocess`/`time.sleep`)
  — temperature/CPU-load/GPU-load-EMA/memory/disk/power(INA3221)/throttle/fan
  parsing, the tailscale cache, the `_update_health` orchestration + state push,
  and the thread lifecycle/module wiring — taking `services/health_monitor.py`
  from 27% to **100%**. Raises the `pixi run test` coverage floor 70% -> 75%
  (75.41% actual).
- `MavlinkConnection` unit coverage (`tests/test_mavlink_connection.py`): the FC
  receiver loop driven with `mavutil.mavlink_connection` patched to a scripted
  fake link — endpoint-scheme normalization, the per-message-type dispatch
  (HEARTBEAT arm/mode, SYS_STATUS, GLOBAL_POSITION_INT with/without fix,
  HOME_POSITION, ATTITUDE, SYSTEM_TIME, COMMAND_ACK, SERVO_OUTPUT_RAW), the
  reconnect-on-`None` path, the disconnect watchdog (`LOST` transition), the
  command-ack wait/confirm/timeout, and the health-broadcast loop — taking
  `services/mavlink/connection.py` from 23% to **100%**. Raises the `pixi run
  test` coverage floor 65% -> 70% (70.31% actual).
- `edge_core.ros_http_bridge.vio_math` (+ `tests/test_vio_math.py`): the VIO
  pose/frame math lifted out of the rclpy-only `node.py` into a pure, 100%-covered
  module — covariance→confidence, the REP-103→NED position/velocity/attitude sign
  conventions, the tilt-compensated magnetometer heading (incl. its degenerate
  no-update cases), and the gimbal-camera→drone-body pose composition. 17 tests
  pin the flight-relevant signs that were previously unreachable by the suite.
- `docker/Dockerfile.jetson` (+ `docker/ros_entrypoint.sh`): the on-board Jetson
  perception/odometry image, layered on the Isaac ROS dev base — ZED SDK 5.2.3,
  GStreamer, the ZED/nvblox/nav2 ROS2 runtime, `isaac_ros_nvblox_utils`, and the
  self-contained NOMAD ROS-HTTP bridge. Resolves the previously dangling
  `docker/Dockerfile.jetson` references in `.github/workflows/docker.yml` and
  `docs/deployment.md`; the Jetson deployment docs were corrected to describe the
  image accurately (perception container on the Isaac ROS base, not an
  all-in-one) and note the `BASE_IMAGE` prerequisite.

### Changed
- Dashboard notifications header (`NotificationPanel`) relaid out as a docked
  AutoSize table (title/unread badge left, Clear right) instead of absolute
  positions with a resize handler, and its fonts routed through `NOMADTheme`.
  Completes the responsive sweep of the operator-facing views — the remaining
  views (Video/Terminal/Calibration) are already docked, the Dashboard is a
  percentage `TableLayoutPanel`, and the Settings dialog is an intentionally
  fixed-size `FixedDialog` with `AutoScroll` tabs.
- Health + Links panels relaid out responsively: `ServiceControlPanel` (services
  as a reflowing name/status/buttons table + percentage log split, no fixed
  920×650), `LinkHealthPanel` (header and settings rows are AutoSize cards with
  wrapping flows instead of hardcoded x-positions that overlapped at narrow
  widths), and `EnhancedHealthDashboard` (status + metrics panels as tables so the
  progress bars stretch and values never clip). All routed through the shared
  `NOMADTheme`/`ControlFactory` helpers; behaviour unchanged.
- Flight Boundaries view relaid out responsively: the fixed-height cards with
  absolutely-positioned children are now AutoSize cards of full-width rows
  (wrapping button/control clusters, explicit-height grids), routed through the
  shared `NOMADTheme`/`ControlFactory` helpers. Fixes overlap and clipping at
  narrow widths / non-default aspect ratios; behaviour and all persisted settings
  are unchanged (the cross-partial `Controls.Find(...)` named controls are kept).
- Gimbal control de-branded from "Caddx" to a generic gimbal (window title,
  sidebar button, file headers/comments) so it drives any `DO_MOUNT_CONTROL`
  mount, and the floating joystick window relaid out with a docked
  `TableLayoutPanel` — a fill joystick pad plus AutoSize rows and wrapping button
  rows — so it fits any window size/aspect ratio without overlap. The `MountMode`
  enum is now shared (was duplicated across `GimbalController` and the window).
- `PayloadControlPanel` (drop + momentary-relay rows) now delegates its
  arm/confirm decision to `PayloadReleaseInterlock`, keeping only the rendering
  and visual-revert timer. Closes the rearchitecture §3.3 GAP "payload release
  logic still mixed with panel chrome" (`docs/safety/partition.md`); behaviour is
  unchanged (3-click drop, 2-click relay fire, 3 s window). Also disposes the
  relay-fire reset timers on teardown (previously leaked).
- The ROS-HTTP bridge node (`ros_http_bridge.node`) is now a thin ROS adapter:
  its `_handle_vio`/`_handle_mag`/`_get_drone_body_pose` callbacks delegate the
  frame and pose arithmetic to the new `vio_math` module instead of computing it
  inline. Behaviour-preserving; the only untested file left in the bridge package
  shrinks to message unpacking and HTTP forwarding.

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
