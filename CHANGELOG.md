<!-- SPDX-License-Identifier: Apache-2.0 -->
# Changelog

All notable changes to NOMAD are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Removed
- [core,ros] The Python ROS-HTTP bridge (`edge_core/ros_http_bridge/`) is
  **deleted** — the C++ `nomad_vehicle_node` adapter (`ros2/nomad_ros`)
  replaces it end-to-end: it owns the MAVLink UDP link, the core velocity
  path (`/nomad/cmd_vel` + VIO health/confidence), and publishes `/nomad/*`
  telemetry. The compose `ros`/`gazebo`/`ros-gpu` profiles now run the node
  (fed by SITL / MAVProxy / a real vehicle on :14552); `Dockerfile.jetson`
  and `Dockerfile.sim-isaac` colcon-build `nomad_core` + `nomad_ros`; the
  systemd unit is `nomad-ros-vehicle.service` with
  `scripts/services/nomad_ros_vehicle.sh`; `scripts/nomad` service renamed
  `ros_http_bridge` → `ros_vehicle`; Isaac API routes manage the node
  (status key `vehicle_running`). The bridge's host unit tests
  (`test_bridge_http_client`, `test_coordinate_math`, `test_mavlink_velocity`,
  `test_mesh_packer`, `test_vio_math`) and its route pinning in
  `test_client_contract.py` were deleted with it. (Deletion gate 1, 2026-09-05)

### Added
- [cli] New `nomad velocity` verb: streams velocity setpoints through the C++
  core (armed + GUIDED + fresh VIO gates) for a duration, then reports the
  watchdog stop (`velocity_active`, `watchdog_reason`).
- [test] `tests/sitl/velocity_loop_closure.py` now drives the C++ CLI velocity
  verb against SITL (motion, watchdog stop, mode-gate refusal) instead of the
  retired Python controller — the loop-closure evidence is fully C++-owned.

### Changed
- [plugin,infra] Flight-controller-generic naming: `CubeOutputController` is
  renamed `OutputController` (it drives generic ArduPilot `DO_SET_SERVO` /
  `DO_SET_RELAY` outputs on any board); the services-status key
  `no_cubepilot` / `cubepilot_present` became `no_flight_controller` /
  `flight_controller_present` on both the edge_core route and the plugin
  panel; profiles and deployment wording no longer name a specific board.
  USB device-id matching in `mavlink_router.sh` intentionally still lists
  real vendor by-id strings (CubePilot, Pixhawk, Holybro, mRo, CUAV,
  RadioLink, ArduPilot) to discover any ArduPilot flight controller.
- [plugin] `CubeOutputController` no longer falls back to the edge_core REST
  vehicle routes (`/api/servo/channel/*/pwm`, `/api/servo/shooter/arm`,
  `/api/servo/shooter/trigger`) — the last C# consumers of the `payload` and
  `mavlink` REST modules (deletion-gate step 2, 2026-09-05). Servo and payload
  release now fail closed when neither the core client nor a live MAVLink
  link is available; `SendServoPwmAsync` became a plain `Task<bool>` wrapper.
- [core] The MAVLink codec no longer hand-maintains dialect tables.
  `third_party/ardupilot-mavlink` is pinned as a submodule at the exact commit
  Copter 4.7.1 compiles against, and `scripts/dev/generate_mavlink.py` runs the
  submodule's own mavgen to produce headers into the gitignored build dir
  (`build/generated/mavlink`, wired into CMake). `src/mavlink/protocol.cpp`
  and `fence.cpp` now use the generated message ids, crc_extras, lengths, and
  per-message pack/decode; NOMAD still owns framing, CRC verification, and the
  send -> ack -> state-verify semantics. Golden wire-frame tests are
  byte-identical (all pass unchanged), and the full `core-sitl-*` battery
  re-passes on Copter 4.7.1 (status, command-flow, mission,
  velocity-watchdog, geofence upload/readback, payload). Regenerate after any
  ArduPilot bump with `pixi run generate-mavlink`.
- [dev] ArduPilot SITL image updated from Copter 4.7.0 to **Copter 4.7.1**
  (`nomad-sitl:copter-4.7.1`, built from the `Copter-4.7.1` tag; the CI
  `sitl.yml` build args and docker-compose reference the new tag). All nine
  `core-sitl-*` scenarios re-verified on 4.7.1 (status, command-flow with
  verified goto, mission, velocity-watchdog, geofence upload/readback,
  payload, link-loss, link-recovery) — no wire behavior changes were needed.
  The dev stack now always forwards a MAVLink copy to host UDP 14572 so the
  link-recovery scenario's relay works without restarting the stack with a
  special `NOMAD_CORE_SITL_PORT`.

### Added
- [dev] Watch SITL runs live in Mission Planner: every `core-sitl-*` scenario
  now prints a hint pointing at the dev stack's operator MAVLink link
  (Mission Planner → CONNECT → TCP → 127.0.0.1:5762, via the compose
  `mp_bridge`). The link is a passive multi-client observer, verified to
  stream MAVLink v2 to the host without disturbing scenarios or the core CLI;
  documented in `tests/sitl/README.md` and `docs/development.md`.
- Core output verbs: `servo <channel> <pwm_us>`, `relay <number> <0|1>`,
  `motor-test <instance> <pwm_us> <timeout_s>`, `gimbal-config <mount_mode>`
  and `user-command <7 values>` CLI commands backed by new `Vehicle` methods
  (`set_relay`, `motor_test`, `configure_gimbal`, `send_user_command`;
  `set_servo` moved to `src/vehicle/output.cpp`). Each validates its input
  (relay 0..15, PWM 500..2500, mount mode 0..4, finite parameters, motor-test
  timeout clamped 0.05..3.0 s) and verifies the ArduPilot acknowledgement.
- `NomadCoreClient` methods `Servo`, `SetRelay`, `MotorTest`, `GimbalConfigure`
  and `SendUserCommand` — fail-closed validation gates before spawn, pinned
  argument vectors in `NomadCoreClientTests`.
- [plugin] CubeOutputController discrete servo/relay commands and
  MotorMusicCommand now route through the C++ core client first
  (acknowledged + verified), with the direct MAVLink and REST paths kept as
  transitional fallbacks; `GimbalController.SetMode` (mount configure) is
  core-first. Dead plugin motor-test code (`SendMotorTestPwm*`,
  `MotorTestCommand`) deleted — the capability moved to the core `motor-test`
  verb. High-rate drag/stick streams stay on direct MAVLink by design
  (debt-noted; a core streaming verb is the follow-up).
- [test] ROS adapter command services: `tests/ros/test_nomad_ros_integration.py`
  grows five tests exercising `/nomad/arm`, `/nomad/disarm`, `/nomad/land`,
  `/nomad/rtl` and ACK-rejection against the stateful MAVLink responder —
  each command is sent through the core, acknowledged, and verified via the
  authoritative vehicle state (armed bit / custom mode), with
  `MAV_RESULT_FAILED` surfacing as a failed service response (8/8 in-image).
- [test] `tests/ros/mavlink_wire.py`: structural MAVLink v1/v2 frame decoding
  shared by the ROS integration tests (COMMAND_LONG ids, velocity setpoint
  vx), keeping the assertions independent of a MAVLink parser.
- `NomadCoreClient` (C# plugin): the Mission Planner-free client for the C++
  core CLI boundary — spawns the `nomad` binary with the pinned
  `goto <lat> <lon> <alt> --endpoint <ep>` vector (invariant-culture F7/F1
  formatting), fails closed on non-finite/out-of-range input and on an
  unavailable core, and passes the API key through as `NOMAD_API_KEY`.
- `pixi run test-plugin-core-client` (+ `scripts/build/test_plugin_core_client.ps1`,
  `mission_planner/tests/coreclient/NomadCoreClientTests.cs`): a `csc`
  assertion harness pinning the client boundary — argument vector, invariant
  formatting, lat/lon/alt validation gates, unavailable-core fail-closed, and
  the live CLI authentication gate (actuation without a key is refused before
  any socket work, exercised when the C++ binary is built). Added as a step in
  the `plugin-tests` CI job.
- [test] Video feed/player coverage: `tests/test_mediamtx_config.py` pins the
  single-stream / multi-consumer MediaMTX contract (exactly one `publisher`,
  always-on path; no primary/secondary), and `tests/test_simple_video_bridge.py`
  gains pipeline-content assertions (single `/stream` target, x264
  `zerolatency`/`latency=0`, resolution/bitrate/fps) plus a rapid-switch
  stability test (60 back-to-back topic switches leave exactly one live pipeline)
  and a same-topic-is-a-no-op (seamless) check.
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
  and asserts the mount tracks the commanded pitch and reaches the configured
  limit — read back from the reported attitude (`GIMBAL_DEVICE_ATTITUDE_STATUS`)
  or, on builds that don't publish it, the mount's servo output. Wired into the
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
- [ros2] [ci] `docker/Dockerfile.sim-ros` now compiles `nomad_core` +
  `nomad_ros` into a colcon workspace (`/ws/install`, sourced by the
  entrypoint), replacing the image's Python-only runtime. The root CMake's
  CTest targets are gated on the `tests/` tree so the subproject build stays
  clean without it.
- [ros2] [test] `tests/ros/test_ros_bridge_integration.py` (Python HTTP
  bridge against a stub) is replaced by
  `tests/ros/test_nomad_ros_integration.py`: it runs the real C++
  `nomad_vehicle_node` against an in-process pymavlink MAVLink responder and
  asserts typed telemetry publication, the fail-closed VIO gate (no feed →
  no setpoint), and that a healthy VIO feed + `/nomad/cmd_vel` delivers a
  FLU-to-FRD `SET_POSITION_TARGET_LOCAL_NED` (vx ~= 1.0) to the vehicle.
  `ros-sim.yml` and `pixi run test-ros-integration` now run this suite; the
  workflow triggers now cover the core + adapter sources. The Python bridge
  deletion remains gated on its Jetson/GPU/systemd deployment tail
  (docs/migration.md Phase 7).
- [core] [transport] `wait_for_state` now consumes the entire pending queue
  per call instead of stopping at the first telemetry frame, and
  `receive_message` drains new datagrams even when the queue is non-empty. A
  slow sampler (the ROS node's telemetry timer at a few hertz) previously fell
  behind the MAVProxy stream, leaving heartbeats unread behind a backlog so
  on-demand freshness checks reported a healthy link as stale and every
  velocity command was refused. Found by the live ROS-against-SITL proof;
  covered by a link-flood regression test in `tests/udp_connection_test.cpp`.
- [ros2] [fix] `nomad_vehicle_node` never set `vio_sample_.updated` in its
  VIO health/confidence callbacks, so `take_latest_vio_sample` always reported
  no feed and every `/nomad/cmd_vel` was refused with "without a VIO feed".
  The flags are now set on each callback; the node drove the live Copter 4.7.0
  SITL vehicle over `/nomad/cmd_vel` end-to-end after the fix.
- [ros2] `nomad_ros` compiles and passes its 12 translation unit tests in a
  ROS 2 Humble image; the live-feed half of deletion-gate step 1
  (docs/migration.md Phase 7) is closed.
- [hardware] NOMAD is no longer documented or discovered as Cube-Orange-specific:
  it targets ArduPilot on any flight controller. The mavlink-router USB discovery
  (`scripts/services/mavlink_router.sh`) now matches common ArduPilot boards
  (Pixhawk, Holybro, mRo, CUAV, RadioLink, ArduPilot) in addition to Cube, and
  plugin payload wording/config comments describe generic ArduPilot
  servo/relay channels (`DO_SET_SERVO` / `DO_SET_RELAY`) rather than Cube
  outputs; the C++ core already used only board-independent MAVLink and
  SERVO1-16/RELAY channels.
- [plugin] [boundary] `GuidedGoto` (BoundaryManager soft-boundary return) now
  routes through `NomadCoreClient` to the C++ core CLI instead of the plugin's
  MAVLink path: the core sends MAV_CMD_DO_REPOSITION with the change-mode flag
  and verifies the arrival position, so a "return to boundary" means the
  vehicle is actually at the target. `FlightModeController.Initialize(config)`
  is wired into plugin load; the dead reflection tail, `SetMember`, and the
  caller-less `SetGuidedMode` were deleted.
- [plugin] Flight Boundaries view spacing tightened further (smaller card
  padding, inter-card margin, section-title gap, and row gaps) so the SOFT/HARD
  boundary and IMPORT/EXPORT sections pack closely without needless scrolling.
- [edge] [plugin] The Jetson now exposes a **single** RTSP stream at the
  canonical path `stream` — replacing the old `/primary` + `/secondary` (and the
  legacy `zed`/`live`/`dynamic`/wildcard) paths. MediaMTX serves it from one
  always-on `source: publisher` path, so any number of viewers can read it
  concurrently while the content (which ROS topic) is switched live via the
  bridge API. The video bridge (`--rtsp-path`/default), `VideoStreamManager`
  default URL, the `/api/video/bridges` JSON key (`primary` → `stream`),
  `/api/stream/info`, the plugin's RTSP URLs + status read, and the env/docs are
  all aligned on `stream`.
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

### Fixed
- [core] [bug] The UDP MAVLink transport was single-consumer: with a
  concurrent telemetry pump (the ROS node's 10 Hz timer) a command
  acknowledgement could be consumed before `send_command`'s waiter saw it,
  timing out commands that a single-threaded CLI never lost.
  `send_command` now holds the receive mutex for the whole exchange and the
  receive/state helpers use locked variants; the fence helpers follow. Found
  by the new ROS command-service tests.
- [plugin] [bug] Gimbal joystick window keeps the NOMAD theme when it loses focus
  — Mission Planner's `ThemeManager` recolors plugin windows on every
  activation/deactivation (turning the buttons grey/green and the background
  light), so the window now re-asserts its palette on focus changes (the same
  pattern the main screen uses). This also fixes the "Center 0°/0°" and
  "Look Down −90°" preset buttons that were left green.
- [plugin] [bug] Gimbal joystick PITCH/ROLL axis labels sit just outside the ring
  in a reserved gutter — clear of the circle without spilling past the pad edge.

### Removed
- [infra] The separate `/primary` and `/secondary` MediaMTX stream paths (and the
  legacy `zed`/`live`/`dynamic` + wildcard `all_others` paths) — collapsed into
  the single `stream` path.

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
