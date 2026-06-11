# NOMAD Refactoring Plan — drift removal, dead-code purge, contract hardening

> Execution plan for branch `baseline-polish` (audit date 2026-06-10). Check items
> off as they land. This document complements [REFACTORING_PROMPT.md](REFACTORING_PROMPT.md)
> (house rules, SC protocol) and defers to [docs/safety/](docs/safety/) on anything
> flight-related; where they conflict, those documents win.
>
> Tier rules apply to every item: SC changes follow the SC protocol (name the
> `SR-*` requirement, tests before the move, `pixi run cov-safety` stays at 100%
> branch). Items are tagged **[SC]**, **[SR]**, or **[NC]**.

---

## Audit summary (what this plan fixes)

The SC core ([edge_core/safety/](edge_core/safety/)) is clean and the adapters
genuinely delegate to it — the partition holds. The volume and drift problems
live one layer out:

- **F1 — Wired-in-name-only services.** ~3,400 lines of Python services are
  packaged and referenced from `app.state` but never initialized:
  `video_stream_manager.py` (709 lines, init never called → every `/api/video/*`
  route returns "not initialized"), `infra/tailscale/src/` (~1,130 lines →
  `/network/status` always returns nulls), `operational_mode.py`,
  `modules/slam/isaac.py`'s `IsaacROSBridge`, `services/ipc.py` (+ `pyzmq` dep),
  `services/logging_service.py`.
- **F2 — Two context objects, one silent wiring break.** `api.py:381` attaches
  the docker-probe helper to `ApiRouteContext`; `IsaacModule` reads it via
  `AppContext.get_service` — never found, so nvblox/bridge process detection
  silently never runs.
- **F3 — Stub endpoints reporting fake success.** `vio.py` `reset_origin` /
  `area/save` / `area/load` / `area/relocalize` return `{"success": true}` while
  doing nothing; `/api/spray/calibration` consumes 1 of ~18 pushed fields and
  fabricates a `"persisted"` answer.
- **F4 — Contract gate is path-only; live breaks exist.** GET
  `/api/servo/camera/tilt` is issued by both clients but only POST exists (405);
  the ROS bridge posts `angle=45.0` to an `int`-typed param (422) — the
  RC-driven camera-tilt mirror is silently dead.
- **F5 — Dead control surface in `create_app`.** 105-line `_mavlink_watchdog_loop`
  never scheduled; 4 of 5 DI setters uncalled.
- **F6 — Boilerplate triplication.** `MavlinkCommands` repeats its connection
  guard 15×; `DualLinkSender.Http.cs` has 4 switch tables + 5 hand-rolled HTTP
  helpers; `CubeOutputController` implements motor-test send 3×.
- **F7 — Ground router gaps.** `_localSock` has no watchdog (one socket error
  permanently kills GCS→aircraft uplink); `ActiveLink` mutated from two threads
  unsynchronized; minor dead code.
- **F8 — Vestigial configuration.** `NOMAD_ENABLE_VISION` / `SERVO_MODE` have
  zero readers; C# `UseELRS` has UI but no behavioral consumer.

Decisions taken (2026-06-10):

- **`IsaacROSBridge` (`modules/slam/isaac.py`): DELETE** (with its tests and
  re-exports).
- **`video_stream_manager` and `infra/tailscale`: wire as real `nomad.modules`
  entry points**, simplified to only what the live routes/dashboard actually
  consume.

---

## Phase 0 — Strengthen the gates first

Convert the bug classes below into red CI *before* touching the code they guard.

- [x] **0.1 [SR] Contract gate: verbs, not just paths.** Extend
  [tests/test_client_contract.py](tests/test_client_contract.py) to extract the
  HTTP method from C# call sites (`GetAsync|PostAsync|DeleteAsync|GetLongRunAsync|PostLongRunAsync|GetStringAsync|PostJsonAsync`
  preceding the route literal) and assert `(method, path)` exists in
  `app.openapi()`, not just the path.
- [x] **0.2 [SR] Contract gate: cover the Python client.** Add a scanner over
  [edge_core/ros_http_bridge/node.py](edge_core/ros_http_bridge/node.py)'s
  `_http_get_json` / `_http_post` route literals and assert `(method, path)`
  against the same openapi schema. (This is where F4a/F4b lived — the C#-only
  gate could not see them.)
- [x] **0.3 [NC] Expect both new checks to fail** on the camera-tilt GET and any
  other latent drift; record the failures in this file under Phase 3 before
  fixing them. Do not extend `KNOWN_DRIFT` for new code.

## Phase 1 — Python deletions (verified dead)

Each deletion commit cites the verifying grep in its message.

- [x] **1.1 [SR]** Delete `_mavlink_watchdog_loop` from
  [edge_core/api.py](edge_core/api.py) (lines ~261–366; defined, never
  scheduled; duplicates `infra/systemd/nomad-mavlink-router.service`). (~105 lines)
- [x] **1.2 [NC]** Delete unused DI setters in `api.py`: `set_isaac_bridge`,
  `set_tailscale_manager`, `set_network_monitor`, `set_camera_service`. Inline
  `set_health_monitor` into
  [health_monitor.py:535-541](edge_core/services/health_monitor.py#L535-L541).
- [x] **1.3 [NC]** Delete [edge_core/services/ipc.py](edge_core/services/ipc.py)
  (373 lines, zero consumers), its `edge_core/__init__.py` re-exports, and the
  `pyzmq` dependency from [pyproject.toml](pyproject.toml).
- [x] **1.4 [NC]** Delete
  [edge_core/services/logging_service.py](edge_core/services/logging_service.py)
  (zero consumers) and its re-exports.
- [x] **1.5 [NC]** Delete
  [edge_core/services/operational_mode.py](edge_core/services/operational_mode.py)
  (init never called), the `mode_manager` websocket branch in
  [system.py:265-270](edge_core/api_routes/system.py#L265-L270), and
  `app.state.mode_manager`.
- [x] **1.6 [NC] DELETE `IsaacROSBridge`** (decision 2026-06-10):
  [edge_core/modules/slam/isaac.py](edge_core/modules/slam/isaac.py) (382 lines),
  `tests/test_isaac_bridge.py`, the optional-import block + `_ISAAC_AVAILABLE`
  exports in [edge_core/__init__.py](edge_core/__init__.py), and
  `app.state.isaac_bridge`. Note: `api_routes/isaac.py` (the `isaac_mgmt`
  container-control module) **stays** — only the unwired bridge class goes.
- [x] **1.7 [SC]** Delete dead `MavlinkCommands` methods (no production callers):
  `send_gimbal_command`, `send_gimbal_rate_command`, `send_obstacle_distance`,
  `send_vision_position_estimate`, `send_vision_speed_estimate`. Update
  `tests/test_safety_command_surface.py` to pin the reduced surface. SC
  protocol applies: behavior-preserving deletion, cite the audit grep.
- [x] **1.8 [SC]** Remove the `_clamp = clamp` back-compat alias in
  [mavlink_velocity.py:61](edge_core/ros_http_bridge/mavlink_velocity.py#L61)
  (update any test imports); delete `MavlinkServo.enable/disable` +
  `ServoController.enable_all/disable_all` (the `_enabled` flag gates nothing)
  and the single-member `ServoFunction` enum in
  [servo.py](edge_core/modules/payload/servo.py).
- [x] **1.9 [NC]** Delete `ModuleRegistry.configure_all` / `register_routes`
  (test-only duplicates of `wire_safe`) and the unused `specs=` parameter of
  `wire_modules`; migrate the one test in `tests/test_module_registry.py`.
- [x] **1.10 [NC]** Delete vestigial config plumbing: `--no-vision` /
  `NOMAD_ENABLE_VISION` and `--servo-mode` / `SERVO_MODE` from
  [main.py](edge_core/main.py), `config/profiles/*.env`,
  `config/nomad.env.example`, `scripts/profile.py`, `scripts/nomad-profile`,
  `scripts/setup/setup_jetson_remote.py`, `docker/Dockerfile.dev`,
  `docker/docker-compose.dev.yml`, and docs. (No edge_core code reads either;
  payload uses `NOMAD_ENABLE_SERVOS`.)
- [x] **1.11 [NC]** Fix `edge_core/__init__.py` `__all__`: `"SystemState"` is
  listed but never imported (`from edge_core import SystemState` raises).
  Import it from `services.models` or drop the entry.

## Phase 2 — Wire the keepers as real modules (decision 2026-06-10)

Both modules follow the `NomadModule` lifecycle
(`configure(ctx)` → `register_routes(app)` → `start()` → `stop()`), register via
the `nomad.modules` entry-point group in [pyproject.toml](pyproject.toml), and
get an `enable_flag`.

- [x] **2.1 [NC] `video` module.** Wrap
  [video_stream_manager.py](edge_core/services/video_stream_manager.py) in a
  `VideoStreamModule` (suggested: `edge_core/services/video_module.py`):
  - [ ] `configure`: construct the manager from ctx config
        (`ISAAC_CONTAINER_NAME`, stream ports); register as service
        `video_stream_manager`; `enable_flag="NOMAD_ENABLE_VIDEO"`,
        `enabled_by_default=True`.
  - [ ] `start`/`stop`: own the watchdog thread lifecycle (replace the
        module-level `init_video_stream_manager` global/singleton; delete the
        deprecated `auto_start` parameter and its warning).
  - [ ] Move route lookup in
        [video_slam.py](edge_core/api_routes/video_slam.py) from the global
        `get_video_stream_manager()` to the registered service / `app.state`.
  - [ ] **Simplify to what the routes consume.** The live consumers are
        `/api/video/bridges`, `/api/video/bridges/start`, `/api/video/source`,
        `/api/video/overlay/{action}` plus the C# callers
        (`GetVideoBridgesStatusAsync`, `StartVideoBridgesAsync`,
        `NOMADVideoView`). Delete manager methods/fields with no caller after
        that wiring (audit each public method; target ≤ ~450 of the current
        709 lines).
  - [ ] Add `tests/test_video_module.py`: module wires, routes respond without
        Docker present (graceful degraded payloads, not 500s).
- [x] **2.2 [NC] `network` module.** Wrap `infra/tailscale/src/`
  (`TailscaleManager`, `NetworkMonitor`) in a `NetworkModule`:
  - [ ] `configure`: `init_tailscale_manager` / `init_network_monitor` from ctx
        config (`GCS_IP` / tailscale env); register services + set
        `app.state.tailscale_manager` / `network_monitor`;
        `enable_flag="NOMAD_ENABLE_NETWORK_MONITOR"`, `enabled_by_default=True`.
  - [ ] `start`/`stop`: own the monitor thread lifecycle.
  - [ ] **Simplify to what `/network/status` and the C# dashboard consume**:
        tailscale `status/ip/hostname/peer_count/latency_ms`, monitor
        `internet_reachable/tailscale_reachable/modem`. Delete unconsumed
        surface (audit `network_monitor.py`'s 732 lines against actual readers;
        target ≤ ~400). Keep the standalone watchdog shell scripts
        (`infra/tailscale/scripts/`) untouched — they are deployment infra, not
        app code.
  - [ ] Add `tests/test_network_module.py`: module wires; `/network/status`
        returns real (or cleanly-degraded) data instead of permanent nulls.
- [x] **2.3 [NC]** Register both in `pyproject.toml`
  `[project.entry-points."nomad.modules"]` and document the enable flags in
  [docs/configuration.md](docs/configuration.md) and `config/nomad.env.example`.
- [x] **2.4 [NC]** Remove the now-satisfied `app.state.* = None` placeholders in
  `create_app` for `tailscale_manager` / `network_monitor` (modules set them),
  and delete `app.state.camera_service` outright (nothing ever sets or reads a
  real one).

## Phase 3 — Python contract and wiring fixes

> Phase 0.3 recorded failures (2026-06-10), from the upgraded gates before any
> fix: C# scanner — `('get', '/api/servo/camera/tilt')` from `SLAM3DView.Data.cs`
> (only POST exists, 405). Python-bridge scanner — `('get',
> '/api/servo/camera/tilt')` from `node.py` `_poll_gimbal_angle`. No other
> latent drift found beyond the existing `KNOWN_DRIFT` ledger. Both fixed by 3.2.

- [x] **3.1 [SR] Fix the silent docker-probe wiring break (F2).** Move the
  `pgrep`-in-container probe into `IsaacModule.configure` (or register it as a
  named `AppContext` service from `create_app`). Delete
  `ApiRouteContext.docker_exec_bash_success`. Add a regression test asserting
  `IsaacModule.cmd_success` is non-None after `wire_modules`. Decide whether
  `ApiRouteContext` itself survives; if `AppContext` can serve the legacy route
  files, unify on it.
- [x] **3.2 [SR] Camera-tilt contract (F4).** In
  [calibration.py](edge_core/api_routes/calibration.py):
  - [ ] `set_camera_tilt(angle: int = 90)` → `angle: float`, validated to
        [0, 180].
  - [ ] Add `GET /api/servo/camera/tilt` returning `{"angle": <current>}` from
        `ServoController.get_camera_tilt()` (503 when uninitialized).
  - [ ] Update [node.py](edge_core/ros_http_bridge/node.py)
        `_poll_gimbal_angle` to read `angle` (drop the phantom
        `feedback_angle` key).
  - [ ] Add the GET route to the internal-bridge allowed routes in `api.py` if
        the bridge polls it (it does, at 10 Hz with 0.1 s timeout).
- [x] **3.3 [SR] Honest `/api/spray/calibration`.** Typed Pydantic request model
  containing only consumed fields (today: `water_pump_relay_number`), with
  `extra="forbid"` (or documented accepted-and-ignored); remove the fabricated
  `"persisted"` key. Trim the C# push payload
  (`NOMADSettingsForm.ServosTab.cs`, `NOMADPlugin.VideoAndLink.cs`) to match;
  the ~18 `Spray*` gains in
  [NOMADConfig.Spray.cs](mission_planner/src/Config/NOMADConfig.Spray.cs) that
  no longer travel become C#-local or get deleted with their UI.
- [x] **3.4 [SR] vio.py stubs stop lying (F3).** `reset_origin`, `area/save`,
  `area/load`, `area/relocalize`: implement against the real ZED service if it
  exists in this baseline; otherwise return 501 and delete the C# callers
  (`ResetVioOriginAsync`, `SaveAreaMapAsync`, `LoadAreaMapAsync`,
  `RelocalizeAreaMapAsync`) and their buttons.
- [x] **3.5 [SC] De-boilerplate `MavlinkCommands`** (behavior-preserving; keep
  `tests/test_mavlink_fence.py` green unchanged):
  - [ ] One connection-guard helper replacing the 15× repeated
        `hasattr(self, "_conn")` + try/except + `logger.debug` pattern,
        preserving exact return-False failure behavior.
  - [ ] Single shared velocity `type_mask` constant in `edge_core.safety`
        (policy currently restated in
        [commands.py:238](edge_core/services/mavlink/commands.py#L238) and
        [mavlink_velocity.py:84](edge_core/ros_http_bridge/mavlink_velocity.py#L84)).
  - [ ] Replace the implicit mixin contract (`hasattr(self, "_conn")`,
        `getattr(self, "state_manager", None)`) with declared abstract
        attributes or a `Protocol` so mypy enforces it.
- [x] **3.6 [SC] Servo adapter tightening.**
  - [ ] Route `MavlinkServo.set_pwm` through
        `edge_core.safety.validate_servo_command` (the camera-tilt path
        currently transmits an unvalidated locally-computed pulse).
  - [ ] Remove the duplicated inline range checks in
        [calibration.py:61-66](edge_core/api_routes/calibration.py#L61-L66) so
        rejection text comes from the SC core only (preserve the message
        verbatim or update the test with it).
  - [ ] Replace `init_servo_controller`'s mutation of the private
        `_controller._mavlink_service` with a constructor arg or public setter.
- [x] **3.7 [SR] Shrink `create_app`.** After 1.1–1.2 and 2.4, `api.py` holds
  only app construction, CORS, auth middleware, and route registration. Target
  ≤ 280 lines (from 399).

## Phase 4 — C# control-path consolidation

- [x] **4.1 [NC] Burn down `KNOWN_DRIFT` (all 11 rows).** Delete callers and
  ledger rows together (the two-way check enforces it). Do not re-add routes
  server-side to satisfy dead buttons:
  - [x] `/api/isaac/launch-nvblox` + `/api/isaac/stop-nvblox` —
        `LaunchNvbloxAsync`, `StopNvbloxAsync`, `StopSlamAsync`,
        ServiceControlPanel nvblox buttons
  - [x] `/api/isaac/logs` — `GetIsaacRosLogsAsync`
  - [x] `/api/slam/status` — `GetSlamStatusAsync`, ServiceControlPanel SLAM row
  - [x] `/api/slam/clear` — `ClearSlamAsync` + SLAM3DView.Actions caller
  - [x] `/api/admin/git-update` — EnhancedHealthDashboard update button
  - [x] `/api/admin/upload-gdrive-token` — Settings uploads tab
  - [x] `/api/detections` — SLAM3DView overlay poll
  - [x] `/api/servo/status` — SLAM3DView servo poll
  - [x] `/api/tools/rviz2/start` + `/stop` — Rviz2View (delete the view if it
        has no remaining function)
- [x] **4.2 [NC]** Delete `DualLinkSender.SendMAVLinkCommand` (private, zero
  callers, ~75 lines) and `GetIsaacRosContainerStatusAsync` (passes a raw
  docker string where a whitelist key is required — cannot ever succeed).
- [x] **4.3 [NC]** Delete `MAVLinkConnectionManager.ProcessHeartbeat`,
  `TrackPacket`, `GetBestAvailableLink` (zero callers); drop the unused `force`
  parameter of `SwitchToLink`.
- [x] **4.4 [NC]** Delete `UseELRS`: config property, settings checkbox
  (`NOMADSettingsForm.VideoTab.cs`), and the two status-text reads in
  `NOMADPlugin.cs`. The "ELRS instead of HTTP" mode does not exist.
- [x] **4.5 [SC-adjacent] Collapse `CubeOutputController` motor-test
  triplication.** `SendMotorTestPwmAsync` becomes the single implementation;
  sync wrapper delegates; `SendMotorTestPwmBatch` loops over it under one lock
  acquisition. Preserve PWM bounds (500–2500) and timeout clamp verbatim.
- [x] **4.6 [SC-adjacent] Eliminate `async void` in the payload command path.**
  [PayloadActions.cs](mission_planner/src/Payload/PayloadActions.cs) methods
  become `async Task` with top-level try/catch logging via `Log`; joystick call
  sites use an exception-observing fire-and-forget helper (see
  [UiAsync.cs](mission_planner/src/UI/UiAsync.cs)).
- [x] **4.7 [SC] Relay-interlock asymmetry decision.** `FireRelayAsync`'s
  direct-MAVLink path fires the pump with UI-confirm only, while the HTTP
  fallback enforces the SR-PAY-03 arm→trigger interlock. Either mirror the arm
  step in C# before the direct send, or record the rationale in
  [docs/safety/](docs/safety/) + a code comment citing the requirement ID. Do
  not leave it implicit.
- [x] **4.8 [NC] Unify `DualLinkSender.Http` plumbing.**
  - [x] One private `SendAsync(HttpMethod, path, body, timeout?)` built on
        `HttpJson`, returning `CommandResult`; the five hand-rolled variants
        (`SendHttpGet`, `SendHttpGetLongRun`, `SendHttpDelete`, `SendHttpPost`,
        `SendHttpPostLongRun`) become one-liners or vanish.
  - [x] Replace the four service-name switch tables with one
        exceptions-`Dictionary<string,string>` + `$"{verb}_{service}"`
        composition.
  - [x] Replace `dynamic` health parsing in `GetHealthAsync` with a typed DTO
        matching the actual `/health` schema — delete the `??` dual-name
        chains and the hardcoded 1 TB disk assumption (surface `disk_free_gb`
        as-is).
- [x] **4.9 [SR] Ground router fixes.**
  - [x] Watchdog for `_localSock`: extend `WatchdogReopen` to rebind it and
        restart `LocalRxLoop` when the loop has exited; tolerate the same
        benign `SocketError` set as `UdpRxLoop` before breaking. (Today one
        socket error permanently kills GCS→aircraft uplink.)
  - [x] Make `SetActiveLink` thread-safe (lock around compare-and-set, or
        marshal `MaybePromoteReceivingLink`'s promotion onto the stats tick) —
        it races between rx threads and the stats thread.
  - [x] Delete the dead ternary at
        [GroundLinkRouter.Rx.cs:399](mission_planner/src/Connectivity/GroundLinkRouter.Rx.cs#L399)
        (unreachable after the line-397 early return).
  - [x] Rename `MavlinkFrameParser.CrcErrors` → `ResyncCount` (no CRC is
        validated); drop the redundant `MavlinkFrame.Payload` alias of `Raw`.
- [x] **4.10 [NC] Pre-emptive splits at the 800-line cap.**
  [NotificationService.cs](mission_planner/src/Notifications/NotificationService.cs)
  (797) and [NOMADConfig.cs](mission_planner/src/Config/NOMADConfig.cs) (791)
  cannot absorb another change. Split along existing seams (rules/state vs.
  delivery; persistence/migration vs. properties — the `.Spray.cs` partial
  pattern already exists). Target < 700 each.

## Phase 5 — Protocol, logging, and config unification

- [x] **5.1 [SR] One request-parsing idiom server-side.** Replace repeated
  `await request.body(); json.loads(...)` blocks (vio.py ×4, calibration.py ×2)
  with Pydantic request models — this also makes the openapi schema strong
  enough for the Phase 0 gate to verify field names/types.
- [x] **5.2 [SR] One error envelope.** Server errors are FastAPI `{"detail": …}`
  everywhere; remove the HTTP-200-with-`{"success": false}` pattern, then
  delete `SendHttpPostLongRun`'s application-error double-parse on the C# side.
- [x] **5.3 [NC] Config truth.** After 1.10 and 4.4, regenerate
  [docs/configuration.md](docs/configuration.md) and `config/nomad.env.example`
  so every documented variable has a reader; consider a `NOMAD_*`-has-a-reader
  test in the spirit of `test_api_reference_sync.py`.
- [x] **5.4 [NC] Stale comment fixes.** [DualLinkSender.cs:54](mission_planner/src/Connectivity/DualLinkSender.cs#L54)
  points the command whitelist at `edge_core/api.py`; it lives in
  [terminal.py](edge_core/api_routes/terminal.py). Sweep similar references
  while touching these files.

## Verification gates (every commit)

- [ ] `pixi run cov-safety` — 100% branch coverage on `edge_core/safety/`
- [ ] `pixi run lint-safety` and SC tests green (`test_safety_partition.py`,
  `test_safety_command_surface.py`, `test_safety_traceability.py`)
- [ ] `pixi run test` (coverage ratchet ≥ 25; raise the floor at the end of the
  plan to the new level minus 1 point) · `lint` · `fmt-check` · `typecheck`
- [ ] C#: `scripts/build/build_plugin_windows.ps1` compiles clean;
  `scripts/build/lint_plugin_deadcode.ps1` reports no new (and fewer) orphans
- [ ] Upgraded contract gate green with **zero `KNOWN_DRIFT` rows** at plan end
- [ ] `pixi run dev-up && pixi run test-api` passes; `pixi run sitl-scenario`
  and `pixi run sitl-fence` pass (SR-VEL / SR-FEN evidence unchanged)
- [ ] Camera-tilt live check: POST then GET `/api/servo/camera/tilt`
  round-trips a float angle
- [ ] No fake success: `grep -rn '"success": True' edge_core/api_routes/` shows
  no handler returning success without performing the named action

## End-state metrics

- [ ] Repo total (`git ls-files '*.py' '*.cs' | xargs wc -l`) down ≥ 4,500 lines
  from ~49,960 (Phases 1+4 deletions ~2,300 verified-dead; 1.6 adds ~400;
  module simplification in Phase 2 ~600+)
- [ ] No source file ≥ 800 lines; the two 79x files < 700 after 4.10
- [ ] `pyzmq` gone from `pyproject.toml`
- [ ] `KNOWN_DRIFT` ledger empty
- [ ] Every `nomad.modules` entry point starts in sim mode
  (`pixi run dev` boots with all modules wired, none half-wired)

---

*When the plan is complete: fold the outcome into [CHANGELOG.md](CHANGELOG.md)
and delete this file, per the fold-and-delete convention used for prior plans.*
