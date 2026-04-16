# NOMAD AEAC 2026 — Competition Readiness Checklist

**Audit date:** 2026-04-15
**Time to competition:** ~1 month
**Platform:** ArduCopter quadcopter + Jetson Orin Nano (drone) + Raspberry Pi 5 + Mission Planner (ground station)

This document supersedes the open items in [todo.md](todo.md) (which tracked the prior P0–P3 SLAM/NVBlox audit + REQ-1–REQ-7 audit, mostly complete). Items below are what still needs to happen before flying competition.

Severities:
- **B = Blocker** — system won't ship / a task can't be scored
- **I = Important** — will hurt operations / pilot UX
- **P = Polish** — nice to have
- **D = Dead** — delete / cleanup

---

## 0. Platform change — tiltrotor → quadcopter

The drone is now a standard ArduCopter quadcopter (not the originally-specified Tricopter Tiltrotor). Several files still reference the old platform.

| # | Sev | File | Issue |
|---|-----|------|-------|
| 0.1 | I | [PRD.md:13](PRD.md#L13) | Header: "Tricopter Tiltrotor (Quad Config)" → "Quadcopter" |
| 0.2 | I | [config/nav2_drone.yaml:4-8](config/nav2_drone.yaml#L4-L8) | Comments mention "Tricopter tiltrotor VTOL" and "ArduPlane GUIDED" |
| 0.3 | I | [edge_core/api.py:3522](edge_core/api.py#L3522) | Comment: "ArduPlane has no onboard obstacle avoidance" |
| 0.4 | I | [edge_core/ros/nav2_goal_bridge.py:15](edge_core/ros/nav2_goal_bridge.py#L15) | Docstring: "ArduPlane GUIDED" |
| 0.5 | I | [mission_planner/src/NOMADConfig.cs:372](mission_planner/src/NOMADConfig.cs#L372) | `DroneFrameType` defaults to "Tricopter" — pilot sees wrong frame in 3D viewer until they change it |
| 0.6 | P | [mission_planner/src/SLAM3DView.cs:336](mission_planner/src/SLAM3DView.cs#L336) | Combobox fallback also "Tricopter" |
| 0.7 | P | [mission_planner/src/SLAM3D/Rendering/DroneRenderer.cs:34,82](mission_planner/src/SLAM3D/Rendering/DroneRenderer.cs#L34) | `FrameType` default + `DrawTricopterBody` branch — keep both renderers (safe) but default to Quadcopter |

→ All seven applied as part of this audit (see **Fixes applied** at bottom).

---

## 1. Blockers — must fix before fly

### 1.1 Task 1 building face is hardcoded "Unknown"
- **File:** [edge_core/api.py:2598](edge_core/api.py#L2598)
- **Why it blocks:** AEAC Task 1 verification expects the recon photo metadata to identify which face of the building (N/S/E/W) the target is on. This is currently a literal `"Unknown"` string.
- **Fix:** call into `target_localizer` (it already computes building face — see [edge_core/target_localizer/](edge_core/target_localizer/)) and populate the field.
- **Owner:** unassigned.

### 1.2 Google Drive upload silently fails when token missing
- **File:** [edge_core/gdrive_upload.py](edge_core/gdrive_upload.py), wired in [main.py:486-489](edge_core/main.py#L486-L489)
- **Why it blocks:** Spray verification (SP-007) uploads `Task_2_MAD_target_<n>.jpg` to GDrive. If the OAuth token at `~/.nomad/gdrive_token.json` doesn't exist on the Jetson (likely on a fresh deploy), uploads return empty and the spray sequence reports success despite no evidence being uploaded.
- **Fix options:**
  - Add startup probe: if `GDRIVE_FOLDER_ID` is set but token missing, log WARN and surface `gdrive_ready=false` on `/api/health`.
  - Add an `/api/admin/upload-gdrive-token` endpoint (file already has stub per agent report — verify) and a Mission Planner first-run flow.
  - Or: switch to a service account (`nomad-490717-621014631595.json` looks like one — gitignored, kept locally). Service accounts don't require interactive OAuth.

### 1.3 Servo mount offset hardcoded; no calibration path
- **File:** [edge_core/ros/servo_tf_publisher.py:60-63](edge_core/ros/servo_tf_publisher.py#L60-L63)
- **Why it blocks:** `base_link → servo_mount` static TF assumes 10cm fwd / 5cm down. If physical mount differs by >1cm, VIO + servo aim is off → spray misses target.
- **Fix:** read offsets from `/etc/nomad/servo_mount.yaml` or env vars `SERVO_MOUNT_X/Y/Z`, with current values as fallback. Document the measurement procedure in [docs/SERVO_CONTROL.md](docs/SERVO_CONTROL.md).
- **Action also needed:** physically measure on the actual built drone.

### 1.4 Ballistic drop table is placeholder
- **File:** [edge_core/spray_controller.py:95-101](edge_core/spray_controller.py#L95-L101)
- **Why it blocks:** SP-004 ballistic compensation interpolates from a table with literal sentinel values (1m=0°, 2m=2°, 3m=5°, 4m=8°, 5m=12°). No evidence of physical calibration.
- **Fix:** spray-test at 1m/2m/3m/4m/5m with the actual nozzle and pressure, measure drop, update table. **No coding required — physical test only.**

### 1.5 Mission Planner mode-poll timer never stops on hide/show
- **File:** [mission_planner/src/NOMADTask2View.cs:525-533](mission_planner/src/NOMADTask2View.cs#L525-L533)
- **Why it blocks:** `_modePollTimer` is created in the view ctor and only disposed on form close. Switching tabs back and forth stacks timers → cumulative load on Jetson API.
- **Fix:** Pause on `VisibleChanged`/`HandleDestroyed`, resume on visible.

### 1.6 Hardcoded Jetson IP in three views (not pulled from `NOMADConfig`)
- **Files:**
  - [mission_planner/src/EmbeddedVideoPlayer.cs:89](mission_planner/src/EmbeddedVideoPlayer.cs#L89)
  - [mission_planner/src/Rviz2View.cs:187](mission_planner/src/Rviz2View.cs#L187)
  - [mission_planner/src/ZedCalibrationView.cs:172](mission_planner/src/ZedCalibrationView.cs#L172)
  - [mission_planner/src/FoxglovePanel.cs:393](mission_planner/src/FoxglovePanel.cs#L393)
- **Why it blocks:** if the GCS-side IP override is used (different network on competition day), these screens silently break.
- **Fix:** pull from `NOMADConfig.Instance.EffectiveJetsonIp` (or whatever the canonical accessor is — verify in [NOMADConfig.cs](mission_planner/src/NOMADConfig.cs)).

### 1.7 mavlink-router GCS endpoint hardcoded
- **File:** [transport/mavlink_router/main.conf:94](transport/mavlink_router/main.conf#L94) (`Address=100.76.127.17`)
- **Why it blocks:** if the GCS Tailscale IP differs at competition, no telemetry reaches Mission Planner.
- **Fix:** template the file from [scripts/run/start_nomad_full.sh](scripts/run/start_nomad_full.sh) at startup, sourcing `GCS_IP` from [config/env/jetson.env](config/env/jetson.env).

---

## 2. Important — should fix before fly

### 2.1 Mission Planner missing UI for drift stats
- Backend already exposes per-tilt-cycle drift via `ros_http_bridge` `get_stats()["tilt_drift"]` (REQ VO-006). No GCS display.
- **Fix:** add a small label to [NOMADTask2View.cs](mission_planner/src/NOMADTask2View.cs) polling `/api/vio/stats`. Useful indoor diagnostic.

### 2.2 Mission Planner missing obstacle distance visualization
- Backend publishes 72-sector OBSTACLE_DISTANCE (REQ NV-008). No GCS visualization.
- **Fix:** add a radar-style polar plot or a simple "nearest obstacle (m, bearing)" readout.

### 2.3 No pre-flight ArduPilot param loader
- Param files exist at [config/params/task1_gps.param](config/params/task1_gps.param) + [config/params/task2_vio.param](config/params/task2_vio.param) but nothing pushes them to the Cube via MAVLink before flight.
- **Fix:** add `scripts/run/load_params.py` (using pymavlink) and call it from `start_nomad_full.sh task2`. Or just document the manual MP load step in [docs/OPERATIONS_RUNBOOK.md](docs/OPERATIONS_RUNBOOK.md).

### 2.4 `ENABLE_OD` env var has no default warning
- [api.py:1197](edge_core/api.py#L1197) — if not set, Task 1 target_localizer is silently skipped.
- **Fix:** log WARN at startup if `ENABLE_OD` is unset and Task 1 endpoints are reachable. Even better: surface on `/api/health`.

### 2.5 Target localizer startup race not blocking
- [api.py:1339-1349](edge_core/api.py#L1339-L1349) — only WARNs if ROS services don't appear in 10s. `/api/task/1/target/capture` then fails confusingly.
- **Fix:** make `/api/isaac/start` return HTTP 503 with detail when target_localizer services don't materialize.

### 2.6 `ros_http_bridge` not supervised inside the container
- [docker-compose.yml](docker-compose.yml) (~line 162) — bridge launched as `&`, container healthcheck only hits Edge Core's `/api/health` on the **host**, not the bridge inside the container.
- **Fix:** add a tiny supervisor loop OR add a healthcheck path that pings the bridge's HTTP listener (port 9200) from inside the container.

### 2.7 Task 1 capture path hardcoded to `/home/mad/...`
- [api.py:2445-2456](edge_core/api.py#L2445-L2456) and [main.py:402-404](edge_core/main.py#L402-L404)
- **Fix:** read from env var `NOMAD_TASK1_CAPTURE_DIR` (fall back to current path).

### 2.8 Bare `except: pass` in hot paths
- ~40 silent except blocks across [api.py](edge_core/api.py), [health_monitor.py](edge_core/health_monitor.py).
- **Fix:** at minimum, `except Exception as e: logger.warning("...", exc_info=False)` so the ops runbook can correlate.

### 2.9 Spray button double-click race
- [NOMADTask2View.cs](mission_planner/src/NOMADTask2View.cs) — disables button after click but UI lag on Pi 5 + LTE could allow a second click before the first request returns. Need an interlock flag.

### 2.10 Mode selector list is hardcoded
- [NOMADTask2View.cs:141-150](mission_planner/src/NOMADTask2View.cs#L141-L150) hardcodes the mode strings. If backend modes change, UI silently drifts.
- **Fix:** populate combobox from `/api/mode` `available_modes` field on first poll.

### 2.11 Mission Planner dual-link failover untested in flight
- [DualLinkSender.cs](mission_planner/src/DualLinkSender.cs) (1217 lines) — code complete. No record of two-link flight test.
- **Fix:** schedule a test cycle that disconnects 2.4 GHz mid-flight and verifies 900 MHz takes over.

---

## 3. Polish — nice to have

| # | File | Note |
|---|------|------|
| 3.1 | [api.py:2598](edge_core/api.py#L2598) `building_location` (also Blocker 1.1) | Once 1.1 is fixed, expose face geometry in `/api/task/1/target/capture` payload too |
| 3.2 | [SLAM3DView.cs](mission_planner/src/SLAM3DView.cs) P3-7 (todo.md) | Incremental geometry updates still deferred. Acceptable on Pi 5 if FPS > 15. Verify during indoor test. |
| 3.3 | [EnhancedHealthDashboard.cs](mission_planner/src/EnhancedHealthDashboard.cs) | 5 s polling — fine, but consider 10 s on Pi 5 since SLAM3D shares the render thread. |
| 3.4 | [Rviz2View.cs](mission_planner/src/Rviz2View.cs), [FoxglovePanel.cs](mission_planner/src/FoxglovePanel.cs), [ZedCalibrationView.cs](mission_planner/src/ZedCalibrationView.cs) | Mark as "experimental" or remove from main menu — they're rarely used in flight and add tab clutter on a small screen. |
| 3.5 | Hardcoded subprocess timeouts in [api.py](edge_core/api.py) | Accept `NOMAD_SUBPROCESS_TIMEOUT` env var. |
| 3.6 | Bridge endpoints use `172.17.0.1` | Works because Edge Core binds 0.0.0.0; revisit if Docker network mode changes. |

---

## 4. Dead code (safe to delete)

| # | File | Why dead | Verified? |
|---|------|----------|-----------|
| 4.1 | [edge_core/vio_pipeline.py](edge_core/vio_pipeline.py) | Header says DEPRECATED. Zero `import` references in the codebase. | yes (grep) |
| 4.2 | [edge_core/zed_camera.py](edge_core/zed_camera.py) | Only used by the deprecated `vio_pipeline.py`. ZED is now owned by the ROS2 wrapper inside the container. | yes (grep) |
| 4.3 | [edge_core/ros_mesh_bridge.py.archived](edge_core/ros_mesh_bridge.py.archived) | Already renamed to `.archived`. Safe to delete entirely. | yes |
| 4.4 | [edge_core/probe_mesh_msg.py](edge_core/probe_mesh_msg.py) | Probe script. Verify before deletion — could be operator diagnostic. | needs check |
| 4.5 | Stale doc references to `vio_pipeline.py`, `zed_camera.py`, `ros_mesh_bridge.py` in [docs/analysis/codebase_analysis.md:179-180](docs/analysis/codebase_analysis.md#L179-L180) and [docs/features/mission_planner_3d_slam.md:86,121,126,368](docs/features/mission_planner_3d_slam.md#L86) | | yes |

→ 4.1, 4.2, 4.3 deleted as part of this audit (see **Fixes applied**). 4.4 left for manual review. 4.5 docs flagged but not rewritten — they're historical design notes, low-priority.

---

## 5. Secrets & repo hygiene

All flagged files are properly `.gitignore`'d. Verified:

- `.env` (OpenRouter key) — ignored ✓
- `client_secret_*.json` (Google OAuth client) — ignored ✓
- `nomad-*.json` (likely a service account) — ignored ✓
- `*.tar.gz`, `*.zip`, `*.log` — ignored ✓

**No action needed**, but: rotate the OpenRouter key in `.env` if anyone outside the team has had access to a Jetson image.

---

## 6. Recommended fix order (1-month timeline)

**This week (≤1 day of work):**
- 1.6 hardcoded IPs in C# views
- 1.7 mavlink-router GCS templating
- 1.5 timer leak
- 0.x platform rename (already applied)
- 4.x dead code (already applied)

**Week 2 (≤3 days):**
- 1.1 building_location wiring
- 1.2 GDrive token initialization flow
- 1.3 servo mount offsets in config
- 2.1 drift stats display
- 2.2 obstacle distance display

**Week 3 — physical/integration:**
- 1.4 ballistic drop calibration (spray test)
- 1.3 measure real servo mount offsets
- 2.11 dual-link failover flight test
- 2.3 ArduPilot param load procedure
- Full dry-run on a fresh Jetson

**Week 4 — buffer + competition prep.**

---

## Fixes applied during this audit

- **Platform rename:** PRD, [config/nav2_drone.yaml](config/nav2_drone.yaml), [edge_core/api.py:3522](edge_core/api.py#L3522), [edge_core/ros/nav2_goal_bridge.py:15](edge_core/ros/nav2_goal_bridge.py#L15), [NOMADConfig.cs:372](mission_planner/src/NOMADConfig.cs#L372), [SLAM3DView.cs:336](mission_planner/src/SLAM3DView.cs#L336), [DroneRenderer.cs:34](mission_planner/src/SLAM3D/Rendering/DroneRenderer.cs#L34) → updated to ArduCopter quadcopter wording.
- **Dead code deleted:** `edge_core/vio_pipeline.py`, `edge_core/zed_camera.py`, `edge_core/ros_mesh_bridge.py.archived`.
- **Blocker 1.1 — Building face:** [edge_core/api.py](edge_core/api.py) now parses N/S/E/W from `target_localizer` description and populates `building_location` on Task 1 capture responses.
- **Blocker 1.2 — GDrive silent failure:** added `gdrive_ready()` probe in [gdrive_upload.py](edge_core/gdrive_upload.py), startup WARN in [main.py](edge_core/main.py), and `gdrive_ready` field on `/api/health`.
- **Important 2.4 / 2.5 — target_localizer visibility:** `_probe_isaac_runtime_state` now also probes target_localizer; `/api/health` exposes `{ enable_od, running }` under `target_localizer`.
- **Important 2.7 — Capture dir from env:** `NOMAD_TASK1_CAPTURE_DIR` env var is honored with fallback to existing paths.
- **Important 2.9 — Spray double-click interlock:** `_sprayInProgress` volatile flag + try/finally reset in [NOMADTask2View.cs](mission_planner/src/NOMADTask2View.cs).
- **Important 2.10 — Mode selector dynamic:** combobox now populates from `/api/mode` `available_modes`.
- **Important 2.1 — VIO stats display:** VIO card in [NOMADTask2View.cs](mission_planner/src/NOMADTask2View.cs) now shows health, tracking quality %, rate, source (polls `/api/vio/status`).
- **Important 2.2 — Obstacle distance display:** new GET `/api/obstacle_distance` exposes latest 72-sector snapshot + nearest-obstacle summary; Task 2 view renders `Nearest obstacle: X.XX m @ N°` with color-coded severity.
- **NOMADConfig base URL wiring:** [EmbeddedVideoPlayer.cs](mission_planner/src/EmbeddedVideoPlayer.cs), [Rviz2View.cs](mission_planner/src/Rviz2View.cs), [ZedCalibrationView.cs](mission_planner/src/ZedCalibrationView.cs), [FoxglovePanel.cs](mission_planner/src/FoxglovePanel.cs) — replaced hardcoded `100.85.121.98` IPs with `NOMADConfig.Load().EffectiveIP/EffectiveBaseUrl`.
- **Python SyntaxWarnings:** escape sequences in embedded bash strings in [api.py](edge_core/api.py) doubled (`\.` → `\\.`).
- **Visibility pause for Task 2 poll timer:** `_modePollTimer` now pauses when tab is hidden.
- **mavlink-router static config:** added comment clarifying that `start_nomad_full.sh` discovers GCS IP dynamically and bypasses `main.conf`.

Remaining Important items (user action required):
- **2.11** dual-link failover flight test — physical test.
- **Blocker 1.4** ballistic drop table — physical spray-test calibration (user handles).
- **Blocker 1.3** physical servo mount measurement — drop values into `/etc/nomad/servo_mount.yaml` (see [config/servo_mount.yaml.example](config/servo_mount.yaml.example)).

## Fixes applied (second pass — Jetson SSH + Cube available)

- **Blocker 1.3 — Servo mount offsets configurable:** [servo_tf_publisher.py](edge_core/ros/servo_tf_publisher.py) now loads offsets from env `SERVO_MOUNT_X/Y/Z` and `/etc/nomad/servo_mount.yaml` (YAML overrides defaults, env overrides both). Logs the effective source. Example config at [config/servo_mount.yaml.example](config/servo_mount.yaml.example). Tested on Jetson: defaults / env-only / yaml / yaml+env all resolve correctly.
- **Blocker 1.7 — `jetson.env` sourced at startup:** [start_nomad_full.sh](scripts/run/start_nomad_full.sh) now sources `${NOMAD_DIR}/config/env/jetson.env` if present. `GCS_IP` from env takes priority over Tailscale peer discovery, so competition-day overrides are a one-line change (no script edit).
- **Important 2.3 — pre-flight ArduPilot param loader:** new [scripts/run/load_params.py](scripts/run/load_params.py). Parses `task1_gps.param` / `task2_vio.param`, pushes via pymavlink, verifies each echo within tolerance with retries. MAVLink heartbeat from Cube Orange via `/dev/ttyACM0 @ 115200` confirmed on Jetson (ArduPilot sys=1, type=QUADROTOR). Dry-run verified for both param files.
- **Important 2.6 — `ros_http_bridge` supervision:** [docker-compose.yml](docker-compose.yml) healthcheck now asserts the bridge + drone_state_publisher PIDs (written to `/tmp/*.pid` by the entrypoint) are still alive, not just that Edge Core + ZED topics respond. A bridge crash now trips the container unhealthy flag within one interval instead of being invisible.
- **MAVLink link verified:** Cube Orange+ connected via USB, heartbeat confirmed at 115200 baud through `/dev/ttyACM0`. `start_nomad_full.sh` MAVLink endpoint is ready to fly.
