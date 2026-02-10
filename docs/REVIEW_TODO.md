# Codebase Review TODO

Generated: 2026-02-09
Status: In Progress

---

## CRITICAL

- [ ] **C1 - API Authentication**: Add API key auth to edge_core endpoints and tighten CORS to GCS origin/IP only
  - Files: `edge_core/api.py`
  - Details: Unauthenticated privileged endpoints with wide-open CORS allow remote command execution

- [ ] **C2 - Remove Hardcoded Sudo Password**: Remove password piping in setup_service.sh
  - Files: `scripts/setup_service.sh`
  - Details: Hardcoded sudo password piped via `sudo -S`

---

## IMPORTANT - Bugs

- [ ] **B1 - MAVLink GPS Population**: Populate `gps_lat/lon/alt` and attitude from `GLOBAL_POSITION_INT` and `ATTITUDE` messages
  - Files: `edge_core/mavlink_interface.py`
  - Details: Task 1 capture relies on GPS fields that are never populated

- [ ] **B2 - Task 1 Truthiness Checks**: Use `is not None` instead of truthiness for GPS 0.0 values
  - Files: `edge_core/api.py` (Task 1 capture section)
  - Details: Legitimate 0.0 values treated as missing

- [ ] **B3 - VIOPipeline setattr Bug**: Fix `setattr(self._status, key)` to include value argument
  - Files: `edge_core/vio_pipeline.py`
  - Details: Missing value in setattr call causes TypeError

- [ ] **B4 - VIOPipeline _command_descent**: Fix nonexistent `api_module._nav_controller` reference
  - Files: `edge_core/vio_pipeline.py`
  - Details: Descent failsafe path will crash

- [ ] **B5 - C# TelemetryInjector null**: Fix `new TelemetryInjector(null)` that always throws
  - Files: `mission_planner/src/NOMADControlPanel.cs`, `mission_planner/src/TelemetryInjector.cs`
  - Details: Telemetry never initializes

- [ ] **B6 - C# ServiceControlPanel Commands**: Fix raw shell commands where API expects whitelisted command names
  - Files: `mission_planner/src/ServiceControlPanel.cs`
  - Details: Terminal command execution likely fails on Jetson API

- [ ] **B7 - C# EffectiveIP Usage**: Use `EffectiveIP` instead of `JetsonIP` when Tailscale enabled
  - Files: `mission_planner/src/EnhancedWASDControl.cs`
  - Details: Water shooter and nozzle servo target wrong host with Tailscale

---

## IMPORTANT - Stale/Old Code

- [ ] **S1 - NavController Deprecation**: Either remove deprecated NavController or remove deprecation notice
  - Files: `edge_core/nav_controller.py`, `edge_core/main.py`
  - Details: Marked deprecated but still initialized and used at runtime

- [ ] **S2 - setup_isaac_ros_zed.sh Paths**: Update workspace path from `~/ros2/isaac_ros_ws` to `~/workspaces/isaac_ros-dev`
  - Files: `scripts/setup_isaac_ros_zed.sh`
  - Details: References old workspace path

- [ ] **S3 - Dockerfile Base Image**: Update from L4T r35 to L4T r36.x for JetPack 6.2
  - Files: `infra/Dockerfile`
  - Details: Base image mismatches current platform

---

## IMPORTANT - Inconsistencies

- [ ] **I1 - Isaac ROS Env Var Name**: Standardize `NOMAD_ENABLE_ISAAC_ROS` vs `ISAAC_ROS_ENABLED`
  - Files: `config/env/jetson.env`, `config/env/jetson.env.example`
  - Details: Different variable names for same setting

- [ ] **I2 - GCS Port Mismatch**: Align GCS_PORT between env and example (14550 vs 14552)
  - Files: `config/env/jetson.env`, `config/env/jetson.env.example`
  - Details: Default port differs

- [ ] **I3 - VIO Confidence Scale**: Normalize confidence scale (0-100 vs 0-1)
  - Files: `edge_core/vio_pipeline.py`, `edge_core/api.py`
  - Details: Mismatched scale at boundaries

- [ ] **I4 - C# Emojis Violation**: Remove emojis from MissionSetupPanel
  - Files: `mission_planner/src/MissionSetupPanel.cs`
  - Details: Violates no-emoji rule, won't render correctly in plugin

---

## IMPORTANT - Security

- [ ] **SEC1 - MediaMTX Auth**: Restrict MediaMTX to Tailscale subnet or add credentials
  - Files: `infra/mediamtx.yml`
  - Details: Unauthenticated publish/read from any IP

- [ ] **SEC2 - Servo PWM Binary Path**: Move from /tmp to locked directory
  - Files: `edge_core/servo_controller.py`
  - Details: Binary in world-writable /tmp vulnerable to replacement

- [ ] **SEC3 - C# Terminal Security**: Add auth or restrict terminal command execution
  - Files: `mission_planner/src/JetsonTerminalControl.cs`
  - Details: Arbitrary command execution without auth/TLS

---

## MINOR

- [ ] **M1 - Remove Duplicate Gemini Script**: Deprecate `process_task1_with_gemini.py`
  - Files: `scripts/process_task1_with_gemini.py`

- [ ] **M2 - Merge Power Mode Scripts**: Keep only JetPack 6.2 compatible version
  - Files: `scripts/fix_power_mode_25w.sh`, `scripts/fix_power_mode_25w_v2.sh`

- [ ] **M3 - Identical Task Branches**: Make task1/task2/all branches differ or merge
  - Files: `scripts/start_nomad_full.sh`

- [ ] **M4 - Legacy Task 1 Image Path**: Fix or remove legacy endpoint with wrong directory
  - Files: `edge_core/api.py`

- [ ] **M5 - Log Cleanup Path Mismatch**: Align cleanup path with actual log locations
  - Files: `edge_core/logging_service.py`

- [ ] **M6 - Duplicate nav_controller.stop()**: Remove duplicate stop call in shutdown
  - Files: `edge_core/main.py`

- [ ] **M7 - Remove Unused Models**: Remove `Task1CaptureRequest.lidar_distance_m` and `Task2ResetRequest`
  - Files: `edge_core/api.py`

- [ ] **M8 - C# Boundary Editor Duplication**: Consolidate BoundaryManager and NOMADViews boundary UI
  - Files: `mission_planner/src/BoundaryManager.cs`, `mission_planner/src/NOMADViews.cs`

- [ ] **M9 - C# Unused Panels**: Remove or integrate unused MissionSetupPanel and BoundaryConfigPanel
  - Files: `mission_planner/src/MissionSetupPanel.cs`, `mission_planner/src/BoundaryManager.cs`

- [ ] **M10 - C# Centralize HttpClient**: Create shared Jetson API service instead of per-control HttpClients
  - Files: Multiple C# files

- [ ] **M11 - C# Theme Duplication**: Use NOMADTheme consistently, remove local color constants
  - Files: `mission_planner/src/NOMADMainScreen.cs`

- [ ] **M12 - C# Hardcoded Polling**: Honor user config for polling intervals
  - Files: `mission_planner/src/JetsonConnectionManager.cs`

- [ ] **M13 - C# VIO Status Inference**: Use actual VIO endpoint instead of connectivity
  - Files: `mission_planner/src/NotificationService.cs`

- [ ] **M14 - setup_service.sh Duplicate Env**: Replace instead of append TAILSCALE_IP
  - Files: `scripts/setup_service.sh`

---

## COMPLETED

- [x] **Fixed Isaac ROS container name**: nomad_isaac_ros_32 -> nomad_isaac_ros (2026-02-09)
- [x] **Fixed Isaac ROS image name**: isaac_ros_dev-aarch64 -> dustynv/ros:humble-ros-base-l4t-r36.2.0 (2026-02-09)
- [x] **Fixed Isaac ROS workspace path**: ~/ros2/isaac_ros_32_ws -> ~/workspaces/isaac_ros-dev (2026-02-09)
