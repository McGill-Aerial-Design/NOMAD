# Codebase Review TODO

Generated: 2026-02-09
Status: Complete (committed 8d376f6, deployed to Jetson)

---

## CRITICAL

- [x] **C1 - API Authentication**: Added APIKeyMiddleware with NOMAD_API_KEY env var + tightened CORS (2026-02-09)
- [x] **C2 - Remove Hardcoded Sudo Password**: Removed password piping, added root check (2026-02-09)

---

## IMPORTANT - Bugs

- [x] **B1 - MAVLink GPS Population**: Added GLOBAL_POSITION_INT and ATTITUDE handlers (2026-02-09)
- [x] **B2 - Task 1 Truthiness Checks**: Changed to `is not None` checks, guarded formatting (2026-02-09)
- [x] **B3 - VIOPipeline setattr Bug**: Fixed `setattr(self._status, key, value)` (2026-02-09)
- [x] **B4 - VIOPipeline _command_descent**: Fixed to use `main_module.nav_controller`, removed bad kwarg (2026-02-09)
- [x] **B5 - C# TelemetryInjector null**: Made constructor accept null gracefully (2026-02-09)
- [x] **B6 - C# ServiceControlPanel Commands**: Replaced raw shell commands with proper API endpoints (2026-02-09)
- [x] **B7 - C# EffectiveIP Usage**: Fixed to use `EffectiveIP` consistently (2026-02-09)

---

## IMPORTANT - Stale/Old Code

- [x] **S1 - NavController Deprecation**: Removed stale deprecation notice (2026-02-09)
- [x] **S2 - setup_isaac_ros_zed.sh Paths**: Updated to `~/workspaces/isaac_ros-dev` (2026-02-09)
- [x] **S3 - Dockerfile Base Image**: Updated to `dustynv/l4t-pytorch:r36.2.0` (2026-02-09)

---

## IMPORTANT - Inconsistencies

- [x] **I1 - Isaac ROS Env Var Name**: Standardized on `NOMAD_ENABLE_ISAAC_ROS` (2026-02-09)
- [x] **I2 - GCS Port Mismatch**: Aligned to 14550 (2026-02-09)
- [x] **I3 - VIO Confidence Scale**: Normalized to 0-1 at API boundary (2026-02-09)
- [x] **I4 - C# Emojis Violation**: Replaced with ASCII text labels (2026-02-09)

---

## IMPORTANT - Security

- [x] **SEC1 - MediaMTX Auth**: Added security documentation for Tailscale ACLs + firewall (2026-02-09)
- [x] **SEC2 - Servo PWM Binary Path**: Moved to `~/.nomad/bin/` (2026-02-09)
- [x] **SEC3 - C# Terminal Security**: Added safe-commands list and confirmation dialog (2026-02-09)

---

## MINOR

- [x] **M1 - Remove Duplicate Gemini Script**: Added deprecation notice (2026-02-09)
- [x] **M2 - Merge Power Mode Scripts**: Added deprecation notice to v1 (2026-02-09)
- [x] **M3 - Identical Task Branches**: Added explanatory comment (2026-02-09)
- [x] **M4 - Legacy Task 1 Image Path**: Fixed to `./data/task1_captures` (2026-02-09)
- [x] **M5 - Log Cleanup Path Mismatch**: Aligned to `DEFAULT_LOG_DIR` and `*.json` (2026-02-09)
- [x] **M6 - Duplicate nav_controller.stop()**: Removed duplicate (2026-02-09)
- [x] **M7 - Remove Unused Models**: Removed `Task2ResetRequest` (kept `lidar_distance_m` -- used by C#) (2026-02-09)
- [x] **M11 - C# Theme Duplication**: Using NOMADTheme consistently now (2026-02-09)
- [x] **M13 - C# VIO Status Inference**: Queries actual `/api/vio/status` endpoint (2026-02-09)
- [x] **M14 - setup_service.sh Duplicate Env**: Uses sed upsert pattern now (2026-02-09)

### Remaining (deferred -- lower priority)

- [ ] **M8 - C# Boundary Editor Duplication**: Consolidate BoundaryManager and NOMADViews boundary UI
- [ ] **M9 - C# Unused Panels**: Remove or integrate unused MissionSetupPanel and BoundaryConfigPanel
- [ ] **M10 - C# Centralize HttpClient**: Create shared Jetson API service instead of per-control HttpClients
- [ ] **M12 - C# Hardcoded Polling**: Honor user config for polling intervals

---

## COMPLETED (Prior)

- [x] **Fixed Isaac ROS container name**: nomad_isaac_ros_32 -> nomad_isaac_ros (2026-02-09)
- [x] **Fixed Isaac ROS image name**: isaac_ros_dev-aarch64 -> dustynv/ros:humble-ros-base-l4t-r36.2.0 (2026-02-09)
- [x] **Fixed Isaac ROS workspace path**: ~/ros2/isaac_ros_32_ws -> ~/workspaces/isaac_ros-dev (2026-02-09)
- [x] **Fixed old Tailscale IP**: 100.75.218.89 -> 100.85.121.98 across all docs, scripts, C# defaults (2026-02-09)
