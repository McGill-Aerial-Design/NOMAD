# NOMAD 3D VSLAM + NVBlox Audit -- Fix TODO

## P0 -- Critical (Must Fix First)

### P0-1: Camera/Drone Frame Mismatch in SLAM3DView
- **File**: `mission_planner/src/SLAM3DView.cs`
- **Problem**: FPV camera, TPV camera, drone marker, and trajectory ribbon all assume Z-up/XY-forward, but actual SLAM data is X-right/Y-down/Z-forward (ROS optical frame).
- **Specifics**:
  - FPV: `UpdateFPVCamera` LookDirection and UpDirection are non-orthogonal for general pitch/roll
  - TPV: `UpdateTPVCamera` chase camera orbits in XY plane instead of the correct frame
  - Drone marker: Arrow built along +X but forward is +Z
  - Drone rotation: ZYX Euler applied in viewport basis without frame remapping
  - Trajectory ribbon: `dir x (0,0,1)` singularity on straight-ahead flight (forward IS +Z)
- **Fix**: Remap ROS frame (X-right, Y-down, Z-forward) to WPF frame (X-right, Y-up, Z-backward) once, then apply rotations. Use quaternion-based orientation.
- [x] Fix FPV camera (UpdateFPVCamera)
- [x] Fix TPV camera (UpdateTPVCamera)
- [x] Fix drone marker arrow orientation (CreateDroneMarker)
- [x] Fix drone rotation transforms (UpdateDroneVisual)
- [x] Fix trajectory ribbon cross product (UpdateTrajectoryVisual)
- [x] Fix orbit camera initialization (seed from current view, not stale state)

### P0-2: WebSocket Drops Mesh Frames When Pose Missing
- **File**: `edge_core/api.py` (around line 591)
- **Problem**: When a mesh frame arrives without `drone_position`, `has_mesh=true` but `has_pose=false`, so the frame is skipped entirely. Valid mesh data is lost.
- **Fix**: Send mesh data even without pose. Set pose to last known position, or send mesh-only frame.
- [x] Fix /ws/slam to not skip mesh when pose is missing

### P0-3: Docker Compose Bypasses Custom Launch Wrapper
- **File**: `docker-compose.yml` (line 102)
- **Problem**: Launches `zed_example.launch.py` directly instead of `nomad_zed_nvblox.launch.py`. Custom OD config (`custom_circle_detection.yaml`, `enable_od`) is dead code.
- **Fix**: Change compose command to use `nomad_zed_nvblox.launch.py` or pass the custom params directly.
- [x] Fix docker-compose.yml to use the custom launch wrapper

---

## P1 -- Important (Should Fix Soon)

### P1-1: Pose Frame Inconsistency Between Mesh-Bundled and Standalone
- **Files**: `edge_core/ros_http_bridge.py`, `edge_core/api.py`
- **Problem**: Mesh-bundled pose comes from TF (odom/map frame); standalone VIO comes from raw odometry. WebSocket mixes them without normalization, causing drone jumps.
- **Fix**: Ensure all pose sources posted to the SLAM WebSocket are in the same reference frame. Add frame_id metadata to poses.
- [ ] Normalize pose frame in ros_http_bridge VIO callback (deferred -- needs Jetson testing)
- [ ] Add frame_id to WebSocket pose messages (deferred -- needs Jetson testing)

### P1-2: Cross-Thread UI Access in SLAM3DView WebSocket Loop
- **File**: `mission_planner/src/SLAM3DView.cs` (line 661)
- **Problem**: WebSocket thread reads `_chkAutoUpdate.Checked` directly (InvalidOperationException risk). `_dronePosition` (struct) written from WS thread, read from UI thread -- non-atomic.
- **Fix**: Cache auto-update flag via Invoke. Use lock or volatile fields for pose data.
- [x] Fix _chkAutoUpdate cross-thread access
- [x] Add synchronization for _dronePosition/_droneYaw/_dronePitch/_droneRoll
- [x] Capture per-frame pose snapshot in BeginInvoke lambda

### P1-3: REST Mesh Fallback Uses Wrong Coordinate Frame
- **File**: `edge_core/api.py` (line 2557)
- **Problem**: `/api/task/2/slam/mesh` falls back to `external_vio_state` (NED coords), but mesh geometry is in ROS frame. Viewer shows drone in wrong position.
- **Fix**: Use `slam_vio_ros_frame` as fallback, matching WebSocket behavior.
- [x] Fix mesh GET endpoint to use slam_vio_ros_frame

### P1-4: String-Keyed Voxel Map (Performance Hotspot)
- **File**: `mission_planner/src/SLAM3DView.cs`
- **Problem**: `Dictionary<string, uint>` with "ix,iy,iz" keys. String.Split + int.Parse in hot path. Heavy GC pressure.
- **Fix**: Use packed `long` key: `((long)(ix+32768) << 32) | ((long)(iy+32768) << 16) | (iz+32768)`. Or use `(int,int,int)` tuple key.
- [x] Replace string keys with packed long keys in _persistedBlocks
- [x] Replace string keys in _occupiedPositions HashSet
- [x] Update all Split/Parse call sites in render loop

### P1-5: No Mesh Update Validation
- **File**: `edge_core/api.py` (line 2482)
- **Problem**: `/api/task/2/slam/mesh/update` stores arbitrary JSON in memory -- no schema/size validation. DoS risk.
- **Fix**: Validate required fields, types, and max payload size.
- [x] Add payload validation to mesh update endpoint

### P1-6: Unbounded Message Buffer in WebSocket Client
- **File**: `mission_planner/src/SLAM3DView.cs` (line 641)
- **Problem**: No max-frame guard on MemoryStream.Write. One oversized mesh frame grows memory permanently.
- **Fix**: Add 10MB max message size check before writing.
- [x] Add max message size guard in WebSocketStreamLoop

### P1-7: No Application-Level Receive Timeout
- **File**: `mission_planner/src/SLAM3DView.cs` (line 631)
- **Problem**: If server stalls without closing, ReceiveAsync hangs indefinitely.
- **Fix**: Use CancellationTokenSource with 30s timeout on ReceiveAsync.
- [x] Add receive timeout with CancellationTokenSource

---

## P2 -- Should Fix

### P2-1: Dead Code -- ros_mesh_bridge.py
- **File**: `edge_core/ros_mesh_bridge.py`
- **Problem**: Not auto-started. Wrong QoS, wrong schema, wrong lifecycle. Delta endpoint in api.py queries this dead bridge.
- **Fix**: Delete or archive. Fix delta endpoint to use app.state from ros_http_bridge.
- [x] Archive ros_mesh_bridge.py (renamed to .archived)
- [x] Fix /api/task/2/slam/mesh/delta to use app.state

### P2-2: Config Overlay Fragility
- **File**: `docker-compose.yml` (line 92-93)
- **Problem**: `cp` over installed `nvblox_base.yaml` instead of passing `nvblox_params_file`.
- **Fix**: Pass `nvblox_params_file:=/config/nvblox_performance.yaml` in launch args (already supported).
- [x] Replace cp overlay with launch parameter

### P2-3: Documentation Inconsistencies
- **Files**: `docs/ISAAC_ROS_NVBLOX_SETUP.md`, `docs/NAV2_INTEGRATION_PLAN.md`, `config/nvblox_performance.yaml`
- **Problem**: Docs cite 0.12m voxels / 15m radius / 10Hz mesh. Config uses 0.15m / 8m / 2Hz. Config header says 10cm but value is 15cm.
- [x] Update ISAAC_ROS_NVBLOX_SETUP.md to match actual config
- [x] Update NAV2_INTEGRATION_PLAN.md voxel size reference
- [x] Fix nvblox_performance.yaml header comment

### P2-4: ZED Readiness Gate is Weak
- **File**: `docker-compose.yml` (lines 107-116)
- **Problem**: Only checks for any image topic, not odometry. Proceeds after 30s even on failure.
- [x] Check for odometry topic in readiness gate
- [x] Fail startup if ZED not ready after timeout

### P2-5: No Failure Supervision for ros_http_bridge
- **File**: `docker-compose.yml` (line 120)
- **Problem**: `wait` only watches nvblox PID. Bridge crash is invisible.
- [x] Add process supervision (trap + monitor both PIDs)

### P2-6: WebSocket Unauthenticated
- **File**: `edge_core/api.py` (line 569)
- **Problem**: `/ws/slam` bypasses HTTP API-key middleware.
- [x] Add token-based auth to WebSocket endpoint (hmac.compare_digest, query param ?token=)

### P2-7: Mesh is Not True Delta Stream
- **Files**: `edge_core/ros_http_bridge.py`, `mission_planner/src/SLAM3DView.cs`
- **Problem**: Bridge sends capped snapshots; client merges additively. Removed geometry never leaves the viewer.
- [x] Add "removed" voxel list to mesh updates
- [x] Client-side: handle voxel removal in UpdateMeshVisual

---

## P3 -- Nice to Fix

### P3-1: FIFO Eviction Unreliable
- **File**: `mission_planner/src/SLAM3DView.cs` (lines 893-901)
- **Problem**: Dictionary.Keys.Take() not guaranteed insertion-ordered on .NET Framework 4.8.
- [x] Replace with LinkedList or Queue for eviction order

### P3-2: Material Cache Unbounded in Block Mode
- **File**: `mission_planner/src/SLAM3DView.cs` (line 1070)
- **Problem**: Block mode stores full 24-bit colors. Cache grows without limit.
- [x] Apply same 4-bit quantization to block colors (nibble * 17)

### P3-3: 4-Bit Color Quantization Darkens Colors
- **File**: `mission_planner/src/SLAM3DView.cs` (lines 876-878)
- **Problem**: `>> 4 << 4` maps 255 to 240 instead of proper nibble expansion (nibble * 17).
- [x] Use nibble * 17 for 4-bit-to-8-bit expansion

### P3-4: Black Voxels Cannot Render as Black
- **File**: `mission_planner/src/SLAM3DView.cs` (line 936)
- **Problem**: Color 0x000000 treated as "missing" and replaced with gray fallback.
- [x] Use uint.MaxValue sentinel instead of overloading 0x000000

### P3-5: BackMaterial Unnecessary on Voxel Cubes
- **File**: `mission_planner/src/SLAM3DView.cs`
- **Problem**: Double-sided rendering adds fill cost where winding is already correct.
- [x] Remove BackMaterial from voxel and block models

### P3-6: Detection Marker Confidence Unclamped
- **File**: `mission_planner/src/SLAM3DView.cs` (lines 558-561)
- [x] Clamp confidence to [0, 1] before computing sphere radius

### P3-7: Full Mesh Rebuild on Every 20-Voxel Batch
- **File**: `mission_planner/src/SLAM3DView.cs`
- [x] Add time-based debounce (250ms min interval, ~4 rebuilds/sec)
- [ ] Move toward incremental geometry updates (deferred -- requires architecture change)

### P3-8: Drone Transform Allocates New Objects Every Frame
- **File**: `mission_planner/src/SLAM3DView.cs` (line 1208)
- [x] Cache Transform3DGroup and child transforms, update values only

### P3-9: Double-Dispatch UI Marshaling
- **File**: `mission_planner/src/SLAM3DView.cs` (lines 755, 791)
- [ ] Marshal once to WPF dispatcher instead of BeginInvoke + ElementHost.Invoke (deferred -- risky refactor)

### P3-10: Isaac Launch Does Not Verify Success
- **File**: `edge_core/api.py` (line 1892)
- **Problem**: `docker exec -d` return code not checked.
- [x] Check return code and verify startup

### P3-11: Freeze Coverage Incomplete
- **File**: `mission_planner/src/SLAM3DView.cs` (lines 403-410, 483-489)
- [x] Freeze drone marker mesh, material, model, and lighting group

### P3-12: Isaac Subprocess Pipes Never Consumed
- **File**: `edge_core/api.py` (line 1778)
- **Problem**: stdout/stderr pipes can fill, blocking the child process.
- [x] Redirect to DEVNULL (fire-and-forget process)

---
---

# Requirements Audit (Sections 2-9)

Full audit of the requirements document against codebase. Audited 2026-03-19.

## Legend
- [x] Implemented and verified in code
- [ ] Missing — needs implementation
- [~] Partially implemented — needs finishing

---

## REQ-1: TF Tree and Servo Integration (TF-001 to TF-007) — ALL PASS

- [x] **TF-001**: servo_mount -> camera_link at >= 50 Hz — `edge_core/ros/servo_tf_publisher.py:91`
- [x] **TF-002**: Feedback angle when available — `servo_tf_publisher.py:177`
- [x] **TF-003**: Fallback to commanded angle with warning — `servo_tf_publisher.py:181-189`
- [x] **TF-004**: Static base_link -> servo_mount (10cm fwd, 5cm down) — `servo_tf_publisher.py:60-63`
  - **ACTION NEEDED**: Physically measure real mounting offsets and update constants
- [x] **TF-005**: ZED factory calibration (launch passes `camera: zed2`) — `nomad_zed_nvblox.launch.py:70`
- [x] **TF-006**: Pure Y-axis rotation, zero translation — `servo_tf_publisher.py:143-158`
- [x] **TF-007**: TF latency < 20ms (50 Hz interval)

---

## REQ-2: Nvblox 3D Mapping (NV-001 to NV-008) — 7/8 PASS

- [x] **NV-001**: Full TF chain (`use_tf_transforms: true`) — both configs
- [x] **NV-002**: Outdoor voxel_size = 0.10 — `config/nvblox_performance.yaml`
- [ ] **NV-003**: Indoor voxel_size = 0.03 — `config/nvblox_indoor.yaml` (NOT YET CREATED)
- [x] **NV-004**: Depth rate limited (10 Hz outdoor, 15 Hz indoor)
- [x] **NV-005**: 3D ESDF (`esdf_mode: "3d"`) — both configs
- [x] **NV-006**: Correct TF prevents phantom geometry (depends on TF-*)
- [x] **NV-007**: TSDF decay + block deallocation — both configs
- [x] **NV-008**: 2D ESDF slice -> OBSTACLE_DISTANCE MAVLink for ArduPilot
  - `edge_core/ros/obstacle_distance_bridge.py` — subscribes to occupancy grid,
    raycasts 72 sectors, sends to Edge Core API
  - `edge_core/mavlink_interface.py:send_obstacle_distance()` — forwards to ArduPilot
  - `edge_core/api.py:POST /api/obstacle_distance` — API endpoint
  - `config/launch/nomad_zed_nvblox.launch.py` — auto-launched with nvblox
  - Supports sector exclusion for spray approach (SP-005)

---

## REQ-3: Visual Odometry (VO-001 to VO-007) — 5/7 PASS

- [x] **VO-001**: Outdoor uses Cube GPS/IMU EKF (architecture-level)
- [x] **VO-002**: Indoor uses cuVSLAM/ZED VIO — `ros_http_bridge.py`
- [x] **VO-003**: Servo TF published for tilt compensation — `servo_tf_publisher.py`
- [x] **VO-004**: Scan-stop-scan protocol — `ros_http_bridge.py:354-357, 568-590`
- [x] **VO-005**: Tracking loss -> level servo after 3s — `ros_http_bridge.py:349-392`
- [x] **VO-006**: Drift < 5cm per tilt cycle — per-cycle measurement added
  - `edge_core/ros_http_bridge.py` — records position at tilt start/end,
    logs warning if drift > 5cm, exposes stats via `get_stats()["tilt_drift"]`
- [x] **VO-007**: ZED IMU independent of tilt (handled by ZED ROS wrapper)

---

## REQ-4: Target Detection (TD-001 to TD-007) — 5/7 PASS

- [x] **TD-001**: Full TF chain for 3D positioning (ZED SDK internal)
- [~] **TD-002**: Median depth in bbox (ZED SDK config-dependent — needs verification)
- [x] **TD-003**: Detection timestamp = image capture time, not inference publish time
  - `ros_http_bridge.py:800-810` uses `msg.header.stamp` (image acquisition time)
  - Falls back to `time.time()` if header stamp is empty
- [x] **TD-004**: 5 Hz normal / 3 Hz throttled — `ros_http_bridge.py:890`
- [x] **TD-005**: HSV + YOLO dual verification, `needs_review` flag — `ros_http_bridge.py:643-754`
- [x] **TD-006**: Dedup within 0.5m, keep higher confidence — `api.py:2146-2161`
- [ ] **TD-007**: Target position relative to building geometry (wall face, corner dist)
  - Low priority — Task 1 specific, requires building model loader

---

## REQ-5: Autonomous Spray (SP-001 to SP-008) — ALL IMPL (needs integration testing)

- [x] **SP-001**: Trigger from > 2m distance — `spray_controller.py:trigger()` validates distance
- [x] **SP-002**: Fully autonomous state machine — `spray_controller.py:_run_sequence()`
  - States: IDLE -> APPROACH -> AIM -> SPRAY -> VERIFY -> UPLOAD -> COMPLETE
- [x] **SP-003**: Visual servoing (drone lateral + servo pitch) — `spray_controller.py:_aim_at_target()`
- [x] **SP-004**: Ballistic drop compensation — `spray_controller.py:BALLISTIC_DROP_TABLE`
  - Interpolates drop angle based on engagement distance
- [x] **SP-005**: Sector exclusion — `spray_controller.py:_update_excluded_sector()`
  - Excludes 30-deg sector centered on target from OBSTACLE_DISTANCE
- [x] **SP-006**: Ground target descent — `spray_controller.py:_approach_target()`
  - Vertical descent, hover at 1.2m, min alt 0.8m enforced
- [x] **SP-007**: Photo + HSV verify + upload — `spray_controller.py:_capture_and_upload()`
  - Filename: `Task_2_MAD_target_<n>.jpg`
  - **ACTION NEEDED**: Wire up Google Drive upload callback
- [x] **SP-008**: Re-spray once on fail — `spray_controller.py:MAX_SPRAY_ATTEMPTS=2`
- API endpoints: `POST /api/spray/trigger`, `POST /api/spray/abort`, `GET /api/spray/status`

---

## REQ-6: Compute Resources (RM-001 to RM-005) — ALL PASS

- [x] **RM-001**: GPU memory monitoring — `health_monitor.py`
- [x] **RM-002**: INT8 TensorRT, 640x480 (external config)
- [x] **RM-003**: ZED depth max 15 Hz, 720p (launch params)
- [x] **RM-004**: CPU monitoring — `health_monitor.py`
- [x] **RM-005**: Thermal throttle at 85C -> 3Hz, resume at 75C — `ros_http_bridge.py:341-890`

---

## REQ-7: Operational Mode Manager (Section 9) — IMPLEMENTED

- [x] **Mode system**: `edge_core/operational_mode.py`
  - `OperationalModeManager` with `switch_mode()` method
  - Coordinates servo, VIO source, nvblox config, obstacle buffer
  - API: `GET /api/mode`, `POST /api/mode/set?mode=outdoor_transit`
  - Validates drone is hovering before nvblox config switches
  - Supports nvblox restart callback for config changes

| Mode | Servo | VIO Source | Nvblox Config | Obstacle Buffer |
|------|-------|-----------|---------------|-----------------|
| outdoor_transit | Fixed 90 deg | Cube GPS EKF | performance.yaml (10cm, 5m) | 2m horizontal |
| outdoor_survey | Sweep -45/+30 | Cube GPS EKF | performance.yaml | 2m horizontal |
| indoor_nav | Fixed 90 deg (scan-stop) | cuVSLAM/ZED VIO | indoor.yaml (3cm, 5m) (NOT YET CREATED) | 0.7m h+v |
| spray_approach | Visual servo | Mode-dependent | keep current | Sector exclusion |
| emergency | N/A (RC kill) | N/A | keep current | Bypassed |

- **nvblox config switching requires node restart** (~2-3s blind window)
  - `switch_mode()` checks groundspeed < 0.2 m/s before allowing
  - **ACTION NEEDED**: Wire up `set_nvblox_restart_fn()` in `main.py`
  - **ACTION NEEDED**: Mission Planner mode selector UI

---

## Remaining Integration Work

1. **Wire up `main.py`** — Initialize `OperationalModeManager`, `SprayController`,
   set callbacks (nvblox restart, photo capture, HSV verify, Google Drive upload)
2. **Google Drive upload** — Implement upload callback for SP-007
   (service account or OAuth, upload to shared drive)
3. **Mission Planner UI** — Mode selector dropdown, spray trigger button,
   drift stats display, obstacle distance visualization
4. **TD-007** — Building geometry output (low priority, Task 1 specific)
5. **Ballistic drop table** — Calibrate with real nozzle measurements
6. **Field testing** — All new features need Jetson deployment and flight testing

---

## Task 2 Page Revamp -- Follow-ups

Items left over after the Task 2 view refactor (Video|Tabs layout, shape
overlay, manual+auto Submit flow, spray-state-driven tilt lock).

### T2-1: Detection list ↔ overlay mismatch
- **Files**: `mission_planner/src/NOMADTask2View.cs`, `edge_core/api.py`,
  `edge_core/ros/simple_video_bridge.py`
- **Problem**: The video overlay now draws shape-based circles (Task 2
  detector) but the spray-target list still pulls from `/api/detections`,
  which is the ZED YOLO/HSV pipeline. Operators select targets that may
  not match what they see drawn on the video.
- **Fix**: Either expose the bridge's shape-detector results through a new
  `/api/task/2/detections` endpoint and have the Detect & Spray tab read
  from there, or feed the shape detector into the existing detection
  pipeline so both UI elements stay in sync.
- [x] Decide source of truth (bridge — shape detector results are authoritative for Task 2)
- [x] Wire detection list to that source (new `/api/task/2/detections` endpoint)
- [x] Verify selection coordinates match overlay-drawn boxes (UI uses same bbox)
- [ ] Follow-up: thread depth lookup so spray controller can run real approach
      instead of `image_only=True` (currently every shape detection is treated
      as already-in-range)

### T2-2: Color-agnostic verification not actually wired
- **File**: `edge_core/task2_circle_verify.py`,
  `edge_core/spray_controller.py` (`_verify_spray`)
- **Problem**: The agreed verification semantics are "any circle shape,
  then check ≥20% average colour change inside it." The new overlay
  matches that, but `_verify_spray` still calls `task2_circle_verify`,
  which has purple→blue HSV gates baked in. Autonomous flow can fail
  verify on a perfectly good spray when the target isn't the expected hue.
- **Fix**: Replace the colour-gated verifier with a colour-agnostic
  before/after delta on the matched circle ROI (mean LAB ΔE or
  ‖ΔBGR‖ over a configurable threshold, default 20%).
- [ ] Add colour-agnostic verifier
- [ ] Switch `_verify_circle_change_fn` over
- [ ] Keep HSV verify as opt-in fallback for sanity

### T2-3: Manual flow has no spray-controller interlock
- **Files**: `mission_planner/src/Task2UploadPanel.cs`,
  `edge_core/task2_spray_artifacts.py`, `edge_core/spray_controller.py`
- **Problem**: Clicking *Manual Start* mid-autonomous-spray silently
  finalises the autonomous artifact session and starts a new one. The
  Abort button only stops the autonomous controller, not the manual
  recording. Operators can foot-gun this trivially.
- **Fix**: Disable manual Start/Stop while spray state ∈
  {approach, aim, spray, verify, upload}; make manual sessions abortable
  from the same Abort button; or merge both flows behind a single
  session state machine.
- [ ] Gate manual buttons on spray-active state
- [ ] Route Abort to whichever session is live
- [ ] Surface the active session source in the Submit panel header

### T2-4: No artifact retention policy
- **File**: `edge_core/task2_spray_artifacts.py`
- **Problem**: `~/.nomad/spray_sessions/` accumulates indefinitely — every
  session keeps two JPEGs and a multi-MB MP4. After a few flight days
  the SD card will fill.
- **Fix**: On session-end (and on startup), keep the last N sessions
  (default 20) and delete the rest. Optionally also enforce a total-size
  cap.
- [ ] Implement retention sweep
- [ ] Add `NOMAD_SPRAY_KEEP_LAST` env override
- [ ] Log how many sessions/MB were freed

### T2-5: Drive upload roundtrip is wasteful
- **Files**: `mission_planner/src/Task2UploadPanel.cs`,
  `edge_core/spray_controller.py` (`_upload_fn`),
  `edge_core/api.py`
- **Problem**: The Submit panel pulls every artifact down from the Jetson
  over the link, then pushes the same bytes back up to Google Drive.
  Wastes bandwidth and is fragile on flaky cell links.
- **Fix**: Add a server-side upload endpoint
  (`POST /api/task/2/spray/upload`) that uploads the requested artifacts
  directly from the Jetson and returns Drive URLs. Mission Planner only
  has to trigger and display results.
- [ ] Implement Jetson-side Drive upload using existing `_upload_fn`
- [ ] Replace `Task2UploadPanel.UploadArtifactsFromJson` round-trip
- [ ] Keep client-side upload as fallback when Jetson lacks token

### T2-6: Shape detector cosmetic rough edges
- **File**: `edge_core/ros/simple_video_bridge.py`
- **Problems**:
  - No spatial dedupe between Hough and contour passes — the same circle
    sometimes gets two boxes.
  - `set_overlay_detectors` clears `_detections` on every call; harmless
    repeated state pushes from the status poll cause a brief flicker.
  - `Task2PayloadPanel` still subscribes to
    `PayloadControlPanel.AutonomousModeChanged` even though nothing
    raises it anymore — works fine via the direct `SetTiltLocked` call,
    but the dead subscription is misleading.
- [ ] Add minDist-style dedupe across detector passes
- [ ] Skip the `_detections` clear when the new state matches the old
- [ ] Drop the dead event subscription in Task2PayloadPanel
