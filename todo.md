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
