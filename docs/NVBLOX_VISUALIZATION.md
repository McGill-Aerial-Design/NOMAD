# NOMAD nvblox Visualization Quick Reference

## Overview
This document describes how to run nvblox 3D mesh visualization with the ZED camera on the Jetson.

There are two startup paths in this repository. **Only one is parity-valid for
comparing Mission Planner against RViz:**

| Path | Launch file | Use it for | Parity with Mission Planner? |
|------|-------------|-----------|------------------------------|
| **Canonical** | `nomad_zed_nvblox.launch.py` (via `start_isaac_ros_auto.sh`) | Production flights, RViz vs Mission Planner parity validation | **Yes — required** |
| Debug-only | `zed_example.launch.py` (upstream nvblox example) | Isolated RViz sanity checks, upstream debugging | **No — do not use for parity** |

The two stacks differ in topic remaps, TF setup, and bridge configuration. Any
comparison between RViz and Mission Planner must start from the canonical path
so both tools see the same ROS graph, frames, and rates.

## Canonical (parity) Start

### From Windows PowerShell
```powershell
ssh mad@100.85.121.98 "bash /home/mad/NOMAD/scripts/run/start_isaac_ros_auto.sh start"
```

This script:
1. Restarts the `nomad_isaac_ros` container with the correct device bindings.
2. Launches `nomad_zed_nvblox.launch.py` inside the container.
3. Starts the `ros_http_bridge.py` with `--mesh-topic /nvblox_node/color_layer_marker`, `--vio-topic /zed/zed_node/odom`, and bridge rate 30 Hz.
4. Leaves the bridge as the single authoritative publisher to Edge Core; do **not** run a second bridge instance from the `zed_example.launch.py` path in parallel.

### Parity Mode for Mission Planner (RViz comparison runs)
For RViz-vs-Mission Planner comparison runs, set the parity flag on the client
so client-side smoothing, jump rejection, and voxel retention caps are bypassed:

```powershell
$env:NOMAD_SLAM3D_PARITY = "1"
# then launch Mission Planner
```

Or at runtime: set `SLAM3DView.ParityMode = true`.

Parity mode:
- Disables `PoseState` smoothing and jump rejection.
- Raises `VoxelMeshBuilder` retention cap to unlimited and disables age-based expiry.
- Uses voxel render scale 1.0 (no quantization padding).
- Drops the mesh rebuild debounce from 250 ms to ~16 ms.

Do not leave parity mode on in normal operation — it raises memory use and
removes filtering that protects against VIO glitches.

## Key Topics (canonical stack)

| Topic | Rate | Description |
|-------|------|-------------|
| `/nvblox_node/color_layer_marker` | bridge-capped at 30 Hz (effective rate varies with map density) | Voxel markers consumed by the ROS HTTP bridge |
| `/nvblox_node/static_map_slice` | ~1 Hz | 2D occupancy slice |
| `/zed/zed_node/odom` | 30 Hz | Visual odometry (source for VIO and drone pose) |
| `/zed/zed_node/imu/data` | ~200 Hz | Calibrated IMU for attitude |
| `/zed/zed_node/imu/mag` | ~50 Hz | Magnetometer heading |

`frame_id` for SLAM visualization is **`odom`** end-to-end: bridge mesh payloads,
`ws_slam` frames, mesh endpoint, and Mission Planner client all agree on this
exact string (see "Frame contract" below).

## Frame contract

All SLAM visualization payloads use `frame_id: "odom"` (REP-103 convention).
- `ros_http_bridge.py` publishes mesh with `"frame_id": "odom"`.
- `edge_core/api.py` `/api/task/2/slam/mesh/update` normalizes to `"odom"` and logs a mismatch if the bridge ever sends something else.
- `edge_core/api.py` `/ws/slam` emits `frame_id: "odom"` on every frame.
- `mission_planner/src/SLAM3DView.cs` expects `"odom"` and logs a warning on anything else.

If you see a mismatch warning, you are running two publishers at once — stop
the debug stack.

## Launch RViz (canonical stack)

```bash
ssh mad@100.85.121.98 "docker exec -e DISPLAY=:1 -e XAUTHORITY=/run/user/1000/gdm/Xauthority nomad_isaac_ros /bin/bash -c 'source /workspaces/isaac_ros-dev/install/setup.bash && rviz2 -d /workspaces/isaac_ros-dev/config/visualization/nomad_zed_nvblox.rviz'"
```

If the NOMAD RViz config does not exist yet, the upstream config at
`/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz`
can be used as a read-only visualization — but keep the canonical stack running;
do not launch the example stack.

## Debug-only path (NOT for parity)

If you need to reproduce an upstream bug against the stock nvblox example, you
can launch `zed_example.launch.py` directly:

```bash
# DEBUG ONLY. Do NOT use while the canonical stack is running.
ssh mad@100.85.121.98 "docker exec -d nomad_isaac_ros /bin/bash -c 'source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 > /tmp/zed_nvblox_full.log 2>&1'"
```

This path does not wire up the NOMAD bridge, so Mission Planner will show
nothing and any parity measurements taken against it are invalid.

## Troubleshooting

### ZED Camera Not Detected
- Check USB connection: `lsusb | grep -i stereo`
- Restart the Docker container: `docker restart nomad_isaac_ros`
- The ZED SDK sometimes needs a fresh USB bind after container restart

### nvblox Mesh Not Showing
- Check mesh topic is publishing: `ros2 topic hz /nvblox_node/color_layer_marker`
- In RViz, set Fixed Frame to `odom`
- Ensure marker display is enabled
- Wait 10-20 seconds for mesh to build up

### Mission Planner shows mesh but RViz does not (or vice versa)
- Confirm only one stack is running: `pgrep -af ros2.launch`
- Check bridge coalescing counter via `GET /api/task/2/slam/mesh?format=summary` — field `mesh_coalesced` on the bridge stats should be near-constant, not rising rapidly.
- Check drop counter in the same summary: `drops`, `last_drop_bytes`, `last_drop_reason`.
- Check WebSocketClient `OversizeDrops` in Mission Planner diagnostics.

### Frame ID warnings in edge_core logs
```
ws_slam pose frame_id mismatch: got 'X', expected 'odom'
mesh/update frame_id mismatch: got 'X', expected 'odom'
```
This means something is publishing to Edge Core with the wrong frame string.
The canonical stack always emits `"odom"` — the most common cause is leaving the
debug stack running simultaneously.

### TF Frame Errors
- The ZED driver publishes frames: `zed_camera_link`, `zed_left_camera_frame`, etc.
- nvblox expects proper TF chain from camera to `base_link`
- `start_isaac_ros_auto.sh` publishes a static transform aliasing `camera_link` → `zed_camera_link` if the URDF chain is delayed.

## Configuration

### Docker Container
- Name: `nomad_isaac_ros`
- Image: `nomad-isaac-ros:latest` (built on top of `isaac_ros_dev-aarch64`)
- Workspace: `/workspaces/isaac_ros-dev/`

### X11 Display
- DISPLAY: `:1`
- XAUTHORITY: `/run/user/1000/gdm/Xauthority`
- Requires `xhost +local:docker` for Docker GUI access

### ZED Camera
- Model: ZED 2i
- Launch parameter: `camera:=zed2`
- Resolution download may take ~7 minutes on first run

## Logs

| Log File | Contents |
|----------|----------|
| `/tmp/zed_nvblox.pid` | PID of the running canonical launch |
| `/tmp/ros_bridge.pid` | PID of the running ros_http_bridge |
| `/tmp/zed_nvblox_full.log` | Debug stack launch output (only when the debug path was used) |
| `/tmp/rviz.log` | RViz startup log |

## Stop Commands

```bash
# Stop all ROS nodes
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros pkill -f 'ros2|rviz'"

# Stop just RViz
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros pkill rviz2"
```
