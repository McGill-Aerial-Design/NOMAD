# Mesh Transmission Diagnostic Guide

## Problem
SLAM3DView showing "Mesh: 0 voxels" or stuck at "waiting for mesh".

## Data Flow
```
nvblox_node                 ros_http_bridge         Edge Core API           Mission Planner
    |                              |                      |                        |
    | /nvblox_node/               |                      |                        |
    | color_layer_marker          |                      |                        |
    | (Marker msg)                |                      |                        |
    |----------------------------->|                      |                        |
    |                              | POST /api/task/2/   |                        |
    |                              | slam/mesh/update    |                        |
    |                              |-------------------->|                        |
    |                              |                      | Store in               |
    |                              |                      | app.state.slam_mesh    |
    |                              |                      |                        |
    |                              |                      | /ws/slam (WebSocket)   |
    |                              |                      |----------------------->|
    |                              |                      |  type="mesh"           |
    |                              |                      |  voxels: [...]         |
```

## Common Causes

### 1. nvblox not publishing
**Symptoms**: No voxels ever appear, "Mesh: 0 voxels" stays forever

**Check**:
```bash
# Inside Isaac ROS container
ros2 topic list | grep nvblox
ros2 topic echo /nvblox_node/color_layer_marker --once
```

**Fix**: Start nvblox or move camera to generate depth data

### 2. ros_http_bridge not running or not subscribed
**Symptoms**: nvblox publishes but Edge Core never receives

**Check**:
```bash
# Check bridge status
curl http://100.85.121.98:8000/api/vio/status | jq '.bridge_stats'
```

**Look for**:
- `mesh_received: 0` → Bridge not subscribing
- `mesh_sent: 0` → Bridge not forwarding

**Fix**: 
```bash
# Start with --enable-mesh
python3 edge_core/ros_http_bridge.py --host 172.17.0.1 --enable-mesh
```

### 3. visualization_msgs not available
**Symptoms**: Bridge logs "visualization_msgs not available - per-voxel mesh disabled"

**Check**:
```bash
# Inside Isaac ROS container
python3 -c "from visualization_msgs.msg import Marker; print('OK')"
```

**Fix**: Install visualization_msgs in Isaac ROS container

### 4. Edge Core not storing mesh
**Symptoms**: Bridge sends but Mission Planner receives nothing

**Check**:
```bash
curl http://100.85.121.98:8000/api/task/2/slam/mesh?format=summary
```

**Expected**:
```json
{
  "available": true,
  "mode": "voxel",
  "block_count": 1234,
  "timestamp": "2026-04-05T..."
}
```

**Fix**: Check Edge Core logs for errors

### 5. WebSocket not sending mesh frames
**Symptoms**: Can see pose updating but no voxels

**Check**: Run diagnose_mesh.py test #4 (WebSocket)

**Fix**: Check `/ws/slam` endpoint is serving mesh frames

## Quick Diagnostic

```bash
# ON JETSON - Run comprehensive diagnostic
python3 scripts/dev/diagnose_mesh.py

# Manual checks:
# 1. ROS topic
ros2 topic hz /nvblox_node/color_layer_marker

# 2. Bridge stats
curl http://100.85.121.98:8000/api/vio/status | jq '.bridge_stats | {mesh_received, mesh_sent}'

# 3. Edge Core storage
curl http://100.85.121.98:8000/api/task/2/slam/mesh?format=summary | jq '{available, mode, block_count}'

# 4. WebSocket (requires websocket-client)
python3 -c "
import websocket
import json
ws = websocket.create_connection('ws://100.85.121.98:8000/ws/slam')
for _ in range(10):
    msg = json.loads(ws.recv())
    print(f'{msg.get(\"type\")}: voxels={len(msg.get(\"mesh\", {}).get(\"voxels\", []))}')
ws.close()
"
```

## Expected Behavior

When working correctly:
1. nvblox publishes CUBE_LIST markers at ~10-30 Hz (when camera moving)
2. ros_http_bridge receives, rate-limits to 30Hz, forwards to Edge Core
3. Edge Core stores latest mesh in `app.state.slam_mesh_data`
4. WebSocket `/ws/slam` sends:
   - `type="pose"` at 30Hz (always)
   - `type="mesh"` when mesh changes (sporadic)
5. SLAM3DView updates stats: "Mesh: 1234 voxels (567 cached)"

## Known Issues

- **Empty voxels after 20 messages**: Normal if camera isn't moving or depth invalid
- **Voxels cap at 8000**: By design, larger scenes are subsampled for performance
- **Mesh not updating**: May need to move camera to generate new depth data
- **frame_id mismatch**: Should be "odom" (ROS) or "ros_optical" (legacy)

## Next Steps

1. Run `python3 scripts/dev/diagnose_mesh.py` on Jetson
2. Check which test fails
3. Follow fix for that specific failure
4. Re-run diagnostic until all pass
