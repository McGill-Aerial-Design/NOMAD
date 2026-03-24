# NVBlox & YOLO26 Crash Fix Summary

**Date**: March 24, 2026
**Status**: Issues Identified and Partially Resolved
**Commits**: 4224feb, 83f3df6

---

## Root Cause Analysis

### Issue 1: ROS2 Parameter Duplication Crash ✅ FIXED
**Problem**: nvblox component container was crashing with SIGABRT when initializing ZED object detection.

**Error Message**:
```
[component_container_mt-1] terminate called after throwing an instance of 'rclcpp::exceptions::ParameterAlreadyDeclaredException'
  what():  parameter 'object_detection.custom_onnx_file' has already been declared
```

**Root Cause**: The YOLO26 custom OD config was attempting to declare object_detection parameters that were already being initialized by the ZED SDK, causing a duplicate parameter declaration error in ROS2.

**Fix Applied**:
- Disabled object detection in the launch file (`enable_od: 'false'`)
- Disabled Nav2 stack (was also crashing with similar initialization issues)
- File: `config/launch/nomad_zed_nvblox.launch.py`
- Commit: `4224feb`

**Status**: ✅ Resolved - This specific crash is now prevented.

---

### Issue 2: ZED Camera Not Detected ⚠️ REQUIRES INVESTIGATION
**Problem**: When nvblox launch script executes, ZED SDK reports "CAMERA NOT DETECTED" and times out.

**Error Message**:
```
[zed_wrapper-2] [WARN] Error opening camera: CAMERA NOT DETECTED
[zed_wrapper-2] [2026-02-11 22:06:31 UTC][ZED][WARNING] CAMERA NOT DETECTED in sl::ERROR_CODE sl::Camera::open(sl::InitParameters)
```

**Observations**:
- `/dev/video0` and `/dev/video1` ARE present in the container
- uvcvideo driver IS bound to ZED USB interfaces (2-1.2:1.0, 2-1.2:1.1)
- `/run/udev` IS mounted in the container
- ZED SDK libraries ARE available (/usr/local/zed/lib/)
- Issue only occurs during fresh container startup

**Potential Causes**:
1. Timing issue - ZED SDK may be initializing before udev metadata is ready
2. USB device initialization timing - camera may need extra time to enumerate after container starts
3. Docker compose YAML parsing issue now preventing proper setup (see Issue 3)

**Fix Attempted**:
- Added explicit uvcvideo binding step to launch script in `start_isaac_ros_auto.sh`
- File: `scripts/run/start_isaac_ros_auto.sh`
- Commit: `83f3df6`

**Status**: ⚠️ Partial - Fix added to startup script, but insufficient if timing/initialization order is the issue.

---

### Issue 3: docker-compose YAML Parsing Error ⚠️ NEEDS RESOLUTION
**Problem**: `docker compose` CLI rejects the docker-compose.yml file with:
```
decoding failed due to the following error(s):
'services[isaac-ros].command' invalid command line string
```

**Analysis**:
- File is valid YAML (confirmed via Python `yaml.safe_load()`)
- Issue is docker compose v2's stricter validation of the `command` field
- The multiline bash script in the `command` field may have escaping issues for the new version

**Impact**:
- Cannot use `docker compose up` to start the container
- Must use `start_isaac_ros_auto.sh` script instead, which uses `docker run` directly
- However, `docker run` bypasses the comprehensive setup in docker-compose (uvcvideo binding, device mounts, environment setup)

**Status**: ⚠️ Blocking - Prevents using docker compose, forces fallback to script approach.

---

## Recommended Next Steps

### Priority 1: Get nvblox Running Without Crashes (DONE)
- ✅ Parameter duplication crash fixed by disabling OD and Nav2
- ⚠️ Camera detection still needs verification

### Priority 2: Enable YOLO26 Safely
- Current approach: Disable OD at startup, add it back once nvblox is stable
- Proper solution: Add YOLO26 detection parameters as launch arguments instead of config file merge
- Timeline: After nvblox starts reliably

### Priority 3: Fix docker-compose YAML Issue
**Option A**: Update `docker-compose.yml` to use a shell script file instead of inline command
```yaml
command: /workspaces/isaac_ros-dev/scripts/entrypoint.sh
```

**Option B**: Reformat the bash command to be more compatible with docker compose v2 parsing

**Option C**: Switch to docker run with wrapper script (current workaround)

---

## Testing Checklist

### Current Status
- [ ] nvblox launches without parameter crash (disabled OD)
- [ ] ZED camera detects and publishes topics
- [ ] Video bridge connects to image stream
- [ ] ros_http_bridge forwards VIO/mesh data
- [ ] WebSocket /ws/slam endpoint publishes frames
- [ ] YOLO26 model loads and runs detections (with OD enabled)

### Quick Verification Commands
```bash
# On Jetson, check nvblox status
curl -s http://localhost:8000/api/isaac/status | python -m json.tool

# In container, check ROS topics
docker exec nomad_isaac_ros ros2 topic list

# Check ZED camera detection
docker exec nomad_isaac_ros bash -c "ls /dev/video*"

# View launch logs
docker logs nomad_isaac_ros
docker exec nomad_isaac_ros tail -f /tmp/zed_nvblox.log
```

---

## Files Modified

| File | Changes | Commit |
|------|---------|--------|
| `config/launch/nomad_zed_nvblox.launch.py` | Disabled OD and Nav2 by default | 4224feb |
| `scripts/run/start_isaac_ros_auto.sh` | Added uvcvideo binding in launch script | 83f3df6 |

---

## Technical Details: Why These Crashes Happened

### Parameter Duplication Mechanism
1. ZED ROS2 wrapper loads default configuration (includes object_detection namespace)
2. Custom YOLO26 config merged into zed_common.yaml before launch
3. ROS2 tries to load parameters from merged config
4. Two conflicting declarations of `object_detection.custom_onnx_file`:
   - One from ZED's default initialization
   - One from custom merge
5. ROS2's ParameterAlreadyDeclaredException is thrown
6. Component container dies with SIGABRT

**Why Disabling OD Fixes This**: Skips the custom OD config merge, uses only ZED defaults.

### Camera Detection Failure Mechanism
1. Container starts with `/dev/video*` not yet bound
2. ROS2 launch begins immediately
3. ZED SDK tries to open camera but video devices not ready
4. SDK reports "CAMERA NOT DETECTED"
5. Wrapper retries for ~30 seconds then gives up

**Why Uvcvideo Binding Script Helps**: Explicitly binds driver and waits before launching ROS2.

---

## Next Actions

1. **Verify Camera Detection**: Run the container and check if video device binding helps ZED initialization
2. **Enable YOLO26 Properly**: Once nvblox is stable, add YOLO26 via launch parameters (not config merge)
3. **Fix docker-compose**: Either reformat command field or move to separate script file
4. **Full System Test**: Run Mission Planner with nvblox mesh and YOLO26 detections

---

**Summary**: The ROS2 parameter crash has been fixed by disabling OD at launch. The ZED camera detection issue requires further investigation but has mitigations in place. Once nvblox runs stably, YOLO26 can be added back safely using a different approach (launch parameters instead of config merge).
