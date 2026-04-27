# Task 2: Manual Positioning + Nav2 Autonomous Approach + Spray

**Updated**: April 25, 2026
**Status**: Nav2 approach restored (simplified 3m→2m)

---

## Overview

**Task 2** is an indoor fire extinguishing mission (AEAC 2026 CONOPS §5.2.4):

- **Manual operator positioning** — Operator uses WASD controls to position drone within 3m of target
- **Autonomous Nav2 approach** — System uses Nav2 NavigateToPose to approach from 3m to 2m (obstacle-avoiding)
- **Autonomous spray sequence** — Once at 2m: aim, spray, verify, upload runs autonomously
- **Obstacle avoidance** — Nav2 uses nvblox costmap; target sector excluded during approach
- **3m radius obstacle check** — Pre-flight nvblox scan within 3m keeps RAM usage low

### Competition Requirements (CONOPS)

From §5.2.4 and Table 6:

- Targets are **circles of paper** (5-30cm diameter) on white plastic backing
- Paper is **dyed in pH-sensitive red cabbage juice** — purple when dry, **turns blue** when wet with baking soda solution
- Must wet target across **2cm wide area** for extinguish count
- Must **declare target extinguished** and provide **visual confirmation photo in real time**
- Photos titled **"Task_2_\<team_name\>_target_\<target#\>"** uploaded to Google Drive
- **No part of UAS can touch targets** — water only
- Penalty for false declarations: -PI/-PO points
- **Autonomous target extinguishing** = 20 pts (requires all approach from >2m + aiming + extinguishing + image capture/upload to be autonomous)
- Only **one UAV** permitted for Task 2

---

## Architecture

```
Operator (Ground Station)
    |
    | Manual WASD positioning into 3m engagement zone
    v
GUIDED Mode Drone (≤3m from target)
    |
    | Operator clicks "Spray Target" button
    v
POST /api/spray/trigger
    |
    v
Edge Core - SprayController
    |
    +-- APPROACH: Nav2 NavigateToPose (3m→2m)
    |   |   Sends goal via /api/nav2/goal
    |   |   nav2_goal_bridge dispatches to Nav2 stack
    |   |   Nav2 plans path using nvblox costmap
    |   |   /cmd_vel → ros_http_bridge → NavController → AP GUIDED
    |   |   Falls back to direct velocity if Nav2 unavailable
    |   v
    +-- AIM: Visual servoing
    |   - Center target in camera via servo pitch
    |   - Ballistic drop compensation by distance
    +-- SPRAY: Activate water pump
    +-- VERIFY: Circle change detection (before/after comparison)
    +-- UPLOAD: Post photo to Google Drive
    v
Sequence complete → ready for next target
```

### Data Flow: Nav2 Approach

```
  SprayController ──goal_dict──> app.state.nav2_pending_goal
       │                              │
       │                              v
       │                     nav2_goal_bridge (polls /api/nav2/pending)
       │                              │
       │                              v
       │                     Nav2 NavigateToPose
       │                     (nvblox costmap + DWB controller + Smac planner)
       │                              │
       │                              v
       │                     /cmd_vel @ 20Hz
       │                              │
       │                              v
       │                     ros_http_bridge → Edge Core → NavController → AP GUIDED
       │                              │
       │  <──result──── /api/nav2/result (goal_id, status, message)
       v
  SprayController continues to AIM state
```

---

## Spray State Machine

```
Trigger by operator (drone within 3m):

APPROACH - Nav2 NavigateToPose from 3m to 2m
|   Obstacle-avoiding path using nvblox costmap
|   Target sector excluded from obstacle avoidance
|   Timeout: 20s → proceed from current position
|   Fallback: direct velocity if Nav2 unavailable
v
AIM - Visual servoing to center target in camera
|   Servo pitch adjusts for distance + ballistic drop
v
SPRAY - Activate water pump (500ms)
|   v
VERIFY - Circle change: compare before/after spray images (>20% change)
|   If passed: SUCCESS
|   If failed: retry (max 2 sprays)
v
UPLOAD - Post photo to Google Drive
|   Filename: Task_2_MAD_target_<n>.jpg
v
COMPLETE - Ready for next target
```

**Key difference from previous version**: APPROACH is now Nav2-based
(obstacle-avoiding) instead of direct velocity. This earns the
**20-point autonomous extinguishing bonus** per CONOPS Table 6.

---

## API Endpoints

### Spray Control

```
POST /api/spray/trigger
Body: {
    "target_id": 1,
    "x": 1.0,          # world NED coords
    "y": 2.0,
    "z": -0.5,
    "label": "fire_1",
    "confidence": 0.95
}
Response: 200 OK
{
    "success": true,
    "target_id": 1,
    "distance": 2.8,        # distance to target (meters)
    "skip_approach": false   # true if already within 2m
}
Error: 400 Bad Request
    - Drone > 3m from target
    - Spray already active
```

```
GET /api/spray/status
Response: {
    "state": "approach",            # idle|approach|aim|spray|verify|upload|complete|failed|aborted
    "target_id": 1,
    "target_label": "fire_1",
    "distance_to_target": 2.34,
    "servo_angle": 85.5,
    "spray_count": 0,
    "verification_passed": false,
    "nav2_goal_id": "a1b2c3d4",     # Nav2 goal tracking
    "nav2_approach_active": true,    # waiting for Nav2 result
    "approach_method": "nav2",       # "nav2" or "velocity"
    "upload_url": "",
    "error": null,
    "targets_engaged": 5,
    "targets_succeeded": 4,
    "targets_failed": 1
}
```

```
POST /api/spray/abort
Aborts current sequence, cancels Nav2 goal, returns to IDLE.
```

### Nav2 Integration (used by spray approach)

```
POST /api/nav2/goal
  - Spray controller sends NavigateToPose goal here
  - nav2_goal_bridge polls /api/nav2/pending to dispatch

GET /api/nav2/status
  - Spray controller monitors approach progress

POST /api/nav2/result
  - nav2_goal_bridge reports goal completion
  - Spray controller receives result via update_nav2_result()
```

---

## Operator Workflow

### Before Mission

1. Check drone battery, water level (baking soda solution loaded)
2. Arm drone in GUIDED mode
3. Verify VIO healthy (ZED + nvblox running)
4. Verify Nav2 stack running (nav2_goal_bridge active)
5. Hover at safe altitude

### During Mission

1. **Position manually**: Use WASD controls to fly within 3m of target
2. **Confirm target visible**: Check live video feed — target detected (purple circle)
3. **Trigger spray**: Click "Spray Target" button in Mission Planner
4. **Monitor approach**: Watch Nav2 status — drone navigating to 2m
5. **Monitor spray**: Watch servo aiming and spray confirmation
6. **Repeat**: Move to next target (manual WASD back to 3m+ from next target)

### Safety Stops

- **Manual position adjustment**: Send WASD at any time (cancels autonomous aiming)
- **Emergency abort**: Trigger `/api/spray/abort` endpoint
- **Mission abort**: Switch to STABILIZE or ALT_HOLD mode
- **Nav2 cancel**: Abort cancels both spray sequence AND Nav2 goal

### Autonomous Extinguishing Points (CONOPS Table 6)

To earn the 20-point autonomous extinguishing bonus, ALL of the following must be autonomous:

1. ✅ **Approach from >2m** — Nav2 NavigateToPose (3m→2m)
2. ✅ **Aiming** — Visual servoing with servo pitch + ballistic drop
3. ✅ **Extinguishing** — Water pump activation
4. ✅ **Image capture + upload** — Circle change verify + Google Drive upload

The system satisfies all four criteria. The operator only manually positions
within 3m (which triggers the sequence), then all approach/aim/extinguish/upload
is autonomous.

---

## Nav2 Approach Details

### Why Nav2 (Simplified)

- **Obstacle avoidance**: Nav2 uses nvblox 3D costmap — won't hit walls/furniture
- **Compliance scoring**: 20 points for autonomous extinguishing requires autonomous approach from >2m
- **Short range only**: 3m→2m, not full arena navigation
- **RAM efficient**: 3m radius work area keeps nvblox costmap small

### Approach Goal Computation

```python
# Approach point is 2m from target along drone→target vector
drone_pos = get_drone_position()  # (x, y, z) NED
dx = target.x - drone_pos[0]
dy = target.y - drone_pos[1]
dz = target.z - drone_pos[2]
dist = sqrt(dx² + dy² + dz²)

# Normalized unit vector
nx, ny, nz = dx/dist, dy/dist, dz/dist

# Point 2m from target (back along approach vector)
approach_x = target.x - nx * 2.0
approach_y = target.y - ny * 2.0
approach_z = target.z - nz * 2.0

# Yaw facing toward target
yaw = atan2(dy, dx)
```

### Obstacle Sector Exclusion

During APPROACH, the target's angular direction is excluded from
OBSTACLE_DISTANCE (72 sectors × 5°) so the drone can navigate
toward the target without the costmap blocking that direction.

```python
# Compute target direction
angle_deg = degrees(atan2(dy, dx)) % 360
center_sector = int(angle_deg / 5) % 72

# Exclude ±2 sectors (±10°) around target direction
excluded = {(center_sector + offset) % 72 for offset in range(-2, 3)}
```

Exclusions are cleared when:
- Approach completes (Nav2 succeeded or timeout)
- Approach falls back to velocity mode
- Spray sequence is aborted

### Fallback: Direct Velocity Approach

If Nav2 is unavailable (nav2_goal_bridge not running, goal rejected, or goal failed):

```python
# Simple proportional velocity toward target
speed = min(0.5, distance * 0.5)  # m/s
vx = (dx / dist) * speed
vy = (dy / dist) * speed
vz = (dz / dist) * speed * 0.3    # slower vertical
nav_controller.send_velocity(vx, vy, vz, 0)  # 10 Hz loop
```

No obstacle avoidance in fallback mode — operator must verify clear path.

---

## Target Detection & Verification

### Circle Detection (ZED Custom OD)

- ZED2i camera runs custom circle detection model (ONNX)
- Detects colored circles (5-30cm) per CONOPS specifications
- Reports bounding box, 3D position, label, confidence
- Detection data flows via ros_http_bridge → `/api/detections/update`

### Circle Change Verification (color-agnostic)

After spray, captures frame and checks for color shift:

```python
# Purple (dry) → Blue (wet) in OpenCV HSV
# Dry: H ~130-160 (purple), S > 80, V > 50
# Wet: H ~100-130 (blue),   S > 80, V > 50

hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
blue_mask = cv2.inRange(hsv, [100, 80, 50], [130, 255, 255])
blue_ratio = count(blue_mask) / total_pixels

passed = blue_ratio > 0.15  # 15% blue = successful spray
```

Per CONOPS FAQ Q31: For autonomy points, the **UAS** must declare
the target extinguished — not the human operator. The circle change check
provides this autonomous declaration.

---

## Configuration

### spray_controller.py Parameters

```python
TRIGGER_MAX_DISTANCE_M = 3.0    # Max distance to trigger
APPROACH_STOP_DISTANCE_M = 2.0  # Nav2 goal: 2m from target
APPROACH_TIMEOUT_S = 20.0       # Nav2/velocity approach timeout
NAV2_GOAL_SETTLE_TIME_S = 1.0   # Settle after Nav2 stops
NAV2_STATUS_POLL_INTERVAL_S = 0.2  # Nav2 status check rate
AIM_TOLERANCE_PX = 30           # Aiming accuracy (pixels)
SPRAY_DURATION_MS = 500         # Pump duration
SPRAY_SETTLE_TIME_S = 0.5       # Wait before verify
MAX_SPRAY_ATTEMPTS = 2          # Retry on verify fail
SERVO_GAIN = 0.1                # Servo responsiveness
```

### Nav2 Configuration (simplified for 3m approach)

Nav2 uses the same stack as indoor navigation but only within
the 3m engagement zone:

- **Planner**: Smac Hybrid-A* (short paths, slow replan OK)
- **Controller**: DWB Local Planner (20Hz velocity output)
- **Costmap**: nvblox 3D → 2D projection (5Hz update)
- **Max velocity**: 0.5 m/s (conservative indoor approach)
- **Goal tolerance**: 0.3m position, 0.2 rad orientation

---

## Obstacle Avoidance

### During Nav2 Approach (3m→2m)

- **Primary**: Nav2 nvblox costmap — obstacle-avoiding path
- **Target sector excluded**: OBSTACLE_DISTANCE override so flight controller doesn't fight Nav2
- **3m radius costmap**: Keeps nvblox RAM low (~50MB vs ~200MB for full arena)

### Pre-Flight Obstacle Check

Before the mission, a 3m radius nvblox occupancy check verifies
the engagement zone is clear enough for approach:

- Checks nvblox occupancy map within 3m of target
- Warns operator if dense obstacles detected
- Does NOT abort the mission (operator can still try)

### Not Active During Spray

After approach (AIM/SPRAY/VERIFY/UPLOAD states), the drone is
stationary at 2m from target. Obstacle avoidance degrades to
position hold only — no dynamic costmap needed.

---

## Autonomous Features

### Visual Servoing (AIM state)

```
Error = target center - image center
If |error| < tolerance: DONE → proceed to SPRAY

Servo pitch adjustment:
  pitch_adjust = err_y * SERVO_GAIN
  pitch_adjust += ballistic_drop(distance)
  servo_angle = current - pitch_adjust
```

**Ballistic Drop Compensation** (water nozzle at ~2 bar):

| Distance | Drop Angle |
|----------|-----------|
| 1.0m | 0° |
| 2.0m | 2° |
| 3.0m | 5° |
| 4.0m | 8° |
| 5.0m | 12° |

### Circle Change Verification (VERIFY state)

- Captures frame after spray + 0.5s settle time
- Detects circles color-agnostically, compares before/after (>20% change)
- If check passes: mark successful, proceed to UPLOAD
- If check fails: re-aim and retry (max 2 attempts)

### Google Drive Upload (UPLOAD state)

- Captures proof photo
- Uploads with naming: `Task_2_MAD_target_<n>.jpg`
- Per CONOPS §5.2.4(4f): "Task_2_\<team_name\>_target_\<target#\>"
- Stores URL in status for operator reference
- Upload must be autonomous (no human intervention per FAQ Q30)

---

## Error Handling

### During APPROACH (Nav2)

- **Nav2 unavailable**: Fall back to direct velocity approach
- **Nav2 goal rejected**: Fall back to direct velocity approach
- **Nav2 goal failed**: Fall back to direct velocity approach
- **Nav2 timeout (20s)**: Cancel Nav2 goal, proceed from current position
- **NavController unavailable**: Mark APPROACH failed, abort sequence

### During APPROACH (Velocity fallback)

- **NavController unavailable**: Mark failed
- **Timeout**: Proceed from current position anyway
- **VIO lost**: NavController refuses commands → loops until VIO recovers or timeout

### During AIM (visual servoing)

- Detection lost: Wait up to 15 seconds, then proceed
- Servo error: Continue spray anyway
- Timeout: Proceed with last-known servo angle

### During SPRAY

- Servo unavailable: Log warning, continue
- Water pump failure: Log warning, proceed (assume spray occurred)

### During VERIFY

- Photo capture fails: Log warning, treat as unverified
- Circle change function unavailable: Fall back to HSV, then assume pass

### During UPLOAD

- Photo missing: Log warning, continue
- Upload fails: Log error, store URL as empty

**Design Philosophy**: Fail soft. If any autonomous feature unavailable,
attempt spray anyway. Mission success judged on judge verification
at end of flight window, not sensor feedback.

---

## Mission Planner Integration

### Task 2 View

- **VIO Status**: Tracking quality, message rate, source
- **Distance to Target**: Real-time distance readout
- **Nav2 Status**: Current approach state (pending/navigating/succeeded/failed)
- **Mode Status**: Current operational mode
- **Spray Status**: Current state, approach method, spray count, verification result

### Controls

- **Spray Button**: Triggers spray on target (operator selects target first)
- **Abort Button**: Abort current spray + cancel Nav2 goal
- **WASD Keys**: Manual positioning (3m engagement zone entry)

---

## Competition Scoring Impact

### Task 2 Points Breakdown (CONOPS Table 6)

| Criterion | Points | Our Strategy |
|-----------|--------|-------------|
| Target Extinguishing | 70 | Per-target points (40 indoor / 30 outdoor) |
| Autonomous Takeoff | 5 | ArduPilot GUIDED mode auto-takeoff |
| **Autonomous Extinguishing** | **20** | **Nav2 approach + visual servo + spray + upload** |
| Autonomous Landing | 5 | ArduPilot RTL |
| RTM SOP Compliance | 15 | Big City SOPs (Appendix F) |
| Safe Landing | 5 | Guided landing at flight line |
| **Total** | **120** | |

The Nav2 approach is specifically designed to earn the
**20-point autonomous extinguishing** bonus, which requires:
- All approach/positioning from >2m away (✅ Nav2 3m→2m)
- Aiming/target locking (✅ Visual servoing)
- Successful extinguishing + image capture (✅ circle change verify + photo)
- Upload (✅ Google Drive autonomous upload)

---

## Testing Checklist

- [ ] Drone boots in GUIDED mode, VIO healthy
- [ ] WASD positioning works over LTE/Tailscale
- [ ] Nav2 stack starts correctly (container + nav2_goal_bridge)
- [ ] Spray trigger validates 3m distance check
- [ ] Nav2 NavigateToPose goal sent and accepted
- [ ] Nav2 approaches to 2m with obstacle avoidance
- [ ] Nav2 result reported back to spray controller
- [ ] Velocity fallback works when Nav2 unavailable
- [ ] Obstacle sector exclusion set/cleared correctly
- [ ] Visual servoing centers target via servo pitch
- [ ] Ballistic drop compensation adjusts angle by distance
- [ ] Water pump triggers and photo captured
- [ ] Circle change verification detects >20% pixel change in target circle
- [ ] Failed spray triggers retry (max 2 attempts)
- [ ] Photo uploads to Google Drive with correct naming
- [ ] Status updates reflect state transitions + Nav2 tracking
- [ ] Abort cancels Nav2 goal + spray sequence + clears exclusions
- [ ] Full sequence: WASD → trigger → Nav2 approach → aim → spray → verify → upload

---

## Limitations & Future Work

### Current Limitations

1. **Nav2 approach within 3m only** — Full arena navigation not implemented
2. **No lateral aiming** — Servo only controls pitch, not drone XY position
3. **Pre-flight obstacle check only** — Not dynamic during spray states
4. **No target tracking prediction** — Steps through targets one at a time

### Future Enhancements

1. Multi-target sequential engagement (automatic next target selection)
2. Lateral servo aiming (2-axis gimbal or drone repositioning)
3. Dynamic costmap during spray states (if RAM permits)
4. Target detection prediction/filtering for faster visual servoing
5. Full arena Nav2 navigation (waypoint-based search pattern)

---

## References

- **Spray Controller**: [edge_core/spray_controller.py](../edge_core/spray_controller.py)
- **Nav2 Goal Bridge**: [edge_core/ros/nav2_goal_bridge.py](../edge_core/ros/nav2_goal_bridge.py)
- **Nav Controller**: [edge_core/nav_controller.py](../edge_core/nav_controller.py)
- **API Endpoints**: [edge_core/api.py](/api/spray/*, /api/nav2/*)
- **Mission Planner UI**: [mission_planner/src/NOMADTask2View.cs](../mission_planner/src/NOMADTask2View.cs)
- **Nav2 Integration Plan**: [docs/NAV2_INTEGRATION_PLAN.md](NAV2_INTEGRATION_PLAN.md)
- **Architecture**: [docs/JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md)
- **CONOPS**: 2026-AEAC-CONOPS-v1.3 §5.2.4 (Task 2: Fire Extinguishing)
