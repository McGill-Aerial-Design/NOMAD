# Task 2: Manual Trigger + ZED-Guided Autonomous Spray

**Updated**: May 9, 2026
**Status**: Classical circle detection + fixed firing geometry; nvblox/YOLO not required for Task 2

---

## Overview

**Task 2** is an indoor fire extinguishing mission (AEAC 2026 CONOPS §5.2.4):

- **Manual operator setup** — Operator flies until the ZED2i sees a target, typically up to about 5m camera range
- **Autonomous approach/alignment** — After the Spray button, the UAS uses the local circle detector, optional ZED depth, and GUIDED velocity/yaw-rate commands to reproduce the calibrated firing view
- **Fixed firing geometry** — The target is driven to a tunable water-landing pixel and camera range before spraying
- **Autonomous spray sequence** — Pre-photo, pump activation, post-photo, wetness/change verification, and upload run without further operator input
- **Field tuning** — Mission Planner exposes the standoff, aim pixel, servo angle, gains, speed limits, timeout, and trigger range

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
    | Manual WASD positioning until target is visible in ZED stream
    v
 GUIDED Mode Drone (target visible, typically ≤5.5m camera range)
    |
    | Operator clicks "Spray Target" button
    v
POST /api/spray/trigger
    |
    v
Edge Core - SprayController
    |
    +-- APPROACH: Direct GUIDED velocity toward target stand-off
    |   |   Uses target position/depth when available
    |   |   Stops near approach point, then refines with visual servoing
    |   v
    +-- AIM: Image visual servoing
    |   - Align target to calibrated water-landing pixel
    |   - Hold calibrated camera range before firing
    |   - Use yaw or lateral velocity for horizontal correction
    +-- SPRAY: Activate water pump
    +-- VERIFY: Circle change detection (before/after comparison)
    +-- UPLOAD: Post photo to Google Drive
    v
Sequence complete → ready for next target
```

### Data Flow: ZED-Guided Approach

```
  RGB snapshot ──> Task2CircleDetector ──> app.state.detected_objects
                                                     │
                                                     v
  Operator trigger ──> SprayController ──> NavController.send_velocity()
                                                     │
                                                     v
                                      MAVLink GUIDED velocity/yaw-rate
       v
  SprayController continues to AIM state
```

---

## Spray State Machine

```
Trigger by operator (target visible, within configured start range):

APPROACH - Direct velocity approach from >2m toward target
|   Timeout: 20s → proceed from current position
v
AIM - ZED visual servoing to calibrated aim pixel and camera range
|   Servo moves to calibrated nozzle fire angle
|   Altitude/yaw/lateral/forward velocity refine the shot geometry
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

**Key difference from previous version**: Task 2 no longer depends on nvblox.
The scoring-critical portion is the autonomous sequence after the operator
clicks Spray: approach from >2m, aim/align, extinguish, verify, and upload.

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
    - Drone > configured trigger range from target
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
    "approach_method": "velocity",
    "upload_url": "",
    "error": null,
    "targets_engaged": 5,
    "targets_succeeded": 4,
    "targets_failed": 1
}
```

```
POST /api/spray/abort
Aborts current sequence, stops movement, returns to IDLE.
```

### Spray Calibration

```
GET /api/spray/calibration
  - Returns current Jetson spray/aim calibration

POST /api/spray/calibration
  - Mission Planner pushes field-tuned spray values
  - Persists to ~/.nomad/calibration/spray_calibration.json by default
```

---

## Operator Workflow

### Before Mission

1. Check drone battery, water level (baking soda solution loaded)
2. Arm drone in GUIDED mode
3. Verify VIO healthy and Task 2 circle detections are updating
4. Verify Mission Planner Spray calibration was pushed to the Jetson
5. Hover at safe altitude

### During Mission

1. **Position manually**: Use WASD controls until the target is visible in the ZED stream
2. **Confirm target visible**: Check live video feed — target detected (purple circle)
3. **Trigger spray**: Click "Spray Target" button in Mission Planner
4. **Monitor approach**: Drone autonomously approaches and aligns to the calibrated firing view
5. **Monitor spray**: Watch servo aiming and spray confirmation
6. **Repeat**: Move to next target and reacquire it in the ZED stream

### Safety Stops

- **Manual position adjustment**: Send WASD at any time (cancels autonomous aiming)
- **Emergency abort**: Trigger `/api/spray/abort` endpoint
- **Mission abort**: Switch to STABILIZE or ALT_HOLD mode
- **Movement stop**: Abort stops spray sequence and sends zero velocity

### Autonomous Extinguishing Points (CONOPS Table 6)

To earn the 20-point autonomous extinguishing bonus, ALL of the following must be autonomous:

1. ✅ **Approach from >2m** — GUIDED velocity approach after operator trigger
2. ✅ **Aiming** — image visual servoing to calibrated water-landing pixel/range
3. ✅ **Extinguishing** — Water pump activation
4. ✅ **Image capture + upload** — Circle change verify + Google Drive upload

The system satisfies all four criteria. The operator only manually positions
until the target is visible and clicks the trigger; approach/aim/extinguish/upload
then run autonomously.

---

## Approach Details

### Why Fixed Firing Geometry

- **Reliability**: One calibrated standoff and aim pixel is easier to validate than a distance/angle table
- **Compliance scoring**: 20 points for autonomous extinguishing requires autonomous approach from >2m
- **Field tuning**: Mission Planner can adjust range, aim pixel, fire angle, gains, and limits during test flights
- **No nvblox/YOLO dependency**: Task 2 uses deterministic circle detection for target-relative alignment

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

During APPROACH, the target's angular direction can be excluded from
OBSTACLE_DISTANCE (72 sectors × 5°) so proximity sensors do not fight the
short motion toward the visible target.

```python
# Compute target direction
angle_deg = degrees(atan2(dy, dx)) % 360
center_sector = int(angle_deg / 5) % 72

# Exclude ±2 sectors (±10°) around target direction
excluded = {(center_sector + offset) % 72 for offset in range(-2, 3)}
```

Exclusions are cleared when:
- Approach completes or times out
- Spray sequence is aborted

### Direct Velocity Approach

The primary Task 2 approach uses simple proportional velocity toward the target:

```python
# Simple proportional velocity toward target
speed = min(0.5, distance * 0.5)  # m/s
vx = (dx / dist) * speed
vy = (dy / dist) * speed
vz = (dz / dist) * speed * 0.3    # slower vertical
nav_controller.send_velocity(vx, vy, vz, 0)  # 10 Hz loop
```

No path planner is active in this Task 2 path — operator must verify a clear
line before clicking Spray.

---

## Target Detection & Verification

### Circle Detection

- Edge Core runs a deterministic Task 2 circle detector on RGB snapshots
- Primary pass segments pale purple/blue red-cabbage paper from white backing
- Hough/contour shape checks supplement the color pass
- Current detections are exposed through `/api/detections` for Mission Planner selection

### Wall Distance / Standoff

The distance source is the ZED wrapper registered depth image:

```text
/zed/zed_node/depth/depth_registered
or
/zed2i/zed_node/depth/depth_registered
```

`target_localizer_node.py` subscribes to this depth image and the rectified RGB
image. For each detected circle it samples valid depth pixels inside the circle
interior and reports the median as `distance_m`. The ROS HTTP bridge subscribes
to `/target_localizer/detection_status_json`, converts those circles into
`/api/detections` entries, and forwards the value as `range_m`.

During spray aiming, `SprayController` uses:

```python
range_error = range_m - target_camera_range_m
vx = range_error * forward_gain
```

So the aircraft moves forward only when the ZED depth says the target wall is
farther than the calibrated firing distance. If ZED depth is missing, the
controller falls back to pixel-only visual alignment and should be treated as a
manual-standoff mode.

### Circle Change Verification

After spray, captures a frame and checks for the baking-soda chemical response
inside the detected circle. Generic image change is only supporting evidence,
because rain or plain water can darken the paper without producing a valid
blue/cyan reaction.

```python
# Purple (dry) → Blue (wet) in OpenCV HSV
# Dry: H ~130-160 (purple), S > 80, V > 50
# Wet: H ~100-130 (blue),   S > 80, V > 50

hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
blue_mask = cv2.inRange(hsv, [100, 80, 50], [130, 255, 255])
blue_ratio = count(blue_mask) / target_circle_pixels
blue_increase = blue_ratio_after - blue_ratio_before

passed = blue_ratio_after >= 0.08 and blue_increase >= 0.05
```

Per CONOPS FAQ Q31: For autonomy points, the **UAS** must declare
the target extinguished — not the human operator. The circle verification
provides this autonomous declaration while reducing false positives from plain
water or rain.

---

## Configuration

### Spray Calibration Parameters

```python
trigger_max_distance_m = 5.5    # Operator can start once target is visible at range
target_camera_range_m = 3.8     # ZED-to-wall range at the calibrated firing view
range_tolerance_m = 0.25        # Acceptable range error before spraying
aim_pixel_x = 640               # Water landing pixel X in the ZED image
aim_pixel_y = 390               # Water landing pixel Y in the ZED image
aim_tolerance_px = 25           # Pixel lock tolerance
servo_fire_angle_deg = 82.0     # Nozzle/camera servo firing angle
forward_gain = 0.45             # Range-to-forward velocity gain
yaw_gain = 0.0025               # Horizontal pixel-to-yaw-rate gain
altitude_gain = 0.0010          # Vertical pixel-to-altitude velocity gain
lock_hold_ms = 700              # Hold aim lock before spraying
align_timeout_s = 20.0          # Proceed after timeout from current alignment
```

These values can be pushed from Mission Planner Settings → Spray. The Jetson
also persists them to `~/.nomad/calibration/spray_calibration.json`.

---

## Obstacle Avoidance

### During Task 2 Approach

- **Primary**: operator chooses a clear line, then autonomy runs a short visible-target approach
- **Target sector excluded**: optional OBSTACLE_DISTANCE override so proximity data does not block the intended line to the target
- **Speed-limited**: forward/lateral/altitude/yaw limits are field-tunable in Mission Planner

### Pre-Flight Obstacle Check

Before the mission, the operator verifies the engagement zone is clear enough
for the short autonomous approach:

- Confirms there is no obstacle between UAS and target
- Confirms the nozzle arm clearance is safe
- Confirms the target can remain in the ZED field of view while approaching

### Not Active During Spray

After approach (AIM/SPRAY/VERIFY/UPLOAD states), the drone should be close to
the calibrated firing geometry. The controller sends only small correction
velocities during aim; no dynamic costmap is required for this Task 2 path.

---

## Autonomous Features

### Visual Servoing (AIM state)

```
Error = target pixel - calibrated water-landing pixel
Range error = ZED camera range - calibrated firing range
If pixel and range remain locked for lock_hold_ms: proceed to SPRAY

Commands:
  vx = range_error * forward_gain
  yaw_rate or vy = pixel_x_error * gain
  vz = pixel_y_error * altitude_gain
  servo_angle = servo_fire_angle_deg
```

The recommended calibration is a single fixed firing geometry. Multi-distance
servo tables remain possible, but they add field burden and should be treated as
a fallback if the fixed standoff cannot meet the wetting requirement.

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

### During APPROACH

- **NavController unavailable**: Mark failed
- **Timeout**: Proceed from current position anyway
- **VIO lost**: NavController refuses commands → loops until VIO recovers or timeout

### During AIM (visual servoing)

- Detection lost: Stop movement briefly and wait for the target to reacquire
- Servo error: Continue with the configured fire angle if movement alignment is good
- Timeout: Stop movement and proceed from the current alignment

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
- **Spray Calibration**: Fixed firing range, aim pixel, gains, limits, and push-to-Jetson control
- **Mode Status**: Current operational mode
- **Spray Status**: Current state, approach method, spray count, verification result

### Controls

- **Spray Button**: Triggers spray on target (operator selects target first)
- **Abort Button**: Abort current spray and stop movement
- **WASD Keys**: Manual positioning until target is visible

---

## Competition Scoring Impact

### Task 2 Points Breakdown (CONOPS Table 6)

| Criterion | Points | Our Strategy |
|-----------|--------|-------------|
| Target Extinguishing | 70 | Per-target points (40 indoor / 30 outdoor) |
| Autonomous Takeoff | 5 | ArduPilot GUIDED mode auto-takeoff |
| **Autonomous Extinguishing** | **20** | **ZED-guided approach + visual servo + spray + upload** |
| Autonomous Landing | 5 | ArduPilot RTL |
| RTM SOP Compliance | 15 | Big City SOPs (Appendix F) |
| Safe Landing | 5 | Guided landing at flight line |
| **Total** | **120** | |

The ZED-guided approach is specifically designed to earn the
**20-point autonomous extinguishing** bonus, which requires:
- All approach/positioning from >2m away (✅ trigger range up to 5.5m)
- Aiming/target locking (✅ Visual servoing)
- Successful extinguishing + image capture (✅ circle change verify + photo)
- Upload (✅ Google Drive autonomous upload)

---

## Testing Checklist

- [ ] Drone boots in GUIDED mode, VIO healthy
- [ ] WASD positioning works over LTE/Tailscale
- [ ] `/api/detections` shows `task2_circle` from the local RGB detector
- [ ] Mission Planner Spray calibration pushes to `/api/spray/calibration`
- [ ] Spray trigger validates configured start range
- [ ] Direct velocity approach starts from >2m and converges toward firing range
- [ ] Visual servoing moves target to calibrated water-landing pixel
- [ ] Gain signs are verified for yaw/lateral and altitude correction
- [ ] Water pump triggers and photo captured
- [ ] Circle change verification detects >20% pixel change in target circle
- [ ] Failed spray triggers retry (max 2 attempts)
- [ ] Photo uploads to Google Drive with correct naming
- [ ] Status updates reflect state transitions
- [ ] Abort stops velocity commands and cancels spray sequence
- [ ] Full sequence: WASD → trigger → approach → aim → spray → verify → upload

---

## Limitations & Future Work

### Current Limitations

1. **No obstacle-planning dependency in Task 2** — Operator must start from a safe line to target
2. **Fixed firing geometry** — Best performance depends on a good wall calibration
3. **Gain signs require flight validation** — Mission Planner exposes negative gains for quick inversion
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
- **Nav Controller**: [edge_core/nav_controller.py](../edge_core/nav_controller.py)
- **API Endpoints**: [edge_core/api.py](/api/spray/*)
- **Mission Planner UI**: [mission_planner/src/NOMADTask2View.cs](../mission_planner/src/NOMADTask2View.cs)
- **Architecture**: [docs/JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md)
- **CONOPS**: 2026-AEAC-CONOPS-v1.3 §5.2.4 (Task 2: Fire Extinguishing)
