# Task 1: Fire Reconnaissance — Competition Guide

**AEAC 2026 | CONOPS v1.3 | NOMAD System**

This document is the definitive reference for operating NOMAD's Task 1 pipeline during competition. It covers: what the CONOPS requires, how NOMAD implements each requirement, step-by-step instructions for competition day, and checklists.

---

## 1. CONOPS Requirements Summary

Task 1 is scored out of **100 points** across 5 criteria:

| Criterion | Max Points | Key Rules |
|-----------|-----------|-----------|
| **Target Detection & Location Accuracy** | 25 | Text-based description of each target's 3D position + colour. Descriptions must use landmarks as references, no GPS/numerical coordinates. **All distances must be expressed to decimetre (0.1m) precision only** — no finer (CONOPS §5.2.3.6b). Accuracy scoring: ≤0.5m=×100%, ≤1m=×75%, ≤1.5m=×50%, >1.5m=×0%. Correct colour=×100%, wrong/missing=×50%. |
| **Equipment Delivery** | 20 | Carry combo of radio (5pt), O₂ tank (5pt), ladder (10pt) to staging areas (32" landing pads). ≤2m from pad=×100%, >2m=×50%. No airdrops. |
| **Distance Flown** | 30 | Laps on a 400m–1km course en route to the scene. Highest laps=30pt, lowest=10pt, linear scale. |
| **Payload Fraction** | 20 | `MIN(PF, 0.35)/0.35 × 20pt` where `PF = payload_weight / total_takeoff_weight`. |
| **Safe Landing** | 5 | All UAVs land safely at flight line. |

### Key Rules from CONOPS §5.2.3

1. **Building data** (GPS, dimensions) provided Thursday night (2026-05-21 2359 ET).
2. **Scene boundary**: 15m from building perimeter, up to 10m AGL.
3. **Target characteristics**: Circle 5–30cm diameter, colours: black, white, red, yellow, blue, green. Outdoor only (no indoor targets in Task 1). May be inclined or partially obstructed.
4. **Target descriptions** must be:
   - Text-based, using **landmarks** as reference points (not GPS/numerical coordinates)
   - 3D position fixed unambiguously (height above ground + horizontal offset from a landmark)
   - Expressed to **decimetre (0.1m) precision only** — no finer (e.g. "1.4m" not "1.37m"). This is a hard format rule, not just an accuracy goal.
   - Uploaded as **`Task_1_MAD_targets.txt`** to team Google Drive folder by end of flight window
   - Format: see CONOPS Appendix D examples
5. **Equipment**: Radio ≤500g, O₂ tank ≤1kg, Ladder ≤3kg. Must be attached at flight line, single run (no return trips). Delivery is not airdrop — UAV must land/touch ground before release.
6. **Up to 2 UAVs** may be used. Lower lap count is used for scoring. Batteries **may not** be swapped.
7. **UAM corridor**: Flight between 20–35m AGL except near takeoff/destination. Must enter/stay ≥30s, transmit radio calls per RTM SOPs.
8. **RTM SOPs compliance**: 15 points at stake. Must follow callsign, takeoff/clearance, corridor, and landing radio procedures (Appendix F).

---

## 2. NOMAD Implementation vs. CONOPS Requirements

### 2.1 Target Detection & Description (25 pts)

| CONOPS Requirement | NOMAD Implementation | Status |
|---|---|---|
| Detect coloured circles (5–30cm, 6 colours) | HSV circle detector in `target_localizer/detectors.py` — HSV segmentation + contour circularity analysis. Classifies: black, white, red, yellow, blue, green. | ✅ Implemented |
| 3D back-projection to building-relative coordinates | `_pixel_to_3d_local()` in `target_localizer_node.py`: ZED depth + servo tilt angle + camera intrinsics → world ENU frame. | ✅ Implemented |
| Building face classification | `_resolve_observed_face()`: GPS+heading → building normal alignment → nearest face (N, NE, E, SE, S, SW, W, NW) | ✅ Implemented |
| Projection onto face for horizontal reference | `building_model.py: project_point_onto_face()` → horizontal offset from left corner of face | ✅ Implemented |
| Ground/roof target handling | `classify_nearest_plane()` checks wall distance, ground z, roof z → labels target as ground/roof/wall | ✅ Implemented |
| Natural-language description | `_generate_description()` outputs CONOPS Appendix D format: `"<Colour> target on the <face> face of the building, <height>m above ground, <distance>m from the <corner>."` | ✅ Implemented |
| Colour cross-verification | `ColorVerifier` double-checks HSV classification on the ROI independently | ✅ Implemented |
| Deduplication (same target seen twice) | 3D distance check within `dedup_radius_m` (default 0.5m) | ✅ Implemented |
| No GPS/numerical coordinates in description | Template engine uses face names, corner names, landmark references only | ✅ Implemented |
| Decimetre accuracy (0.1m) | Height and distance rounded to 1 decimal place | ✅ Implemented |
| Save to `Task_1_MAD_targets.txt` | `~/save_targets` service writes the .txt file with format `Target A: <description>` | ✅ Implemented |
| Upload to Google Drive | `Task1UploadPanel.cs` — in-Mission-Planner grid editor + upload via `GoogleDriveUploadService` | ✅ Implemented |
| N-corner polygon building (not just rectangle) | `building_model.py` accepts arbitrary polygon corners via `building.corner_names/lats/lons` | ✅ Implemented |
| Corner naming in descriptions | `Corner.description_name` property — e.g. "NW corner", "A corner" | ✅ Implemented |
| On-site corner GPS calibration | Mission Planner → Configuration → "BUILDING CORNER CALIBRATION" card: fly above corners, click "CAPTURE CORNER GPS", then "Apply to Model" | ✅ Implemented |
| Runtime building model rebuild | `POST /api/task/1/building/corners/apply` → `/target_localizer/set_building_corners` ROS2 service rebuilds BuildingModel without node restart | ✅ Implemented |

**Score analysis — CRITICAL GAP**: Our system computes "distance from corner" by comparing the target's GPS-derived 3D position against a building model whose corner GPS coordinates we manually estimate from satellite imagery. **The camera never sees the building corners** — it only sees the colored circle. The "1.8m from the NW corner" description is entirely a math result from the prior model, not a visual measurement.

**CONOPS provides only a single lat/lon to identify which building (not where on the building**, Q&A #6), plus dimensions to reconstruct the 3D exterior (Q&A #7). No door/window/landmark locations are given.

This means our corner GPS coordinates are only as good as our satellite imagery analysis. A 5° orientation error on a 10m building shifts corner positions by ~0.87m — enough to push every target description from the ×100% tier (≤0.5m) into the ×75% tier (≤1.0m). A 10° error gives ~1.74m error → ×0% tier.

**Mitigation — On-Site Corner Calibration (✅ Implemented)**:

The **Building Corner Calibration** feature (Configuration tab → "BUILDING CORNER CALIBRATION" card) lets the operator fly the drone above each visible building corner and click "CAPTURE CORNER GPS" to record that corner's position using the drone's GPS. Once ≥3 corners are captured, clicking "Apply to Model" rebuilds the building model at runtime — no YAML editing or node restart required.

| Step | Action |
|---|---|
| 1 | Fly the drone directly above a building corner (use the ZED camera feed to align) |
| 2 | In Mission Planner → Task 1 → **Configuration** tab → type corner name (NW/NE/SE/SW for rectangles, A/B/C/... for L-shapes or polygons >4 corners) |
| 3 | Click **CAPTURE CORNER GPS** |
| 4 | Repeat for each visible corner |
| 5 | Click **Apply to Model** (requires ≥3 corners) |

The drone's Cube Orange RTK GPS provides ~1-2cm horizontal accuracy — far better than satellite imagery estimation (~1-2m error). With on-site calibration, the building model geometry error drops to drone GPS accuracy + hover offset (likely <0.5m), keeping descriptions within the ×100% scoring tier.

**API Endpoints**:
- `POST /api/task/1/building/corner` — save one corner (name + lat/lon; supports any number of corners for arbitrary polygon buildings)
- `GET /api/task/1/building/corners` — list saved corners
- `DELETE /api/task/1/building/corners` — clear all corners
- `POST /api/task/1/building/corners/apply` — rebuild building model from saved corners

**Additional mitigations** (fallback if on-site calibration is unavailable):
1. **Walk to corners with handheld GPS**: If you can't fly, walk to corners with a phone GPS and manually enter values in `competition.yaml`.
2. **Satellite imagery with Google Earth Pro**: Measure orientation from the heading tool. Multiple cross-checks reduce orientation error.
3. **Describe targets relative to face name, not just corners**: Face names (north, south, etc.) are less sensitive to corner position error than exact corner offsets.
4. **Manual fallback for corner offsets**: If the automated corner-offset numbers look wrong in the GCS preview, edit the description text before submission.

ZED depth error at 3–5m range is ~1–2% (3–10cm), which is small compared to the building model uncertainty. **The dominant error source is the building geometry prior, not the depth sensor.** Colour accuracy is cross-verified; the penalty for wrong colour is ×50%, so the `ColorVerifier` cross-check remains the key colour accuracy lever.

### 2.2 Equipment Delivery (20 pts)

| CONOPS Requirement | NOMAD Implementation | Status |
|---|---|---|
| Carry equipment to scene | UAV physically carries payloads (manual attachment at flight line) | ✅ Mechanical |
| Drop at staging areas (32" landing pads) | `PayloadControlPanel.cs` — "Drop Payload 1/2" buttons trigger MAVLink `DO_SET_RELAY` commands → GPIO relays | ✅ Implemented |
| Multiple payloads on separate pads | Two relay channels (`GPIO_PAYLOAD1_PIN`, `GPIO_PAYLOAD2_PIN`) + manual third payload drop | ✅ Implemented |
| No airdrop (must land/touch ground first) | Operator procedure — must land or touch ground before pressing drop button | ⚠️ Operator-enforced |
| Ladder (up to 3kg, 15×60×120cm) | Physical mount — operator attaches at flight line | ⚠️ Mechanical |

**Score analysis**: Ladder (10pt) + O₂ tank (5pt) + radio (5pt) = 20 pts max with ≤2m accuracy. Landing accuracy depends on pilot skill and positioning.

### 2.3 Distance Flown / Laps (30 pts)

| CONOPS Requirement | NOMAD Implementation | Status |
|---|---|---|
| Fly laps en route to scene | Pilot-controlled via Mission Planner waypoints / ArduPilot AUTO mode | ✅ Operational |
| Lap counting | ArduPilot mission planning (waypoint sequences) | ✅ Operational |
| Course waypoints provided Thursday night | Must be entered into Mission Planner before flight | ⚠️ Manual entry |

**Score analysis**: 30 pts is the single largest criterion. If all teams complete the same number of laps, ranking resolves by time. Flying more laps (risk/reward) or flying faster matters.

### 2.4 Payload Fraction (20 pts)

| CONOPS Requirement | NOMAD Implementation | Status |
|---|---|---|
| `PF = payload_weight / total_takeoff_weight` | Determined by physical build — total weight / carried payload weight | ✅ Physical |
| `MIN(PF, 0.35)/0.35 × 20pt` | Maximised by carrying the heaviest possible payload (ladder=3kg) on the lightest possible airframe | ✅ Design |

**Score analysis**: PF of 0.35 gives 20/20. If total takeoff weight is ~8.5kg and we carry the ladder (3kg), PF = 3/8.5 = 0.353 → capped at 0.35 → full marks. This is achievable.

### 2.5 Safe Landing (5 pts)

| CONOPS Requirement | NOMAD Implementation | Status |
|---|---|---|
| All UAVs landed safely at flight line | Standard ArduPilot RTL / LAND mode | ✅ Operational |

### 2.6 RTM SOPs Compliance (15 pts — scored in Task 2, but SOPs apply to ALL flights)

| SOP Requirement | NOMAD Approach | Status |
|---|---|---|
| Callsign registration | Register `NOMAD 406K` (or similar) before flight | ⚠️ Manual |
| Radio check on startup | Operator says: "NOMAD 406K to base, radio check" | ⚠️ Manual |
| Takeoff clearance request | Operator says: "NOMAD 406K to base, request takeoff for firefighting mission." | ⚠️ Manual |
| UAM corridor entry/exit calls | Operator says: "NOMAD 406K entering corridor" / "NOMAD 406K has left the corridor" | ⚠️ Manual |
| "Operating near the building" call | Operator says: "NOMAD 406K operating near the building" | ⚠️ Manual |
| Landing clearance request | Operator says: "NOMAD 406K to base, request landing." | ⚠️ Manual |
| Emergency procedures | Operator follows correct radio responses | ⚠️ Manual |

**Note**: These are **operator procedures**, not software. One error = 10/15 pts, two errors = 0/15 pts. PRINTED cheat sheets with the exact phrasing are essential.

---

## 3. System Architecture for Task 1

### 3.1 Data Flow

```
ZED 2i Camera (Jetson, inside Isaac ROS container)
    │
    ├── RGB image ──────────► target_localizer_node (HSV circle detection)
    ├── Depth image ─────────► target_localizer_node (3D back-projection)
    └── Camera intrinsics ──► target_localizer_node (pixel→3D math)
    
CubePilot (MAVROS)
    ├── GPS (lat/lon) ──────► target_localizer_node (building-relative coords)
    ├── Heading ────────────► target_localizer_node (face selection)
    └── Local pose ─────────► target_localizer_node (fallback if no GPS)

Servo angle (/servo/angle) 
    └────────────────────────► target_localizer_node (depth tilt compensation)

    ┌─── On operator button press ───┐
    │                                │
    ▼                                ▼
HSV Detection ──► 3D Back-projection ──► Building Face ──► Description
    │                                                    │
    ▼                                                    ▼
Annotated image saved                   Target record appended
(e.g. target_A.jpg)                     to in-memory list
                                                        │
                    ┌─── On "Save Targets" ───┐         │
                    │                         ▼         │
                    │          Task_1_MAD_targets.txt    │
                    │                         │         │
                    ▼                         ▼         │
         Upload to Google Drive ←─── Mission Planner Submit Tab
```

### 3.2 Key Software Components

| Component | Location | Role |
|-----------|----------|------|
| **target_localizer_node.py** | `edge_core/target_localizer/` | Main ROS 2 node. HSV detection, 3D back-projection, face classification, description generation, deduplication, file output. |
| **building_model.py** | `edge_core/target_localizer/` | Arbitrary N-corner polygon building geometry. Face naming (8 compass buckets), corner references, plane classification (wall/ground/roof), nearest-reference search. |
| **detectors.py** | `edge_core/target_localizer/` | `CircleDetector` (HSV segmentation + contour analysis), `LandmarkDetector` (YOLO, disabled for Task 1), `ColorVerifier` (independent cross-check). |
| **geospatial.py** | `edge_core/` | Pure math: GPS↔NED conversions, raycasting, Haversine distance, bearing. |
| **competition.yaml** | `edge_core/target_localizer/config/` | Building corners, dimensions, team name, HSV tuning params. |
| **building_corners.json** | `/home/mad/targets/` | Corner calibration data written by API, read by `set_building_corners` service. Created on-site during flight. |
| **Edge Core API** | `edge_core/api.py` | REST endpoints: `/api/task/1/target/capture`, `/api/task/1/target/save`, `/api/task/1/target/model`, `/api/task/1/captures`, image download. |
| **NOMADTask1View.cs** | `mission_planner/src/` | GCS UI: GPS status, capture button, gallery, AI description viewer, building location config. |
| **Task1UploadPanel.cs** | `mission_planner/src/` | GCS UI: target grid editor, description entry, preview, Google Drive upload. |
| **PayloadControlPanel.cs** | `mission_planner/src/` | GCS UI: payload drop buttons (relay 1/2), water shooter, nozzle servo slider. |
| **DualLinkSender.cs** | `mission_planner/src/` | HTTP/MAVLink dual-path for sending capture commands. |

### 3.3 File Outputs

| File | Location | Content |
|------|----------|---------|
| `Task_1_MAD_targets.txt` | `/home/mad/targets/` on Jetson (or `output_dir` param) | Competition submission file. Format: `Target A: <description>` |
| `Task_1_MAD_targets_debug.txt` | Same dir | Raw 3D coords, face label, confidence — for post-flight review |
| `target_<letter>.jpg` | `/home/mad/targets/{timestamp}/` | Annotated detection image with bounding box and colour label |
| Metadata JSON | Downloaded to GCS `Documents/NOMAD/Task1/` | GPS, heading, gimbal angles, AI description |

---

## 4. Pre-Competition Setup

### 4.1 Thursday Night (after CONOPS releases data)

After 2359 ET on Thursday May 21, competition organizers provide:
- Building GPS coordinates (single lat/lon pair)
- Building dimensions
- Lap course waypoints
- Flight boundaries (soft + hard)
- Altitude limit (≤400ft AGL)

**You must do the following:**

- [ ] **1. Enter building GPS** in `competition.yaml`:
  ```yaml
  building:
    center_lat: <from CONOPS>
    center_lon: <from CONOPS>
    height: <from CONOPS, in meters>
  ```

- [ ] **2. Compute building corner polygon**: From satellite imagery of Area XO, determine the building's shape (L-shape, T-shape, rectangle). Compute GPS coordinates for each corner:
  ```yaml
  building:
    corner_names: ["NW", "SW", "SE", "NE"]  # or ["A", "B", "C", ...]
    corner_lats: [45.xxx, 45.xxx, ...]
    corner_lons: [-75.xxx, -75.xxx, ...]
  ```
  If you can only approximate a rectangle, leave `corner_names: []` and set:
  ```yaml
  building:
    rectangle:
      length: <long axis, meters>
      width: <short axis, meters>
      orientation_deg: <long-axis heading, deg CW from true north>
  ```

- [ ] **3. Enter lap waypoints** into Mission Planner as a waypoint mission.

- [ ] **4. Enter flight boundaries** into ArduPilot geofence (polygon fence via Mission Planner).

- [ ] **5. Set building location in Mission Planner**: Task 1 → Configuration tab → enter lat/lon → Save.

- [ ] **6. Calibrate HSV ranges** if time permits: `python3 tools/hsv_tuner.py --image <sample>`

- [ ] **7. Rebuild target_localizer**: `colcon build --packages-select target_localizer`

- [ ] **8. Verify building model**:
  ```bash
  ros2 run target_localizer target_localizer_node --ros-args --params-file config/competition.yaml
  # In another terminal:
  ros2 service call /target_localizer/print_model std_srvs/srv/Trigger
  ```
  Check that face headings and corner names match the satellite view.

- [ ] **9. Upload presentation** to Google Drive by 2359 ET.

### 4.2 Friday Morning (Before Flight)

- [ ] **1. Start Isaac ROS container**:
  ```bash
  cd ~/NOMAD && ./scripts/run/start_nomad_full.sh task1
  ```

- [ ] **2. Verify RTK/GPS fix** on CubePilot (3D Fix or better, ≥8 satellites)

- [ ] **3. Verify ZED camera streaming**: Check RTSP feed at `rtsp://100.85.121.98:8554/primary`

- [ ] **4. Verify target_localizer services are registered**:
  ```bash
  ros2 service list | grep target_localizer
  # Should show:
  # /target_localizer/capture_target
  # /target_localizer/save_targets
  # /target_localizer/print_model
  ```

- [ ] **5. Test one capture** at the flight line (no targets, but verify no errors):
  ```bash
  ros2 service call /target_localizer/capture_target std_srvs/srv/Trigger
  ```
  Expected: `success: false, message: "No colored circles detected"` (normal — no targets at flight line).

- [ ] **6. Verify Google Drive upload**:
  ```powershell
  # On Windows GCS
  Invoke-WebRequest -Uri 'http://100.85.121.98:8000/api/gdrive/status' -UseBasicParsing
  ```
  Should return `"available": true`.

- [ ] **7. Load Task 1 ArduPilot params**:
  ```powershell
  # Via Mission Planner: Config → Load Params → task1_gps.param
  # OR via script on Jetson:
  python3 scripts/run/load_params.py config/params/task1_gps.param
  ```

- [ ] **8. Set geofence** in Mission Planner (soft + hard boundaries from CONOPS).

- [ ] **9. Program lap waypoints** into mission.

- [ ] **10. Print RTM SOPs cheat sheet** (see §6 below).

- [ ] **11. Attach payloads** to UAV at flight line (radio, O₂ tank, ladder — choice of combo).

---

## 5. Competition Flight Procedure

### 5.1 Pre-Flight Window (30 minutes before)

1. **Arrive at flight line** 60 min before window (per CONOPS advice).
2. **Power on** Jetson → wait for Edge Core (`systemctl status nomad`).
3. **Power on** CubePilot → verify MAVLink in Mission Planner.
4. **Start Isaac ROS** (should be running from `start_nomad_full.sh task1`).
5. **Verify comms**: Tailscale, RTSP video, Edge Core health.
6. **Verify geofence** is loaded.
7. **Attach payloads** to UAV.
8. **Ready UAV** at edge of flight line for immediate rollout.

### 5.2 Flight Window Opens — RTM SOPs Sequence

> **IMPORTANT**: The radio operator must NOT be the pilot-in-command. Assign one person as dedicated radio operator.

| Step | Radio Call | Notes |
|------|-----------|-------|
| 1 | **"NOMAD 406K to base, radio check."** | ATC replies with signal/clarity rating (1–5). |
| 2 | **"NOMAD 406K to base, request takeoff for firefighting mission."** | Wait for clearance. |
| 3 | **"Cleared to takeoff, NOMAD 406K."** | Acknowledge clearance. Then take off. |
| 4 | **"NOMAD 406K takeoff complete."** | After takeoff. |
| 5 | **"NOMAD 406K entering corridor."** | When entering UAM corridor (20–35m AGL). Must stay ≥30s. |
| 6 | — | **Fly laps** en route to scene. Count laps! |
| 7 | **"NOMAD 406K has left the corridor."** | When departing corridor to approach building. |
| 8 | **"NOMAD 406K operating near the building."** | When entering the 15m search volume. |

### 5.3 At the Scene — Equipment Delivery

1. **Locate staging areas** (32" landing pads) via camera/visual.
2. **Land at first staging area** — UAV must be on ground or touching.
3. **Press "Drop Payload 1"** in Mission Planner → NOMAD → Task 1 → Payload Controls.
4. **Take off**, fly to second staging area, land.
5. **Press "Drop Payload 2"**.
6. **Repeat** for third payload if carrying 3 items (may need manual release).
7. **Confirm no part of UAS left behind.**

### 5.4 At the Scene — Building Corner Calibration (⚠️ Do this BEFORE targeting)

> **Critical**: The "distance from corner" in each target description depends on the building model's corner GPS coordinates. If these are estimated from satellite imagery, they may be off by 1-2m, pushing descriptions into lower scoring tiers. Calibrating corners on-site using the drone's RTK GPS eliminates this error.

1. **Fly the drone above the first visible building corner** — use the ZED camera feed to align.
2. **In Mission Planner → Task 1 → Configuration → "BUILDING CORNER CALIBRATION"**:
   - Type the corner name (presets: NW/NE/SE/SW for rectangles, A/B/C for polygons, or type custom)
   - Click **CAPTURE CORNER GPS**
3. **Repeat** for each visible corner (minimum 3 corners required for a polygon (supports N-corner buildings: L-shapes, T-shapes, etc.)).
4. **Click "Apply to Model"** — rebuilds the building model at runtime.
5. **Verify**: The corner list and model summary should match the building you see.

> If you calibrated corners from satellite imagery Thursday night and verified them, you can skip this step. But on-site calibration is always more accurate.

### 5.5 Camera Geometry — How to Position the Drone (READ THIS)

> **The ZED camera can only tilt up to 45° downward. It cannot look straight down.**
> This is the single biggest source of operator positioning errors. Read this section before your first pass.

#### Camera tilt convention

| Servo command | Camera direction | Use case |
|--------------|-----------------|----------|
| 1450 us | +45° (up) | Looking at high walls / rooftop |
| 1250 us | 0° (level/horizontal) | Default / transit |
| 700 us | −45° (down, maximum) | Ground targets / low walls |

**Set the camera to maximum down (700 us / −45°) before approaching any target area.**
The Mission Planner Task 1 capture tab shows the current tilt angle and estimated ground-ahead distance in real time.

#### Where does the camera look?

At **−45° tilt**, the camera's center line hits the ground at a **horizontal distance equal to the drone's altitude**.

```
Drone altitude 5m → ground intersection 5m ahead of the drone
Drone altitude 3m → ground intersection 3m ahead of the drone
```

The on-screen crosshair always marks this intersection point — center the target in the crosshair before pressing capture.

#### Positioning for ground targets

The drone **cannot hover directly above a ground target** and see it in the crosshair. The sightline is always 45° forward, never straight down.

```
Target on ground:
  → Fly forward of the target by approximately your AGL altitude
  → Example: target at ground level, drone at 4m AGL → position drone ~4m past the target
```

#### Positioning for wall targets

| Target height | Drone AGL | Recommended standoff |
|--------------|-----------|---------------------|
| Low (0–1m) | 3–4m | 3–4m from wall |
| Mid (1–3m) | 4–5m | 4–5m from wall |
| High (3–5m) | 5–6m | 5–8m from wall |

For upper-wall targets: if the drone is too close to the wall, the target will be **above the top of the image**. Move further back.

#### GCS tilt display

The capture tab shows: `Tilt: -45° | Gnd: 5.2m fwd`
- **Tilt** — actual servo angle read from the flight controller (SERVO_OUTPUT_RAW), not commanded
- **Gnd: X.Xm fwd** — estimated ground intersection ahead at current tilt + current altitude
- Use this to judge your forward offset when approaching ground targets, or to confirm the camera is aimed where you think it is for wall targets

#### Capture button states

| Button appearance | Meaning |
|------------------|---------|
| **Green — ● CAPTURE (N circles)** | HSV detector found N colored circles — ideal, capture now |
| **Blue — CAPTURE (crosshair)** | No circles detected — will use frame center depth as position |

Both states always allow capture. The crosshair fallback handles partially obstructed circles or unusual colors that the detector misses — center the target manually and press capture.

---

### 5.6 At the Scene — Target Detection & Description

> This is the core automated pipeline. One operator triggers captures, reviews results, and confirms/rejects targets.

**Per-target workflow:**

1. **Set camera tilt to maximum down** (700 us / −45°) using the servo slider in Mission Planner.
2. **Pilot positions UAV** using the geometry in §5.5 — check the tilt display for ground-ahead distance.
3. **Center the target in the crosshair** (visible on the live video feed in the capture tab).
4. **Operator clicks capture** in Mission Planner → Task 1 → Capture tab.
   - Green button = circles detected, capture is HSV-guided
   - Blue button = no circles, capture uses crosshair depth (still valid — press it)
   - This calls `/api/task/1/target/capture` → triggers `/target_localizer/capture_target` ROS service.
5. **System processes** (takes ~1–2s):
   - HSV circle detection finds the coloured circle (or crosshair fallback if not detected)
   - Depth back-projection computes 3D world position
   - Building face classifier determines which wall face
   - Description generator produces natural-language text
   - Annotated image saved; target appended to list
6. **Operator reviews** the description shown in the Mission Planner result box:
   - Verify **colour** is correct (×50% penalty if wrong)
   - Verify **face name** matches visual observation
   - Verify **description reads unambiguously** in 3D space
7. **If target is a duplicate**: System auto-detects and skips (within 0.5m of existing target).
8. **Rotate/adjust position** and capture next target.

**Repeat** for each visible target. Order does not matter — targets are labeled A, B, C... in order of first detection.

### 5.7 Return & Landing

1. Fly back toward flight line.
2. **"NOMAD 406K to base, request landing."** → Wait for clearance.
3. **"Cleared to land, NOMAD 406K."** → Acknowledge and land.
4. **"NOMAD 406K landed."**

### 5.8 Post-Landing — Submit Targets (CRITICAL — must complete before window closes)

1. **Save targets to file**:
   - Click **"Save Targets"** in Mission Planner Task 1 view, or
   - Call `POST /api/task/1/target/save` via Edge Core API, or
   - SSH to Jetson:
     ```bash
     ros2 service call /target_localizer/save_targets std_srvs/srv/Trigger
     ```
   - This writes `/home/mad/targets/Task_1_MAD_targets.txt`

2. **Review the .txt file** before uploading:
   ```bash
   cat /home/mad/targets/Task_1_MAD_targets.txt
   ```
   Expected format:
   ```
   Target A: Blue target on the north face of the building, 3.2m above ground, 1.8m from the NW corner.

   Target B: Red target on the west face of the building, 1.5m above ground, 0.9m to the left of the door when facing the building from outside.

   Target C: Green target on the ground near the south face of the building, 2.1m from the SE corner.
   ```

3. **Fix any descriptions** that look wrong:
   - In Mission Planner → Task 1 → **Submit** tab: edit description text directly in the grid.
   - Or manually edit the .txt file on the Jetson via SSH.
   - **Decimetre precision check**: Ensure ALL distances are expressed to 0.1m only (one decimal place). No "1.37m" — must be "1.4m". No integers alone like "2m" — must be "2.0m". This is a hard CONOPS rule (§5.2.3.6b).
   - **No GPS coordinates**: Ensure no lat/lon or numerical coordinate system appears anywhere in the file. Only landmark-based relative descriptions are allowed.

4. **Upload to Google Drive**:
   - In Mission Planner → Task 1 → **Submit** tab → **"Upload to Google Drive"** button.
   - This uploads `Task_1_MAD_targets.txt` + any selected target images.
   - Alternative: manually upload the file from `/home/mad/targets/` to the team Google Drive folder.

5. **Confirm upload** completed before the flight window closes!

---

## 6. RTM SOPs Cheat Sheet (PRINT THIS)

> **Callsign**: `NOMAD 406K` (register with Big City before flight)

### 6.1 Startup

```
[After receiving radio]
YOU: "NOMAD 406K to base, radio check."
[Waits for ATC: "NOMAD 406K, four by five" (or similar)]
```

### 6.2 Takeoff

```
YOU: "NOMAD 406K to base, request takeoff for firefighting mission."
ATC: "NOMAD 406K, you are cleared for takeoff."
YOU: "Cleared to takeoff, NOMAD 406K."
[Take off]
YOU: "NOMAD 406K takeoff complete."
```

### 6.3 UAM Corridor

```
[When entering 20-35m AGL corridor]
YOU: "NOMAD 406K entering corridor."
[MUST stay in corridor ≥30 seconds]

[When departing corridor]
YOU: "NOMAD 406K has left the corridor."
```

### 6.4 Approaching Building

```
[When entering 15m search volume]
YOU: "NOMAD 406K operating near the building."
```

### 6.5 Landing

```
YOU: "NOMAD 406K to base, request landing."
ATC: "NOMAD 406K, you are clear for landing."
YOU: "Cleared to land, NOMAD 406K."
[Land]
YOU: "NOMAD 406K landed."
```

### 6.6 Emergencies

**Airspace deconfliction**:
```
ATC: "Base to NOMAD 406K, traffic inbound from heading XXX, altitude XXm; correct course."
YOU: "Correcting course, NOMAD 406K."
[Fly ±90° from TRAFFIC_HDG for ≥10m]
YOU: "Flying heading XXX, NOMAD 406K."
[Adjust altitude ≥15m from TRAFFIC_ALT]
YOU: "Climbing/descending to XXm, NOMAD 406K."
[Wait for: "Base to NOMAD 406K, traffic avoided."]
```

**Yield to medevac**:
```
ATC: "Base to NOMAD 406K, priority flight inbound, fly XXm heading/direction XXX, climb to 50m."
YOU: "Confirm fly XXm heading/direction XXX, climb to 50m, NOMAD 406K."
[Execute]
YOU: "NOMAD 406K to base, yielded."
[Hold until: "Base to NOMAD 406K, continue mission."]
```

**Ground all**:
```
ATC: "Base to all, land immediately."
[LAND IMMEDIATELY]
YOU: "NOMAD 406K to ATC, landed."
[Wait until: "Base to all, resume flights."]
```

### 6.7 Callsign Abbreviation Rule

- First call in a new exchange: use **full callsign** (`NOMAD 406K`).
- Subsequent calls within the same continuous back-and-forth: may use **abbreviated** (`406K`).
- After exchange ends: resume **full callsign** at start of next exchange.

---

## 7. Fallback Procedures

### 7.1 Target Localizer Node Fails

If the automated pipeline fails during the flight window:

1. **Capture images manually** via Mission Planner's ZED snapshot or RTSP screenshot.
2. **Note GPS position and heading** for each target (displayed in Mission Planner HUD).
3. **Write descriptions by hand** using the building dimensions and observation:
   ```
   Target A: Red target on the east face of the building, 2.5m above ground, 1.0m from the southeast corner.
   ```
4. **Create `Task_1_MAD_targets.txt`** manually:
   ```bash
   nano /home/mad/targets/Task_1_MAD_targets.txt
   ```
5. Upload to Google Drive.

The format is simple enough to do by hand in 5 minutes. The annotated detection images and debug log in `/home/mad/targets/` provide raw 3D coordinates to reference.

### 7.2 No GPS Fix (No RTK)

The target_localizer falls back to **local pose** (ZED odometry or MAVROS local_position) for face selection and coordinate conversion. Descriptions will still be generated but may use "local pose" as the reference frame instead of geodetic GPS.

### 7.3 ZED Camera Not Available

If the ZED is not publishing topics:
1. Check topic list: `ros2 topic list | grep zed`
2. Check camera in container: `docker exec nomad_isaac_ros ls /dev/video*`
3. Try USB rebind: `start_nomad_full.sh` includes camera retry logic
4. If unrecoverable: use a handheld camera for target photos and write descriptions manually.

### 7.4 Edge Core API Unresponsive

```bash
ssh mad@100.85.121.98
sudo systemctl restart nomad
```

The target_localizer ROS services are independent of Edge Core — they continue working even if the API is restarted.

---

## 8. Score Maximization Strategy

### 8.1 Target Accuracy (25 pts — most technically demanding)

- **Hover at 3–5m distance** from each target (ZED depth is most accurate at close range).
- **Face the target directly** — the face classifier works best when the drone heading opposes the wall normal.
- **Review colour on GCS immediately** — if the detected colour looks wrong on the annotated image, note the true colour and edit the description before submission.
- **Back-propagation accuracy**: At 5m range, ZED 2i depth error ≈1% → 5cm horizontal error → well within 0.5m ×100% tier.
- **Decimetre rounding risk**: Since CONOPS mandates 0.1m precision only, rounding can shift a near-boundary description across a tier cutoff. Example: true distance 0.46m → described as 0.5m; if judge measures 0.51m, you drop to ×75%. **Aim for <0.4m true positional error** so rounding can't push you over 0.5m.
- **Ambiguity kills**: An ambiguous height (e.g., "on the west face, 0.7m right of the door" without height) gets 0 pts. Always ensure the description fixes the position in 3D.

### 8.2 Equipment Delivery (20 pts)

- **Carry all 3 items** (ladder + O₂ tank + radio = 20 pts max).
- **Land on/near each pad** — accuracy bonus for ≤2m.
- **Assign payloads intelligently**: Highest-value item (ladder, 10pts) to closest/easiest pad.

### 8.3 Distance / Laps (30 pts — highest single criterion)

- **Fly at least 1 lap** (0 laps = 0 pts for distance AND you're excluded from the linear scale).
- **More laps = more distance points**, but costs time. Balance with target detection time.
- **Speed matters** for tie-breaking: fastest lap time wins ties.
- **Consider: 2 laps is often the sweet spot** — demonstrates capability while leaving time for targets.

### 8.4 Payload Fraction (20 pts)

- **Carry the ladder** (3kg). On our ~8.5kg airframe, PF ≈ 0.35 → full marks.
- **Lift all payloads off ground** before flying — UAV must lift off with everything attached.

### 8.5 RTM SOPs (15 pts)

- **One error drops to 10/15, two errors to 0/15.** This is easy points if you follow the script exactly.
- **PRINT the cheat sheet** (§6 above). Practice it in a dirt-dive rehearsal.
- **Assign a dedicated radio operator** who is NOT the pilot.

### 8.6 Scoring Priority

1. **RTM SOPs** (15 pts — almost free if you follow procedure)
2. **Safe landing** (5 pts — standard operation)
3. **Payload fraction** (20 pts — design decision, carry the ladder)
4. **Laps** (30 pts — fly ≥1 lap, more if time allows)
5. **Equipment delivery** (20 pts — fly to pads, land, release)
6. **Target detection** (25 pts — technically hardest, best ROI for our automated system)

---

## 9. Mission Planner Task 1 UI Guide

### Capture Tab
- **GPS Status**: Shows fix type (3D/DGPS/RTK), satellite count, position.
- **Payload Controls**: Drop Payload 1/2, Shoot Water, Nozzle Angle slider.
- **CAPTURE PHOTO WITH METADATA**: Triggers target detection + description. Results show description text. Gallery shows thumbnails of captured images.
- **Gallery**: Click thumbnail for context menu (Open Image / View AI Description). AI description auto-generates if enabled in config.

### Submit Tab
- **Target Grid**: Editable table with columns: #, Colour (dropdown), Description, Image.
- **"+ Add Target"** / **"- Remove"**: Manually add/delete targets.
- **"Preview TXT"**: Shows the exact content that will be uploaded as `Task_1_MAD_targets.txt`.
- **"Upload to Google Drive"**: Uploads the .txt file + target images.
- **"Load From Captures"**: Auto-populates the grid from local capture metadata.

### Configuration Tab
- **Building Location**: Enter lat/lon manually or use "Use Current" button (from UAV GPS position).

---

## 10. API Endpoints Reference (Task 1)

| Method | Endpoint | Purpose |
|--------|----------|---------|
| `POST` | `/api/task/1/target/capture` | Trigger target detection + description. Returns structured metadata with image path, GPS, heading. |
| `POST` | `/api/task/1/target/save` | Save all targets to `Task_1_MAD_targets.txt`. |
| `GET` | `/api/task/1/target/model` | Print building model summary (faces, corners, landmarks). |
| `POST` | `/api/task/1/capture` | Legacy capture endpoint (fallback to detections cache if target_localizer unavailable). |
| `GET` | `/api/task/1/captures` | List all capture folders (timestamp dirs). |
| `GET` | `/api/task/1/images/{folder}/{filename}` | Download captured image/metadata from specific capture folder. |
| `POST` | `/api/servo/camera/tilt?angle={0-180}` | Set camera servo angle (for aiming at targets). |
| `POST` | `/api/servo/shooter/trigger?duration_ms={ms}` | Trigger water pump (Task 1 not needed, but available). |
| `GET` | `/api/task/1/images/{filename}` | Legacy image download (backward compat). |

---

*Last updated: 2026-04-26*
*NOMAD — McGill Aerial Design, AEAC 2026*
