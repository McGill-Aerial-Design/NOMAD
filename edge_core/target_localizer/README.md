# Target Localizer - AEAC 2026 Task 1

Automated target detection, 3D localization, and natural-language description
generation for the AEAC 2026 Fire Reconnaissance task.

## What It Does

One button press while the drone faces a colored circle target:

1. **HSV detection** finds the circle and classifies its color (black, white, red, yellow, blue, green)
2. **ZED 2i depth** + servo tilt angle + drone GPS/heading back-projects the detection to 3D world coordinates
3. **Building model** determines which face the target is on and its position relative to corners and landmarks
4. **Template engine** generates a ConOps-compliant natural language description
5. **Deduplication** ensures the same target seen from multiple angles is only recorded once

Meanwhile, **YOLO landmark detection** runs in the background during the survey orbit, automatically populating the building model with doors and windows so descriptions reference real landmarks instead of just corners.

## Architecture

```
ZED 2i RGB ──────> HSV Circle Detector ──> 3D Back-projection ──> Building Model ──> Description
ZED 2i Depth ─────────────────────────────┘                        ↑
Servo Angle ──────────────────────────────┘                        │
Drone GPS + Heading ──────────────────────┘                        │
                                                                   │
ZED 2i RGB ──────> YOLO Landmark Detector ──> Auto-register ───────┘
                   (background, 2 Hz)          doors, windows
```

## Output Format

Matches ConOps Appendix D exactly:

```
Target 1: Blue target on the north face of the building, 3.2m above ground,
1.8m from the northwest corner.

Target 2: Red target on the west face of the building, 1.5m above ground,
0.9m to the left of the door when facing the building from outside.

Target 3: Green target on the ground near the south face of the building,
2.1m from the southeast corner.
```

## Quick Start

### Build
```bash
cd ~/catkin_ws/src  # or your ROS 2 workspace
cp -r target_localizer .
cd ..
colcon build --packages-select target_localizer
source install/setup.bash
```

### Launch (update building params Thursday night)
```bash
ros2 launch target_localizer target_localizer.launch.py \
    building_lat:=45.3220 \
    building_lon:=-75.7600 \
    building_length:=12.0 \
    building_width:=8.0 \
    building_height:=5.0 \
    building_orientation:=45.0 \
    yolo_model:=/home/mad/models/landmarks.onnx
```

### During Flight

Capture a target (bind this to a joystick button or MissionPlanner plugin):
```bash
ros2 service call /target_localizer/capture_target std_srvs/srv/Trigger
```

Check the building model state:
```bash
ros2 service call /target_localizer/print_model std_srvs/srv/Trigger
```

Save the final .txt file after landing:
```bash
ros2 service call /target_localizer/save_targets std_srvs/srv/Trigger
```

Then upload `Task_1_MAD_targets.txt` from the output directory to Google Drive.

## HSV Calibration

Before flight, calibrate color ranges for competition lighting:

```bash
# From a sample image
python3 tools/hsv_tuner.py --image target_photo.jpg

# Or live from a webcam
python3 tools/hsv_tuner.py --camera 0
```

Update the `HSV_RANGES` dict in `target_localizer/detectors.py` with the tuned values.

## Building Orientation

The `building_orientation_deg` parameter is the heading of the building's long axis,
measured clockwise from north:

- `0` = long axis points north-south
- `90` = long axis points east-west
- `45` = long axis points northeast-southwest

Determine this from Google Maps satellite view of Area XO or measure with a compass on-site.

## Files

```
target_localizer/
├── target_localizer/
│   ├── __init__.py
│   ├── building_model.py       # Building geometry, faces, landmarks, projections
│   ├── detectors.py            # HSV circle detector + YOLO landmark detector
│   └── target_localizer_node.py # Main ROS 2 node
├── launch/
│   └── target_localizer.launch.py
├── config/
│   └── competition.yaml        # Competition params + operational checklist
├── tools/
│   └── hsv_tuner.py            # Interactive HSV calibration tool
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

- ROS 2 Humble
- OpenCV (python3-opencv)
- NumPy
- cv_bridge
- MAVROS (for drone pose data)
- ZED ROS 2 Wrapper (for camera data)
- Optional: YOLO ONNX model for landmark detection

## Notes

- The node works without a YOLO model. It falls back to corner-only descriptions.
  Landmarks just make descriptions more precise and natural.
- The debug output file includes raw 3D coordinates for post-flight analysis.
- Annotated detection images are saved for each capture, useful for verifying
  color classification and reviewing before submission.
- If the automated pipeline fails at competition, use the debug images and
  coordinates to manually write descriptions. The format is simple enough
  to do by hand in 5 minutes.
