# Task 1 Photo Capture with Metadata

**Feature Category**: Task 1 - Outdoor Reconnaissance  
**Status**: Production Ready  
**Last Updated**: February 2, 2026

## Overview

Comprehensive photo capture system for Task 1 outdoor reconnaissance, featuring:
- GPS, AHRS, and gimbal metadata capture
- EXIF metadata embedding in JPEG files
- Target-localizer ROS 2 capture flow for current competition logic
- Mission Planner UI integration

---

## Architecture

### Data Flow
```
Pilot Trigger → Mission Planner → Edge Core API → target_localizer ROS 2 node
                    ↓
          Capture target + build description + save target file
                    ↓
        Mission Planner displays ROS2 result and local metadata
```

### Folder Structure
```
data/task1_captures/
├── 20260202_120000/
│   ├── photo.jpg              (ZED camera capture with EXIF)
│   └── metadata.json          (comprehensive telemetry)
├── 20260202_120130/
│   ├── photo.jpg
│   └── metadata.json
└── 20260202_120245/
    ├── photo.jpg
    └── metadata.json
```

---

## Implementation Details

### Edge Core (Python)

#### API Endpoint
```
POST /api/task/1/target/capture
```

**Response**:
```json
{
  "success": true,
  "output": "Added 2 target(s):\nRed target ..."
}
```

The legacy `/api/task/1/capture` alias is still available for compatibility, but the Mission Planner Task 1 view now uses the target-localizer endpoint directly.

#### EXIF Tags Embedded
- GPS Latitude/Longitude (WGS84 DMS format)
- GPS Altitude (meters)
- DateTime (YYYY:MM:DD HH:MM:SS)
- ImageDescription (AHRS data: heading, pitch, roll)

#### Dependencies Added
```python
piexif>=1.1.3  # EXIF metadata embedding
```

#### Key Functions
- `_gps_to_exif()` - Converts decimal GPS to EXIF DMS format
- `task1_capture()` - Main capture endpoint handler

#### Files Modified
- `edge_core/requirements.txt` - Added piexif
- `edge_core/api.py` (lines 98-656) - EXIF helper + enhanced endpoint
- `config/env/jetson.env.example` - Building location config

---

### Mission Planner (C#)

#### Button Enhancement
- Label: "CAPTURE PHOTO WITH METADATA"
- Enlarged size for prominence
- Quick access also in Control Panel
- Uses the target-localizer ROS 2 capture response when the new Task 1 pipeline is active

#### Metadata Display
8-line comprehensive format:
```
[OK] Capture Successful
Time: 2026-02-02T12:00:00.123456Z
Position: 45.123456, -75.654321 @ 100.5m
AHRS: Hdg=45.0 Pitch=-5.2 Roll=2.1
Gimbal: Pitch=-45.0 Yaw=0.0
Building: Competition Building A
Folder: data/task1_captures/20260202_120000
Image: photo.jpg
```

When the backend returns the newer target-localizer response, Mission Planner shows the ROS2 output directly instead of trying to reconstruct the old image download path.

#### Gallery Enhancement
- Thumbnail view of captured photos
- Hover tooltip with metadata
- Click to view full-size
- Gallery auto-refreshes after capture

#### Files Modified
- `mission_planner/src/NOMADTask1View.cs` - Task 1 capture view
- `mission_planner/src/ServiceControlPanel.cs` - Target localization controls

---

## Deployment

### Jetson Setup

```bash
# 1. SSH to Jetson
ssh mad@100.85.121.98
cd ~/NOMAD

# 2. Pull latest code
git pull origin main

# 3. Install EXIF library
pip3 install piexif>=1.1.3

# 4. Configure building location
nano config/env/jetson.env
# Add:
TASK1_BUILDING_NAME=Competition Building A
TASK1_BUILDING_LAT=45.123456
TASK1_BUILDING_LON=-75.654321

# 5. Restart Edge Core
sudo systemctl restart nomad
sudo systemctl status nomad
```

### Windows Ground Station

```powershell
# 1. Navigate to NOMAD directory
cd C:\Users\Youssef\Documents\Code\MAD\NOMAD

# 2. Build Mission Planner plugin
.\scripts\build\build_plugin_windows.ps1

# 3. Verify plugin installed
Get-ChildItem "$env:LOCALAPPDATA\Mission Planner\plugins\NOMADPlugin.dll"

# 4. Restart Mission Planner
```

---

## Testing

### Edge Core API Test
```bash
# Test capture endpoint
curl -X POST http://100.85.121.98:8000/api/task/1/capture

# Check folder created
ls -lh data/task1_captures/

# Verify EXIF embedded
sudo apt install libimage-exiftool-perl
exiftool data/task1_captures/*/photo.jpg
```

### Mission Planner UI Test
1. Open NOMAD → Task 1 View
2. Click "CAPTURE PHOTO WITH METADATA"
3. Verify either the 8-line metadata block or the ROS2 target-localizer output appears
4. Check thumbnail in gallery when the legacy image payload is returned
5. Hover for metadata tooltip
6. Click thumbnail for full view

---

## Configuration

### Building Location (Jetson)
Edit `config/env/jetson.env`:
```bash
TASK1_BUILDING_NAME=Competition Building A
TASK1_BUILDING_LAT=45.123456
TASK1_BUILDING_LON=-75.654321
```

### Camera Settings
ZED camera automatically selected via `SystemState.camera` interface.

---

## Metadata Schema

### metadata.json
```json
{
  "timestamp": "2026-02-02T12:00:00.123456Z",
  "gps": {
    "lat": 45.123456,
    "lon": -75.654321,
    "alt": 100.5
  },
  "ahrs": {
    "heading_deg": 45.0,
    "pitch_deg": -5.2,
    "roll_deg": 2.1
  },
  "gimbal": {
    "pitch_deg": -45.0,
    "yaw_deg": 0.0
  },
  "building_location": "Competition Building A (45.123456, -75.654321)",
  "photo_path": "data/task1_captures/20260202_120000/photo.jpg"
}
```

---

## Known Issues

None - feature is production-ready.

---

## Future Enhancements

- [ ] Panorama stitching from multiple captures
- [ ] QR code/barcode detection in images
- [ ] Automatic building detection from GPS proximity
- [ ] HDR image capture mode
- [ ] RAW format option for post-processing

---

## Related Documentation

- [Task 1 AI Processing](task1_ai_processing.md) - AI-powered scene description
- [../architecture.md](../architecture.md) - System architecture overview
- [../JETSON_DEPLOYMENT.md](../JETSON_DEPLOYMENT.md) - Jetson setup guide

---

*NOMAD - McGill Aerial Design AEAC 2026*
