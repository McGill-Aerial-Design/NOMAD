# NOMAD Documentation

## Directory Structure

### `/features/`
- [task1_photo_capture.md](features/task1_photo_capture.md) - Task 1 photo capture with metadata and EXIF embedding

### Root docs
- [architecture.md](architecture.md) - Overall system architecture
- [OPERATIONS_RUNBOOK.md](OPERATIONS_RUNBOOK.md) - Operations runbook (setup, competition day, troubleshooting)
- [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) - Jetson deployment guide
- [JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md) - Navigation stack architecture
- [ISAAC_ROS_ZED_SETUP.md](ISAAC_ROS_ZED_SETUP.md) - Isaac ROS + ZED camera setup guide
- [VIDEO_STREAMING.md](VIDEO_STREAMING.md) - Video streaming system
- [NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md) - 3D mapping visualization
- [DUAL_LINK_FAILOVER.md](DUAL_LINK_FAILOVER.md) - Redundant telemetry setup
- [SERVO_CONTROL.md](SERVO_CONTROL.md) - Camera tilt servo and water shooter control
- [TASK1_COMPETITION_GUIDE.md](TASK1_COMPETITION_GUIDE.md) - Task 1 competition-day operations, checklists, CONOPS compliance

---

## Quick Reference by Topic

### Getting Started
1. [../README.md](../README.md) - Project overview
2. [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) - Deploy to Jetson
3. [OPERATIONS_RUNBOOK.md](OPERATIONS_RUNBOOK.md) - Operations and competition day

### Task 1 (Outdoor Recon)
- [TASK1_COMPETITION_GUIDE.md](TASK1_COMPETITION_GUIDE.md) - **Competition guide** — operations, checklists, CONOPS compliance, scoring strategy
- [features/task1_photo_capture.md](features/task1_photo_capture.md) - Photo capture system
- [../edge_core/target_localizer/README.md](../edge_core/target_localizer/README.md) - Target localizer ROS 2 node (detection, 3D positioning, description generation)

### Task 2 (Indoor Autonomous)
- [JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md) - Navigation architecture
- [NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md) - 3D mapping

### System Architecture
- [architecture.md](architecture.md) - System design
- [DUAL_LINK_FAILOVER.md](DUAL_LINK_FAILOVER.md) - Telemetry redundancy
- [VIDEO_STREAMING.md](VIDEO_STREAMING.md) - Video streaming

---

## Configuration File Reference

Key configuration files and their purposes:

| Config File | Purpose | Task | Details |
|-------------|---------|------|----------|
| `config/nvblox_performance.yaml` | Unified nvblox (all tasks) | Task 1 + Task 2 | 10cm voxels, 10Hz updates, 3m local map, dynamic mapping with 3D ESDF |
| `config/profiles/task1_outdoor.params` | Task 1 operational config | Task 1 | GPS geofencing, outdoor navigation params |
| `config/profiles/task2_indoor.params` | Task 2 operational config | Task 2 | Indoor nav2 constraints, obstacle avoidance safety limits |
| `config/nav2_drone.yaml` | Nav2 stack configuration | Task 2 | Navigation controller, planner, recovery behaviors |
| `config/video_streams.json` | MediaMTX RTSP streams | All | ZED camera RTSP, mask stream URLs for Mission Planner |
| `config/nomad.env` | Environment variables (single source of truth) | All | Tailscale IPs, home paths, port configuration, autostart flags |


---

## Documentation Standards

### File Naming
- Use lowercase with underscores: `task1_photo_capture.md`
- Be descriptive but concise
- Avoid redundant prefixes (no "IMPLEMENTATION_", "TASK1_", etc.)

### Structure
Each feature document should include:
1. Overview with status
2. Architecture/data flow
3. Implementation details
4. Deployment instructions
5. Configuration options
6. Testing procedures
7. Troubleshooting
8. Related documentation links

### Updates
- Keep "Last Updated" date current
- Mark status clearly (Production Ready, Beta, Planned)
- Update cross-references when moving files

---

## Contributing

When adding new documentation:
1. Choose appropriate category folder
2. Follow naming conventions
3. Use consistent structure
4. Add entry to this README
5. Update cross-references in related docs

---

*NOMAD - McGill Aerial Design AEAC 2026*
