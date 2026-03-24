# NOMAD Documentation

## Directory Structure

### `/features/`
- [task1_photo_capture.md](features/task1_photo_capture.md) - Task 1 photo capture with metadata and EXIF embedding
- [task1_ai_processing.md](features/task1_ai_processing.md) - AI-powered scene description (Gemini/Ollama)

### `/planning/`
- [dependency_injection.md](planning/dependency_injection.md) - Dependency injection refactoring notes
- [NAV2_INTEGRATION_PLAN.md](NAV2_INTEGRATION_PLAN.md) - Nav2 integration planning

### `/analysis/`
- [codebase_analysis.md](analysis/codebase_analysis.md) - MCP-based codebase analysis report

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

---

## Quick Reference by Topic

### Getting Started
1. [../README.md](../README.md) - Project overview
2. [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) - Deploy to Jetson
3. [OPERATIONS_RUNBOOK.md](OPERATIONS_RUNBOOK.md) - Operations and competition day

### Task 1 (Outdoor Recon)
- [features/task1_photo_capture.md](features/task1_photo_capture.md) - Photo capture system
- [features/task1_ai_processing.md](features/task1_ai_processing.md) - AI scene description

### Task 2 (Indoor Autonomous)
- [JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md) - Navigation architecture
- [NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md) - 3D mapping

### System Architecture
- [architecture.md](architecture.md) - System design
- [DUAL_LINK_FAILOVER.md](DUAL_LINK_FAILOVER.md) - Telemetry redundancy
- [VIDEO_STREAMING.md](VIDEO_STREAMING.md) - Video streaming

### Development
- [planning/dependency_injection.md](planning/dependency_injection.md) - Refactoring plans

---

## Configuration File Reference

Key configuration files and their purposes:

| Config File | Purpose | Task | Details |
|-------------|---------|------|----------|
| `config/nvblox_performance.yaml` | Primary nvblox (default) | Task 1 + visualization | 5cm voxels, 30Hz updates, 8m radius, static TSDF mapping |
| `config/nvblox_indoor.yaml` | Indoor-optimized nvblox | Task 2 indoor | 3cm voxels, 15Hz depth, 5m radius, dynamic occupancy for people |
| `config/profiles/task1_outdoor.params` | Task 1 operational config | Task 1 | GPS geofencing, outdoor navigation params |
| `config/profiles/task2_indoor.params` | Task 2 operational config | Task 2 | Indoor nav2 constraints, obstacle avoidance safety limits |
| `config/nav2_drone.yaml` | Nav2 stack configuration | Task 2 | Navigation controller, planner, recovery behaviors |
| `config/video_streams.json` | MediaMTX RTSP streams | All | ZED camera RTSP, mask stream URLs for Mission Planner |
| `config/env/jetson.env` | Environment variables | All | Tailscale IPs, home paths, port configuration |

### Switching nvblox Profiles

By default, `nvblox_performance.yaml` is active. To switch to indoor profile:

```bash
# Option 1: Manual copy inside container
docker exec nomad_isaac_ros bash -c \
  'cp /workspaces/isaac_ros-dev/config/nvblox_indoor.yaml \
      /opt/ros/humble/.../nvblox/nvblox_base.yaml' 
rosnode kill /nvblox_node  # Restart occurs automatically (~2-3s blind)

# Option 2: Edit start_isaac_ros_auto.sh before launch
# Change NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_indoor.yaml
```
- [analysis/codebase_analysis.md](analysis/codebase_analysis.md) - Code quality

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
