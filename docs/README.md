# NOMAD Documentation Structure

## Overview

This directory contains comprehensive documentation for the NOMAD drone system organized into logical categories.

---

## Directory Structure

### `/features/`
User-facing feature documentation with deployment guides and usage instructions.

- [task1_photo_capture.md](features/task1_photo_capture.md) - Task 1 photo capture with metadata and EXIF embedding
- [task1_ai_processing.md](features/task1_ai_processing.md) - AI-powered scene description (Gemini/Ollama)

### `/implementation/`
Technical implementation details for developers.

- [mcp_setup.md](implementation/mcp_setup.md) - MCP server configuration and setup

### `/planning/`
Design documents and refactoring plans.

- [dependency_injection.md](planning/dependency_injection.md) - Dependency injection refactoring notes

### `/analysis/`
Codebase analysis reports and quality assessments.

- [codebase_analysis.md](analysis/codebase_analysis.md) - MCP-based codebase analysis report

### `/` (Root)
Architecture and system-level documentation.

- [architecture.md](architecture.md) - Overall system architecture
- [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) - Jetson deployment guide
- [JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md) - Navigation stack architecture
- [ISAAC_ROS_ZED_SETUP.md](ISAAC_ROS_ZED_SETUP.md) - Isaac ROS + ZED camera setup guide
- [COMPETITION_SETUP.md](COMPETITION_SETUP.md) - Competition day procedures
- [MULTI_STREAM_VIDEO.md](MULTI_STREAM_VIDEO.md) - Video streaming configuration
- [NAV2_INTEGRATION_PLAN.md](NAV2_INTEGRATION_PLAN.md) - Nav2 integration planning
- [NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md) - 3D mapping visualization
- [DUAL_LINK_FAILOVER.md](DUAL_LINK_FAILOVER.md) - Redundant telemetry setup
- [SERVO_CONTROL.md](SERVO_CONTROL.md) - Camera tilt servo and water shooter control
- [PRD.md](PRD.md) - Product requirements document

---

## Quick Reference by Topic

### Getting Started
1. [../README.md](../README.md) - Project overview
2. [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) - Deploy to Jetson
3. [COMPETITION_SETUP.md](COMPETITION_SETUP.md) - Competition checklist

### Task 1 (Outdoor Recon)
- [features/task1_photo_capture.md](features/task1_photo_capture.md) - Photo capture system
- [features/task1_ai_processing.md](features/task1_ai_processing.md) - AI scene description

### Task 2 (Indoor Autonomous)
- [JETSON_NAV_ARCHITECTURE.md](JETSON_NAV_ARCHITECTURE.md) - Navigation architecture
- [NAV2_INTEGRATION_PLAN.md](NAV2_INTEGRATION_PLAN.md) - Nav2 integration
- [NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md) - 3D mapping

### System Architecture
- [architecture.md](architecture.md) - System design
- [DUAL_LINK_FAILOVER.md](DUAL_LINK_FAILOVER.md) - Telemetry redundancy
- [MULTI_STREAM_VIDEO.md](MULTI_STREAM_VIDEO.md) - Video streaming

### Development
- [implementation/mcp_setup.md](implementation/mcp_setup.md) - MCP servers
- [planning/dependency_injection.md](planning/dependency_injection.md) - Refactoring plans
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
