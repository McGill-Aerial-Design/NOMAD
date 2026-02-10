# Architecture Outline

## Domains
- Transport (A): tailscale, link bonding/failover via 4G/WiFi + ELRS.
- Edge Core (B): FastAPI orchestrator, watchdog/time-sync, Task 1 recon logic, Task 2 VIO + CV + gimbal control.
- Mission Planner (C): Centralized control plugin with RTSP video viewer, ELRS tunneling, indoor nudge, telemetry injection, task controls.

## Process Separation
- `edge_core/orchestrator`: API + state management; watchdog for vision/VIO.
- `edge_core/vision`: ZED VIO + YOLO + gimbal PID + exclusion map.
- `transport`: mavlink routing; keep FC-facing ports stable.

## Data Flow (high-level)
- FC UART → mavlink-router → Orchestrator + Vision + Mission Planner (UDP/ELRS).
- Vision outputs detections + VIO → Orchestrator decisions → FC setpoints/gimbal commands.
- Mission Planner plugin provides centralized control, receives RTSP video streams, and sends task triggers via HTTP API.

Keep each module small: one responsibility per package, tests in `tests/` mirroring package paths.

## Performance Considerations

### Video Latency Chain

Current video pipeline:
```
ZED -> Isaac ROS -> ROS Topic -> Python Bridge -> TCP Socket -> FFmpeg Encode -> MediaMTX RTSP -> Network -> Mission Planner (LibVLC)
```

**Risk**: Each hop adds latency. Achieving glass-to-glass latency under 200ms is challenging with this many serialization/deserialization steps.

**Mitigation**: If latency becomes critical, consider a direct GStreamer pipeline from the ZED wrapper to RTSP, bypassing the Python Bridge/TCP hop. This trades custom overlay capability for lower latency.

### Resource Contention on Jetson

The Jetson Orin Nano runs multiple concurrent workloads:
- Docker (Isaac ROS VSLAM + Nvblox + YOLO)
- Python Edge Core (FastAPI + MAVLink handling)
- FFmpeg / GStreamer (Software H.264 encoding -- Orin Nano lacks NVENC)
- MAVLink Router

**Risk**: Thermal throttling. If the GPU is maxed out by Isaac ROS, software video encoding may increase CPU load, or the CPU may throttle, affecting the Python orchestrator.

**Mitigation**: The `health_monitor.py` monitors thermals and can trigger alerts. Consider GPU workload scheduling or reducing Isaac ROS update rates under thermal pressure.
