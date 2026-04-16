This document outlines the software architecture and requirements for **NOMAD** (**N**etworked **O**perations for **MAD**).

This specification is designed to be included in the **AEAC Phase 1 Design Paper** under the "System Integration" and "Mission Strategy" sections. It adheres to **Clean Architecture** principles, ensuring modularity, testability, and separation of concerns.

***

# Product Requirements Document (PRD)

| **Project Name** | **NOMAD** (Networked Operations for MAD) |
| :--- | :--- |
| **Team** | McGill Aerial Design (MAD) |
| **Competition** | AEAC Student Competition 2026 |
| **Platform** | Jetson Orin Nano / ZED 2i / ArduCopter Quadcopter |
| **Software Stack** | Python 3.13.9 (Free-Threading), C# .NET |
| **Version** | **3.0.0** |
| **Status** | **APPROVED FOR IMPLEMENTATION** |

---

## 1. Executive Summary

**NOMAD** is a distributed Command and Control (C2) architecture designed to bridge the "Air Gap" between the pilot and the intelligent edge. It utilizes a hybrid network strategy: a high-bandwidth 4G/LTE link for real-time video/API control (Task 2), and a low-latency ELRS dual-band link (2.4GHz + 900MHz Gemini) for safety-critical telemetry and control.

The system is designed for **two distinct task configurations**:

1.  **Task 1 (Outdoor Recon):** Traditional RC pilot control via ELRS directly to ArduPilot. Jetson is mounted for video streaming and imaging only. GPS/RTK positioning. No autonomous navigation.

2.  **Task 2 (Indoor Extinguish):** Jetson-centric autonomous navigation using Isaac ROS Nav2/Nvblox. ArduPilot operates in GUIDED mode as flight controller only. VIO positioning indoors. YOLO target detection. WASD controls available for human intervention over LTE if needed.

**Key Architecture Principle:** Task 1 uses traditional RC control. Task 2 uses Jetson for navigation with ArduPilot as a low-level flight controller.

---

## 2. System Architecture Scope

The system follows **Clean Architecture** principles, divided into three logical domains:

1.  **Domain A: Transport Layer (Connectivity)** - MAVLink routing, ELRS telemetry, and 4G/LTE communication.
2.  **Domain B: Edge Core** - The Onboard Orchestrator, VIO pipeline, and NavController running on Jetson Orin Nano. **Full autonomy stack deployed for Task 2 only. Task 1 uses Jetson for video/imaging only.**
3.  **Domain C: Mission Planner Integration** - A custom plugin providing telemetry display, WASD backup control (Task 2), and task interfaces.

---

## 3. Functional Requirements

### 3.1 Domain A: Transport Layer (Connectivity)

*Objective: Route MAVLink messages between Flight Controller and Ground Station.*

#### **A.1 Both Tasks: MAVLink + Tailscale**
*   **[NET-01] MAVLink Routing:**
    *   `mavlink-router` on Jetson routes telemetry between FC, Edge Core, and Ground Station.
    *   **Input:** USB/UART from Cube Orange.
    *   **Output 1:** UDP `127.0.0.1:14550` - Edge Core NavController
    *   **Output 2:** UDP `127.0.0.1:14551` - Vision Process
    *   **Output 3:** UDP `<Ground_IP>:14550` - Mission Planner (via Tailscale)

*   **[NET-02] Tailscale VPN:**
    *   4G/LTE USB modem provides Internet connectivity.
    *   Tailscale mesh VPN enables secure communication to Ground Station.
    *   Exposes: HTTP API (8000), MAVLink (14550), RTSP (8554), SSH (22).

*   **[NET-03] ELRS Fallback:**
    *   ELRS remains active as backup control link.
    *   If 4G/Tailscale fails, pilot can take manual RC control.

#### **A.2 Task 1 Specific: GPS/RTK**
*   **[T1-NET-01] RTCM Corrections:**
    *   RTK corrections delivered via Mission Planner MAVLink injection (`GPS_INJECT_DATA`).
    *   Operator must verify RTK Fixed (`fix_type=6`) before flight.

---

### 3.2 Domain B: Edge Core

*Objective: Jetson Orin Nano running Python 3.13.9 for video streaming (Task 1) and autonomous operations (Task 2).*

**NOTE:**
- **Task 1:** Jetson provides video streaming and imaging. Flight controlled by pilot via ELRS RC.
- **Task 2:** Jetson handles autonomous navigation via Nav2/Nvblox. ArduPilot operates in GUIDED mode. WASD controls available for LTE-based human intervention if needed.

#### **B.1 Orchestrator & State Management**
*   **[EDGE-ORCH-01] API Interface:** FastAPI service exposing endpoints for status, health, and Task 2 control.
*   **[EDGE-ORCH-02] Watchdog:** Monitor Vision/VIO process health. Auto-restart on hang without severing MAVLink connection.
*   **[EDGE-TIME-01] Time Sync:** NTP client (Chrony) for system time via 4G. Fallback to GPS time from MAVLink if 4G unavailable.

#### **B.2 VIO Integration (Task 2)**
*   **[T2-NAV-01] ZED Visual-Inertial Odometry:**
    *   Dedicated process interfacing with ZED SDK.
    *   Transform ZED pose to NED frame.
    *   Stream `VISION_POSITION_ESTIMATE` to Flight Controller at **30Hz**.
    *   Process pinned to high-priority CPU cores.

*   **[T2-NAV-02] EKF Source Configuration:**
    *   Indoor Source Set (SRC2): ExternalNav for position, velocity, and yaw.
    *   Magnetometer disabled indoors (`EK3_SRC2_YAW=6`).
    *   VIO health monitored via EKF_STATUS_REPORT variances/innovations.

*   **[T2-NAV-03] Jetson-Centric Navigation:**
    *   **ArduPilot operates as flight controller only** - no internal path planning.
    *   Isaac ROS Nav2/Nvblox generates velocity commands (`/cmd_vel`).
    *   Edge Core NavController forwards commands to ArduPilot GUIDED mode.
    *   Full obstacle avoidance via Nvblox 3D costmap.
    *   See `docs/JETSON_NAV_ARCHITECTURE.md` for detailed architecture.

#### **B.3 Target Detection & Engagement (Task 2)**
*   **[T2-CV-01] YOLO Detection:**
    *   YOLOv8 model detecting Purple (dry) and Blue/Green (wet) pH targets.
    *   Inference on Jetson GPU.

*   **[T2-FIRE-01] Firing Solution:**
    *   Trigger water pump only when:
        1. Target class is "Purple" (dry)
        2. Target NOT in Exclusion Map
        3. Aiming error < 5 degrees
        4. Range 1.5m - 2.5m

*   **[T2-MEM-01] Exclusion Map:**
    *   Log sprayed target 3D coordinates in VIO frame.
    *   Ignore future detections within 0.5m of stored coordinates.

*   **[T2-AIM-01] Gimbal Visual Servoing:**
    *   PID loop on camera feed to center target.
    *   Send `MOUNT_CONTROL` MAVLink messages for gimbal control.

#### **B.4 Safety & Failsafes (Task 2)**
*   **[T2-SAFE-01] VIO Failure Response:**
    *   If VIO unhealthy for >3 seconds (innovation/variance thresholds exceeded):
        1. Switch EKF source to fallback (Baro/IMU only)
        2. Set flight mode to `ALT_HOLD`
        3. Alert pilot for manual takeover
    *   **NOT:** Auto-land (may hit obstacles), RTL (no GPS indoors), motor kill (unsafe)

*   **[T2-SAFE-02] Thermal Protection:**
    *   If Jetson temp > 85°C: Throttle YOLO inference, never throttle VIO.

*   **[T2-SAFE-03] Brownout Protection:**
    *   If battery < critical threshold: Disable Vision Engine (GPU) to preserve flight power.

---

### 3.3 Domain C: Mission Planner Integration

*Objective: C# plugin for centralized control and telemetry display.*

#### **C.1 Common Features (Both Tasks)**
*   **[MP-PLUG-01] Telemetry Display:** Show flight data, GPS status, battery, etc.
*   **[MP-PLUG-02] ELRS Control:** Direct MAVLink via ELRS for failsafe control.

#### **C.2 Task 1 Features (With Jetson)**
*   **[MP-T1-01] RTK Status:** Display GPS fix type, RTK Fixed/Float indicator.
*   **[MP-T1-02] WASD Pilot Control:** Velocity commands via NavController to ArduPilot GUIDED mode.
*   **[MP-T1-03] Video Streaming:** RTSP streams from ZED camera via Tailscale.
*   **[MP-T1-04] Jetson Health:** CPU/GPU temp, network status monitoring.

#### **C.3 Task 2 Features (With Jetson)**
*   **[MP-T2-01] Jetson Health Tab:** CPU/GPU temp, VIO confidence, network status.
*   **[MP-T2-02] WASD Nudge Control:** `SET_POSITION_TARGET_LOCAL_NED` velocity commands for indoor repositioning.
*   **[MP-T2-03] Task 2 Controls:** Start/stop autonomous mode, exclusion map reset.
*   **[MP-T2-04] Video Streaming:** RTSP streams from ZED camera via Tailscale.
*   **[MP-T2-05] Network Monitor:** Tailscale status, 4G signal strength, latency.

---

## 4. Hardware Configuration

### 4.1 Task 1 Configuration (Outdoor GPS)
```
Drone (No Jetson):
├── Cube Orange Flight Controller
├── GPS with RTK capability
├── ELRS 900MHz + 2.4GHz Dual Receiver
└── Standard FPV camera (optional)

Ground Station:
├── Mission Planner
├── ELRS Gemini TX (2.4GHz + 900MHz)
├── NTRIP client for RTCM corrections
└── USB connection to ELRS TX
```

### 4.2 Task 2 Configuration (Indoor VIO)
```
Drone (With Jetson):
├── Cube Orange Flight Controller
├── Jetson Orin Nano
├── ZED 2i Stereo Camera
├── 4G/LTE USB Modem
├── ELRS Dual Receiver (backup)
├── Gimbal + Water Pump
└── NVMe storage

Ground Station:
├── Mission Planner + NOMAD Plugin
├── ELRS Gemini TX
├── Tailscale VPN client
└── 4G/WiFi connectivity
```

---

## 5. ELRS Configuration

### 5.1 Preflight (Parameter Work)
```
Packet Rate: 500 Hz
Telemetry Ratio: 1:2
Expected Param Download: ~30 seconds
```

### 5.2 Flight Mode
```
2.4 GHz: 500 Hz, 1:2 telemetry (primary)
900 MHz: 150 Hz, 1:8 telemetry (backup/range)
Gemini auto-switching enabled
```

---

## 6. Compliance Checklist

*   **[COMP-01] Evidence:** Mission logs downloadable via HTTP (Task 2) or SD card (Task 1).
*   **[COMP-02] Safety Failsafe:** VIO failure triggers ALT_HOLD, not auto-land or RTL.
*   **[COMP-03] Manual Override:** ELRS always provides pilot authority regardless of 4G status.

---

## 7. Development Architecture

```
TASK 1 (Outdoor - RC Pilot Control):
+---------------------------------------------------------------+
|  Ground Station                                                |
|  +-- Mission Planner (telemetry)                              |
|  +-- ELRS TX (RC control) ----> ELRS RX on drone              |
+---------------------------------------------------------------+
                              |
+---------------------------------------------------------------+
|  Drone                      |                                  |
|  +-- Cube Orange <----------+ (standard flight modes)         |
|  +-- Jetson (video streaming only via Tailscale)              |
+---------------------------------------------------------------+

TASK 2 (Indoor - Jetson Autonomous Navigation):
+---------------------------------------------------------------+
| Ground Station                                                 |
| +-- Mission Planner + Plugin                                   |
| +-- WASD backup control (intervention over LTE)               |
| +-- Tailscale VPN <--4G--> Jetson                             |
|                              |                                 |
| JETSON ORIN NANO             |                                 |
| +-- Isaac ROS (Docker)       |                                 |
| |   +-- ZED VSLAM (VIO)      |                                 |
| |   +-- Nvblox (3D Map)      |                                 |
| |   +-- Nav2 (Path Planning) --> /cmd_vel                     |
| |   +-- ros_http_bridge ------------+                          |
| |                                   |                          |
| +-- Edge Core (Python)              |                          |
|     +-- VIO Pipeline ---------------+--> VISION_POSITION_EST  |
|     +-- NavController <-------------+                          |
|     +-- MavlinkService -------------+--> SET_POSITION_TARGET   |
|     +-- FastAPI (HTTP)              |                          |
|                                     |                          |
+-- mavlink-router                    |                          |
|       |                             |                          |
+-------+-----------------------------|---------------------------+
        |                             |
        v                             v
  Cube Orange (GUIDED mode)    Ground Station
  +-- Attitude Control         +-- Status Display
  +-- Motor Mixing             +-- WASD Backup
  +-- Failsafe Logic           +-- Manual Override
```
