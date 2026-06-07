# NOMAD Mission Planner Plugin

**Comprehensive Control Interface for NOMAD Operations**

This C# plugin integrates directly with Mission Planner to provide full control of the NOMAD drone system including task execution, embedded video streaming, remote terminal access, MAVLink dual-link failover, and indoor manual control.

## Features

### Full-Screen Sidebar Interface
The primary interface features a modern sidebar navigation design:
- **Dashboard**: Quick overview of all critical systems at a glance
- **Video Feed**: Embedded RTSP streaming with configurable payload controls
- **Terminal**: Remote command execution on Jetson
- **System Health**: Real-time Jetson monitoring (CPU/GPU temps, memory, network)
- **Link Status**: Dual-link failover monitoring
- **Boundary**: Soft / hard geofence editing and live monitoring
- **Settings**: Plugin configuration

### Pop-Out Window Support
- The NOMAD main screen can be popped out into a separate window
- Ideal for multi-monitor setups
- Access via **NOMAD -> Pop Out NOMAD Window** menu

### MAVLink Dual Link Failover
- **Primary Link**: LTE/Tailscale via Jetson mavlink-router
- **Secondary Link**: RadioMaster transmitter (UDP 14550 or COM port)
- **Merged Link**: NOMAD plugin local router on UDP 14600 for Mission Planner
- **Automatic Failover**: Switches links on connection loss
- **Health Monitoring**: Real-time latency and packet loss tracking
- **Manual Override**: Switch links manually via Link Status panel
- **Auto-Reconnect**: Returns to preferred link when available

### RadioMaster Connection Options
- **UDP Mode**: Listens on port 14550 for MAVLink packets
- **COM Port Mode**: Direct serial connection to RadioMaster receiver
  - Windows: COM3, COM4, etc.
  - Linux: /dev/ttyUSB0, /dev/ttyACM0, etc.
- Automatic detection and switching between modes

### Configurable Payload Controls
The Video view hosts a payload panel backed by Cube Orange servo / relay outputs.
Payloads are fully configurable (channels, PWM limits, live servo sliders, relay
pump) — see [Configuration](#configuration).

### Core Functionality
- **Embedded Video**: Built-in RTSP player with low-latency streaming (LibVLC)
- **Telemetry Display**: Real-time position and connection status
- **Dual-Link Communication**: HTTP API over Tailscale or MAVLink via ELRS for redundancy

## Project layout

`src/` is organized by responsibility. Namespaces are flat
(`NOMAD.MissionPlanner`, plus `NOMAD.MissionPlanner.Core`), so a file's folder is
independent of its namespace. The build file is `src/NOMADPlugin.csproj` — an
old-style .NET Framework 4.8 project that lists every source file explicitly, so
**adding or moving a file means updating its `<Compile Include>` entry**.

| Folder | Contents |
|--------|----------|
| `src/Plugin/` | `NOMADPlugin` (the `MissionPlanner.Plugin.Plugin` entry point) and `NOMADMainScreen` (full-page sidebar shell + pop-out window) |
| `src/Config/` | `NOMADConfig` (settings persistence) and `NOMADSettingsForm` (settings dialog) |
| `src/UI/` | Shared UI scaffolding: `NOMADViewBase` (base view), `NOMADTheme` (theming), `UiAsync` (UI-thread helpers) |
| `src/Views/` | Top-level page views: Dashboard, Health, Links, Terminal, Video, Boundary, plus `SLAM3DView`, `Rviz2View`, `ZedCalibrationView` |
| `src/Panels/` | Embeddable control panels: service control, link health, health dashboard, EKF source, Jetson health / terminal |
| `src/Connectivity/` | Links + transport: MAVLink / Jetson connection managers, `JetsonApiService`, `JetsonStateStream`, `DualLinkSender`, `GroundLinkRouter` |
| `src/Control/` | Flight / gimbal control: flight mode, gimbal (+ joystick window), guided RTH landing, cube outputs |
| `src/Input/` | Joystick services: `NomadJoystickService`, `SerialJoystickBridge` |
| `src/Payload/` | `PayloadActions` and `PayloadControlPanel` (drop / water shooter / nozzle) |
| `src/Geofence/` | `BoundaryManager` (soft/hard boundary monitor), `MissionConfig` (boundary + mission models), `MapOverlayManager`, `MPFenceUploader` |
| `src/Media/` | `EmbeddedVideoPlayer` (LibVLC RTSP), `SnapshotManager`, `BuildingViewer3D` |
| `src/Notifications/` | `NotificationService`, `NotificationPanel` (toasts), `AudioAlerts` (chimes / TTS) |
| `src/Telemetry/` | `TelemetryInjector` (HUD / `STATUSTEXT` injection) |
| `src/Core/` | Module SDK: `INomadModule`, `INomadView`, `ModuleHost`, `NomadModuleContext`, `NomadViewDescriptor` (see [`src/Core/README.md`](src/Core/README.md)) |
| `src/SLAM3D/` | 3D nvblox mesh viewer — rendering, data, camera, models, network |
| `src/Modules/` | Example pluggable modules (see [`src/Modules/README.md`](src/Modules/README.md)) |

> This is the public baseline branch; some competition-specific views (e.g. the
> Task 1 / Task 2 tabs and Google Drive upload) are not included here. The full
> implementations live on the `AEAC2026` branch.

## Installation

### Prerequisites

1. **Mission Planner** - Download from [ardupilot.org](https://ardupilot.org/planner/docs/mission-planner-installation.html)
2. **Visual Studio 2022** - For building the plugin (Community edition is free)
3. **.NET Framework 4.7.2 or 4.8**
4. **(Optional) LibVLCSharp** - For enhanced embedded video playback

### Enabling Embedded Video (Windows)

If the Video tab opens an external VLC window instead of showing the stream inside Mission Planner, the embedded LibVLC player is not available. To enable embedded playback:

1. Add the `LibVLCSharp.WinForms` NuGet package to `src/NOMADPlugin.csproj` and restore packages.
2. Install VLC (matching system bitness) from https://www.videolan.org/vlc/ — this provides the native `libvlc.dll` and related files.
3. Rebuild the plugin (Release) and ensure the `LibVLCSharp.WinForms.dll` and native libvlc binaries are copied next to `NOMADPlugin.dll` in the Mission Planner plugins folder.
   - Alternatively, include the libvlc redistributable (DLLs and `plugins` folder) in the plugin directory.
4. Restart Mission Planner. The Video tab should now show an embedded player.

If embedded playback still fails, the plugin will fall back to opening VLC or FFplay externally; check the debug output for a message describing why LibVLC failed to initialize (missing native lib or assembly mismatch).

Tip: For automated deployments, package the `libvlc` redistributables with your plugin or document the matching VLC version to install on operator machines.

### Automated libVLC packaging

To make embedded playback easy for operators, the repository contains helper scripts that can fetch and package LibVLC and LibVLCSharp for Windows:

- `mission_planner/packaging/fetch-libvlc.ps1` — downloads the latest `VideoLAN.LibVLC.Windows` (native redistributables) and `LibVLCSharp`/`LibVLCSharp.WinForms` managed assemblies from NuGet, and places them into `packaging/libvlc-windows/` and `third_party/libvlc/` respectively.
- `mission_planner/packaging/copy-libvlc.ps1` — copies native libvlc DLLs and the `plugins/` folder into the build output `src/bin/Release`.
- `mission_planner/packaging/copy-managed-libs.ps1` — copies managed `LibVLCSharp*.dll` into the build output.

Usage:

1. Run the fetcher (one-time per developer machine):
   ```powershell
   .\mission_planner\packaging\fetch-libvlc.ps1 -Arch win-x64
   ```
2. Build using the included build script — it will automatically copy the managed and native files into the plugin output and deploy them to `%LOCALAPPDATA%\Mission Planner\plugins`:
```powershell
.\scripts\build\build_plugin_windows.ps1
```

This ensures the embedded player can initialize libVLC at runtime. If you prefer not to include the redistributables in the repository, operators can instead install VLC on their machines and the plugin will detect the native libs automatically.

### Building

**Quick build (recommended):** from the repo root, run the bundled task — it
locates MSBuild, builds Release, and deploys the DLL (plus LibVLC redistributables
if present) to the Mission Planner plugins folder:

```powershell
pixi run build-plugin
# equivalently: .\scripts\build\build_plugin_windows.ps1
```

**In Visual Studio:**

1. Open `src/NOMADPlugin.csproj` in Visual Studio

2. Restore NuGet packages:
   - Right-click solution → "Restore NuGet Packages"
   - Required: `Newtonsoft.Json`
   - Optional: `LibVLCSharp.WinForms` for embedded video

3. Add Mission Planner references:
   - Right-click "References" → "Add Reference"
   - Browse to your Mission Planner installation folder
   - Add: `MissionPlanner.exe`, `MissionPlanner.Comms.dll`, `MAVLink.dll`

4. Build the solution (Release mode recommended):
   ```
   Build → Build Solution (Ctrl+Shift+B)
   ```

5. Copy the output DLL to Mission Planner plugins folder:
   ```powershell
   Copy-Item "bin\Release\NOMADPlugin.dll" "$env:LOCALAPPDATA\Mission Planner\plugins\"
   ```
   Or: `C:\Program Files (x86)\Mission Planner\plugins\`

## Configuration

Settings stored in:
```
%LOCALAPPDATA%\Mission Planner\plugins\nomad_config.json
```

Example:
```json
{
  "JetsonIP": "192.168.1.100",
  "JetsonPort": 8000,
  "TailscaleIP": "100.100.100.100",
  "UseTailscale": false,
  "RtspUrlPrimary": "rtsp://192.168.1.100:8554/live",
  "RtspUrlSecondary": "rtsp://192.168.1.100:8554/gimbal",
  "VideoNetworkCaching": 100,
  "PreferredVideoPlayer": "Embedded",
  "UseELRS": false,
  "HttpTimeoutSeconds": 5,
  "DefaultTab": "Dashboard",
  "DebugMode": false
}
```

## Usage

### Opening the Control Interface

**Method 1: Menu Bar (Pop-Out Window)**
- Click **NOMAD -> Pop Out NOMAD Window**
- Opens the full NOMAD interface in a separate, movable window
- Ideal for multi-monitor setups

**Method 2: FlightData Tab**
- NOMAD tab appears in FlightData actions panel
- Provides the same interface integrated into Mission Planner

### Dashboard Tab

The dashboard provides:
- Connection status indicator (green = connected)
- Quick action buttons for common operations
- System status cards showing VIO and GPS state
- Video preview (loads automatically when Jetson is online)
- Activity log with recent events

### Video Tab

The Video tab combines the ZED stream with the payload control panel:

**Video Features:**
- **Play/Stop**: Start and stop video streams
- **Snapshot**: Capture current frame
- **Fullscreen**: Expand video to full window

**Payload Controls:**
- One control group per configured payload — drop / retract buttons for servo
  payloads, or a live PWM slider for aiming servos
- Relay trigger for a water pump (configurable duration)
- Configure payloads, channels and PWM limits under **Settings -> Payloads**

### Terminal Tab

Execute commands on the Jetson remotely:
- Quick commands dropdown for common operations
- Command history (up/down arrows)
- Output with color-coded errors
- Safe command whitelist in production mode

### Health Tab

Real-time monitoring:
- **CPU/GPU**: Temperature, load, frequency
- **Memory**: Used/total with percentage
- **Disk**: Free space and usage
- **Power**: Current draw in watts
- **Network**: Tailscale status and IP
- **Thermal**: Warning/critical indicators

### Settings

1. Click **NOMAD -> Settings** in the menu
2. Configure connection settings:
   - Jetson IP/Port
   - Tailscale IP (if using Tailscale VPN)
   - RTSP URLs for video streams
3. Configure RadioMaster connection:
   - **UDP Mode**: Default port 14550
   - **COM Port Mode**: Select serial port and baud rate
4. Configure LTE MAVLink input:
   - **UDP Mode**: Default port 14560
5. Configure Mission Planner's main connection:
   - **UDP Mode**: Default port 14600 for the plugin's merged router output
6. Click **Test Connection** to verify
7. Click **OK** to save

### RadioMaster ELRS Connection

The plugin supports two connection modes for RadioMaster ELRS:

**UDP Mode (Default):**
- RadioMaster transmitter outputs MAVLink over USB
- NOMAD plugin listens for RadioMaster input on UDP port 14550
- Mission Planner connects to the plugin's merged output on UDP port 14600
- Works when RadioMaster is in WiFi bridge mode

**COM Port Mode:**
- Direct serial connection to RadioMaster receiver
- Select the appropriate COM port (Windows) or /dev/ttyUSB* (Linux)
- Baud rate typically 420000 for ELRS or 115200 for standard
- Better for direct connections without network stack overhead

## Linux Support

The plugin runs on Linux via Mono with Mission Planner:

| Feature | Windows | Linux |
|---------|---------|-------|
| Core UI (WinForms) | Full | Full (Mono) |
| HTTP API calls | Full | Full |
| SSH Terminal | Full | Full |
| MAVLink protocol | Full | Full |
| LibVLC Video | Bundled | System libvlc |
| Serial Ports | COM1, COM2... | /dev/ttyUSB0... |

**Linux Installation:**
1. Install Mission Planner via Mono
2. Install `libvlc-dev` package for video support
3. Configure serial port paths in settings

## Architecture

```
+----------------------------------------------------------+
|                    Mission Planner                        |
|  +----------------------------------------------------+  |
|  |              NOMAD Plugin                          |  |
|  |  +---------+ +---------+ +---------------------+   |  |
|  |  | Dash /  | | Boundary| |   Video + Payload   |   |  |
|  |  | Health  | | / SLAM  | |   (LibVLC + servos) |   |  |
|  |  +----+----+ +----+----+ +----------+----------+   |  |
|  |       |           |                 |              |  |
|  |       v           v                 v              |  |
|  |  +---------------------------------------------+   |  |
|  |  |           DualLinkSender                    |   |  |
|  |  |  HTTP (Tailscale) -or- MAVLink (ELRS)       |   |  |
|  |  +------------------+--------------------------|   |  |
|  +---------------------|---------------------------+  |
+------------------------|-----------------------------+
                         |
    +--------------------|----------------------+
    |                    v                      |
    |  +------------------------------------+   |
    |  |        Jetson Orin Nano            |   |
    |  |  +------------------------------+  |   |
    |  |  |    NOMAD Edge Core API       |  |   |
    |  |  |    (FastAPI on port 8000)    |  |   |
    |  |  +------------------------------+  |   |
    |  |  +------------------------------+  |   |
| | | MediaMTX RTSP Server | | |
| | | (port 8554: /primary) | | |
    |  |  +------------------------------+  |   |
    |  +------------------------------------+   |
    +------------------------------------------+
```

## MAVLink Commands (ELRS Mode)

| Command ID | Name | Parameters |
|------------|------|------------|
| 31010 | `CMD_NOMAD_TASK1_CAPTURE` | p1: heading, p2: gimbal, p3: lidar |
| 31011 | `CMD_NOMAD_TASK2_RESET` | (none) |
| 31012 | `CMD_NOMAD_TASK2_HIT` | p1: x, p2: y, p3: z |

## Troubleshooting

- **Plugin Not Loading**: Check DLL is in correct plugins folder
- **Connection Failed**: Verify Jetson IP/port, check Tailscale is connected
- **Video Not Playing**: Ensure LibVLC DLLs are in plugin folder
- **COM Port Not Found**: Check RadioMaster connection type in settings
- **Dashboard Button Hidden**: Ensure plugin is updated to v3.1+
- **Keys Stuck**: Move mouse outside the keyboard panel to release all keys
