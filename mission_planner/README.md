# NOMAD Mission Planner Plugin

**Comprehensive Control Interface for NOMAD Operations - AEAC 2026**

This C# plugin integrates directly with Mission Planner to provide full control of the NOMAD drone system including task execution, embedded video streaming, remote terminal access, MAVLink dual-link failover, and indoor manual control.

## Features

### Full-Screen Sidebar Interface (v3.1)
The primary interface features a modern sidebar navigation design:
- **Dashboard**: Quick overview of all critical systems at a glance
- **Task 1: Recon**: GPS-based outdoor reconnaissance with snapshot capture
- **Task 2: Extinguish**: VIO-based indoor navigation with target tracking
- **Video Feed**: Embedded RTSP streaming with WASD controls
- **Terminal**: Remote command execution on Jetson
- **System Health**: Real-time Jetson monitoring (CPU/GPU temps, memory, network)
- **Link Status**: Dual-link failover monitoring
- **Settings**: Plugin configuration

### Pop-Out Window Support
- The NOMAD main screen can be popped out into a separate window
- Ideal for multi-monitor setups
- Access via **NOMAD -> Pop Out NOMAD Window** menu

### MAVLink Dual Link Failover
- **Primary Link**: LTE/Tailscale via Jetson mavlink-router
- **Secondary Link**: RadioMaster transmitter (UDP 14550 or COM port)
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

### Enhanced WASD Nudge Controls
The Video view includes comprehensive manual drone control:
- **Movement (WASD)**: Forward, Back, Left, Right
- **Altitude (Up/Down Arrows)**: Climb/Descend
- **Yaw (Left/Right Arrows)**: Rotate left/right
- **Visual Feedback**: Real-time key press visualization
- **Mouse-Based Activation**: Hover over the keyboard panel to enable
- **Payload Controls**: Drop mechanisms and water pump control

### Core Functionality
- **Task 1 (Recon)**: Capture snapshot and calculate target position relative to landmarks
- **Task 2 (Extinguish)**: Manage exclusion map and target hit registration
- **Embedded Video**: Built-in RTSP player with low-latency streaming (LibVLC)
- **Telemetry Display**: Real-time position and connection status
- **Dual-Link Communication**: HTTP API over Tailscale or MAVLink via ELRS for redundancy

## Files

| File | Description |
|------|-------------|
| `src/NOMADPlugin.cs` | Main plugin class (implements `MissionPlanner.Plugin.Plugin`) |
| `src/NOMADMainScreen.cs` | **Full-page sidebar interface with pop-out support** |
| `src/NOMADDashboardView.cs` | **Dashboard overview panel** |
| `src/NOMADViews.cs` | **Individual view implementations (includes WASD controls)** |
| `src/DualLinkSender.cs` | HTTP and MAVLink communication handler |
| `src/MAVLinkConnectionManager.cs` | Dual link failover manager (UDP + COM port) |
| `src/LinkHealthPanel.cs` | Link health monitoring UI |
| `src/EmbeddedVideoPlayer.cs` | Built-in RTSP video player |
| `src/JetsonTerminalControl.cs` | Remote terminal interface |
| `src/EnhancedHealthDashboard.cs` | Health monitoring display |
| `src/EnhancedWASDControl.cs` | **Full keyboard control with visual feedback** |
| `src/NOMADConfig.cs` | Configuration persistence |
| `src/NOMADSettingsForm.cs` | Settings dialog |
| `src/NOMADPlugin.csproj` | Project file |

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
   .\mission_planner\src\build_and_deploy.ps1
   ```

This ensures the embedded player can initialize libVLC at runtime. If you prefer not to include the redistributables in the repository, operators can instead install VLC on their machines and the plugin will detect the native libs automatically.

### Building

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

### Task 1: Outdoor Recon

1. Ensure GPS fix is acquired
2. Navigate the drone to view a target
3. Go to **Task 1** tab
4. Click **CAPTURE SNAPSHOT**
5. Position and image are logged

### Task 2: Indoor Extinguish

1. Go to **Task 2** tab
2. Click **Reset VIO Origin** at your start position
3. Use the Enhanced WASD controls (in Video tab) for precise positioning
4. Click **Reset Exclusion Map** to clear engaged targets

### Video Tab with WASD Controls

The Video tab combines streaming with drone control:

**Video Features:**
- **Play/Stop**: Start and stop video streams
- **Snapshot**: Capture current frame
- **Fullscreen**: Expand video to full window

**WASD Nudge Controls:**
1. Hover mouse over the keyboard visual panel
2. Enable via the toggle switch
3. Use keys:
   - **W/S**: Forward/Backward
   - **A/D**: Left/Right strafe
   - **Up/Down Arrows**: Climb/Descend altitude
   - **Left/Right Arrows**: Yaw rotation
4. Keys automatically release when mouse leaves the panel
5. Visual feedback shows which keys are pressed

**Payload Controls:**
- **Drop Payload 1/2**: Trigger linear actuators
- **Shoot Water**: Activate water pump
- **Nozzle Angle**: Servo position slider (0-180 degrees)

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
4. Click **Test Connection** to verify
5. Click **OK** to save

### RadioMaster ELRS Connection

The plugin supports two connection modes for RadioMaster ELRS:

**UDP Mode (Default):**
- RadioMaster transmitter outputs MAVLink over USB
- Mission Planner listens on UDP port 14550
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
| WASD Nudge Control | Full Win32 API | Event-based (works) |
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
|  |  | Task 1  | | Task 2  | |   Video + WASD      |   |  |
|  |  | Capture | |  Map    | |   (LibVLC + Keys)   |   |  |
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
    |  |  |   MediaMTX RTSP Server       |  |   |
    |  |  |   (port 8554: /live)         |  |   |
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

### WASD Nudge MAVLink Messages

The WASD controls send the following MAVLink messages:

| Action | Message Type | Notes |
|--------|-------------|-------|
| Forward/Back | `SET_POSITION_TARGET_LOCAL_NED` | Velocity in body frame |
| Left/Right | `SET_POSITION_TARGET_LOCAL_NED` | Strafe velocity |
| Climb/Descend | `SET_POSITION_TARGET_LOCAL_NED` | Vertical velocity |
| Yaw Left/Right | `SET_POSITION_TARGET_LOCAL_NED` | Yaw rate |

**Requirements:**
- Drone must be in GUIDED mode
- MAVLink connection must be active
- Speed adjustable via slider (0.1 - 2.0 m/s)

## Troubleshooting

- **Plugin Not Loading**: Check DLL is in correct plugins folder
- **Connection Failed**: Verify Jetson IP/port, check Tailscale is connected
- **Video Not Playing**: Ensure LibVLC DLLs are in plugin folder
- **Nudge Not Working**: Check drone is in GUIDED mode, verify MAVLink connection
- **COM Port Not Found**: Check RadioMaster connection type in settings
- **Dashboard Button Hidden**: Ensure plugin is updated to v3.1+
- **Keys Stuck**: Move mouse outside the keyboard panel to release all keys

