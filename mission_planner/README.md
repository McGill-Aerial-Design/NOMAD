# NOMAD Mission Planner Plugin

**Comprehensive Control Interface for NOMAD Operations - AEAC 2026**

This C# plugin integrates directly with Mission Planner to provide full control of the NOMAD drone system including task execution, embedded video streaming, remote terminal access, MAVLink dual-link failover, and indoor manual control.

## Features

### Full Control Page
- **Dashboard Tab**: System overview with quick actions and status cards
- **Task 1 Tab**: GPS-based outdoor reconnaissance with snapshot capture
- **Task 2 Tab**: VIO-based indoor navigation with WASD controls
- **Video Tab**: Embedded RTSP streaming (no external VLC required)
- **Terminal Tab**: Remote command execution on Jetson
- **Health Tab**: Real-time Jetson health monitoring (CPU/GPU temps, memory, network)

### MAVLink Dual Link Failover (NEW)
- **Primary Link**: LTE/Tailscale via Jetson mavlink-router
- **Secondary Link**: RadioMaster transmitter (UDP 14550)
- **Automatic Failover**: Switches links on connection loss
- **Health Monitoring**: Real-time latency and packet loss tracking
- **Manual Override**: Switch links manually via Link Status panel
- **Auto-Reconnect**: Returns to preferred link when available

### Core Functionality
- **Task 1 (Recon)**: Capture snapshot and calculate target position relative to landmarks
- **Task 2 (Extinguish)**: Manage exclusion map and target hit registration
- **Embedded Video**: Built-in RTSP player with low-latency streaming
- **Indoor Nudge**: WASD keyboard controls for manual indoor positioning
- **Telemetry Display**: Real-time position and connection status
- **Dual-Link Communication**: HTTP API or MAVLink via ELRS for redundancy

## Files

| File | Description |
|------|-------------|
| `src/NOMADPlugin.cs` | Main plugin class (implements `MissionPlanner.Plugin.Plugin`) |
| `src/NOMADFullPage.cs` | Full-page tabbed control interface |
| `src/NOMADControlPanel.cs` | Quick access popup panel |
| `src/DualLinkSender.cs` | HTTP and MAVLink communication handler |
| `src/MAVLinkConnectionManager.cs` | **Dual link failover manager** |
| `src/LinkHealthPanel.cs` | **Link health monitoring UI** |
| `src/EmbeddedVideoPlayer.cs` | Built-in RTSP video player |
| `src/JetsonTerminalControl.cs` | Remote terminal interface |
| `src/EnhancedHealthDashboard.cs` | Health monitoring display |
| `src/WASDNudgeControl.cs` | WASD keyboard control handler |
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

**Method 1: Menu Bar**
- Click **NOMAD → Open Full Control Page**

**Method 2: Quick Panel**
- Click **NOMAD → Open Control Panel** (smaller popup)

**Method 3: FlightData Context**
- NOMAD tab may appear in FlightData actions panel

### Dashboard Tab

The dashboard provides:
- Connection status indicator (green = connected)
- Quick action buttons for common operations
- System status cards showing VIO and GPS state
- Activity log with recent events

### Task 1: Outdoor Recon

1. Ensure GPS fix is acquired
2. Navigate the drone to view a target
3. Go to **Task 1** tab
4. Click **📸 CAPTURE SNAPSHOT**
5. Position and image are logged

### Task 2: Indoor Extinguish

1. Go to **Task 2** tab
2. Click **Reset VIO Origin** at your start position
3. Use WASD controls for precise positioning:
   - **W/S**: Forward/Backward
   - **A/D**: Left/Right
   - **Q/E**: Rotate
   - **R/F**: Up/Down
4. Click **Reset Exclusion Map** to clear engaged targets

### Video Tab

The embedded video player supports:
- **Play/Stop**: Start and stop video streams
- **Snapshot**: Capture current frame
- **Fullscreen**: Expand video to full window
- **Latency Control**: Adjust network caching (lower = less delay)
- **External Player**: Fall back to VLC if needed

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

1. Click **NOMAD → Settings** in the menu
2. Configure connection settings, video preferences
3. Click **Test Connection** to verify
4. Click **OK** to save


### Task 2: Extinguish

1. Click **Reset Exclusion Map** at the start of a new run
2. Manually register hits with X, Y, Z coordinates and **Hit** button

### Video Streaming

1. Select video player: VLC (recommended), FFplay, or System Default
2. Click **Open Primary** for ZED/Navigation view
3. Click **Open Secondary** for Gimbal/Targeting view

### Indoor Nudge

1. Check **Enable Indoor Nudge (WASD)**
2. Use W/A/S/D for forward/left/back/right movement
3. Adjust nudge speed (default 0.5 m/s)

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Mission Planner                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │              NOMAD Plugin                        │   │
│  │  ┌─────────┐ ┌─────────┐ ┌────────────────────┐ │   │
│  │  │ Task 1  │ │ Task 2  │ │   Video Streams    │ │   │
│  │  │ Capture │ │  Map    │ │  (VLC/FFplay)      │ │   │
│  │  └────┬────┘ └────┬────┘ └─────────┬──────────┘ │   │
│  │       │           │                │            │   │
│  │       ▼           ▼                ▼            │   │
│  │  ┌──────────────────────────────────────────┐   │   │
│  │  │           DualLinkSender                 │   │   │
│  │  │  HTTP API ──────────┐  MAVLink/ELRS     │   │   │
│  │  └──────────────────────┼───────────────────┘   │   │
│  └──────────────────────────┼──────────────────────┘   │
└──────────────────────────────┼─────────────────────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     ▼                     │
         │  ┌──────────────────────────────────┐     │
         │  │        Jetson Orin Nano          │     │
         │  │  ┌─────────────────────────────┐ │     │
         │  │  │    NOMAD Edge Core API      │ │     │
         │  │  │    (FastAPI on port 8000)   │ │     │
         │  │  └─────────────────────────────┘ │     │
         │  │  ┌─────────────────────────────┐ │     │
         │  │  │   MediaMTX RTSP Server      │ │     │
         │  │  │   (port 8554: /live /gimbal)│ │     │
         │  │  └─────────────────────────────┘ │     │
         │  └──────────────────────────────────┘     │
         └───────────────────────────────────────────┘
```

## MAVLink Commands (ELRS Mode)

| Command ID | Name | Parameters |
|------------|------|------------|
| 31010 | `CMD_NOMAD_TASK1_CAPTURE` | p1: heading, p2: gimbal, p3: lidar |
| 31011 | `CMD_NOMAD_TASK2_RESET` | (none) |
| 31012 | `CMD_NOMAD_TASK2_HIT` | p1: x, p2: y, p3: z |

## Troubleshooting

- **Plugin Not Loading**: Check DLL is in correct plugins folder
- **Connection Failed**: Verify Jetson IP/port, check firewall
- **Video Not Playing**: Ensure VLC is installed and in PATH
- **Nudge Not Working**: Check ELRS connection, drone must be in GUIDED mode

