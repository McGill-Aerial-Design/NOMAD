# Neighborhood inspection with NOMAD

Fly a short ArduPilot mission over a rendered neighborhood, inspect a roof through
NOMAD's gimbal camera, and command a marker output after the audience cuts a
telemetry route. This runs the actual Mission Planner host, NOMAD plugin, ArduPilot
SITL and Project AirSim Neighborhood environment. The camera shows the current
rendered scene; its pose follows observed controller servo outputs.

Mission Planner owns flight controls and missions. ArduPilot executes the flight.
NOMAD supplies its normal video, gimbal, payload and link-router controls.

![Neighborhood inspection through NOMAD](images/inspection.png)

## Prepare the laptop

Run commands from the repository root. Required:

- Windows x64 with Mission Planner **1.3.83** in its standard installation folder.
- Visual Studio/MSBuild, Roslyn and the .NET Framework 4.8 targeting pack described
  in the [development guide](../docs/development.md).
- Python **3.11**, available through `py -3.11`.
- GStreamer x64 at `C:\gstreamer\1.0\msvc_x86_64`, including `x264enc`,
  `rawvideoparse`, `tcpclientsrc` and `rtspclientsink`; Mission Planner's own
  installed GStreamer runtime is also required.
- The qualified Windows **ArduCopter V4.8.0-dev b7ba8bca** runtime directory,
  containing `ArduCopter.exe`, its `cyg*.dll` dependencies and `copter.parm`
  directly in that directory. A rolling SITL download is not interchangeable.

Setup verifies these SHA-256 values:

| File | Required SHA-256 |
|---|---|
| `ArduCopter.exe` | `C5D95E95684D3AC0A6FA7315DD23D005D1BFCA81D1CB156EAC113D3C49822D75` |
| `copter.parm` | `5E01345B45D1C6190B28BECE5638BBDD4CF1CCE35E05BBBF480AB24D2B51AA0E` |

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\conference\setup.ps1 `
    -SitlRuntimeDirectory 'C:\path\to\qualified-windows-sitl'
```

Setup needs Internet access and space for a copied Mission Planner installation,
the simulator, Python environment and run evidence. It downloads the packaged
[Project AirSim 1.0.1 Neighborhood](https://github.com/iamaisim/ProjectAirSim/releases/tag/v1.0.1)
and [MediaMTX 1.21.2](https://github.com/bluenviron/mediamtx/releases/tag/v1.21.2),
verifies their archive hashes, and installs the pinned Python clients. Unreal
Editor, a source build of Unreal and paid services are not required. Retain the
downloaded components' licenses and notices.

The prepared host, plugin, simulator configuration and identity manifest live in
`build/conference/prepared`. Setup builds and copies the plugin into that host;
it does not install it into the operator's Mission Planner. Repeat setup after
changing the plugin or prepared files. Launch verifies the manifest hashes.

## Rehearse the complete check

Close other Mission Planner sessions first. Check calibration without starting
processes, then request the full local simulator rehearsal:

```powershell
.\build\conference\python\Scripts\python.exe .\conference\camera.py --check
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\conference\run.ps1 -Check
```

The full check normally arms, takes off to 25 m in GUIDED, and switches to AUTO
to execute the uploaded survey leg. It requires both mission progress and changed
GPS position before testing the real NOMAD camera, gimbal and payload controllers.
It switches between both live camera inputs, observes the gimbal's channel 7/8 outputs, cuts the radio route, and operates
the marker through the remaining route: explicit Safe, two confirmations to
activate, then explicit Safe again. It restores routes, returns with RTL to the
clear launch site, observes landed state, normally disarms and observes disarmed state.

After Mission Planner closes, the launcher deliberately stops one owned encoder
and requires the camera bridge to report failure and stop its sibling encoder.
That camera failure is expected only at the end of `-Check`. A successful run
requires both the native rehearsal and this lifecycle check to complete.

Each invocation creates `build/conference/run-<timestamp>-<id>`. Inspect
`result.json`, `lifecycle.json` and the command's exit status together. The final
`camera-status.json` records the deliberate encoder failure, not a failed flight
check. Screenshots and diagnostic logs remain in that run directory. These are
software integration results for the pinned simulator cohort, not aircraft qualification.

## Present the inspection

![Chase camera during the survey](images/flight.png)

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\conference\run.ps1
```

The launcher opens the neighborhood renderer and a separate Mission Planner host
with one conference aircraft profile. It connects through NOMAD's router, uploads
the short mission and opens NOMAD's Video & Payloads page with the inspection
stream playing. Wait for normal ArduPilot pre-arm readiness; startup `ready`
does not establish flight readiness.

| Person | Responsibility |
|---|---|
| Presenter | Explain the inspection and let the audience choose the camera aim and telemetry fault |
| Mission Planner operator | Review and fly the mission; use NOMAD gimbal, marker and Safe controls |
| Fault/evidence operator | Select the Conference menu fault, show Link Status and controller output telemetry |

1. In native **Plan**, use **Read** to review the uploaded mission. It contains a
   25 m takeoff, a short survey leg, a 90-second inspection loiter and RTL.
   If showing fence setup, stage a reviewed polygon through NOMAD Fence Setup,
   then use native MP Write/Read against SITL while disarmed. Fence setup is a
   separate operator demonstration, not part of the automated check.
2. Use native Mission Planner controls to arm normally, take off in GUIDED to
   25 m, and enter AUTO for the uploaded mission. Show the actual HUD, position
   and mission progress as ArduPilot reaches the inspection area.
3. Open NOMAD's Gimbal control and use MAVLink targeting to aim at a roof. Select
   **Inspection gimbal** or **Chase camera** from the video input dropdown.
   The inspection camera follows actual SITL output: channel 7 is pitch and
   channel 8 is roll. Both feeds are live renders of this simulation.
4. Ask the audience which route to interrupt. The top-level **Conference** menu
   offers **Cut telemetry radio**, **Cut telemetry network**, **Cut both telemetry
   routes** and **Restore routes**. Each selects a complete route state; cutting
   one explicitly enables the other. Show NOMAD Link Status and repeat a gimbal
   movement through the surviving route.
5. In NOMAD's payload controls, select **Secure marker** first. Select
   **Release marker** twice to satisfy its confirmations. Observe controller
   channel 9 at 2000 PWM, then select **Secure marker** and observe 1000 PWM. This demonstrates
   the release output; no physical marker or delivery is modeled.
6. Restore routes. Complete the mission's RTL or request native RTL to return to
   the clear launch site; do not descend onto the roof inspection area. Observe
   landed state, then disarm normally and observe disarmed state. This cohort sets
   `DISARM_DELAY=0`; do not wait for automatic disarming.

Close the conference Mission Planner window to end the session. The launcher
requests camera/link shutdown and stops only its owned processes. It verifies
preservation of the installed plugin and the named operator configuration/profile
files; it does not claim every host cache or log is unchanged.

## Operating limits

- Once prepared, the run needs no Internet service. MP uses cached map tiles;
  check coverage before presenting. The rendered neighborhood does not depend
  on those tiles. Missing tiles do not change the controller's position telemetry.
- Use an offline or trusted network. The owned forwarding gates, RTSP server and
  camera encoder sockets use loopback, but the simulator components also expose
  wildcard listeners: SITL's sensor input includes `0.0.0.0:9003`, and Project
  AirSim's server can listen on `::`. The launcher does not change firewall rules.
- The telemetry paths are local byte-forwarding gates into one real controller.
  They exercise the actual NOMAD router, not RF range, LTE behavior or independent
  physical redundancy. Both-down operation follows the configured ArduPilot
  failsafes; rehearse it separately before selecting it during a flight.
- Camera transport and its controller-output observer are independent of those
  two telemetry gates. Video can continue while the operator's telemetry is lost.
  A moving image alone does not establish a healthy command path.
- Stock SITL does not implement NOMAD's optional custom TERMINATE mode. This
  rehearsal does not qualify RC/HID hardware, QuadPlane behavior, real camera or
  payload calibration, fence containment, physical landing or termination.

For the plugin's normal controls and storage behavior, see
[operations](../docs/operations.md). For the simulator interfaces and configuration,
see the [Project AirSim ArduPilot guide](https://github.com/iamaisim/ProjectAirSim/blob/v1.0.1/docs/controllers/ardupilot.md).
