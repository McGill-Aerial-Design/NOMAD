# NOMAD Mission Planner Plugin

A drop-in plugin for [Mission Planner](https://ardupilot.org/planner/) that adds
the NOMAD ground-control integration. The working source uses a local C++ core
client and direct media playback; it does not require the removed Edge Core API.
An existing packaged DLL may predate that migration; verify its build identity.

## Install

1. Build the plugin from the repository root with `pixi run build-plugin-only`.
   From this folder, stage the built DLL beside the installer:

   ```powershell
   Copy-Item -LiteralPath ..\src\bin\Release\NOMADPlugin.dll -Destination .\NOMADPlugin.dll
   ```

2. Close Mission Planner. From this folder, run:

   ```powershell
   powershell -ExecutionPolicy Bypass -File INSTALL.ps1
   ```

   This copies `NOMADPlugin.dll` into Mission Planner's installation plugins
   folder (`C:\Program Files (x86)\Mission Planner\plugins`).
3. Start Mission Planner and open the NOMAD panel from the **Tools** menu.

### Manual install

Copy `NOMADPlugin.dll` into
`C:\Program Files (x86)\Mission Planner\plugins\` yourself, then restart Mission
Planner.

## Requirements

- Windows with Mission Planner installed (built against **1.3.83**).
- .NET Framework 4.8 (ships with current Mission Planner / Windows).

Core-backed operations also need a compatible configured NOMAD C++ executable.
Video needs its selected playback/runtime dependencies. Qualify the complete
package at G8; a DLL-only install does not establish task readiness. Installation
changes the local Mission Planner deployment and requires operator authorization.
