# NOMAD Mission Planner Plugin

A drop-in plugin for [Mission Planner](https://ardupilot.org/planner/) that adds
the NOMAD ground-control integration, linking Mission Planner to the NOMAD Edge
Core companion-computer service.

## Install

1. Close Mission Planner.
2. From this folder, run:

   ```powershell
   powershell -ExecutionPolicy Bypass -File INSTALL.ps1
   ```

   This copies `NOMADPlugin.dll` into your per-user plugins folder
   (`%LOCALAPPDATA%\Mission Planner\plugins`) — no admin needed.
3. Start Mission Planner and open the NOMAD panel from the **Tools** menu.

### Manual install

Copy `NOMADPlugin.dll` into `%LOCALAPPDATA%\Mission Planner\plugins\` yourself,
then restart Mission Planner.

> Keep only one copy. If `NOMADPlugin.dll` also sits in
> `C:\Program Files (x86)\Mission Planner\plugins\`, Mission Planner loads the
> plugin twice — delete the Program Files copy.

## Requirements

- Windows with Mission Planner installed (built against **1.3.83**).
- .NET Framework 4.8 (ships with current Mission Planner / Windows).

The plugin references only Mission Planner's own assemblies and the .NET
Framework, so no extra files are needed beyond `NOMADPlugin.dll`.
