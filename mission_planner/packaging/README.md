# NOMAD Mission Planner Plugin

The NOMAD plugin is a Windows Mission Planner client. Supported command requests
use runtime IPC to `nomad-runtime`; router telemetry and native Mission Planner
controls remain separate paths. For current limits, see
[qualification status](../../docs/qualification.md).

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

The local build task compiles only; it does not make a release archive or install
the DLL. The repository release workflow packages the plugin and standalone
ground router separately. A `v*` tag publishes release assets; manual workflow
dispatch uploads downloadable artifacts. See the [development workflow](../../docs/development.md).

## Ground router host

The plugin does not contain or launch the ground router. Download the separate
`NOMADLinkRouter` release package, edit `router.example.json` for the physical
links and consumers, then run `nomad-link-router.exe router.example.json` in an
independently supervised process. Keep `Nomad.LinkRouter.dll` and the example
configuration beside the executable. The Mission Planner status panel connects
to loopback TCP `127.0.0.1:14610`; native Mission Planner uses UDPCl to the
configured `mission_planner` consumer, normally port `14600`.

The standalone host enforces `mission_planner` as receive-only. An explicit host
JSON entry with `AllowOutbound` omitted (default true) or set true is rejected;
the sample sets it false while preserving the separate command-capable
`nomad_core` consumer. Mission Planner's obsolete `IntegratedFlightMode` field
is removed during config migration and never rewrites host configuration. The
plugin installer below
installs only `NOMADPlugin.dll`; it does not install or register a router service.

### Manual install

Copy `NOMADPlugin.dll` into
`C:\Program Files (x86)\Mission Planner\plugins\` yourself, then restart Mission
Planner.

## Requirements

- Windows with Mission Planner installed (built against **1.3.83**).
- .NET Framework 4.8 (ships with current Mission Planner / Windows).
- The separately distributed `NOMADLinkRouter` host package for multi-link routing.

Runtime-backed requests also need one running, compatible `nomad-runtime` with
the same configured IPC endpoint. See the [operations guide](../../docs/operations.md).
Plugin video uses Mission Planner's GStreamer wrapper and SkiaSharp frame support.
The installer deploys only NOMADPlugin.dll; GStreamer is supplied by the
Mission Planner installation. The Jetson ROS image bridge and MediaMTX are
separate video infrastructure. Qualify the complete package at G8; a DLL-only
install does not establish task readiness. Installation changes the local
Mission Planner deployment and requires operator authorization.
