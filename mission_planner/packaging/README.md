# NOMAD Mission Planner Plugin

The NOMAD plugin is a Windows Mission Planner client. Supported command requests
use runtime IPC to `nomad-runtime`; router telemetry and native Mission Planner
controls remain separate paths. For current limits, see
[qualification status](../../docs/qualification.md).

## Install

The local `pixi run build-plugin-only` task compiles without installing. Use a
complete release manifest and its plugin archive for deployment. The installer
now forwards explicit actions to the release deployment tool; a loose DLL is
not an installable release. Use Python 3.13 and the reviewed deployment tools
from the release or the corresponding source checkout.

```powershell
$arguments = @{
    Root = 'C:\ProgramData\NOMAD\deployment'
    MissionPlanner = 'C:\Program Files (x86)\Mission Planner'
    DeploymentTool = '.\scripts\release\deploy.py'
}
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action verify `
    -Manifest '.\release-manifest.json' -Package '.\NOMADPlugin.zip'
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action stage `
    -Manifest '.\release-manifest.json' -Package '.\NOMADPlugin.zip'
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action status
# Close Mission Planner before this explicit activation.
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action activate -Release 'vX.Y.Z'
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action rollback
.\mission_planner\packaging\INSTALL.ps1 @arguments -Action cleanup -Release 'vOLD.VERSION'
```

Use the actual manifest filename and release version. Verification binds package
SHA-256, component, platform, runtime IPC expectation and Mission Planner target.
Checksums establish correlation and integrity; they are not publisher signatures.
Protect the deployment root with administrator-controlled ACLs before use; do not
put operator state in this directory. Activation refuses an unmanaged existing
DLL: stage the exact existing release and use `-Action adopt -Release 'vX.Y.Z'`
to record it only after its deployed bytes match. Otherwise preserve the existing
installation and obtain its original verified package before upgrade.

Staging keeps immutable release payloads without changing the installed DLL.
Activation and rollback require Mission Planner closed, validate the installed
Mission Planner version (**1.3.83**), and replace only
`plugins\NOMADPlugin.dll` using a temporary file and same-directory atomic replace.
A locked DLL fails clearly. The tool never kills or starts Mission Planner.
Rollback restores the retained exact previous DLL and checks its digest before
reporting success. Cleanup refuses the active and rollback releases.

The installer preserves Mission Planner settings, other plugins and AppData
`nomad_config.json`, including `CoreClientCredential`. It does not remove the
legacy AppData DLL; an operator must resolve duplicate discovery separately.
AppData is neither a package payload nor rollback storage. Configuration migration
remains the plugin's existing load behavior when the operator later opens it;
deployment never rewrites or rolls back configuration files.

After an interrupted activation, inspect `status` and explicitly run `recover`
before another activation. See the canonical [operations guide](../../docs/operations.md)
for deployment journal states, recovery and activation ordering. Tagged publication
and manual artifact generation are described in the
[development workflow](../../docs/development.md).

## Ground router host

The plugin does not contain or launch the ground router. Download the separate
`NOMADLinkRouter` release package. Keep authoritative router JSON outside the
versioned programs, derive it once from `router.example.json`, then run
`nomad-link-router.exe <external-router.json>` in an independently supervised
process. Keep `Nomad.LinkRouter.dll` beside the executable. The Mission Planner status panel connects
to loopback TCP `127.0.0.1:14610`; native Mission Planner uses UDPCl to the
configured `mission_planner` consumer, normally port `14600`.

The standalone host enforces `mission_planner` as receive-only. An explicit host
JSON entry with `AllowOutbound` omitted (default true) or set true is rejected;
the sample sets it false while preserving the separate command-capable
`nomad_core` consumer. Mission Planner's obsolete `IntegratedFlightMode` field
is removed during config migration and never rewrites host configuration. The
plugin installer below
installs only `NOMADPlugin.dll`; it does not install or register a router service.

### Software qualification

CI uses temporary fake Mission Planner installations, including paths with
spaces and `Program Files (x86)`, to verify exact A → B → A DLL replacement and
unchanged settings/credentials and unrelated plugins. It checks closed-process,
unsupported-target and malformed-DLL rejection. It does not start the Mission
Planner GUI. Real-host acceptance still checks administrator ACLs, loaded DLL
locking, actual Mission Planner discovery and plugin startup with retained settings.

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
