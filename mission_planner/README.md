# Mission Planner integration

`mission_planner/` is the C# operator UI, management and status client. Supported
typed vehicle requests use local IPC to the long-running C++ `nomad-runtime`.
The client does not own vehicle policy or a direct command transport.

The plugin may own:

- operator views and configuration;
- telemetry presentation;
- management and the currently supported typed command controls that call the NOMAD runtime client;
- GCS-native link, video, log, and display workflows.

It must not become a second source of vehicle, mission, or safety logic. The C++
core/runtime is the command boundary; Mission Planner is replaceable.

## Build

The plugin requires Windows, MSBuild, .NET Framework 4.8, and Mission Planner
reference assemblies:

```powershell
pixi run build-plugin-only
```

This compiles the plugin and writes `src/bin/Release/NOMADPlugin.dll` without
copying, creating, or deleting files in the Mission Planner installation. Run
`pixi run test-plugin-build-only` to check the build-only dispatch against an
isolated deny-write installation. That test uses the C# compiler bundled with
Visual Studio MSBuild. For installation steps, see
[the packaging guide](packaging/README.md) and
[operations](../docs/operations.md).

Use `lint-plugin` and focused `test-plugin-*` tasks for non-deploying checks.
`NomadCoreClient` uses only the C++ runtime over versioned loopback JSON Lines
IPC. It performs HELLO negotiation and sends the typed requests supported by
protocol v1. If the runtime is unavailable, commands fail closed; Mission Planner
does not launch the CLI or fall back to native MAVLink or direct vehicle writes.
Requests with an unknown outcome are not replayed. GuidedGoto is unavailable
until runtime protocol v1 adds a typed navigation request, and the boundary
monitor tells the operator to take manual control. The local API-key setting is
a nonempty actuation gate, not IPC authentication. Gimbal angle targeting uses
typed runtime requests; maintenance parameter paths and native
Mission Planner/RC controls remain outside this client boundary. Global authority
handover is open.
For CONOPS v1.0, the dedicated GCS display must show live aircraft position and
competition area (AE27-OPS-004). The LAND-as-termination recipe and descent-speed
settings are removed. The monitored termination button and hard-boundary request
report termination unavailable and send no substitute aircraft command. Direct
vehicle-fence upload/clear is removed; only visual export to the Plan map remains.
Flight-controller fence installation/readback is not exposed through runtime
IPC v1. A future request must use the C++ core and complete integrated authority
and containment qualification. Do not use these controls as flight termination.
Aircraft-wide authority arbitration, termination and hardware acceptance remain
unqualified; see [qualification status](../docs/qualification.md).

Core-client loopback protocol checks are available through
`pixi run test-plugin-core-client`; the other pure helper checks are available
through `test-plugin-*` Pixi tasks. See
[the canonical architecture](../docs/architecture.md) and
[development workflow](../docs/development.md) for ownership and verification.

## Multi-Link routing configuration

Mission Planner is a management/status client for the standalone ground router.
It never starts or stops the router process, binds physical links, or configures
route selection and failover. The router host and plugin are separate release
packages; `pixi run build-ground-router` writes the host, library, example
configuration, and README to `build/ground-router/`.

Configure and supervise `nomad-link-router.exe` separately. Mission Planner's
native MAVLink connection uses UDPCl to the router's `mission_planner` consumer
(default `127.0.0.1:14600`). The C++ runtime uses its distinct command-capable
route: the router's `nomad_core` consumer at `14602` feeds the runtime listener
at `14601`. Mission Planner's status panel reconnects to loopback management
TCP `127.0.0.1:14610`, reports stale/unavailable status, and can select an
enabled link or return to automatic selection.

The standalone host enforces the `mission_planner` consumer as receive-only for
all profiles. An explicit entry with `AllowOutbound` omitted (default true) or
set true is rejected at startup; the example sets it false and leaves
`nomad_core` command-capable. Mission Planner has no aircraft MAVSDK endpoint or
router-topology settings. Its `RouterLocalPort` and `ManagementPort` configure
only its loopback router clients; update the host's matching consumer endpoint
when changing the UDP port. Profile migration removes retired topology and
`IntegratedFlightMode` fields; legacy `RouterMode=Embedded` is rejected.

## Opt-in modules

Reusable functionality belongs in the base NOMAD platform. Competition-, event-
and mission-specific functionality should normally be an opt-in module unless
there is a strong reason for it to be reusable platform functionality. Modules
must use the C++ core authority/safety boundary; they must not call MAVLink
directly or create independent vehicle-command ownership.

The SDK in `src/Core` provides metadata, enable flags, dependency order, shared
configuration/context, sidebar views/actions and configure/start/stop lifecycle.
Register modules in the existing plugin host. Stop releases background resources
in reverse dependency order; the screen disposes cached module views.
`src/Modules/ExampleModule.cs` is development documentation by example, disabled
by default. Set `NOMAD_PLUGIN_EXAMPLE_MODULE=1` before launching Mission Planner
to show its example view and action. It reads configuration and sends no vehicle
commands. Modules without workers may inherit the base no-op Start/Stop methods;
modules that start workers must stop and dispose them.
