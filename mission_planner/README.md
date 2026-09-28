# Mission Planner integration

`mission_planner/` is the current C# ground-station integration. It is
transitional while NOMAD moves vehicle behavior into the standalone C++ core.

The plugin may own:

- operator views and configuration;
- telemetry presentation;
- mission and command controls that call the NOMAD client boundary;
- GCS-native link, video, log, and display workflows.

It must not become a second source of vehicle, mission, or safety logic. The C++
core is the product boundary; Mission Planner is replaceable.

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
[the packaging guide](packaging/README.md).

Use `lint-plugin` and focused `test-plugin-*` tasks for non-deploying checks.
`NomadCoreClient` supports `LegacyOneShot` and `PersistentRuntime`. Persistent
mode connects to the C++ runtime over versioned loopback JSON Lines IPC, performs
HELLO negotiation, then issues typed requests without spawning `nomad`. It does
not automatically retry a request whose response is lost. The compatibility
mode remains available until a deployment selects persistent mode. The runtime
protocol is local-only and does not authenticate clients. Direct gimbal,
maintenance parameter paths remain; global authority handover is open.
For CONOPS v1.0, the dedicated GCS display must show live aircraft position and
competition area (AE27-OPS-004). The LAND-as-termination recipe and descent-speed
settings are removed. The monitored termination button and hard-boundary request
report termination unavailable and send no substitute aircraft command. Direct
vehicle-fence upload/clear is removed; only visual export to the Plan map remains.
Flight-controller fence installation/readback belongs through the C++ core and
still needs integrated authority and containment qualification. Do not use these controls as flight termination.
Aircraft-side activation, authority and acceptance remain migration GAP-05/06.

Core-client loopback protocol checks are available through
`pixi run test-plugin-core-client`; the other pure helper checks are available
through `test-plugin-*` Pixi tasks. See
[the canonical architecture](../docs/architecture.md) and
[development workflow](../docs/development.md) for ownership and verification.

## Multi-Link routing configuration

Link Status now renders configured physical links and provides manual selection
per stable ID. Existing LTE/RadioMaster fields remain a compatibility input; add
`RouterLinks` and `RouterConsumers` to the plugin JSON for additional links.
See the [shared router reference](../infra/transport/ground_router/README.md) for
field names, a complete host JSON example, port ownership and recovery policy.
The plugin's default C++ listener is now loopback `14601`, separate from physical
RadioMaster `14550`; the router feeds it from `14602`.

For standalone ownership set `RouterMode` to `Standalone`, configure the
loopback management endpoint (default `127.0.0.1:14610`), and connect native MP
via UDPCl to the host's `14600`. MP restart then leaves the host running while
the plugin reconnects to status/events and marks stale data explicitly. The UI
can select an enabled link or return to automatic selection; endpoint, consumer,
and policy changes require a host restart. Embedded mode remains the default and
keeps plugin-owned lifetime. Do not start both modes against the same endpoints
or simultaneous CLI processes that bind the same consumer endpoint.

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
