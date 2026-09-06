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
pixi run build-plugin
```

Pure helper checks are available through the `test-plugin-*` Pixi tasks. See
[the canonical architecture](../docs/architecture.md) and
[development workflow](../docs/development.md) for ownership and verification.
