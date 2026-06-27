<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Mission Planner — Module SDK

This folder is the **module SDK** for the NOMAD Mission Planner plugin. It lets
self-contained feature sets contribute sidebar views to the NOMAD screen without
editing `NOMADMainScreen`, mirroring the Jetson-side `edge_core.core` module
system so the two ends of NOMAD share one mental model.

| Mission Planner (C#) | Jetson (`edge_core.core`, Python) |
| --- | --- |
| `INomadModule` / `NomadModuleBase` | `NomadModule` / `BaseModule` |
| `NomadModuleMetadata` | `ModuleMetadata` |
| `NomadModuleContext` | `AppContext` |
| `ModuleHost` | `ModuleRegistry` |
| `NomadViewDescriptor` | FastAPI route registration |

## Pieces

- **`NomadModuleContext`** — dependency container + feature-flag lookup handed to
  a module at `Configure()` time. Resolve services by type (`Get<T>()`,
  `Require<T>()`) or name. `IsEnabled(flag, default)` gates optional behavior.
- **`INomadModule` / `NomadModuleBase`** — a module declares `Metadata` (name,
  version, `Requires`, `EnableFlag`, `EnabledByDefault`), resolves dependencies in
  `Configure()`, and yields `NomadViewDescriptor`s from `GetViews()`. Optional
  `Start()` / `Stop()` run in dependency order (stop reversed).
- **`NomadViewDescriptor`** — one sidebar entry: either an in-place **View**
  (created lazily via a factory and cached) or an **Action** (e.g. opens a
  floating window). Carries `Id`, `ButtonText`, `Title`, and a `Section` heading.
- **`INomadView`** *(optional)* — lifecycle hooks (`OnActivated` /
  `OnDeactivated`) for a view. Views that only need periodic refreshes can
  implement the existing `IUpdatableView` instead — the host's update timer drives
  whichever the active view implements.
- **`ModuleHost`** — registers modules, filters by allow-list + enable flags,
  resolves a dependency order (topological sort; throws on cycles, missing or
  disabled dependencies), and aggregates their view descriptors.

## Writing a module

```csharp
using System.Collections.Generic;
using NOMAD.MissionPlanner.Core;

public sealed class MyModule : NomadModuleBase
{
    private NOMADConfig _config;

    public override NomadModuleMetadata Metadata => new NomadModuleMetadata
    {
        Name = "my-module",
        EnableFlag = "NOMAD_MODULE_MY",   // opt-in via env var / setting
        EnabledByDefault = false,
    };

    public override void Configure(NomadModuleContext context)
        => _config = context.Require<NOMADConfig>();

    public override IEnumerable<NomadViewDescriptor> GetViews()
    {
        yield return NomadViewDescriptor.View(
            "MyView", "My View", "My Feature", "TOOLS",
            () => new MyUserControl(_config));
    }
}
```

Register it where the plugin builds its host (`NOMADPlugin.BuildModuleHost`):

```csharp
host.Register(new MyModule());
```

## How it plugs into the screen

`NOMADPlugin.BuildModuleHost()` builds a `ModuleHost`, registers the available
modules, calls `Configure()` + `StartAll()`, and publishes it via
`NOMADMainScreen.SetStaticModuleHost(host)`. When the screen builds its sidebar,
it **appends** each module's descriptors after the built-in entries — so modules
extend the screen rather than replace it. With no host (or no enabled modules),
only the built-in entries appear.

The bundled [`../Modules/ExampleModule.cs`](../Modules/ExampleModule.cs) is a
working module wired in this way: it adds an "Example Module" page and action to
the sidebar (gated by `NOMAD_PLUGIN_EXAMPLE_MODULE`, on by default).
