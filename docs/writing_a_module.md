# Writing a Module

NOMAD's plugin system lets you add new capabilities as self-registering modules.
Modules are discovered at startup via the `nomad.modules` Python entry-point group.

## Python module SDK

### Module interface

A module is any object with a `metadata` attribute and the four lifecycle hooks
below (the `NomadModule` protocol in `edge_core/core/module.py`). The hooks run in
this order and are all synchronous:

```python
from edge_core.core import AppContext, ModuleMetadata

class MyDetectorModule:
    # Identity + gating. `requires` lists other module names that must start
    # first; `enable_flag` is an optional NOMAD_* env flag that turns it on/off.
    metadata = ModuleMetadata(name="my_detector", version="0.1.0")

    def configure(self, ctx: AppContext) -> None:
        """Read config and grab core services from the context, e.g.:
             ctx.require_service("state_manager")
             ctx.require_service("mavlink")
             ctx.require_service("health_monitor")
             ctx.require_service("video_stream_manager")
           ctx.get_config("NOMAD_...") reads configuration."""

    def register_routes(self, app) -> None:
        """Add FastAPI routes/routers. Called after configure()."""

    def start(self) -> None:
        """Start background tasks. Called when the app starts up."""

    def stop(self) -> None:
        """Clean up resources. Called on shutdown, in reverse order."""
```

Most modules subclass `BaseModule`, which supplies no-op defaults so you only
override the hooks you need. Service names are whatever each built-in module
registers with `ctx.register_service(name, obj)` — e.g. `state_manager`,
`mavlink`, `health_monitor`, `video_stream_manager`, `time_sync`.

### Registration

Register your module in your package's `pyproject.toml`:

```toml
[project.entry-points."nomad.modules"]
my_detector = "my_package.module:MyDetectorModule"
```

### Enable / disable

Modules are controlled via `config/nomad.env`:

```bash
# Allow-list — only these modules load (comma-separated)
NOMAD_MODULES=my_detector,payload

# Or disable individual modules by name
NOMAD_ENABLE_MY_DETECTOR=false
```

If no allow-list is set, all discovered entry-point modules are loaded.
If `NOMAD_MODULES` is set, only the listed modules load (subject to their
`requires` dependencies being satisfied).

### Sample module

A minimal third-party module is provided at `examples/sample_module/`. It
subclasses `BaseModule` (which supplies no-op lifecycle defaults) and declares
its identity with a `ModuleMetadata` instance:

```python
from typing import Any

from edge_core.core import AppContext, BaseModule, ModuleMetadata


class SampleModule(BaseModule):
    metadata = ModuleMetadata(
        name="sample",
        version="0.1.0",
        description="Minimal example NOMAD module.",
    )

    def __init__(self) -> None:
        self.started = False

    def configure(self, ctx: AppContext) -> None:
        self.ctx = ctx

    def register_routes(self, app: Any) -> None:
        from fastapi import APIRouter

        router = APIRouter(prefix="/api/sample", tags=["sample"])

        @router.get("/ping")
        async def ping() -> dict[str, Any]:
            return {"module": self.metadata.name, "started": self.started, "reply": "pong"}

        app.include_router(router)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False
```

## C# module pattern

The Mission Planner plugin mirrors the same idea: an `INomadModule` (in
`mission_planner/src/Core/`) has `Metadata` plus `Configure` / `GetViews` /
`Start` / `Stop`. Instead of returning a single control, a module contributes one
or more sidebar entries as `NomadViewDescriptor`s. Derive from `NomadModuleBase`
for no-op defaults:

```csharp
using System.Collections.Generic;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

public class MyCustomModule : NomadModuleBase
{
    public override NomadModuleMetadata Metadata =>
        new NomadModuleMetadata { Name = "my_custom", Version = "1.0.0" };

    public override IEnumerable<NomadViewDescriptor> GetViews()
    {
        // id, button label, header title, sidebar section, view factory
        yield return NomadViewDescriptor.View(
            "my_custom", "My View", "My View", "TOOLS",
            () => new MyCustomControl());
    }
}
```

Register the module with a `ModuleHost` before the NOMAD screen is shown; the host
resolves dependencies and the screen builds the sidebar from the descriptors:

```csharp
moduleHost.Register(new MyCustomModule());
```
