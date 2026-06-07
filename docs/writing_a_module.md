# Writing a Module

NOMAD's plugin system lets you add new capabilities as self-registering modules.
Modules are discovered at startup via the `nomad.modules` Python entry-point group.

## Python module SDK

### Module interface

Every module implements the `NomadModule` protocol:

```python
class NomadModule:
    """Protocol for NOMAD modules."""

    name: str               # unique module identifier (e.g. "my_detector")
    version: str            # semver string
    requires: list[str]     # module names this depends on (optional)

    def configure(self, ctx: "AppContext") -> None:
        """Called during startup. 'ctx' provides access to core services:
           - ctx.state       (global state manager)
           - ctx.mavlink     (MAVLink interface)
           - ctx.nav         (navigation controller)
           - ctx.health      (health monitor)
           - ctx.video       (video stream manager)
        """

    def register_routes(self, app: "FastAPI") -> None:
        """Add FastAPI routes. Called after configure()."""

    async def start(self) -> None:
        """Start background tasks. Called when the app is ready."""

    async def stop(self) -> None:
        """Clean up resources. Called during shutdown."""
```

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

The Mission Planner plugin uses a similar module interface for views:

```csharp
public interface INomadModule
{
    string Name { get; }
    string Version { get; }
    Control CreateView();
}

public class MyCustomView : INomadModule
{
    public string Name => "My Custom View";
    public string Version => "1.0.0";

    public Control CreateView()
    {
        var panel = new UserControl();
        // ... build your UI ...
        return panel;
    }
}
```

New views register themselves through the `ModuleHost` registry:

```csharp
ModuleHost.Register(new MyCustomView());
```

The integration is built on the existing `NOMADViewBase` + view registration
pattern in `NOMADMainScreen.cs`. See `mission_planner/src/Core/` and
`mission_planner/src/Modules/` for reference implementations.
