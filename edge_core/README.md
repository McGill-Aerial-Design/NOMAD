# Transitional Edge Core

`edge_core/` is the current Python/FastAPI implementation while NOMAD migrates to
the C++ core described in [the architecture](../docs/architecture.md) and
[the migration plan](../docs/migration.md).

It currently provides the hardware-free sim path, REST routes, Python MAVLink
services, safety decision tests, health/video services, and the ROS HTTP bridge.
These are migration references, not new extension points.

Use this path only for transitional checks:

```bash
pixi run dev
pixi run test-fast
```

Do not add new module registries, service layers, REST vehicle commands, or
parallel vehicle logic here. Port behavior to the C++ core or place genuine
experimentation in `python/`.
