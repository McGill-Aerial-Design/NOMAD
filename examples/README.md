# Examples

mavsdk_connectivity_smoke.cpp demonstrates MAVSDK connect/status only. It is the
qualification consumer of the transport the core uses; it does not qualify
vehicle commands by itself.

Use the non-installed `nomad-qualification` driver and
`scripts/dev/core_sitl_*` runners for direct command-flow qualification against
an isolated SITL instance. The installed CLI uses runtime IPC and is not a
standalone aircraft-control example. See [development](../docs/development.md),
[architecture](../docs/architecture.md), and
[qualification status](../docs/qualification.md). Keep examples small; the
removed Python module pattern is not a template.
