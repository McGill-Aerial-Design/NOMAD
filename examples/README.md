# Examples

mavsdk_connectivity_smoke.cpp demonstrates MAVSDK connect/status only. It is the
qualification consumer of the transport the core uses; it does not qualify
vehicle commands by itself.

Use the core CLI and scripts/dev/core_sitl_* runners for current command-flow
examples after isolated SITL startup is qualified. See
[development](../docs/development.md) and
[MAVSDK adoption](../docs/mavsdk-adoption.md).
Keep examples small; the removed Python module pattern is not a template.
