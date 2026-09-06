# Transport configuration

`mavlink_router/main.conf` is the current deployment configuration for routing
flight-controller MAVLink between the companion computer and clients.

It is transitional infrastructure. The C++ core will own its own connection
boundary and support serial, UDP, and TCP without exposing router details to the
`Vehicle` API.

Use placeholders for endpoint values and keep deployment-specific addresses in
local ignored configuration. See [operations](../../docs/operations.md) and
[architecture](../../docs/architecture.md).
