# Transport configuration

`mavlink_router` contains the onboard serial/IP service configuration. It
forwards flight-controller traffic to onboard clients and the ground station.
`ground_router` is a separate ground-computer process that owns multi-link
selection, failover, and consumer routing. Keep these services at their existing
endpoints; they do not replace one another.

The C++ connection is the MAVSDK transport and the CLI configures it from UDP
endpoint schemes (udp, udpin, udpout). MAVSDK can carry other transports without
changing Vehicle policy, but native serial/TCP support is not yet production
behavior. Validate loop prevention, duplicate handling and link capacity at G3.
Keep real endpoints in ignored configuration. See
[operations](../../docs/operations.md) and [architecture](../../docs/architecture.md).
