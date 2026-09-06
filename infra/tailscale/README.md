# Tailscale integration

Tailscale is an optional deployment network. It may carry MAVLink, a future
client connection, SSH, or video between a companion computer and ground
station. It is not a NOMAD core dependency.

The Python monitors and shell setup helpers are transitional infrastructure used
by the current Edge Core deployment. Keep real auth keys, hosts, and ACLs outside
the repository. Detailed setup is intentionally kept out of the canonical product
docs until a supported deployment requires it.

See [operations](../../docs/operations.md) for the supported boundary and
[development](../../docs/development.md) for the migration policy.
