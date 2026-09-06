# Infrastructure

`infra/` contains deployment support that is outside the NOMAD core:

- `systemd/` service templates used by the transitional runtime;
- `transport/mavlink_router/` routing configuration;
- `tailscale/` optional network monitoring and setup scripts;
- `mediamtx.yml` and log rotation configuration.

The target deployment is one C++ core process plus only the adapters a vehicle
needs. Keep network, container, VPN, and service-manager choices out of the core.

See [operations](../docs/operations.md) for the canonical deployment model and
[migration](../docs/migration.md) for deletion/consolidation gates.
