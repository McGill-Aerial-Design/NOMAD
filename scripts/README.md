# Scripts

Scripts are deployment, setup, simulation, and development utilities. They are
not the NOMAD vehicle core.

- `scripts/dev/` contains local diagnostics and the size/complexity report.
- `scripts/setup/` provisions deployment environments.
- `scripts/build/` contains plugin and image helpers.
- `scripts/services/` and `scripts/nomad` manage the transitional service stack.
- `scripts/hardware/` contains explicit hardware utilities.

The target CLI is the C++ `nomad` executable. The current service dispatcher is
kept only until the migration deletes the Python multi-service runtime.

Use the canonical [development](../docs/development.md),
[operations](../docs/operations.md), and [migration](../docs/migration.md)
documents instead of adding another service guide.
