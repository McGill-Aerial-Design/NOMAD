# Configuration

`nomad.env.example` is the committed transitional runtime template. Copy it to
`config/nomad.env` for local use; the real file is ignored and must never contain
committed secrets.

The target C++ core starts with command-line arguments and sensible defaults.
External configuration remains a deployment concern and must not leak into core
vehicle behavior.

See [operations](../docs/operations.md) and [migration](../docs/migration.md).
