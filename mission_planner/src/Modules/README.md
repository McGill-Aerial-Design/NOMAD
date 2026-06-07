<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Mission Planner — Modules

Feature modules that plug into the NOMAD screen through the
[module SDK](../Core/README.md). Each module bundles a related set of sidebar
views (and/or actions) and declares its own metadata and enable flag.

## Reference module

- **[`AEAC2026/`](AEAC2026/AEAC2026Module.cs)** — the AEAC-2026 competition view
  set, moved to the `AEAC2026` branch on GitHub. Kept in the source tree here for
  reference; not enabled by default in this branch.

## Adding your own module

1. Create `Modules/<YourModule>/<YourModule>Module.cs` deriving from
   `NomadModuleBase` (see the [SDK README](../Core/README.md) for the shape).
2. Add the `.cs` file to `NOMADPlugin.csproj` (`<Compile Include="..." />`) —
   the project uses an explicit file list, not globbing.
3. Register it in `NOMADPlugin.BuildModuleHost()` with `host.Register(...)`.
4. Gate it with an `EnableFlag` so builds that don't want it are unaffected.
