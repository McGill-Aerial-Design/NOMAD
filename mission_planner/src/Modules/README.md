<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Mission Planner — Modules

Feature modules that plug into the NOMAD screen through the
[module SDK](../Core/README.md). Each module bundles a related set of sidebar
views (and/or actions) and declares its own metadata and enable flag.

## Example module

- **[`ExampleModule.cs`](ExampleModule.cs)** — a minimal, working module. It adds
  an "Example Module" page and a sample action to the NOMAD sidebar, reads the
  plugin config from the shared context, and is gated by the
  `NOMAD_PLUGIN_EXAMPLE_MODULE` env flag (on by default). Use it as the template
  for your own modules.

## Adding your own module

1. Create `Modules/<YourModule>.cs` deriving from `NomadModuleBase` (see
   [`ExampleModule.cs`](ExampleModule.cs) and the [SDK README](../Core/README.md)
   for the shape). The project compiles every `.cs` under `src/`, so no
   `NOMADPlugin.csproj` edit is needed.
2. Register it in `NOMADPlugin.BuildModuleHost()` with `host.Register(...)`.
3. Gate it with an `EnableFlag` so builds that don't want it are unaffected.
