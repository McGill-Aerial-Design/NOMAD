# Development

The product is a C#/.NET Framework 4.8 DLL for the reviewed Mission Planner host
in `scripts/release/mission-planner-target.json`. Windows builds require Visual
Studio/MSBuild, its Roslyn compiler and the .NET Framework targeting pack, plus
that installed host's reference assemblies. Python/Pixi serves checks/docs only.
No C++ toolchain, MAVSDK, Docker, backend service or independent CLI is needed.

## Build without installation

```powershell
pixi run build-plugin-only
```

The script cleans/builds Release, writes source SHA/MP target identity under
`build/release/` and leaves `mission_planner/src/bin/Release/NOMADPlugin.dll`.
All host references are non-copy-local. The artifact contains no host/dependency
binaries and the script never writes an MP installation. Direct IDE builds use
fallback development metadata if generated identity is absent; record the actual
checked source and local changes with any artifact. Installation is a separate
explicit [manual closed-MP operation](operations.md#manual-plugin-installation).

## Product checks

Run the focused check first, then relevant gates once for the final coherent diff.
The host-dependent gates require the actual reviewed MP assemblies. UI checks use
isolated Windows/STA fixtures; router checks bind loopback test sockets. Profile
fixtures use temporary paths and must not touch real operator files.

```powershell
pixi run test-plugin-build-only
pixi run build-plugin-only
pixi run lint-plugin
pixi run test-plugin-gimbal
pixi run test-plugin-payload
pixi run test-plugin-config-validation
pixi run test-plugin-geometry
pixi run test-plugin-audio
pixi run test-plugin-version
pixi run test-ground-router-logic
pixi run test-plugin-gimbal-ui
pixi run test-plugin-router-ui
pixi run test-plugin-video
```

Configuration validation includes the aircraft-profile suite; use
`pixi run test-aircraft-profiles` alone for a focused profile check.
Gimbal includes native target/stale-input/fault tests. Payload covers native
mapping/ACK faults, trim matching, pending Safe draining, confirmations, HID edges
and controller repeat lifetime. Router covers parser, dedup/failover, one MP egress,
parameter pinning and ownership disposal. Geometry/video/profile tests retain
concrete failure cases rather than obsolete service/IPC assertions.

## Documentation and tooling

```text
pixi run lint
pixi run format-check
pixi run test-python
pixi run docs-build
pixi run precommit
```

Python tests retain document links and source requirement provenance. CI runs the
actual Windows product gates and uploads only the DLL; Linux runs Python/docs checks.
Old C++/MAVSDK/SITL/release jobs are removed rather than reporting fabricated success.
Repository protection is external state: obsolete required contexts must be aligned
under separate authorization before merge; source edits do not change protection.

Preserve ignored builds, operator settings and other worktrees. Plans, handoffs and
run histories belong in ignored `local/`. Use [qualification](qualification.md) to
separate software checks from hardware/competition acceptance; no build/test deploys.
