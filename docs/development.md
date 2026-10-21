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
pixi run test-plugin-native-preflight
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

## Manual native simulator acceptance

With Mission Planner closed, an explicitly installed reviewed DLL, and the active
saved profile's router, HID, switch bindings, arrow inputs and configured automatic
HUD video disabled, run:

```powershell
pixi run test-plugin-native-sitl -ExpectedSource <installed-source-sha> -ExpectedPluginSha256 <installed-dll-sha256> -Vehicle Copter
```

This optional Windows check never installs or builds. It requires the reviewed
Mission Planner 1.3.83 installation and the cached Windows ArduCopter simulator
under `Documents/Mission Planner/sitl`. The runner checks the installed DLL hash
and the fixture checks its embedded source identity. The simulator is pinned to
`ArduCopter V4.8.0-dev (b7ba8bca)`, SHA256
`C5D95E95684D3AC0A6FA7315DD23D005D1BFCA81D1CB156EAC113D3C49822D75`.
The cached `copter.parm` is also pinned to SHA256
`5E01345B45D1C6190B28BECE5638BBDD4CF1CCE35E05BBBF480AB24D2B51AA0E`.
`-Vehicle Plane` reuses the finite check with built-in Plane defaults plus the same
synthetic output parameters. Its cached binary is `ArduPlane V4.7.0-dev (32752687)`,
SHA256 `1D7E98968226D0A5C33CA845594B4C153D0B89DDBF48BF8E5EE3C1F350621D27`.
Each result records the vehicle, binary banner/hash, cached defaults hash if used,
and actual host executable version, checked against the reviewed target before launch.
Both use 10 Hz output/status requests: Plane's default 50 Hz scheduler limits
MAVLink streaming to 40 Hz, so a 50 Hz request is invalid. The same 10 Hz test passes
on both pinned simulators; no vehicle-specific retry or fallback is used.
These cached development builds provide simulator evidence, not aircraft qualification.

Each attempt has fresh simulator state and temporary MP XML under
`build/tests/native-sitl`. The runner refuses an existing MP process, starts only
its own simulator/host, uses loopback TCP, and applies synthetic payload settings
in memory. Before either process starts, it strictly loads the selected profile through the
exact installed DLL and rejects those startup owners plus debug popups. No plugin
Init or Start runs during that preflight. The separate native-preflight gate tests
the actual guard with synthetic data and all four primary/backup loading candidates.
Immediately before MP launch, it rechecks the DLL and named preservation set:
`Documents/Mission Planner/config.xml`, `nomad_aircraft_profiles.json` and its `.bak`,
`nomad_config.json` and its `.bak`, and `NOMAD/geofence_config.json` under the MP
local application-data plugin directory. It checks that same set after exit;
this does not claim all host/cache/log state is unchanged.
It does not open USB/serial devices, arm, or write real controller parameters.
The temporary MP XML disables automatic connections, other plugin files, speech
and update checks, uses cached maps and directs host logs to the attempt directory.
Normal host startup/scheduling/close own Init, Loaded, Loop and Exit. Passing also
requires actual MP process exit code zero. The check
observes all five payload behaviors, native timed return, explicit Safe and gimbal
servo outputs using fresh `SERVO_OUTPUT_RAW`/`RELAY_STATUS` plus native command ACKs.
Gimbal submission is checked separately from observed output. Bidirectional motion
checks positive and negative endpoints with native neutral return. ReadOnly and
retired-generation checks assert no outbound output command. A firmware that rejects
a required output/status stream fails before payload actions; inspect the recorded
message ID/native ACK result rather than treating absent relay telemetry as success.
Allowlisted `result.json` retains failures and numeric output observations; raw
telemetry logs/transcripts are not test artifacts. Review `passed` and process
cleanup before proceeding. A timeout requests normal close of only the owned MP.

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
