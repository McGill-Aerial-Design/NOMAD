# Development

The product is a C#/.NET Framework 4.8 DLL for the reviewed Mission Planner host
in `scripts/release/mission-planner-target.json`. Windows builds require Visual
Studio/MSBuild, its Roslyn compiler and the .NET Framework targeting pack, plus
that installed host's reference assemblies. Python/Pixi serves checks/docs only.
The optional aircraft firmware has separate [native build/check instructions](native-termination.md).

## Build without installation

```powershell
pixi run build-plugin-only
```

The script cleans/builds Release, writes source SHA/MP target identity under
`build/release/` and leaves `mission_planner/src/bin/Release/NOMADPlugin.dll`.
The plugin release version is defined once in `scripts/build/build_plugin_windows.ps1`
and embedded in the assembly, file, informational and Mission Planner display versions.
All host references are non-copy-local. The artifact contains no host/dependency
binaries and the script never writes an MP installation. Direct IDE builds use
fallback development metadata if generated identity is absent; record the actual
checked source and local changes with any artifact. Installation is a separate
explicit [closed-MP operation](operations.md#plugin-installation).

Build and check the single offline Windows setup after building the DLL:

```powershell
pixi run build-installer
pixi run test-plugin-installer
```

Setup is a separate .NET Framework 4.8 executable compiled with the same Roslyn toolchain.
It embeds the exact DLL, version/hash and notices, requests Windows administrator consent,
and writes under `build/installer/` without installing anything. Its tests use owned temporary
hosts for actual install/update/backup, invalid input, process/file locks, reparse guards and
download-marker removal; package tests inspect the actual EXE resources, manifest and UI.
Profiles and aircraft settings are outside its write scope.

## Product checks

Run the focused check first, then relevant gates once for the final coherent diff.
The host-dependent gates require the actual reviewed MP assemblies. UI checks use
isolated Windows/STA fixtures; router checks bind loopback test sockets. Profile
fixtures use temporary paths and must not touch real operator files.
UI checks exercise owned inactive menus, actual host theme opt-outs and sampled Shift
gesture state without desktop input injection. Video checks cover named-input round trips and strict nested configuration, independent
aircraft/settings drafts, both receiver pipeline paths, source replacement and stale
frame disposal. They use fake pipelines and do not open operator camera feeds.

```powershell
pixi run test-plugin-build-only
pixi run build-plugin-only
pixi run lint-plugin
pixi run test-plugin-gimbal
pixi run test-plugin-payload
pixi run test-plugin-native-preflight
pixi run test-plugin-config-validation
pixi run test-plugin-geometry # coordinate validation and actual native Fence AddCommand staging
pixi run test-plugin-fence # actual native editor mode switching and synthetic parameter read/write peer
pixi run test-plugin-audio
pixi run test-plugin-version
pixi run test-ground-router-logic
pixi run test-plugin-gimbal-ui
pixi run test-plugin-ui
pixi run test-plugin-router-ui
pixi run test-plugin-video
pixi run test-plugin-termination
```

Configuration validation includes the aircraft-profile suite; use
`pixi run test-aircraft-profiles` alone for a focused profile check.
Termination uses the built DLL and actual MP assemblies with an owned synthetic loopback peer, without
installing the plugin or launching MP. It checks native capability discovery, readbacks, command ACKs,
fresh heartbeat naming and target retirement; it does not qualify aircraft termination or touchdown.
Gimbal includes native target/stale-input/fault tests. Payload covers native
mapping/ACK faults, trim matching, pending Safe draining, confirmations, HID edges
and controller repeat lifetime. Router covers parser, dedup/failover, one MP egress,
parameter pinning and ownership disposal. Geometry/video/profile tests retain
invalid-input and lifecycle fault cases.

## Documentation screenshots

Build the plugin, then render its actual controls from a fresh Windows PowerShell
process. No computer-use automation or desktop capture is needed:

```powershell
powershell -STA -NoProfile -ExecutionPolicy Bypass -File scripts/docs/capture_screenshots.ps1
```

The script writes five PNGs to `docs/images/` and capture metadata/DLL/image checksums under
ignored `build/screenshots/`. It uses explicit example profiles, reserved `.invalid`
URLs and an unstarted notification service. The dashboard has no video input;
settings are drafts and the router remains stopped. It neither starts the plugin
nor opens an aircraft/camera connection or saves operator settings. The settings
constructor enumerates HID device names but does not acquire or start them.
Captures are WinForms control renders, not a connected Mission Planner session.
Review them visually and retain those distinctions in captions.

To render the setup window without invoking installation:

```powershell
powershell -STA -NoProfile -File scripts/docs/capture_installer_screenshot.ps1 -InstallerPath build/installer/NOMAD-0.3.1-Setup.exe
```

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
the actual guard with synthetic data and both current collection loading candidates.
Immediately before MP launch, it rechecks the DLL and named preservation set:
`Documents/Mission Planner/config.xml`, and `nomad_aircraft_profiles.json` and its `.bak`
under the MP local application-data plugin directory. It checks that same set after exit;
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
The separate [firmware workflow](../.github/workflows/firmware.yml) runs native
qualification and prepares source-bound CubeOrangePlus packages. Its focused
packager integrity tests are included in `test-python`; those unit tests do not
substitute for the full [native build/release procedure](native-termination.md#build-and-release-automation).
Repository protection is external state; required checks and reviews must be
verified on the actual pull request before merge.

Preserve ignored builds, operator settings and other worktrees. Plans, handoffs and
run histories belong in ignored `local/`. Use [qualification](qualification.md) to
separate software checks from hardware/competition acceptance; no build/test deploys.

Native Fence draft tests skip host constructors and use real MP editor methods and grid types; only the selected-port global and output generation are isolated. They assert zero transport writes and cache preservation.
The native Fence gate also constructs the actual editor with isolated settings/cache-only
maps and uses the built DLL plus a synthetic loopback parameter peer. It checks unsaved
Mission/Fence/Rally rows, metadata, addressed writes/readbacks, armed/stale/retired refusal
and stop-on-failure. It neither starts Mission Planner nor connects to an aircraft.
Controlled-termination cases exercise disabled/unconfigured view edits, native discovery,
both vehicle descent limits, required commissioning fields and stop-on-mismatch/retirement;
they assert that no ordinary landing, RC-channel or GCS-loss parameter is written.
