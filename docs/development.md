# Development

The product is a C#/.NET Framework 4.8 DLL for the reviewed Mission Planner host
in `scripts/release/mission-planner-target.json`. Windows builds require Visual
Studio/MSBuild, its Roslyn compiler and the .NET Framework targeting pack, plus
that installed host's reference assemblies. Python/Pixi serves development and simulator tooling only.
The optional aircraft firmware has separate [native build/check instructions](native-termination.md).

## Build without installation

```powershell
pixi run build-plugin-only
```

The script cleans/builds Release, writes source SHA/MP target identity under
`build/release/` and leaves `mission_planner/src/bin/Release/NOMADPlugin.dll`.
The plugin release version is defined once in `scripts/build/build_plugin_windows.ps1`
and embedded in the assembly, file, informational and Mission Planner display versions.
All host references are non-copy-local. The artifact embeds the pinned CPU vision
runtime, model and their notices, and the script never writes an MP installation.
`prepare_vision.ps1` verifies the archive and individual asset SHA256 values in its
build cache before compilation. The installed plugin performs no downloads. Direct IDE builds use
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
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build/test_plugin_storage.ps1
pixi run test-plugin-geometry # coordinate validation and actual native Fence AddCommand staging
pixi run test-plugin-fence # actual native editor mode switching and synthetic parameter read/write peer
pixi run test-plugin-audio
pixi run test-plugin-version
pixi run test-ground-router-logic
pixi run test-plugin-gimbal-ui
pixi run test-plugin-ui
pixi run test-plugin-simulation
pixi run test-plugin-router-ui
pixi run test-plugin-video
pixi run test-plugin-vision
pixi run test-plugin-termination
```

Configuration validation includes the aircraft-profile suite; use
`pixi run test-aircraft-profiles` alone for a focused profile check.
An optional local capture probe uses an existing GStreamer runtime without opening a camera:

```powershell
powershell -NoProfile -File scripts/build/test_plugin_video.ps1 -CaptureProbeRuntime <existing-gstreamer-prefix>
```

The prefix contains `bin/` and `lib/gstreamer-1.0/`. This check parses the actual pipeline
in NULL state, verifies UTF-8 device-path containment and passively enumerates Windows
capture interfaces. Streaming, format negotiation and unplug/busy behavior require a
separate operator-selected camera check; the probe never enters READY or PLAYING.
The storage check uses fresh CLR processes for default, custom and invalid host
user-data settings. It checks the default path without opening operator files;
custom profile save/backup/load and logging use owned directories. An invalid
explicit custom root must fail without falling back to operator storage.
Termination uses the built DLL and actual MP assemblies with an owned synthetic loopback peer, without
installing the plugin or launching MP. It checks native capability discovery, readbacks, command ACKs,
fresh heartbeat naming and target retirement; it does not qualify aircraft termination or touchdown.
Gimbal includes native target/stale-input/fault tests. Payload covers native
mapping/ACK faults, trim matching, pending Safe draining, confirmations, HID edges
and controller repeat lifetime. Named input checks cover button/pair and two/three-position
axis decoding, fresh centered gimbal/position input, stable binding authorization,
neutral-separated confirmations across list reorder, conflicting input Safe and
retired asynchronous batches. Listen checks use synthetic snapshots for fresh
button/axis detection, held/noisy baseline rejection, timeout, loss and connection
opening; they do not acquire an operator device. Router covers parser, dedup/failover, one MP egress,
parameter pinning and ownership disposal. Geometry/video/profile tests retain
invalid-input and lifecycle fault cases.

The plugin embeds the ONNX Runtime 1.24.4 DirectML build: its CPU provider runs the detector and
DirectML (pinned redistributable 1.15.4, loaded by path only for the GPU tracker) serves the opt-in GPU
Object tracker. Re-exporting the model does not reproduce the tested graph bytes, so the tested export
is a pinned build input: `prepare_vision.ps1` downloads release asset
[`tracker-model-efficienttam-ti512-1`](https://github.com/McGill-Aerial-Design/NOMAD/releases/tag/tracker-model-efficienttam-ti512-1)
(EfficientTAM licence and change notice inside; an identical mirror sits on the YoussGm3o8 fork), verifies
the archive and every file hash, and every build embeds it.
Mission Planner's GDAL loads a 2017 Visual C++ runtime before plugins start, and Windows reuses a loaded DLL
by name, so ONNX Runtime (built with toolset 14.44) failed with error 1114 inside the real host.
`prepare_vision.ps1` disables ORT's embedded isolation manifest (resource ID 2 → 3, pinned hashes), and
`VisionRuntime` loads ORT in an activation context that redirects it to a private copy of the installed
runtime (System32, 14.44 or later, else a clear error). The vision gate checks detection, and with
`NOMAD_TEST_TAM_MODEL` the GPU tracker, in a fresh process after loading GDAL's runtime first.
Set `NOMAD_TEST_TAM_MODEL` to the export folder to add the GPU tracker checks (NVIDIA adapter selection,
the setting creating the tracker from the embedded model, moving positives, adjacent occluders, a
simultaneous twin, removal without memory writes, three-frame return, a small turning object, reselection
and a long-loss lookalike); without it the gate prints an explicit SKIP. The setting, Area isolation and
the no-model refusal are checked in every build.
Vision checks run the pinned CPU model and verify preprocessing, decoding,
invalid profiles/tensors, native load/inference cancellation and recovery. Video
checks exercise matching frame ownership, selection clone replacement, cancellation/failure,
native tracker loss, bounded angle steps, click/drag geometry, observations and manual/source
retirement. The UI clears an unready initialization after two seconds. Readiness still
requires a fresh matching result; stale results and timeout admit no gimbal commands.
Tracking response checks independently project a world target through delayed camera
motion, aircraft yaw reversals and additive/opposing translation. They verify
downward-camera geometry, mirrored-image calibration, rate bounds and settling.
The Object orbit ray checks use 5/15 m altitude and 5/10 m/s at a 20 m radius, with
p95 image-center error at most 5% and peak at most 8%; the stricter 3.5% peak goal
is reported separately. These are controller tests, not rendered detector qualification.
Busy-request coverage checks the next independently calculated angular correction.
Object checks follow moving/scaling texture and a deforming face or rotating thin shaft
under independent background changes. They require verified positive coverage, bounded aim
error and actual working-view updates, then reject an aiming-point occluder after learning.
Removed shafts cannot be replaced by unchanged background. A sub-patch HD animal blob turning
side-on to head-on must stay verified with whole-object centre/box overlap; its removal, a stale gap
return without three matches, and a nearby indistinguishable twin must not be accepted. Tracking retains
the whole-object centre for Object and the immutable point for Area. Off-centre padded
selections and exact uniform shafts cover both initialization and working-view promotion.
The photographed watering-can outline is independently annotated in the first panorama;
camera-ray projection supplies later whole-object extents. Object uses an 8px centre
tolerance; Area retains its 5px canonical-point tolerance. Neither changes wrong-identity
or three-frame recovery requirements. Global return checks retain three-frame confirmation
and duplicate-placement rejection. Area search prioritizes the optional coarse hint,
then falls back globally in the same frame and independently checks another placement.
Each frame performs at most five homography fits, each bounded to 750 RANSAC iterations,
with at most 1000 descriptors per image at a 640-pixel maximum dimension. Native tests
measure the fit count and latency while a persistent wrong hint leaves unique visual
verification and gimbal command admission intact. A verified mismatch beyond that
image-scaled search neighborhood disables the ground prior for the selection.
Optional `-TrackingFrames` and `-TrackingGroundTruth`
arguments to `test_plugin_vision.ps1` require 120 aligned PNG/ground-truth frames and
all 119 subsequent updates without reseeding. Keep external benchmark media in ignored
test storage with its provenance. Detector scores and synthetic centering do not
establish performance on another camera, scene or physical mount.

Rendered-image replays need the separate post-inference semantic scorer;
positive-coverage replay alone does not establish pixel accuracy. Original receipt gaps,
returned frame-owned boxes and all wrong admitted points remain in the evidence.

## Documentation screenshots

Build the plugin, then render its actual controls from a fresh Windows PowerShell
process. No computer-use automation or desktop capture is needed:

```powershell
powershell -STA -NoProfile -ExecutionPolicy Bypass -File scripts/docs/capture_screenshots.ps1
```

The script writes the Dashboard, Fence, settings, input editor and Video PNGs to `docs/images/`
and capture metadata/DLL/image checksums under
ignored `build/screenshots/`. It uses explicit example profiles, reserved `.invalid`
URLs and an unstarted notification service. The dashboard has no video input;
settings are drafts and the router remains stopped. It neither starts the plugin
nor opens an aircraft/camera connection or saves operator settings. The settings
constructor enumerates HID device names but does not acquire or start them.
Captures use one shared offscreen WinForms render host that cannot activate; each render
checks the foreground window is unchanged. Only captured files enter the image manifest.
These are control renders, not a connected Mission Planner session.
Review them visually and retain those distinctions in captions.

To render the setup window without invoking installation:

```powershell
powershell -STA -NoProfile -File scripts/docs/capture_installer_screenshot.ps1 -InstallerPath build/installer/NOMAD-0.5.0-Setup.exe
```

## Manual native simulator acceptance

The optional NOMAD **Testing & Simulation** page loads editable scenarios and starts a prepared
WSL simulator workspace. Follow `scripts/simulation/README.md` in the source checkout
for prerequisites, backend limits and configuration. Its focused Windows gate tests inactive
controls, native connection admission and owned process lifecycle without launching MP or WSL.
Python launcher/world checks run on CI with simulator dependencies mocked; visible renderer and
actual SITL/native-decoder probes are separate local checks, not hosted CI or flight qualification.

### Owned simulation test harness

The development harness under `scripts/simulation/` uses an owned copy of reviewed MP
assemblies, isolated settings/cache/log paths and fresh simulator state. It never starts
the installed MP application, installs the plugin or opens an operator aircraft/device.
Its controlled native-API cohort exercises the actual MP connection/reader and NOMAD
controllers. It does not establish full MP application or plugin startup/shutdown lifecycle.

```powershell
pixi run build-plugin-only
pixi run test-simulation-host
powershell -NoProfile -File scripts/simulation/run_headless.ps1
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -LastLetter
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Scene photo-board -VideoCase Detect
```

The simulator runs require the separately prepared owned WSL workspace and pinned
sources. `scripts/simulation/README.md` describes acquisition/build commands, actual
supported cases and known qualification gaps.
Rendered correctness is an explicit gate: failed positive coverage, unavailable truth or
unavailable capture timing must not be described as a passing end-to-end qualification.

`test-simulation-host` is the fast Windows CI probe; it starts no MP application, simulator
or connection. Python CI tests source/binary manifests, the independent geometry oracle,
RTP byte preservation and Linux process-group ownership. Rendered and real-SITL runs remain
local opt-in checks with owned process/port cleanup and preservation hashes in their results.

### Installed-host manual acceptance

With Mission Planner closed, an explicitly installed reviewed DLL, and the active
saved profile's router, enabled input bindings, arrow inputs and configured automatic
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
actual Windows product gates and uploads the DLL and offline setup; Linux runs Python/docs checks.
The separate [firmware workflow](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/.github/workflows/firmware.yml) runs native
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
