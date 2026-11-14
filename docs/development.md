# Development

Read [architecture](architecture.md) for code ownership and Mission Planner APIs.
Builds/tests never install the plugin or open operator aircraft connections.
Run commands from the repository root. Preserve operator profiles and ignored evidence.

## Build without installation

Windows needs the reviewed Mission Planner host from
[target metadata](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/release/mission-planner-target.json),
Visual Studio/MSBuild with Roslyn and the .NET Framework 4.8 targeting pack, plus Pixi.
The build checks the host version in `%ProgramFiles(x86)%\Mission Planner` before preparing assets or compiling.
Both plugin and worker use that verified folder, including the worker's copied support libraries.
Python/Pixi supports development, documentation and simulator tools.

```powershell
pixi run build-plugin-only
pixi run build-installer
pixi run test-plugin-installer
```

The first command produces `mission_planner/src/bin/Release/NOMADPlugin.dll` and
`build/release/plugin-build.json`, binding its hash to the base commit, product
source hashes (including compiled XAML), reviewed host and every embedded media bundle file. The release version lives in
`scripts/build/build_plugin_windows.ps1`. Host references are non-copy-local.
The installer build verifies that proof against the current source before
embedding the DLL and notices into `build/installer/NOMAD-version-Setup.exe`.
The script builds the media worker before embedding it in the plugin. Direct IDE
builds require a prepared bundle; use the script for checked artifacts.

`prepare_vision.ps1` verifies pinned archives and every asset before compilation.
The DLL embeds a hash-verified worker executable, configuration and existing private
managed dependencies. The worker embeds ONNX Runtime 1.24.4 (DirectML build), DirectML 1.15.4,
OpenCvSharp, YOLOX Nano, the EfficientTAM Ti512 export and license notices.
Detection uses the CPU provider; GPU Object tracking is explicit per camera.
The worker also embeds the pinned 0.7 MB ViTTrack CPU template model and Apache license
for small or poorly separated selections; this requires no GPU or additional runtime.
The installed plugin extracts verified assets and performs no downloads.
`VisionRuntime` retains native C-runtime isolation inside the child (GStreamer can
also load a conflicting C runtime);
it requires an installed System32 Visual C++ runtime 14.44 or later.
Preserve that activation-context/load order when changing native dependencies.

Installation is a separate [closed-MP operation](operations.md#plugin-installation).

## Product checks

Run the focused check first, then relevant gates once for the final diff.
[Windows CI](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/.github/workflows/csharp.yml)
is the complete executable gate list. Host-dependent checks require reviewed MP
assemblies; UI checks run in isolated STA fixtures and router checks use loopback sockets.

| Change | Commands (`pixi run <task>`) |
|---|---|
| Build/tooling | `test-plugin-build-only`, `build-plugin-only`, `lint-plugin` |
| Profiles/configuration/storage | `test-plugin-config-validation` (includes aircraft profiles, storage, setup editors and health observations); `test-aircraft-profiles` for a focused run |
| Gimbal/payload/HID | `test-plugin-gimbal`, `test-plugin-payload`, `test-plugin-gimbal-ui` |
| UI and routing | `test-plugin-ui`, `test-plugin-router-ui`, `test-ground-router-logic` |
| Fence/editor | `test-plugin-geometry`, `test-plugin-fence` |
| Video/vision | `test-plugin-video`, `test-plugin-vision` |
| Alerts/version/termination | `test-plugin-audio`, `test-plugin-version`, `test-plugin-termination` |
| Simulation | `test-simulation-host`, `test-plugin-simulation` |
| Installer | `build-installer`, `test-plugin-installer` (includes package/resource checks) |

Fixtures establish known state, perform one action and observe independent results
with deadlines and diagnostic failures. Retain invalid, boundary and fault cases.
Use temporary storage and owned resources; no test should rewrite real profiles.
Report passed, failed, skipped and unavailable separately.

The video gate also starts the real embedded child with synthetic capture/inference
fixtures. It holds constructors, initialization, update, capture and teardown, checks
UI/preview deadlines at 30/60/120 FPS, injects process loss and verifies one plain-only
recovery, shared receiver ownership, native parent-module exclusion and actual worker
CPU inference. `-Filter <case substring>` runs a focused video case. Fixtures never
open a camera or real aircraft. The parent pixel pool is bounded to 512 MiB allocated
and 128 MiB idle; frame dimensions retain the existing 16,384/64M-pixel ceilings.

The private worker checks heartbeat/frame progress at 2 s, selection initialization
at 5 s, cold media runtime/model preparation at 120 s and cleanup at 2 s. Parent Stop never waits
on these deadlines; background completion confirms resource retirement or owned
process exit and closes its process handle. Startup, heartbeat and control/handshake
deadlines do not require an available parent thread-pool worker. The video gate
exhausts a two-thread pool during startup, heartbeat and a stalled resource handshake,
then checks the existing one-recovery path and final handle retirement.
Missing upstream samples keep the capture polling and do not trigger
a restart loop. Cleanup and extraction preserve operator files.

The vision gate checks the pinned CPU detector and native runtime load order.
Set `NOMAD_TEST_TAM_MODEL` to a prepared EfficientTAM export folder for the optional
NVIDIA/GPU cohort; otherwise that cohort explicitly skips. Optional
`-TrackingFrames` / `-TrackingGroundTruth` arguments accept an external 120-frame
replay with aligned truth. Keep private media/provenance in ignored storage.
Synthetic controller checks and positive replay coverage do not establish real-scene accuracy.

For an explicit CPU performance check on the target computer:

```powershell
powershell -NoProfile -File scripts/build/test_plugin_vision.ps1 -BenchmarkTracking
```

This runs the production CPU trackers against moving/scaling Object, thin HD Object,
perspective Area and rendered/decode deer fixtures. Each cohort warms up for eight updates, then requires
120/120 independently correct aims and p95 update time at or below 33.3 ms. Reports
include median, p95, maximum and CPU time; initialization and fixture rendering are
outside the update measurement. It is separate from the ordinary CI gate because
timing depends on the target CPU and contention. The video gate also reports actual
CPU tracker latency across the private worker IPC on HD frames. Neither check measures
camera exposure, network latency, native ACK rate or physical mount response.

The video gate also exercises the position-servo feedback controller against an
independent PWM plant with moving targets, aircraft turns, camera delay, mechanical
lag, deadband, reversed outputs and angle-calibration error. A loopback peer using
the reviewed MP helpers delays the initial servo ACKs, omits later ACKs and measures
fresh-frame command cadence. Busy, target-loss, recovery and Stop cases check state
reset and prevent replay. These checks do not qualify a physical servo's tuning.

The video gate's optional `-CaptureProbeRuntime <existing-gstreamer-prefix>` parses
the pipeline in NULL state and enumerates device metadata. It never starts a camera.
Real capture, exposure latency and mount response need separate acceptance.

## Manual native simulator acceptance

The optional **Testing & Simulation** page uses a separately prepared WSL workspace.
[Simulation tools](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/simulation/README.md)
cover preparation, scenarios, owned headless/rendered checks and backend limits.
Normal plugin startup requires no simulator, WSL or Python.

The owned harness uses copied reviewed MP assemblies, isolated settings and fresh
SITL state. It exercises native connection/reader/controller APIs without starting
the installed MP application. It does not establish full application lifecycle.
Rendered checks must report missing truth, failed positive coverage and cleanup failure.

The separate `test-plugin-native-sitl` task starts the actual installed host against
pinned cached Copter/Plane simulators. It requires an explicitly installed reviewed DLL,
closed MP, disabled startup input/router/video owners, and explicit
`-ExpectedSource`, `-ExpectedPluginSha256` and `-Vehicle` arguments. Read
[the runner](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/build/test_plugin_native_sitl.ps1)
before an authorized run. It never builds or installs. Simulator outputs remain software evidence.

## Documentation and repository checks

```text
pixi run lint
pixi run format-check
pixi run test-python
pixi run docs-build
pixi run precommit
git diff --check
```

Docs explain implemented behavior and CONOPS obligations; proposed task approaches
belong in [Brainstorming](brainstorming.md). Keep run histories, plans and handoffs
in ignored `local/`. Link published pages to repository source with full repository
URLs. Link and CONOPS tests validate references/provenance, not competition compliance.

To refresh actual control renders after building:

```powershell
powershell -STA -NoProfile -File scripts/docs/capture_screenshots.ps1
```

It uses inactive example controls, writes `docs/images/` and puts identity metadata
under `build/screenshots/`. Review images visually; they are not a connected host session.
For setup, use `scripts/docs/capture_installer_screenshot.ps1 -InstallerPath <built-exe>`.

The optional firmware has separate [build/release gates](native-termination.md#build-and-release-automation).
A release report must identify exact source, binaries and actual checks, including
local changes. Verify required checks/reviews on the PR before merge; software
evidence does not replace [aircraft acceptance](safety.md#aircraft-acceptance-remains-open).
