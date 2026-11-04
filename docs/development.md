# Development

Read [architecture](architecture.md) for code ownership and Mission Planner APIs.
Builds/tests never install the plugin or open operator aircraft connections.
Run commands from the repository root. Preserve operator profiles and ignored evidence.

## Build without installation

Windows needs the reviewed Mission Planner host from
[target metadata](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/release/mission-planner-target.json),
Visual Studio/MSBuild with Roslyn and the .NET Framework 4.8 targeting pack, plus Pixi.
Python/Pixi supports development, documentation and simulator tools.

```powershell
pixi run build-plugin-only
pixi run build-installer
pixi run test-plugin-installer
```

The first command produces `mission_planner/src/bin/Release/NOMADPlugin.dll` and
`build/release/plugin-build.json`, binding its hash to the base commit, product
source hashes and reviewed host. The release version lives in
`scripts/build/build_plugin_windows.ps1`. Host references are non-copy-local.
The installer build verifies that proof against the current source before
embedding the DLL and notices into `build/installer/NOMAD-version-Setup.exe`.
Direct IDE builds have fallback development identity; use the script for checked artifacts.

`prepare_vision.ps1` verifies pinned archives and every asset before compilation.
The DLL embeds ONNX Runtime 1.24.4 (DirectML build), DirectML 1.15.4,
OpenCvSharp, YOLOX Nano, the EfficientTAM Ti512 export and license notices.
Detection uses the CPU provider; GPU Object tracking is explicit per camera.
The installed plugin extracts verified assets and performs no downloads.
`VisionRuntime` isolates ONNX Runtime from MP's older loaded Visual C++ runtime;
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

The vision gate checks the pinned CPU detector and native runtime load order.
Set `NOMAD_TEST_TAM_MODEL` to a prepared EfficientTAM export folder for the optional
NVIDIA/GPU cohort; otherwise that cohort explicitly skips. Optional
`-TrackingFrames` / `-TrackingGroundTruth` arguments accept an external 120-frame
replay with aligned truth. Keep private media/provenance in ignored storage.
Synthetic controller checks and positive replay coverage do not establish real-scene accuracy.

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
