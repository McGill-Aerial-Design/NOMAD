# Owned NOMAD simulation checks

Optional development tools for SITL, rendered video and native Mission Planner API
checks. They never install the DLL or connect to real aircraft/USB/COM devices.
Normal NOMAD operation requires none of these tools.

The owned harness copies reviewed MP assemblies and uses isolated settings and fresh
simulator state. It exercises native connection/reader/mount APIs without constructing
the full MP application. Full host lifecycle and aircraft acceptance are separate.

## Preparation

Windows requires the reviewed MP host, VS/Roslyn/.NET Framework 4.8 and a built plugin.
The headless wrapper uses the default installed host location and **WSL Ubuntu-24.04**;
the plugin page supports selectable distributions/paths. Rendered wrapper runs also
require the existing pinned host archive at
`C:/ProgramData/Mission Planner/gstreamer-1.0-x86_64-1.14.4.zip`. There is no download fallback.

```powershell
pixi run build-plugin-only
pixi run test-simulation-host
pixi run test-plugin-simulation
```

Linux needs Gazebo Harmonic development packages, OpenCV development files and
`gz.transport13`/`gz.msgs10` Python bindings. Task 1 also needs NumPy, OpenCV, Pillow
and GDAL. Use a prepared environment; these scripts do not install system packages.

In WSL, set `NOMAD_REPO` to the absolute checkout path and choose an owned workspace:

```bash
export NOMAD_REPO=/mnt/c/path/to/NOMAD
export SIM_ROOT="$HOME/nomad-simulation"
bash "$NOMAD_REPO/scripts/simulation/prepare_linux.sh" "$SIM_ROOT" --prepare
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" ardupilot
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" gazebo
# Optional headless dynamics backend:
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" last-letter
python3 "$NOMAD_REPO/scripts/simulation/test_last_letter.py" "$SIM_ROOT"
```

Source/dependency pins and build manifests bind binaries and shared libraries to
source. Rebuild after changing inputs; do not bypass mismatches or run `cmake --install`.
Builds remain in the owned workspace with a 10 GiB initial source/build ceiling.
Harmonic 8.14+ supports the generated world's serial PostUpdate policy; a major
Gazebo migration requires reviewing bindings/plugins rather than changing a version label.

## Mission Planner simulation page

Open **NOMAD -> Testing & Simulation**, choose this `scripts/simulation` directory,
a prepared Linux workspace and a scenario. Read-only detection lists installed WSL
distributions and preserves overrides; it installs nothing. Settings are saved in
`nomad_simulation.json`, separately from aircraft profiles.

**Start** waits for simulator/video readiness, then optionally asks native MP to
connect. An existing open/read-only/replay link blocks admission. Use MP's ordinary
setup/flight/mission pages. Page/profile changes do not stop the shared simulator;
**Stop simulation** and unload clean up only that run's recorded processes.
A dispatched native helper may finish after Stop.

The preview uses the normal decoder and cannot steer. To track through the normal
controllers, copy the run's changing video URL into a calibrated simulation profile.
Avoid competing UDP viewers. Headless SITL/last_letter supplies no camera.
**Show Gazebo** requires WSLg GUI/OpenGL readiness. If its context fails, disabling
the GUI may permit server-rendered video; that does not repair the driver path.
Inspect **Open logs** for startup diagnostics and incomplete cleanup.

## Scenarios

Copy an example from `scenarios/` and edit it; **Load scenario** accepts another JSON.
The parser in `interactive.py` and world generator in `interactive_world.py` own the schema.

| Fields | Contract |
|---|---|
| `schema`, `name`, `backend`, `frame` | Schema 1; backend `sitl`, `gazebo` or `last-letter`; one dynamics owner |
| `home`, `system_id` | Home is `[latitude, longitude, altitude_m, heading_deg]`, or a loaded world's georeference; system ID defaults to 201 |
| `defaults`, `defaults_local` | Paths relative to the Linux workspace and scenario file respectively |
| `world`, `aircraft_model`, `camera_model`, `spawn` | Prepared local SDF/assets; ENU heading 0 and a free aircraft. Spawn is six values when the world has no matching aircraft pose |
| `aircraft_model_local` | Model directory resolved relative to the scenario file; use it for repo-owned models instead of workspace paths |
| `vehicle`, `wind` | `ArduCopter` (default) or `ArduPlane`, selecting the recorded SITL binary; Gazebo-only steady ENU wind in m/s, each component within ±30 and injected into the generated world |
| `physics_hz` | Use 1000 for flight tests. 400 is camera-only experimentation and fails the pinned Copter's gyro pre-arm requirements |
| `camera` | Even width 16..1920, height 16..1080, FPS 1..30; optional `bitrate_kbps` 100..50000 and `use_cuda` |
| `wsl_graphics_adapter` | Optional WSLg D3D12 device-name substring for this run; verify actual server/GUI renderer logs |
| `last_letter_model` | Prepared recipe for the separate headless backend |

Generated worlds cap `SceneBroadcaster` at 30 Hz so the GUI does not receive every
1000 Hz physics step; `physics_hz` still sets the physics and IMU rate.

Camera rate is per simulation second; a slow simulation delivers fewer wall-clock
frames. Resolution/aspect changes affect geometry and require matching calibration.
NVENC needs an existing `nvh264enc` plugin/driver. Keep machine-specific adapter and
asset paths in local scenario copies. Changing physics, scenery or encoding changes
evidence conditions; speed alone does not establish healthy autopilot timing.

### Tiltrotor tricopter fixture

`scenarios/tiltrotor.json` (calm) and `tiltrotor-wind.json` (steady 4 m/s from the
west) select ArduPlane and the repo-owned `models/tiltrotor_tricopter`. The
`ardupilot` build records both Copter and Plane; `vehicle` selects a verified binary.
The fixture's geometry, controls and limitations are documented beside the
[model](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/simulation/models/tiltrotor_tricopter/README.md).

## Running and inspecting

Close the normal MP application for the owned headless harness. From Windows:

```powershell
powershell -NoProfile -File scripts/simulation/run_headless.ps1
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -LastLetter
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Scene photo-board -VideoCase Detect
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural -VideoCase Object -ObjectMotion orbit -ObjectAltitude 5 -ObjectSpeed 5
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -DurationSeconds 600
```

`-LinuxWorkspace` overrides the default `~/nomad-simulation`. Prepare natural assets
below before `-Scene natural`. `-Visible` shows owned windows; `-Preview` adds an
interactive period excluded from automated evidence. Object runs use 1000 Hz and
cannot combine with endurance; rendered and last_letter cannot run together.

Each run creates `build/tests/simulation/run-<id>` with source/binary identity,
configuration, clocks, results and logs. Review case success, preservation checks
and cleanup separately. Windows Job Objects and Linux run-marked process groups
bound ownership. Never replace cleanup with process-name kills or `wsl --shutdown`.
The test-only encoded-video relay carries RTP packets through WSL TCP to Windows UDP;
it is separate from the native MP telemetry connection. Backpressure drops whole
datagrams and is recorded; a decoder can need the next keyframe.

Rendered runs encode at `-EncoderKbps` (100..50000, default 800; recorded as
`camera.encoder_kbps`). Above 2000 kbps the relay needs a 4 MiB UDP receive buffer,
otherwise keyframe bursts overflow and video stays grey; the run refuses to start
until `net.core.rmem_max` is raised. The setting does not persist across WSL restarts:

```powershell
wsl -d Ubuntu-24.04 -u root sysctl -w net.core.rmem_max=8388608
```

A prepared local workspace can also exercise the page's launcher/handshake/Stop:

```powershell
powershell -STA -NoProfile -File scripts/build/test_plugin_simulation.ps1 -WslDistribution <distro> -LinuxWorkspace <absolute-linux-workspace> -LauncherScenario scripts/simulation/scenarios/medicine-hat.json
```

`-DecoderRuntime <existing-gstreamer-prefix>` additionally requires decoded frames.
Optional `NOMAD_SIMULATION_TIMING_PROBE=1` checks native disarmed timing/pre-arm
observations; clear it after the run. See the script's parameters for probes against
an already owned simulator. No full operator-session lifecycle is established here.

## Replay and independent scoring

Replay initializes once and never reseeds. It opens no aircraft connection/camera:

```powershell
powershell -NoProfile -File scripts/simulation/run_replay.ps1 -Dataset <owned-run-directory> -Folder decoded
powershell -NoProfile -File scripts/simulation/run_replay.ps1 -Dataset <owned-run-directory> -Mode Object -Folder decoded -Selection 'x,y,width,height'
```

Use explicit ROI coordinates from the first saved image for Object. Set
`NOMAD_SIMULATION_OBJECT_TRACKER=GPU` to opt into the embedded GPU tracker for
replay/native cases. Receipts and stale gaps are retained; sampled recordings can
omit live warmup/history. Reports include returned boxes, aims, image hashes and timing.

Positive coverage alone is insufficient. Run independent scoring in WSL:

```bash
python3 "$NOMAD_REPO/scripts/simulation/analyze_rendered.py" /mnt/c/path/to/owned/area-run
python3 "$NOMAD_REPO/scripts/simulation/score_object_replay.py" --dataset /mnt/c/path/to/dataset --replay /mnt/c/path/to/replay
```

Area's contract is its original fixed point; Object's is whole-object bounds/centre.
Missing/ambiguous truth and wrong accepted points fail rather than disappearing
from the denominator. Oracle poses/masks never enter NOMAD vision or steering.
Keep each run's declared contract; a newer scorer does not reinterpret old evidence.

## Assets and Medicine Hat Task 1 world

Explicit acquisition runs in WSL; runtime world generation never downloads assets:

```bash
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" scene
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" ground
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" panorama
# Optional public planar footage; keep footage/annotations local:
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" video
python3 "$NOMAD_REPO/scripts/simulation/task1_world.py" --prepare --preview --seconds 120
```

`assets.json` pins URLs, hashes and licenses. The natural scene uses CC0 grass,
car/barrel assets. Courtyard panorama views test rotation/return. The optional TMT
planar book video has no located redistribution grant; retain it in ignored storage.

`task1_world.py` prepares coarse Sentinel-2 site colour, NRCan CDEM terrain, CC0
ground photographs and twelve textured deer proxies. Outputs include `task1.sdf`,
`boundary.geojson`, terrain/materials, `scene.json`, camera/overview renders and clocks
under `build/tests/task1-world/run-<id>`. It starts no MP, SITL or physical camera.
Live NOMAD video still requires the existing test bridge; Gazebo topics are not inputs.

The scene approximates Medicine Hat. Coarse terrain/site colours, generic textures,
simplified deer/tags and box collisions do not reconstruct the actual course or
establish clustering/scoring rules. Datums, withheld course details, readable tags
and real-camera appearance need independent acceptance. The bundled COCO vehicle
profile has no deer class; relabeling horse/sheep is not a deer detector test.

Preserve attribution from `task1-site.json`: **Contains modified Copernicus Sentinel
data 2025**; **Canadian Digital Elevation Model, Natural Resources Canada**;
deer model **Innerscene, CC0-1.0**. Ground photos: [Sparse Grass](https://polyhaven.com/a/sparse_grass),
Amal Kumar; [Withered Grass](https://polyhaven.com/a/withered_grass) and
[Dirt](https://polyhaven.com/a/dirt), Charlotte Baglioni; [CC0](https://polyhaven.com/license).
Detailed natural/panorama fixture credits are in [vision fixtures](../../mission_planner/tests/vision/fixtures/README.md).

## Limits and checks

Python tests mock simulator dependencies; native fixtures verify controlled API
behavior. Neither qualifies full MP lifecycle, real-time rendering, actual aircraft
dynamics, physical gimbal response, outdoor moving-object tracking or competition use.
Decoder receipt does not prove fresh exposure or physical latency. last_letter's
stock plus-frame recipe is not a measured aircraft model. GUI and vision can compete
for the same GPU; identify visible/headless and hardware cohorts separately.

Run outcomes belong in ignored evidence with source/configuration identities.
Preserve failed runs; report failed/skipped/unavailable separately. Keep deployment
URLs, profiles and raw private imagery/logs out of source control.
[Development checks](../../docs/development.md) list product gates; Python tooling
checks run through `pixi run test-python`.
