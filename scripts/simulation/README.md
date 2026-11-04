# Owned NOMAD simulation checks

These are optional development tools. Normal plugin startup and installation require none of
them. They never install the DLL or connect to real aircraft/USB/COM devices. Do not run the older
installed-host SITL script as a substitute for these owned-state checks.

The controlled Windows fixture uses real MP1.3.83 assemblies and its native connection,
packet reader and mount helpers. It does not construct `MainV2`: full MP startup binds an
unconditional HTTP listener, starts discovery and enumerates devices. That lifecycle remains
outside this fixture's qualified scope.

## Mission Planner simulation page

Open **NOMAD → Testing & Simulation** after installing a separately checked plugin build.
Choose the `scripts/simulation` folder from your own NOMAD checkout. Opening the page lists
installed WSL distributions, detects the selected or Windows-default distribution and its Python,
and fills the simulator folder as that Linux user's `~/nomad-simulation`. **Detect setup** repeats
the read-only check; saved distribution/workspace/interpreter overrides are preserved. Other
distributions are listed without starting them or scanning their files. Paths are saved separately in
`nomad_simulation.json`; aircraft profiles are never changed by the page. No simulator tools
are bundled into the DLL or installed automatically.
If the default simulator folder is not prepared, detection says so; it does not download or build
toolchains. Choose an existing prepared folder or follow Preparation below.

Prepare that workspace using the commands below before Start. Gazebo requires Harmonic's
`gz.transport13` / `gz.msgs10` Python bindings in the selected interpreter (system Python, or a
compatible environment exposing system packages). **Show Gazebo** opens its visible WSLg GUI.
Start waits for simulator readiness and camera packets before optionally asking native MP to
connect. Connect is also available separately. Use MP's ordinary setup, flight and mission pages.
With **Show Gazebo**, readiness also requires a GUI camera pose published after rendering starts.
WSLg GUI children explicitly select GLX integration unless overridden. The inherited Qt loop and
OpenGL diagnostics are recorded per run; the launcher does not select a software renderer.
If WSLg cannot create the separate GUI's OpenGL context, disabling **Show Gazebo** still permits
the server's rendered camera feed. This is a workaround, not a repair of the GUI driver path.
Switching NOMAD pages/profile does not stop the simulator; Stop simulation and unload clean up
only the recorded run's processes. The configured time limit is 1..1440 minutes; artifacts have
a 1 GiB ceiling. Logs and explicit cleanup errors remain available through Open logs.

The dropdown scans `scenarios/*.json`; **Load scenario** accepts another local JSON file.
Copy and edit an example to configure your location, aircraft, parameters and imagery. There
are three backends: `sitl`, `gazebo`, and headless `last-letter`. `schema`, `name`, `backend`,
`frame` and a home `[latitude, longitude, altitude_m, heading_deg]` define a basic SITL recipe.
`system_id` defaults to 201. `defaults` paths are relative to the Linux workspace;
`defaults_local` paths are relative to the scenario file. Absolute Linux asset paths also work.
The workspace's validated build manifests select binaries/source hashes; examples do not
discover or download arbitrary binaries at runtime. Direct-binary frames differ from
`sim_vehicle.py` recipes: the prepared Gazebo plugin uses `JSON:127.0.0.1`.

For Gazebo, `aircraft_model` and optional `camera_model` identify prepared local SDF model
directories. The stock Iris retains its native three-axis gimbal joint controllers. `world`
or **Load world** selects local scenery; its existing matching aircraft include is replaced
with the owned configured model. Its georeference takes precedence over example coordinates.
Worlds must use ENU spherical heading 0, local prepared assets, and a free aircraft without
static/world joints. A six-value `spawn` is required when no existing aircraft pose is available.
Custom model dependencies/plugins must already exist; arbitrary downloaded worlds are not
automatically made compatible. Natural scenery uses the pinned CC0 grass asset prepared with
`prepare_assets.py scene`; Medicine Hat uses `task1_world.py --prepare` inputs. These are editable
examples, not product defaults or guaranteed replicas of a real site.

On WSLg laptops, Mesa may choose an integrated GPU even when a faster GPU is available. An optional
`"wsl_graphics_adapter": "<device-name substring>"` in your own Gazebo scenario selects the renderer
adapter using `MESA_D3D12_DEFAULT_ADAPTER_NAME`, only for that run's Gazebo server and GUI. Blank or
absent preserves the inherited/platform choice. This requires WSLg's D3D12 renderer and a scenario
restart; it changes no system power/driver settings. Keep machine-specific choices in your own
scenario copy and rebase its relative asset/defaults paths. See Microsoft's
[WSLg GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg).
The requested environment is recorded in `session.json`; verify actual selection in both
`gazebo-home/.gz/rendering/ogre2.log` and `gui-home/.gz/rendering/ogre2.log` (`GL_RENDERER`).
Keep image geometry, camera rate, scene/seed and physics settings unchanged when comparing speed.

Gazebo scenarios accept `"physics_hz": 400` or `1000` (default). The 400 Hz option uses a
2.5 ms physics step for camera-only experiments; it reduces physics, effective IMU sampling and
actuator-exchange resolution. With the pinned Copter's normal 400 Hz control loop, it fails the
gyro pre-arm check: the measured backend must reach at least 720 Hz. Use 1,000 Hz for the normal
flight scenario. Do not disable arming checks or lower the control-loop rate to conceal this mismatch.
`Main loop slow` separately means ArduPilot's simulation-clock loop is below 90% of its configured
rate; Gazebo's wall-clock real-time factor alone cannot establish that check. Camera settings and
the 1× speed target stay unchanged. Qualify dynamics-dependent results separately.

An optional `"camera": {"width": 1920, "height": 1080, "fps": 30}` overrides the run-owned
camera model. Omission preserves that model's settings. Even dimensions from 16 up to 1920×1080
and integer rates from 1 to 30 are supported. The rate is per simulation second; a slow simulation
delivers fewer frames per wall-clock second. Horizontal FOV and camera transforms stay unchanged;
switching from 4:3 to 16:9 narrows the vertical FOV. Operator calibration is not changed automatically.
Upstream fixes its encoder at 800 kb/s. A recorded NOMAD patch (`patches/ardupilot_gazebo-*.patch`,
applied to an exported build copy; the pinned checkout stays clean) adds an optional `bitrate` element.
Set `"bitrate_kbps"` (100..50000) in the camera object; omission keeps 800. Rendered launcher runs
take `-EncoderKbps`, recorded as `camera.encoder_kbps`. Higher bitrate preserves more detail but is
a simulator setting: match the real link before using it as qualification evidence. Rebuild with
`build_linux.sh <workspace> gazebo` after the patch changes; runs refuse a stale patch binding.
The camera object also accepts `"use_cuda": true` for the upstream NVIDIA NVENC encoder;
this requires the existing GStreamer `nvh264enc` plugin and a supported NVIDIA driver. Omission
preserves the camera model's encoder selection. It does not change system graphics settings.

Gazebo previews actual camera pixels using the normal NOMAD decoder. The preview is uncalibrated
and cannot steer a gimbal. Copy its video URL into an explicitly configured simulation profile's
Video input for normal profile-bound tracking/control; do not run both UDP viewers simultaneously.
The URL/port changes for each fresh session. Camera capture time remains unavailable in the
product Bitmap path. Headless SITL/last_letter has no camera preview. last_letter uses
`last_letter_model`; it cannot be combined with Gazebo dynamics without a separately tested pose
bridge, which this page does not supply. Stop never cancels an already dispatched native helper.

The encoded-video relay keeps one pending TCP packet and a bounded socket send buffer. A slow
viewer can lose complete incoming RTP datagrams while that pending packet drains; it cannot
corrupt the length framing or block simulation ingestion. `session.json` records `video_relay`
counters, including `dropped_backpressure`. A decoder may need the next keyframe after packet loss.

Fast Windows gate (no MP application, WSL, simulator or device opened):

```powershell
pixi run test-plugin-simulation
pixi run python -m pytest -o addopts='' tests/test_simulation_launcher.py tests/test_simulation_tooling.py tests/test_simulation_task1.py tests/test_simulation_object_motion.py
```

For a prepared local workspace, the opt-in production-launch probe exercises the same WSL command
builder/session as the page, native MP handshake, video-relay readiness and Stop in isolated test
state. It also checks a path containing spaces and readable Windows-side WSL errors. It opens
visible Gazebo; publication uses an owned native link collection rather than the running MP window:

```powershell
powershell -STA -NoProfile -File scripts/build/test_plugin_simulation.ps1 -WslDistribution <distro> -LinuxWorkspace <absolute-linux-workspace> -LauncherScenario scripts/simulation/scenarios/medicine-hat.json
```

Set `$env:NOMAD_SIMULATION_TIMING_PROBE='1'` for the isolated native fixture's timing/pre-arm check.
It reads back `SCHED_LOOP_RATE=400` and the pinned firmware's `ARMING_SKIPCHK=0`, requests status
and simulation boot-time telemetry, waits 30 host seconds, then invokes `RUN_PREARM_CHECKS` and
observes another 30 host seconds. Every measured SYS_STATUS must report healthy gyro and enabled,
healthy pre-arm checks; heartbeats must remain disarmed and arrive within three host seconds,
with status within two. Command acceptance alone cannot pass. `timing-health.json` retains raw
masks, status text, separate simulation/host clocks and failures. It never arms or writes parameters.
Unset the variable after the opt-in run. This checks the isolated native-library path, not full
Mission Planner operator-session performance or physical flight.

Use `-OutputDirectory <new-owned-path-with-spaces>` to exercise generated world resources under
the same kind of path as MP's user-data folder. Add `-DecoderRuntime <existing-gstreamer-prefix>`
to require three decoded camera frames as well as camera-relay readiness. Generated SDF assets
use literal absolute filesystem paths: this Gazebo resolver does not decode escaped file URLs.
For scenario copies outside the tools folder, also pass `-LauncherToolsDirectory scripts/simulation`.
Keep the output path short enough for native MP/.NET Framework file operations; deeply nested
test logs can exceed their Windows path-length limit.

If startup fails, Open logs retains `startup-error.log` even when the launcher has not started.
The page reports both Windows-side WSL diagnostics and Linux command errors. An early launcher
exit shows its final error line; simulator failures identify the process and its log file.

An optional actual native-handshake/decoder probe can target only an already running owned,
disarmed simulator. Supply its readiness record's host/ports and an existing extracted GStreamer
prefix containing `bin/` and `lib/gstreamer-1.0/`; it opens no MP window:

```powershell
powershell -STA -NoProfile -File scripts/build/test_plugin_simulation.ps1 -NativeHost <owned-ip> -NativePort <owned-tcp-port> -VideoTcpPort <owned-video-tcp-port> -DecoderRuntime <existing-prefix> -OutputDirectory <new-owned-directory>
```

## Preparation

Use Windows11, reviewed MP1.3.83 reference files, VS2022/net48 and WSL2 Ubuntu24.04 with
Gazebo Harmonic 8.15 development packages (the checked local runtime). Generated worlds use
Gazebo's serial PostUpdate policy, available since 8.14, to avoid per-step worker synchronization.
Camera resolution/rate, lighting, collision geometry and physics steps stay unchanged. Older
Harmonic releases ignore this policy. Loaded custom worlds retain their own policy choice.
Gazebo Jetty is a separate major-version migration: the current Python bindings, fixture plugins
and GUI paths target Harmonic. The build scripts refuse missing/changed source pins;
they do not run system package installers. No ROS, CUDA container stack or paid service.

```powershell
pixi run build-plugin-only
pixi run test-simulation-host
```

From WSL, with `NOMAD_REPO` set to the absolute checkout path and `SIM_ROOT` to a new owned
directory (default `~/nomad-simulation`):

```bash
bash "$NOMAD_REPO/scripts/simulation/prepare_linux.sh" "$SIM_ROOT" --prepare
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" ardupilot
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" gazebo
bash "$NOMAD_REPO/scripts/simulation/build_linux.sh" "$SIM_ROOT" last-letter
python3 "$NOMAD_REPO/scripts/simulation/test_last_letter.py" "$SIM_ROOT"
```

Pins: AP `fb773567896014da007bca58358704d4a0d8516f`, official Gazebo plugin
`80dc9664dda7efd6c5d1a71a79198d1d6b708c0e`, last_letter
`13baa815e971abbcc568bea1e265026b44686273`. The script also pins required AP gitlinks and
last_letter's yaml-cpp/DataTamer/NLopt dependencies. Builds stay under the owned workspace;
never run `cmake --install`. The initial source/build ceiling is10GiB. Existing source trees
and installed MP files are read-only inputs and must remain untouched.

Build manifests bind source to executable and local shared-library hashes. Rebuild after
changing build metadata/schema; do not bypass checksum errors. Record compiler/package
versions and compare only evidence from the same source, fixture and binary configuration.

## Running and inspecting

```powershell
powershell -NoProfile -File scripts/simulation/run_headless.ps1
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -LastLetter
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural -VideoCase Occlusion
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural -VideoCase Occlusion -PhysicsHz 400
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Scene photo-board -VideoCase Detect
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural -VideoCase Object -ObjectMotion orbit -ObjectAltitude 5 -ObjectSpeed 5
powershell -NoProfile -File scripts/build/test_video_route.ps1
```

Override only the owned source location with `-LinuxWorkspace` if needed. Runs get fresh
`build/tests/simulation/run-<id>` directories and separate ProgramData/user-data identifiers.
They validate the host version, current DLL source, logging sinks, ownership markers and
reserved ports. Existing MP causes refusal. Every wrapper result reports the preserved
operator config/profile/log set independently from case success and cleanup.

Native mount checks require observed SITL201/1, disarmed state and independent servo9/10/11
transitions. `Submitted` is not an ACK or measured mechanism pose. Last_letter uses a plus
quad; the rendered Iris uses an X quad. Stock source SITL is a separate cohort from NOMAD's
patched Cube firmware. None of these checks establish physical flight or competition approval.

Area/detector cases keep the body fixed and drive actual gimbal joint velocities from observed
ArduPilot outputs, with100ms delay,80ms response and finite speed/travel. Source/frame clocks,
joint samples and rendered PNGs are independent oracle records. The seed is173; thresholds,
calibration and texture generation must not be changed to rescue failed runs.

The Object case reuses the prepared Task1 CC0 deer mesh and ear tag with a predefined center
selection on the declared1.2m torso point. `ObjectMotion` is `orbit`, `flyby`, `approach`, or the
intentionally constrained `direct` pass;
altitude is5/15m and speed5/10m/s. Orbit radius and normal fly-by offset are20m. An owned trigger
starts independent aircraft-body velocities; the orbit adds10°/8s body-yaw variation so a fixed
gimbal cannot pass. The native MP→ArduPilot→output→bounded joints→camera path stays intact.
This is kinematic disturbance evidence, not autopilot free-flight/dynamics qualification.
The declared broadside approach moves from(0,−35)m to(0,−8)m with90° body yaw and±5°/8s yaw
disturbance; the deer does not turn or move. Initial selection is a predefined physical patch,
projected once from the known setup geometry; live labels never initialize or update the tracker.

A coincident semantic camera labels the selected deer, another deer, tag and occluder independently.
Labels never enter NOMAD. Raw RGB/mask stamps must align within2ms, with a2s host pairing deadline;
decoded receipts are aligned by the existing conservative image-fingerprint criterion. The oracle
requires95% correct target coverage over all motion frames, zero wrong accepted targets, actual
body/joint/output variation, bounded joint travel/rate and centering p95 within5% of each dimension.
The direct-pass negative instead requires a correct positive prefix and observed infeasible geometry.
Object cases keep the operator camera's fixed1920×1080/30FPS/2rad optics and NVENC at `-EncoderKbps` (default800)
encoder; Area/detector cases retain640×480/90°/10FPS. Camera and full-resolution label stamps are
recorded independently of host receipts. The fingerprint uses the same normalized64×48 point grid
at either resolution. One-pixel mask tolerance means one native sensor pixel, without scaling it.
Full RGB PNGs are capped at120 samples; six initial and the first120 motion-stage decoded images
are requested for replay through the bounded writer. Every processed raw capture retains metadata
and a compressed full-resolution label map. Inspect writer/capture gaps and the1GiB recording cap.
The old640 ground-selection failure remains diagnostic negative evidence, not an HD result.
Retention does not establish readable3cm glyphs or sustained30 wall-clock FPS. `world-manifest.json`,
semantic masks and `oracle-object-frames.json` retain the declared scope.

The researched WSL-NAT direct UDP path failed a bounded positive-control route probe.
The test-only bridge therefore carries unchanged H264/RTP packets through an owned WSL
TCP15761 listener to Windows loopback UDP15600. It is not a product transport or aircraft
reader. AP TCP15760 is the actual native MP path; its Linux server binds all interfaces
inside the existing WSL-NAT boundary. No firewall/portproxy/system network changes are made.

Windows Job Objects fence owned children. Linux run-marked sessions stop on stdin EOF,
STOP or termination, including child process groups and descendants surviving a leader. Normal close and forced
cleanup are distinct outcomes. Never substitute process-name kills or `wsl --shutdown`.

For recorded datasets (one initialization, no reseeding):

```powershell
powershell -NoProfile -File scripts/simulation/run_replay.ps1 -Dataset <owned-run-directory> -Folder decoded
```

The diagnostic Area replay uses the same fixed144x144 selection as the rendered case;
`-Folder frames -First <zero-based-index>` instead reads raw renderer captures. It reports
positive coverage, not independent geometry. The offline geometry oracle is separate:

```bash
python3 "$NOMAD_REPO/scripts/simulation/analyze_rendered.py" /mnt/c/absolute/owned/run-directory
```

That oracle uses actual Gazebo X-forward/Y-left/Z-up sensor poses, a declared plane and
independent image correspondence. Ambiguous camera poses or invalid clocks stay unavailable.
It never feeds world positions/boxes/poses into NOMAD. Capture latency is unavailable in the
current Bitmap-only product pipeline; raw-image transport receipt is not exposure time.

The natural scene uses a 15m-scale photographic grass/rock tile, an actual 3D car and
barrels. Assets total3.32MB, are CC0 and have exact URLs/hashes in `assets.json`. From WSL:

```bash
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" scene
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" video
```

Scene preparation is required before `-Scene natural`. The second command downloads
the5.10MB TMT `nl_bookI_s1` public planar video and its four-corner annotations, then
prints a fresh owned dataset path under ignored `local/simulation/datasets/`.
Use its Windows path with `run_replay.ps1 -Dataset <path> -Folder frames`.
Replay uses an owned Job Object and a ten-minute process deadline, including native-call hangs.
All1299 original800x600 frames are decoded once, hashed and used in order, with one
fixed55x70 interior selection. The independent oracle projects that initial canonical
point through the annotated quadrilaterals. The gate requires95% verification and
zero accepted errors above5px (annotation plus fitting tolerance), after the first
two confirmation frames. There is no reseeding, dropped hard interval or retry tuning.
Keep the footage/annotations local: an explicit redistribution grant was not located.

For the pinned CC0 photographed360-degree Courtyard source and deterministic views:

```bash
python3 "$NOMAD_REPO/scripts/simulation/prepare_assets.py" panorama
```

It uses the existing OpenCV/NumPy test environment and records source/frame hashes,
fixed selection and camera poses. The checked-in JPEG views run in the normal native
vision gate without downloads. Pixel truth comes from immutable world rays, while
the generator samples the equirectangular photograph. This tests rotation and return,
without implying a physical camera, independently moving object or flight result.

`-Visible` shows Gazebo and the actual NOMAD player during the case. Add `-Preview` for
a two-minute interactive preview afterward. Closing the preview ends it early. The automated
evidence stops before that preview, so later operator interactions cannot enter its
pass/fail metrics. Headless and visible performance cohorts must be identified separately.
The server starts first; the GUI attaches after the first camera capture, in a separate
owned session/home. The Qt XCB/GLX choice is explicit and recorded in `linux.json`.
GUI exit fails a visible case; no automatic restart or graphics fallback rescues it.

The occlusion case triggers one fixed world-level opaque barrier trajectory after the
initial steering case. Its actual link pose and initially observed fixed local collision
bounds accompany capture metadata. It is a controlled occluder proxy, not an aircraft
dynamics model. The independent oracle tests camera-to-original-plane ray segments against
that actual box, requires at least five fully covered frames with no verification, then
clear-view recovery and the normal accuracy/coverage gates. Partial/uncertain overlap is
reported separately; accepted partial views still need the correct original point.
New requests while Searching/Outside are forbidden; an already-dispatched helper may finish.
Recovery's host-observed unverified interval is bounded separately from unavailable optical
capture-to-recovery timing. No ground-truth pose enters NOMAD tracking or steering.

Replays and native Object cases use the GPU Object tracker when `NOMAD_SIMULATION_OBJECT_TRACKER=GPU`;
native cases set the same profile setting an operator chooses and use the model embedded in the built
plugin (see docs/video.md). Records then
carry `object_tracker_provider` and the tracker's admission/timing diagnostic. The visible Gazebo GUI
competes with it for the same GPU; on the reference laptop that lowered the displayed rate below the
native case's 8 FPS floor, while the hidden-GUI cohort met it.

`run_replay.ps1 -Mode Object -Folder decoded -First 0` with one explicit
`-Selection 'x,y,width,height'` replays a complete owned rendered dataset through the Object
tracker the factory selects (the GPU tracker when `NOMAD_SIMULATION_OBJECT_TRACKER=GPU`).
It initializes once on the first saved image, then processes every later saved PNG.
Original receipt intervals reproduce the worker's non-increasing/0.5-second gap rule.
Unsaved warmup frames then appear as a stale-video gap that fences the tracker, which is
stricter than the live history.
This is a sampled-image regression: missing warmup frames and stride2 recordings do not
reproduce the live tracker history. Replay opens no aircraft connection, port or camera;
the normal MP application can remain open. Runtime/cache writes stay in the owned host.

`replay-frames.jsonl` binds each returned box, aim and synchronous tracker diagnostic
to its PNG hash and source receipt clock. `update_ms` excludes file hashing and drawing.
Its positive-coverage result alone does not establish target accuracy. After the replay
process exits, run the separate semantic scorer (paths below are WSL paths):

```bash
python3 -B scripts/simulation/score_object_replay.py --dataset build/tests/simulation/run-DATASET_ID --replay build/tests/simulation/run-REPLAY_ID
```

The scorer rejects omitted/reordered images, changed receipt/gap data, RGB fingerprint
mismatches and inconsistent capture intrinsics. It checks all plausible paired masks,
retains Searching frames in the positive denominator and fails missing/ambiguous truth.
The unchanged point gate requires95% correct foreground aims with zero wrong admitted
aims and the existing one-native-pixel square border. A separate whole-object gate
requires95% of frames with selected-object bbox IoU at least0.5; both must pass this
sampled-replay suite. Neither new gate reinterprets older live results. Point-error p95
includes wrong admitted aims; its pinhole angular bound is not physical mount accuracy.
Bindings include the template, initial ROI/options, RGB, capture metadata and mask hashes.
New Object-motion manifests explicitly declare `target_contract: whole_object_bounds`.
These require95% selected-object bbox/centre coverage, bbox IoU at least0.5, zero accepted
wrong instances and a perception centre error inside the controller's existing per-axis2%
image deadband. The rendered closed-loop centring check uses the independent target centre,
not NOMAD's desired point. A box centre between legs or inside a hole is valid when that
whole object is verified. Historical manifests without this tag retain the original
surface-point gate; those outcomes are never relabelled as passes. Surface-pixel diagnostics
remain separately available. Area continues using its immutable canonical point.
Scoring has a90-second deadline,2,000saved-image/10,000capture limits and two OpenCV
threads. Failures write `object-replay-error.json`; qualification failure still exits nonzero.

Rendered Area/occlusion runs automatically invoke the independent oracle with a55s Linux
deadline after owned simulator cleanup. Missing correspondence/geometry fails the run.

For the separate continuous stability/resource cohort:

```powershell
powershell -NoProfile -File scripts/simulation/run_headless.ps1 -Rendered -Visible -Scene natural -DurationSeconds 1800
```

It preserves one selection/stream/intent for30minutes, with no retry/reselection, model
cycling or source restart. Each completed60s window requires8 displayed FPS and95%
verification. The first5minutes warm the process; subsequent1Hz observations require
private growth<=100MiB,slope<=2MiB/minute and process/GDI/USER-handle growth<=20 each.
Sampling coverage must reach95%, with no gap>3s. No forced GC rescues the measurement.
Raw/decoded PNGs are limited to120 each; compact metadata continues. Recordings/diagnostics
have a1GiB ceiling, while copied runtimes are accounted separately. The run has a
40minute elapsed-time qualification limit; native work uses the remaining budget,
while preparation I/O is not interrupted by an absolute watchdog. Cleanup must
finish<=10s. Source/model lifecycle tests remain separate.
The bounded initial steering slice gets the geometric oracle; the long segment establishes
stability/resources rather than independent accuracy of every30minute frame.

Artifacts include `manifest.json`, `probe.json`, `native.json`, `linux.json`, `result.json`,
`video-frames.jsonl`, independent capture metadata/PNGs, joint samples and resource samples.
Keep exact endpoints/raw local logs private; publish only synthetic allowlisted summaries.

## Medicine Hat Task 1 world

The separate `task1_world.py` perception scene uses the published AEAC 2027 C1 boundary,
coarse site colour from Sentinel-2 (7 May 2025, native 10 m pixels), a complete coarse NRCan
CDEM surface, and twelve stationary textured deer proxies. Sparse grass, dry grass and
dirt photographs add ground detail at their declared 2 m tile scale. The original site
colours guide at most 24 matte material tints and approximate surface choices; this is
not a measured vegetation/soil map. Six deer have antlers; four carry
two-sided two-character ear tags with nominal 3 cm glyphs. A fixed seed defines clusters,
isolated placements and headings. All decoys must lie inside the published polygon.

From the existing WSL checkout:

```bash
python3 scripts/simulation/task1_world.py --prepare --preview --seconds 120
```

First preparation downloads the pinned 8.56 MB CC0 deer, derives its antlerless variant,
reads a small window of the public satellite COG using the installed GDAL library, and
issues sixteen small CDEM profile requests. It also prepares three pinned, unmodified
1K CC0 ground JPEGs totalling 2,824,754 bytes. Existing WSL NumPy/OpenCV/Pillow/Gazebo are
required. It installs nothing. Later runs verify the frozen input hashes and create a
fresh world under `build/tests/task1-world/run-<id>/`; changed inputs/definitions fail.
The ground cache is separate from those frozen site inputs. To prepare only the new
textures for an already prepared site:

```bash
python3 scripts/simulation/prepare_assets.py ground
```

Runtime generation, including the Mission Planner scenario launcher, never downloads them:
missing or changed photos produce a prerequisite error. Each fresh world copies the three
original JPEGs beside `terrain.obj` and `terrain.mtl`, using simple relative texture names
even when the output path contains spaces or Unicode. Materials reuse the original 8,192
terrain triangles, vertices and normals. They add no grass meshes, displacement, collision
objects or physics plugins. UVs repeat in world metres, avoiding a large site-wide texture
atlas. `scene.json` records photo URLs/hashes/licences, the material palette/cell assignment,
tile scale and original colour-map hash. Ground generation consumes no deer-placement RNG.
The 64×64 coarse colour cells still have hard material boundaries in the overview;
photographic detail improves the close view but does not interpolate those site tints.

The output includes `task1.sdf`, `boundary.geojson`, terrain geometry/materials, `scene.json`, and
two real rendered images: `camera.png` (1280×720 inspection view) and `overview.png`
(1024×1280 north-up overview). Raw camera topics are `/nomad/task1/rgb` and
`/nomad/task1/overview`. The preview starts separate owned server/GUI sessions and
stops only those sessions on close, failure or its deadline. Camera simulation stamps
and host receipt clocks are recorded separately. Interactive GUI changes occur after
the evidence images and cannot enter their static oracle. No MP, SITL or physical
camera/aircraft is started; this scene does not establish the native gimbal command path.
NOMAD does not directly consume Gazebo topics: the rendered images are available for
offline CV; a live feed still needs the existing test video bridge integration.

This is **not an exact reconstruction**: the actual search polygon/laps are withheld
until 13 May 2027; fine site-specific grass texture, current structures and surveyed terrain
are unavailable. Generic photographic detail does not establish how the actual Medicine Hat
ground looks. It has no grass-blade geometry or wind. Partial 1 m HRDEM coverage is not mixed with CDEM's CGVD28
datum. The terrain is relative to a local CDEM baseline; ellipsoid altitude is unavailable.
The model is a scaled deer proxy, not the unknown official plastic decoy. Body collisions
are boxes. Cluster membership is a fixture, not an interpretation of the unresolved
competition clustering rule. Higher-resolution licensed site imagery and held-out camera
footage remain needed to qualify actual close-range appearance.

`task1-site.json` records primary URLs, dates, licences, dimensions and hashes.
Credit: **Contains modified Copernicus Sentinel data 2025**; **Canadian Digital Elevation
Model, Natural Resources Canada**; deer model **Innerscene, CC0-1.0**. No Esri/Google
imagery is cached. Ground photos: [Sparse Grass](https://polyhaven.com/a/sparse_grass),
Amal Kumar; [Withered Grass](https://polyhaven.com/a/withered_grass) and
[Dirt](https://polyhaven.com/a/dirt), Charlotte Baglioni; [Poly Haven CC0](https://polyhaven.com/license).
Their versioned pins are separate in `assets.json`. Ground truth stays in `scene.json`, outside NOMAD inputs.
The built-in YOLOX COCO vehicle profile has **no deer class**. Genuine deer/decoy-trained
YOLOX weights are required for deer detection; horse/sheep relabelling is not a detector test.

```powershell
pixi run python -m pytest tests/test_simulation_task1.py
```

## Current qualification limits

- The original rendered failures and their raw/decoded replays are retained. The Area
  tracker now learns bounded verified views and uses checked optical-flow continuity,
  with unchanged spatial-support gates and an original identity witness. Permanent
  colored/natural image regressions accompany the positive and foreground-rejection tests.
- The rendered acceptance case establishes only a short fixed-body camera/mount result,
  not flight or full MP startup qualification. Exact run outcomes belong under ignored
  `local/simulation/` and the immutable attempt artifacts.
- Detection cases need positive coverage for each declared positive preset. Report a
  stricter preset's missed positive separately from correct class/threshold filtering.
- Fresh receipts of frozen pixels do not prove fresh capture. The fast video test records
  that observability limit; no passing freeze detector is claimed.
- last_letter protocol tests reproduce upstream oversized32-channel truncation/acceptance
  and zero-rate fallback. Its valid-frame timing result is not untrusted-input qualification.
- Broader body motion, rendered Object scenarios, aircraft-transition and optional
  board-emulation qualification is not complete. The public-video case is a planar book,
  not the intended aircraft camera or outdoor moving-object acceptance.
  Full MP lifecycle, actual tilt-tricopter dynamics and physical gimbal behavior are unqualified.
