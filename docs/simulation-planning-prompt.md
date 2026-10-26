# Simulation planning-agent prompt

Copy this prompt into an AI coding agent with access to the NOMAD checkout.
The requested output is a researched implementation plan, including concrete
automated-test designs. It does not authorize simulator installation or implementation.

```text
You are planning a production-quality simulation test system for NOMAD, a Windows
C#/.NET Framework Mission Planner plugin. Research the current upstream projects
and inspect NOMAD before recommending an architecture. Produce a plan, not code.

Objective
Compare Project AirSim and Aerial Autonomy Stack for rendered end-to-end NOMAD
testing. Decide which is the best fit, or justify a smaller alternative. Also plan
how current last_letter can complement that renderer with headless flight-dynamics
and ArduPilot SITL tests. Define automated tests for NOMAD's detection, Object/Area
tracking, gimbal control, motion estimates and video UI.

Start with the actual checkout
- Read AGENTS.md, local/HANDOFF.md, local/TODO.md, docs/architecture.md,
  docs/video.md, docs/development.md and docs/qualification.md.
- Verify HEAD, remotes, worktrees, local changes and reviewed host metadata in
  scripts/release/mission-planner-target.json. Do not overwrite unrelated work.
- Read the owning symbols/callers/tests in Vision/, Media/, GimbalController,
  OutputController, the router, tests/vision, tests/video and scripts/build.
- Use actual current code, not this prompt's description, as the contract.
  Existing features include named YOLOX presets/live swapping, Object CSRT with
  appearance rejection, frozen-reference planar Area tracking, retained selection,
  explicit tracking intent, three-frame visual recovery, canonical-point aiming,
  optional GPS/HOME/altitude ground search hints, and a camera-relative heading line.
- The heading display is a short orbiting line, with no circle/arrowhead. Tracking
  uses one fixed-width Start tracking / Stop tracking toggle. Do not reintroduce
  keyboard aircraft-flight controls or conference-specific product UI.

Research primary sources and pin the evidence
- Project AirSim: https://github.com/iamaisim/ProjectAirSim
  https://github.com/iamaisim/ProjectAirSim/blob/main/docs/controllers/ardupilot.md
- Aerial Autonomy Stack: https://github.com/JacopoPan/aerial-autonomy-stack
- last_letter: https://github.com/Georacer/last_letter
  https://github.com/Georacer/last_letter/tree/master/frontends/ardupilot
- ArduPilot: https://github.com/ArduPilot/ardupilot
- Optional board-emulation tier:
  https://github.com/ArduPilot/ardupilot/pull/33975
  https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/renode-emulation.rst
- Optional traffic scenario context, kept out of the initial tracking workstream:
  docs/brainstorming.md and https://github.com/ArduPilot/ardupilot/pull/33955

Prefer current repository README/source over stale wiki assumptions. Record each
inspected commit/release, date and supporting path/link. Separate documented,
source-supported, reproduced, untested and missing capabilities. Do not equate
SimpleFlight/PX4/JSBSim support with ArduPilot support, or main-branch features with
published binaries. Recheck all launch commands before proposing them as runnable.

Evaluate rather than assume
1. Inspect actual Windows/WSL/Linux, CPU/RAM/GPU/VRAM and driver constraints using
   read-only commands. A 6-GB-class laptop is the target constraint to verify, not
   a claim that a published benchmark will fit. No paid services or hardware.
2. Compare setup/maintenance/license/asset costs, ArduPilot version compatibility,
   reset/stepping/headless operation, CI feasibility and deterministic control.
3. Compare camera intrinsics/distortion/extrinsics, streaming into MP GStreamer,
   optical delay/capture timestamps, image/segmentation/depth ground truth, moving
   objects, occluders and repeatable grass/wall/landing-gear scenes.
4. Inspect real gimbal axes, actuator commands, mount feedback and camera transforms.
   For AAS inspect sensor_gimbal, camera code, /gimbal_state and /gimbal_pitch_cmd;
   determine yaw/pitch/roll support instead of inferring it from pitch alone.
   Upstream YOLO boxes may be an independent oracle, not a replacement for NOMAD's
   real detector or evidence that NOMAD processed pixels.
5. Evaluate quadcopter first, then the intended front-two-motor tilt-tricopter
   QuadPlane. Distinguish fixed-wing, VTOL and transition coverage. A generic quad
   model cannot qualify the actual tiltrotor's dynamics or transition behavior.
6. Check current last_letter Plane/Copter recipes and ArduPilot bridge. The current
   README includes the candidate command:
   sim_vehicle.py -v ArduCopter --model last_letter:ardupilot_copter
   Verify prerequisites, versions, bridge protocol, models, timing and lifecycle.
   Evaluate ardupilot_copter_linear only for its declared research envelope.
   Explain what last_letter can test without a rendered camera, and what needs
   the selected renderer. Do not make it an obligatory second rendering stack.
7. Assess Renode only as an optional real-firmware/driver tier. Verify exact board
   support, source pins and prerequisites; do not treat it as camera rendering.

Architecture boundaries
- Mission Planner owns the aircraft connection, MAVLink reader and normal flight
  setup/commands. ArduPilot owns flight execution, mount behavior and failsafes.
  NOMAD sends custom controls through its existing captured native MP APIs.
- A development test harness may orchestrate isolated simulators and sensor
  bridges. Keep that outside product startup/install/runtime dependencies. Add
  only connectors the chosen test path actually needs; no generic simulator bus,
  second NOMAD flight backend, independent product reader or MAVSDK revival.
- The command path under test must be NOMAD UI/controller -> native MP -> ArduPilot
  -> observed mount/servo/backend output -> simulated gimbal -> camera pixels.
  Do not move the camera directly from NOMAD's desired target to fake success.
- Ground-truth poses, boxes and world coordinates belong to the independent oracle.
  Do not feed them into NOMAD detection/tracking/control to make tests pass.
- Model gimbal rate/travel, response lag, limits, frame conventions and actuator
  saturation. CADDX reports can be demanded targets, not measured pose. Distinguish
  submitted command, native acceptance, output telemetry and actual rendered pose.
- Preserve source/stream/aircraft/profile/selection generations, manual takeover,
  tracking ownership, native request serialization and existing failsafes. An
  already-dispatched host helper may finish after Stop; do not claim cancellation.
- Keep normal operator profiles/cache/MP installation and real USB/COM aircraft
  untouched. Plan owned MP state, disabled automatic connections, isolated ports,
  fresh simulator state, explicit binaries/source hashes, process deadlines and
  cleanup that stops only owned processes. No physical arming/output/flash/reset.
- The old conference branch/evidence may be read as reference if present. Do not
  restore its work queue, copy demo flight UI into main or overwrite preserved data.
- Traffic/DAA is a future idea, not an existing NOMAD adapter. Current firmware may
  disable scripting; do not adopt an experimental Lua planner or change native
  termination policy as a side effect of this simulation plan.

Define a layered automated qualification matrix
A. Fast native unit/geometry/ownership tests and deterministic image replay.
B. Headless ArduPilot SITL/last_letter telemetry and mount-output checks.
C. Small rendered end-to-end camera scenarios on the target laptop.
D. Optional heavier GPU/board-emulation scenarios only if justified.
Reuse existing tests and separate what each layer can establish.

Include the following cases, with positive coverage as well as false-accept checks:
- Named model A->B->C switching, cold/failed/canceled loads, class filtering,
  live video continuity, old-result rejection and resource disposal.
- Moving/scaling Object targets, changing appearance, illumination and foreground
  crossings. A wrong occluder/landing-gear target must never be accepted as success.
- Fixed Area under independent aircraft translation/altitude/yaw and gimbal
  pitch/yaw/roll. Check the canonical world-plane point rather than bbox center.
- Repeating grass/wall patterns with and without distinctive context; duplicate
  placements, parallax, minification, partial/full occlusion and out-of-frame return.
- Retained identity, three verified recovery frames, zero newly admitted steering
  while Searching/Outside, and no automatic rearming after Stop/manual/stale input.
- Wrong ground plane, wrong/coarse GPS prior, GPS loss, HOME change, missing pose,
  reported/commanded-pose mismatch and capture/receipt delay. Hints cannot establish
  visual verification, relocate the anchor or starve valid global matching.
- Heading wrap at +/-180, calibrated axis signs, asymmetric travel, slow/saturated
  gimbal, multiple tracking owners, read-only/retired/wrong aircraft and unload.
- Stream restart/dropout, reordered/stale packets and frozen upstream pixels while
  decoded receipts remain fresh. Expose current capture-time observability limits;
  do not invent a passing freeze detector that the actual product lacks.
- Embedded/full-screen heading-line direction and stale/unknown suppression, fixed
  tracking-toggle width/state, model/editor drafts and consistent theme.
- Long-run memory/CPU/process cleanup and repeated fresh-state runs.

For every test specify fixtures, actual command path, independent truth, one action,
observations, numeric thresholds with rationale, deadline/diagnostics, artifacts and
pass/fail. Include accepted-target pixel/angular error, false acceptance, verified
coverage, reacquisition time, request-rate/travel compliance, video latency/FPS,
CPU/RAM/VRAM and cold/warm duration. An always-searching tracker cannot pass a
positive case. Desired pose equaling desired pose is not an independent oracle.
Use reproducible seeds and no hidden reseeding/parameter tweaks to rescue a run.
Record camera capture, simulation and host receipt clocks separately; define truth
alignment and reset semantics rather than assuming wall-clock arrival timings.
Source-bound software/SITL evidence is not physical flight or competition approval.

Deliverable
Write one concise researched plan under ignored local/ (for example
local/simulation/PLAN.md), with:
1. Current-state findings and versioned primary-source capability matrix.
2. Recommended renderer plus the justified complementary role of last_letter.
3. Small architecture/ownership diagram and concrete files/symbols to touch.
4. Prioritized test matrix and complete first vertical-slice acceptance criteria.
5. Bounded implementation phases/PRs, exact prerequisite/build/run/check commands,
   local-versus-CI split, resource budgets, failure cleanup and evidence manifests.
6. Essential unknowns/risks and alternatives only where they affect the decision.

Do not install/download large toolchains, build simulators, launch an existing
Mission Planner installation, change operator data/parameters, implement code,
commit/push/merge, deploy or flash. Small existing isolated checks are acceptable
if needed to falsify a claim. This task authorizes research and a plan only.
```

The related [brainstorming page](brainstorming.md) records the traffic/DAA idea
and source qualifications separately from this simulator-planning task.
