# Safety and qualification status

This is the current summary of what the repository proves and what remains
unqualified. The source-arbitration qualification slice started from `main` at
`34d93335c41a000d78a323436e9027704bffc160` after PR #52. It adds a disarmed
production-runtime SITL scenario and a physical qualification procedure without
changing the production command surface or aircraft parameters. The executable
mutation scenario uses Copter; current runtime v1 mutations are unsupported for
QuadPlane, and that capability gate remains intact.
A test, passing workflow or simulator result proves only the boundary and
revision it actually exercised. This page is not flight authorization.

## Evidence levels

| Evidence | What it establishes | What it does not establish |
|---|---|---|
| Unit and software integration tests | C++ policy against fake transports; request validation and lifecycle; client translation; profile/template handling; ROS and Mission Planner software boundaries | MAVLink delivery to a real controller, radio behavior, physical actuation or flight safety |
| Deterministic MAVLink peer | MAVSDK wire encoding, selected retry/admission behavior and independently counted UDP frames against a local peer | Flight-controller acceptance, aircraft outcome, RF behavior or pilot takeover |
| ArduPilot SITL | Simulated state transitions and observed outcomes for the exact simulator profile, source revision and scenario | Hardware behavior, radio independence, physical takeover, payload mechanics or competition readiness |
| Hardware or flight qualification | Only the aircraft, firmware, equipment, procedure and outcomes observed in a reviewed record | Any untested profile, phase, fault or configuration |

Do not promote evidence from one row to the next. The [safety case](safety.md)
contains stable requirement mappings and fault-path detail. Dated logs,
historical task names and run-specific reports remain in the
[migration evidence archive](migration.md).

## Proven software boundaries

The [source-arbitration model](source-arbitration.md) lists every intended
source, the software/SITL/bench/flight distinctions, controller mechanisms,
unresolved production RC inputs and the later hardware procedure. `handback`
explicitly returns software authority to NOMAD after `revoke`; it does not
establish pilot control. External mode changes do not revoke runtime ownership.

- The production runtime starts without an admitted software command owner.
  Typed mutations require explicit authority admission and carry the runtime
  incarnation, vehicle session, owner, generation, sequence and expiry. Revoke
  and explicit handback advance the generation. Reconnect, runtime restart and
  response-cache eviction do not restore authority or make an old request
  executable. See [runtime IPC](runtime-ipc.md) and
  [runtime authority tests](../tests/runtime_ipc_test.cpp).
- The pinned MAVSDK fork carries that admission through `COMMAND_LONG` and
  `COMMAND_INT` retries to final UDP delivery. The independent peer fixtures
  observe suppressed frames after revocation, including retry and queued-send
  cases. This guarantee is scoped to those command encodings and the tested UDP
  path; it is not a general fence for every MAVLink message or transport. See
  [`test-mavsdk-authority-wire`](development.md#mavsdk-transport-and-authority-checks),
  [wire fixtures](../scripts/dev/mavsdk_authority_wire_fixture.py), and the
  [MAVSDK provenance record](mavsdk-dependencies.md).
- Product profiles set `NOMAD_INTEGRATED_FLIGHT=1`, which inhibits direct
  actuation by the separately built `nomad-qualification` test tool. That tool
  is non-installed and exists for SITL and transport qualification. Template
  checks are in [test_deployment_profiles.py](../tests/test_deployment_profiles.py).
- The installed `nomad` CLI uses runtime IPC only. Mission Planner's supported
  plugin requests also use runtime IPC. Unsupported v1 requests, including
  GuidedGoto, return unavailable without a direct-vehicle fallback. See the
  [runtime client regression](../tests/test_runtime_wiring.py) and
  [Mission Planner client tests](../mission_planner/tests/coreclient/NomadCoreClientTests.cs).
- The standalone ground router rejects command egress from its
  `mission_planner` consumer. It still transports traffic; it does not authorize
  flight actions, authenticate consumer names or block unrelated external
  sources. See [router review tests](../mission_planner/tests/duallink/RouterReviewTests.cs).
  The aircraft-side `mavlink-router` is a different process.
- ROS 2 uses a receive-only MAVLink observer and publishes validated GPS and
  battery samples. It exposes no vehicle command or VIO submission interface.
  Its direct observer socket is separate from runtime IPC because protocol v1
  does not yet supply the source measurements ROS needs. See the
  [ROS integration suite](../tests/ros/test_nomad_ros_services.py) and
  [observer contract tests](../tests/test_ros_observation_boundary.py).
- Core and adapter tests cover decision-specific telemetry freshness, invalid
  values, payload release interlocks and failure handling. The pinned QuadPlane
  landing operation requires post-command descent, landed-state telemetry,
  disarm and a stable final envelope; an ACK alone is not reported as touchdown.
  These are software and simulator proofs, not physical payload or aircraft
  outcomes. See [core safety tests](../tests/safety_test.cpp) and
  [QuadPlane landing tests](../tests/vehicle/quadplane/quadplane_vtol_landing_test.cpp).

## SITL and ROS readiness

| Workflow | Trigger and scope | Evidence limit |
|---|---|---|
| [test.yml](../.github/workflows/test.yml) | Pull requests and pushes: Python, C++ core, and deterministic MAVSDK qualification | No live flight-controller or hardware evidence |
| [lint.yml](../.github/workflows/lint.yml) | Pull requests and pushes: changed-source checks, C++ tests, Ruff, ShellCheck, and the strict docs build | Does not qualify vehicle behavior |
| [ros-sim.yml](../.github/workflows/ros-sim.yml) | Pull requests and relevant pushes: CPU ROS Humble image plus the real observer node and an in-process MAVLink responder | Tests translation, freshness and receive-only behavior; no live SITL, camera, GPU or flight |
| [sitl.yml](../.github/workflows/sitl.yml) main push | Path-triggered reduced Copter connectivity smoke | The successful exact-base run [36659200606](https://github.com/YoussGm3o8/NOMAD/actions/runs/36659200606) at `042c980` covered MAVSDK connect/status only; full SITL jobs were skipped for the push event |
| [sitl.yml](../.github/workflows/sitl.yml) schedule / manual dispatch | Full Copter scenarios, disarmed simulator RC-fault delivery probe and pinned QuadPlane operation chain | The latest completed full run before `042c980` was [36576342974](https://github.com/YoussGm3o8/NOMAD/actions/runs/36576342974) at `cd9eb4e`; it is evidence for that SHA, not a full run at `042c980` |
| [docker.yml](../.github/workflows/docker.yml) manual dispatch | Jetson ARM64 image and Isaac ROS GPU image on self-hosted runners | Jetson requires an ARM64 runner; Isaac ROS requires its base image and an NVIDIA GPU. Standard hosted CI has neither |

The pinned QuadPlane run exercises ArduPlane 4.7.1 commit
`dbe792162d06cab66c3475fd5556bf7a120f119e` with the `quadplane-tilttri`
profile and `Q_ENABLE=2`: identity and telemetry, arm and VTOL takeoff,
VTOL-to-fixed-wing transition, a two-point fixed-wing route, recovery,
fixed-wing-to-VTOL transition and QLAND landing. The independent qualification
driver/operator establishes some setup states, including AUTO; this is not an
autonomous end-to-end Task 1 flight. See the
[SITL workflow](../.github/workflows/sitl.yml),
[SITL task list](../pixi.toml), and [scenario notes](../tests/sitl/README.md).

The exact-base CPU ROS integration run
[36659200578](https://github.com/YoussGm3o8/NOMAD/actions/runs/36659200578)
passed at `042c980`. It uses the CPU-only `nomad-sim-ros` image and an
in-process MAVLink responder. The node is an observer; this run is not a real
ArduPilot SITL flight or a perception/VIO qualification.

Local Copter SITL requires Docker and the `nomad-sitl:copter-4.7.1` image;
the Compose file documents how to build it. The pinned QuadPlane image is built
from `docker/Dockerfile.sitl-plane`. CPU ROS tests require Docker to build
`nomad-sim-ros:latest`, but do not require the SITL stack. GPU/Jetson workflows
need their compatible hardware and base images. See the exact local commands in
[development](development.md#sitl-and-live-mavsdk-smoke).

## Runtime source qualification evidence

The new [runtime authority SITL scenario](../scripts/dev/core_sitl_authority.py)
and its [run procedure](../tests/sitl/README.md#runtime-authority-and-independent-source)
are wired into scheduled/manual Copter CI with a lighter PR guard/observer
gate. The local disarmed Copter run on 2026-09-30 passed at implementation SHA
`aff6d152f3f73e0ef45dd5ae8eefa9ee5e23863b`, with firmware
`dbe792162d06cab66c3475fd5556bf7a120f119e`. Its source-dirty flag records
preserved pre-existing workspace changes; this is not clean-checkout evidence.
The same scenario passed from a clean hosted checkout in
[manual SITL run 36672382568](https://github.com/YoussGm3o8/NOMAD/actions/runs/36672382568)
at that exact implementation SHA. The `runtime-authority-sitl` artifact records
all nine checks, actual parameter readbacks and source IDs; its scope is
`disarmed_pinned_copter_runtime_software_authority_boundary_only`.
Fresh FC output observations matched runtime requests, source 250 changed FC
modes while NOMAD was admitted and revoked, and two observed retries with
dropped FC ACKs stopped after revoke. Link recovery changed the session without
restoring an owner; runtime restart rejected old context and required new
admission. The initial simulator output was restored and observed after runtime
shutdown. These results complete the tested Copter/software slice only.

The full manual run above finished with the complete pinned QuadPlane chain
passing, including the disarmed RC-fault probe and QLAND landing. Its Copter job
passed authority, telemetry, connectivity, command flow, mission, velocity
watchdog, payload, link-loss, zero-delivery and link-recovery checks, then failed
the existing GCS-heartbeat relay cadence assertion: an observed announcement
interval was `0.000s`, below the unchanged 0.9 s bound. The later velocity-loop,
geofence-containment and geofence-upload checks were skipped. Three isolated
local closed-gate checks passed without reproducing this failure; its cause is
unresolved. Retain the failed hosted record: this PR does not claim a complete
Copter workflow pass or relax the heartbeat gate.

## Not yet equivalent to qualification

Native GCS
source-250 mode acceptance in a disarmed simulator is simulated external-source
evidence, not physical pilot takeover. Copter output evidence does not qualify
a QuadPlane runtime mutation; protocol v1 currently has none that can execute.
The standalone production router and
aircraft-side router are absent from this scenario's direct simulator topology.

- Runtime authority is one admitted **software source** for typed requests to
  that runtime. It is not whole-aircraft authority. Native Mission Planner
  controls, pilot/RC input, ArduPilot behavior, maintenance tools and external
  MAVLink sources remain outside that guarantee.
- The retry gate does not prove RC/ELRS priority, physical pilot takeover or
  handback. It covers `COMMAND_LONG`/`COMMAND_INT` retries through tested final
  UDP delivery. Offboard setpoints and fence transfers are not runtime IPC v1
  commands and do not share that final-send guarantee.
- The final production RC channel map, physical RC/ELRS arbitration, complete
  C2-loss policy and production termination behavior remain unqualified. The
  plugin's termination requests report unavailable and send no substitute
  command. The disarmed `SIM_RC_FAIL` probe proves simulator fault delivery and
  restoration only.
- The QuadPlane operation chain is pinned SITL evidence. It does not qualify
  every transition/recovery scenario on real aircraft, transition-phase
  termination, QuadPlane link-loss response, generic mode/takeoff/goto/land or
  the complete Task 1 flight.
- No hardware or flight qualification is recorded for the product profiles,
  manual takeover, termination, physical payload release, gimbal motion,
  aircraft endurance or competition readiness. Profile tests validate
  configuration files; they do not prove a deployed system works.

Track unresolved work in [TODO](../TODO.md). Keep test, peer, SITL and hardware
records tied to their exact revision and profile.
