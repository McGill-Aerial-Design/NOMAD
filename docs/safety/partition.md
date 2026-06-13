<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# Safety Partition — SC / SR / NC

This is the core architectural decision of the safety case. Every source file in
NOMAD belongs to exactly one of three criticality tiers. The tier determines how
much rigor applies to it (tests, review, change control) and — critically — what
it is **allowed to call**.

## The tiers

| Tier | Definition | Rigor |
|------|-----------|-------|
| **SC — Safety-Critical** | Code whose misbehavior can command unintended aircraft motion, payload/relay actuation, or suppress a failsafe. | Full: testable requirements + traceability, enumerated failure modes, fault injection, ~branch coverage, change control. |
| **SR — Safety-Related** | Code that feeds SC code or the operator's safety picture (freshness sources, health, link failover, the auth boundary, trusted telemetry). | Strong: typed, tested, reviewed; defined failure outputs; no formal traceability matrix. |
| **NC — Non-Critical** | Everything else: dashboards, video, 3D mesh, terminal, config UI, docs tooling. | Normal hygiene: lint, types, basic tests. |

## The partitioning rule

> **NC code may never call into SC code except through a narrow, validated SC
> API. SC code may never import NC code.**

This is the software analogue of DO-178C / ARP4754A partitioning. It is the
property that keeps a crashing video widget from ever touching a velocity
setpoint. This rule is **enforced by tooling**:
[`tests/test_safety_partition.py`](../../tests/test_safety_partition.py) asserts
the SC core (`edge_core/safety/`) imports only the standard library and itself,
and that the rest of `edge_core/` reaches it only through its public API. It runs
on every PR — a violation fails CI.

## Tier membership — Python edge (`edge_core/`)

### SC — Safety-Critical

| File / symbol | Why SC |
|---------------|--------|
| **[safety/](../../edge_core/safety/)** — `limits.py`, `gates.py`, `watchdog.py`, `envelope.py`, `geofence.py`, `payload.py` | **The extracted SC decision core.** Dependency-light, pure-logic: velocity/yaw clamps + finite checks (`limits`), freshness/arm/mode gates (`gates`), command-timeout/VIO-stale failsafe (`watchdog`), the single "is this command allowed?" entry point (`envelope`), boundary containment (`geofence`), and payload validation + release interlock (`payload`). 100% branch coverage, CI-gated. Misbehavior here = uncommanded motion, flyaway, or unintended release. |
| [ros_http_bridge/mavlink_velocity.py](../../edge_core/ros_http_bridge/mavlink_velocity.py) — `MavlinkVelocityController` | The autonomous velocity command path's I/O **adapter** (threads, MAVLink link, HEARTBEAT parse). After Phase 2 it owns no safety decisions itself — it snapshots state under its lock and defers to `safety.evaluate` / `safety.watchdog_decision`. Still SC because it is the thing that actually transmits setpoints. |
| [services/mavlink/commands.py](../../edge_core/services/mavlink/commands.py) — `arm_disarm`, `set_mode`, `land`, `takeoff`, `send_velocity_command`, `send_global_position_target`, `send_position_target`, `trigger_payload`, `set_relay` | Direct MAVLink command surface: arm/disarm, mode changes (incl. LAND), takeoff, position/velocity targets, servo + relay actuation. Each can move the aircraft or fire the payload. |
| [modules/payload/servo.py](../../edge_core/modules/payload/servo.py) — `ServoController.trigger_water_shooter`, `arm_release`, `set_channel_pwm`, `MavlinkServo.set_pwm` | Payload / relay actuation **adapter**: the decisions (channel/PWM ranges, duration clamp, arm→release interlock) live in `safety/payload.py`; this file owns MAVLink I/O, the interlock state under its lock, and the de-energize-in-`finally` guarantee. |

> **Geofence containment (SR-FEN-02).** The pure containment decision lives at
> [safety/geofence.py](../../edge_core/safety/geofence.py) (point-in-polygon,
> distance-to-boundary, keep-in margin, `evaluate_position`) and is **wired into
> the command path**: `send_global_position_target` / `send_position_target` in
> [services/mavlink/commands.py](../../edge_core/services/mavlink/commands.py)
> refuse targets outside the optional `NOMAD_FENCE_POLYGON` boundary. The
> lat/lon→planar projection happens in the adapter via
> [services/geospatial.py](../../edge_core/services/geospatial.py); the SC core
> stays frame-agnostic. The FC's own fence (uploaded from the C# side) remains
> the independent backstop. SITL proof: `pixi run sitl-fence` — see
> [hazards.md](hazards.md) H-05 for what has actually been run.

### SR — Safety-Related

| File / symbol | Why SR |
|---------------|--------|
| [api.py](../../edge_core/api.py) — API-key auth middleware | The auth boundary. An unauthorized command is a hazard (DO-326A spirit). Covered by [tests/test_auth_middleware.py](../../tests/test_auth_middleware.py). |
| [services/health_monitor.py](../../edge_core/services/health_monitor.py) | The operator's health/safety picture; feeds go/no-go decisions. |
| [services/mavlink/connection.py](../../edge_core/services/mavlink/connection.py) | Link transport the SC command surface rides on; heartbeat freshness source. |
| VIO / odometry freshness source feeding `MavlinkVelocityController.note_vio` (the VIO module + [api_routes/vio.py](../../edge_core/api_routes/vio.py)) | The freshness signal the velocity gate trusts. Stale/false-healthy here defeats an SC gate. |
| [services/geospatial.py](../../edge_core/services/geospatial.py) | Pure geo math that may feed targeting/containment decisions. Pure functions, well-tested ([tests/test_geospatial.py](../../tests/test_geospatial.py)). |
| [services/time_manager.py](../../edge_core/services/time_manager.py) | Timestamps underpinning every freshness/staleness judgement. |

### NC — Non-Critical

Most of the rest: [services/video_stream_manager.py](../../edge_core/services/video_stream_manager.py),
[api_routes/streaming.py](../../edge_core/api_routes/streaming.py),
[api_routes/terminal.py](../../edge_core/api_routes/terminal.py),
[api_routes/video_slam.py](../../edge_core/api_routes/video_slam.py),
[services/logging_service.py](../../edge_core/services/logging_service.py),
the SLAM/mesh paths, and the module SDK plumbing in [core/](../../edge_core/core/)
(the SDK *wires* modules but does not itself command the aircraft).

## Tier membership — C# ground station (`mission_planner/src/`)

### SC — Safety-Critical

| File | Why SC |
|------|--------|
| [Control/FlightModeController.cs](../../mission_planner/src/Control/FlightModeController.cs) | Commands FC mode changes from the GCS. |
| [Control/CubeOutputController.cs](../../mission_planner/src/Control/CubeOutputController.cs) | Drives Cube servo/relay outputs (payload actuation from the GCS). |
| [Payload/PayloadReleaseInterlock.cs](../../mission_planner/src/Payload/PayloadReleaseInterlock.cs) — the drop / momentary-relay arm→confirm interlock | The release decision goes through a **pure, unit-tested SC interlock**: N deliberate clicks within a rolling window authorize exactly one actuation, then it disarms. `PayloadControlPanel` is now chrome that renders the returned outcome and drives the visual revert. Rearchitecture §3.3 — **closed** (decision logic extracted from the panel; covered by [tests/PayloadReleaseInterlockTests.cs](../../mission_planner/tests/PayloadReleaseInterlockTests.cs), the GCS counterpart of the edge-side `safety/payload.py` interlock). |
| Geofence enforcement: [Geofence/BoundaryManager.cs](../../mission_planner/src/Geofence/BoundaryManager.cs), [Geofence/MPFenceUploader.cs](../../mission_planner/src/Geofence/MPFenceUploader.cs), [Geofence/MapOverlayManager.cs](../../mission_planner/src/Geofence/MapOverlayManager.cs) | Define and upload the containment boundary to the FC. The boundary the aircraft is actually held to. |

> **REMOVED — `GuidedRthLandingController.cs`.** The previous GCS-side RTH/landing
> state machine was removed (it was orphaned — constructed nowhere — and poorly
> implemented). RTH/landing will be **re-implemented** later as a pure, testable
> state machine per the original §3.3 intent. Until then, return-to-home relies
> on ArduPilot's own RTL/LAND modes (the certified inner layer). No NOMAD SC code
> currently owns the RTH/landing sequence.

### SR — Safety-Related

`DualLinkSender` transport/failover (link redundancy the SC commands ride on),
[Control/GimbalController.cs](../../mission_planner/src/Control/GimbalController.cs)
(non-flight actuation but FC-commanding), telemetry the operator trusts.

### NC — Non-Critical

`Views/`, `Panels/` (chrome), `Media/`, `SLAM3D/Rendering/`, terminal, config UI.

## Defense in depth

NOMAD's SC code is the **outer** layer of safety. ArduPilot on the FC is the
inner, certified layer and enforces its own arming checks, GUIDED-mode
preconditions, geofence, and failsafes independently. NOMAD's job is to never
*command* something unsafe and to **relinquish to the FC** on any doubt — not to
be the last line of defense. Every SC failure mode in [hazards.md](hazards.md)
resolves to one of: command zero velocity, hold, or hand back to the FC.
