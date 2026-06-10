<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# Traceability — requirement → code → test

The most aviation-like artifact in the safety case and the cheapest to keep
honest. Each row maps a safety requirement from [requirements.md](requirements.md)
to the **code symbol** that implements it and the **test** that proves it.

In a later phase (rearchitecture plan §4.2) a CI test will parse this table and
fail if a referenced code symbol or test name disappears — that is what keeps the
matrix from rotting. **Until that gate lands, this table is maintained by hand;
treat a missing test cell as a real gap, not a formatting nicety.**

Columns:
- **Requirement** — ID from [requirements.md](requirements.md).
- **Code symbol** — `module::symbol` implementing it.
- **Test** — `test_file::test_name` proving it, or **(none)**.

Post-Phase-2: the velocity-path decision logic now lives in the pure
`edge_core/safety/` package (100% line coverage), so most cells below point there
rather than at the threaded `mavlink_velocity.py` adapter.

| Requirement | Code symbol | Test |
|-------------|-------------|------|
| SR-VEL-01 | `safety/limits.py::VelocityLimits.clamp_command` (`max_velocity_xy`) | `tests/test_safety_limits.py::test_clamp_command_clamps_each_axis_independently`; `tests/test_safety_envelope.py::test_allows_and_clamps_before_converting` |
| SR-VEL-02 | `safety/limits.py::VelocityLimits.clamp_command` (`max_velocity_z`, `max_yaw_rate`) | `tests/test_safety_limits.py::test_clamp_command_clamps_each_axis_independently` |
| SR-VEL-03 | `safety/limits.py::clamp` (raises on non-finite) → `evaluate` rejects "nonfinite" | `tests/test_safety_limits.py::test_clamp_rejects_nonfinite`; `tests/test_safety_envelope.py::test_rejects_nonfinite_command` |
| SR-VEL-04 | `safety/envelope.py::evaluate` (FLU→FRD negation) | `tests/test_safety_envelope.py::test_allows_and_frame_converts_when_all_gates_pass`; `tests/test_mavlink_velocity.py::test_submit_frame_conversion_negates_y_z_yaw` (adapter) |
| SR-VEL-05 | `safety/envelope.py::evaluate` (armed/guided gates) | `tests/test_safety_envelope.py::test_rejects_when_not_armed`, `::test_rejects_when_not_guided`, `::test_armed_gate_precedes_mode_gate` |
| SR-VIO-01 | `safety/gates.py::vio_ready` via `safety/envelope.py::evaluate` | `tests/test_safety_gates.py::test_vio_ready_requires_all_three_conditions`; `tests/test_safety_envelope.py::test_rejects_when_vio_unhealthy`, `::test_rejects_when_vio_low_confidence`, `::test_rejects_when_vio_stale` |
| SR-VIO-02 | `safety/watchdog.py::watchdog_decision` (VIO-stale branch) | `tests/test_safety_watchdog.py::test_vio_stale_stops_with_reason` |
| SR-LNK-01 | `safety/gates.py::heartbeat_fresh` via `evaluate` | `tests/test_safety_gates.py::test_heartbeat_stale_past_timeout`; `tests/test_safety_envelope.py::test_rejects_when_heartbeat_stale`; `tests/test_mavlink_velocity.py::test_submit_rejected_without_heartbeat` (adapter) |
| SR-LNK-02 | `safety/watchdog.py::watchdog_decision` (`command_timeout_s`) | `tests/test_safety_watchdog.py::test_command_timeout_stops_with_reason`, `::test_at_timeout_boundary_does_not_stop` |
| SR-LNK-03 | `mavlink_velocity.py::MavlinkVelocityController.stop` → `_send_stop` | `tests/test_mavlink_velocity.py::test_stop_sends_zero_velocity` |
| SR-FEN-01 | C# `Geofence/MPFenceUploader.cs`, `BoundaryManager.cs` | **(none)** — FC-side, manual |
| SR-FEN-02 | `safety/geofence.py::is_contained`, `point_in_polygon`, `distance_to_boundary` (**primitive only; not wired**) | `tests/test_safety_geofence.py` (suite) |
| SR-PAY-01 | `modules/payload/servo.py::ServoController.set_channel_pwm`, `MavlinkServo.set_angle` | **(none)** — H-06 GAP |
| SR-PAY-02 | `modules/payload/servo.py::ServoController.trigger_water_shooter` (`finally` de-energize) | **(none)** — H-06 GAP |
| SR-PAY-03 | *(no interlock yet)* | **(none)** — H-06 GAP |
| SR-SEC-01 | `services/mavlink/commands.py::MavlinkCommands` (surface contains no failsafe-disable) | **(none)** — by construction |
| SR-SEC-02 | `api.py` auth middleware | `tests/test_auth_middleware.py` (suite) |
| SR-SEC-03 | *(target — §4.6)* | **(none)** |

## Coverage summary

| | Count |
|---|---|
| Requirements with a proving test | 13 |
| Requirements implemented but untested / partial (🟡) | 3 |
| Requirements not yet implemented (🔴) | 3 |

`edge_core/safety/` itself is at **100% line and branch coverage** (limits, gates,
watchdog, envelope, geofence), enforced by the `cov-safety` gate (see below). The
remaining untested cells are the not-yet-extracted payload path (SR-PAY-*) and
the security-as-safety targets (SR-SEC-01/03).

## Machine-checked mappings (normative — parsed by CI)

The block below is the **source of truth** that
`tests/test_safety_traceability.py` parses on every test run: it imports each
code symbol and asserts each test exists. If a referenced symbol or test is
renamed/removed without updating this block, CI fails — that is what keeps the
human table above from rotting (DO-178C §6 bidirectional traceability).

Format: `REQ | python.module.path:Dotted.Symbol | tests/test_file.py::test_name`.
Only the ✅ requirements (real code + proving test) appear here; 🟡/🔴 rows have
nothing to check yet.

```traceability
SR-VEL-01 | edge_core.safety.limits:VelocityLimits.clamp_command | tests/test_safety_limits.py::test_clamp_command_clamps_each_axis_independently
SR-VEL-02 | edge_core.safety.limits:VelocityLimits.clamp_command | tests/test_safety_limits.py::test_custom_limits_are_respected
SR-VEL-03 | edge_core.safety.limits:clamp | tests/test_safety_limits.py::test_clamp_rejects_nonfinite
SR-VEL-04 | edge_core.safety.envelope:evaluate | tests/test_safety_envelope.py::test_allows_and_frame_converts_when_all_gates_pass
SR-VEL-05 | edge_core.safety.envelope:evaluate | tests/test_safety_envelope.py::test_rejects_when_not_armed
SR-VEL-06 | edge_core.safety.gates:heartbeat_from_vehicle | tests/test_safety_gates.py::test_heartbeat_from_vehicle_rejects_gcs
SR-VIO-01 | edge_core.safety.gates:vio_ready | tests/test_safety_gates.py::test_vio_ready_requires_all_three_conditions
SR-VIO-02 | edge_core.safety.watchdog:watchdog_decision | tests/test_safety_watchdog.py::test_vio_stale_stops_with_reason
SR-LNK-01 | edge_core.safety.gates:heartbeat_fresh | tests/test_safety_gates.py::test_heartbeat_stale_past_timeout
SR-LNK-02 | edge_core.safety.watchdog:watchdog_decision | tests/test_safety_watchdog.py::test_command_timeout_stops_with_reason
SR-LNK-03 | edge_core.ros_http_bridge.mavlink_velocity:MavlinkVelocityController.stop | tests/test_mavlink_velocity.py::test_stop_sends_zero_velocity
SR-FEN-02 | edge_core.safety.geofence:is_contained | tests/test_safety_geofence.py::test_is_contained_with_keep_in_margin
SR-SEC-02 | edge_core.api:create_app | tests/test_auth_middleware.py::test_no_key_blocks_remote
```

## Verification roadmap (remaining)

1. Wire `safety/geofence.py` into the command path (SR-FEN-02 enforcement) and
   extract/test the payload guarantees (SR-PAY-01/02) + interlock (SR-PAY-03).
2. Run the SITL loop-closure scenarios for the failsafe branches against the
   `pixi run dev-up` stack (arm→GUIDED→velocity step→watchdog timeout→confirm
   stop; geofence approach→confirm containment). The compose profile exists; the
   scripted scenarios are the open item.
