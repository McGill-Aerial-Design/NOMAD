<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# Traceability — requirement → code → test

The most aviation-like artifact in the safety case and the cheapest to keep
honest. Each row maps a safety requirement from [requirements.md](requirements.md)
to the **code symbol** that implements it and the **test** that proves it.

The normative `traceability` code block below is parsed by
`tests/test_safety_traceability.py` on every test run, which fails if a mapped
code symbol or test disappears (DO-178C §6) — that is what keeps the matrix from
rotting. The human-readable table here is its companion, maintained by hand;
treat a missing test cell as a real gap, not a formatting nicety.

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
| SR-VEL-06 | `safety/gates.py::heartbeat_from_vehicle` (filter HEARTBEAT to commanded autopilot) via `evaluate` | `tests/test_safety_gates.py::test_heartbeat_from_vehicle_rejects_gcs` |
| SR-VIO-01 | `safety/gates.py::vio_ready` via `safety/envelope.py::evaluate` | `tests/test_safety_gates.py::test_vio_ready_requires_all_three_conditions`; `tests/test_safety_envelope.py::test_rejects_when_vio_unhealthy`, `::test_rejects_when_vio_low_confidence`, `::test_rejects_when_vio_stale` |
| SR-VIO-02 | `safety/watchdog.py::watchdog_decision` (VIO-stale branch) | `tests/test_safety_watchdog.py::test_vio_stale_stops_with_reason` |
| SR-LNK-01 | `safety/gates.py::heartbeat_fresh` via `evaluate` | `tests/test_safety_gates.py::test_heartbeat_stale_past_timeout`; `tests/test_safety_envelope.py::test_rejects_when_heartbeat_stale`; `tests/test_mavlink_velocity.py::test_submit_rejected_without_heartbeat` (adapter) |
| SR-LNK-02 | `safety/watchdog.py::watchdog_decision` (`command_timeout_s`) | `tests/test_safety_watchdog.py::test_command_timeout_stops_with_reason`, `::test_at_timeout_boundary_does_not_stop` |
| SR-LNK-03 | `mavlink_velocity.py::MavlinkVelocityController.stop` → `_send_stop` | `tests/test_mavlink_velocity.py::test_stop_sends_zero_velocity` |
| SR-FEN-01 | C# `Geofence/MPFenceUploader.cs`, `BoundaryManager.cs` | **Manual (FC-side)** — boundary uploaded to and enforced by the FC fence before flight; verified operationally. No automated NOMAD test by nature (it is a flight-controller function NOMAD configures, not owns). |
| SR-FEN-02 | `safety/geofence.py::evaluate_position` (pure decision over `is_contained`); enforced by `services/mavlink/commands.py::_fence_allows_global` / `_fence_allows_local` in the position-target senders | `tests/test_safety_geofence.py::test_evaluate_position_rejects_outside_boundary` (+ fault-injection siblings); `tests/test_mavlink_fence.py` (adapter suite); SITL: `tests/sitl/geofence_containment.py` — **run & PASS** (2026-06-11, commit f69a137; now gates nightly via `.github/workflows/sitl.yml`) |
| SR-PAY-01 | `safety/payload.py::validate_servo_command`; enforced by `modules/payload/servo.py::ServoController.set_channel_pwm` | `tests/test_safety_payload.py::test_validate_servo_command_rejects_bad_channel` (+ siblings); `tests/test_payload_servo.py::test_set_channel_pwm_rejects_out_of_range` |
| SR-PAY-02 | `safety/payload.py::clamp_release_duration`; `servo.py::trigger_water_shooter` (`finally` de-energize) | `tests/test_safety_payload.py::test_clamp_release_duration_rejects_nonfinite`; `tests/test_payload_servo.py::test_pump_deenergized_when_sleep_raises`, `::test_relay_on_failure_still_attempts_off` |
| SR-PAY-03 | `safety/payload.py::arm_release` / `evaluate_release` (interlock state machine); `servo.py::ServoController.arm_release` / `trigger_water_shooter`; `/api/servo/shooter/{arm,trigger}` routes; C# fire-button confirm | `tests/test_safety_payload.py::test_release_requires_prior_arm` (+ siblings); `tests/test_payload_servo.py::test_release_without_arm_sends_nothing`, `::test_arm_is_consumed_by_each_attempt` |
| SR-SEC-01 | `services/mavlink/commands.py::MavlinkCommands` (surface contains no failsafe-disable) | `tests/test_safety_command_surface.py::test_no_failsafe_disabling_commands` |
| SR-SEC-02 | `api.py` auth middleware | `tests/test_auth_middleware.py` (suite) |
| SR-SEC-03 | `api.py` middleware `_COMMAND_PATH_PREFIXES` (auth-on-loopback + audit log) | `tests/test_auth_middleware.py::test_command_path_requires_auth_even_on_loopback`, `::test_command_path_requests_are_audit_logged` |

## Coverage summary

| | Count |
|---|---|
| Requirements met (✅) | 19 |
| &nbsp;&nbsp;— proven by an automated test | 18 |
| &nbsp;&nbsp;— verified manually, FC-side (SR-FEN-01) | 1 |
| Partial (🟡) | 0 |
| Not yet implemented (🔴) | 0 |

`edge_core/safety/` itself is at **100% line and branch coverage** (limits, gates,
watchdog, envelope, geofence, payload), enforced by the `cov-safety` gate (see
below). The one requirement without an automated NOMAD test is SR-FEN-01: the
boundary is uploaded by the C# `MPFenceUploader` and enforced by the FC fence, and
is verified manually before flight — inherent to it being a flight-controller
function NOMAD configures rather than owns.

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
SR-FEN-02 | edge_core.safety.geofence:evaluate_position | tests/test_safety_geofence.py::test_evaluate_position_rejects_outside_boundary
SR-PAY-01 | edge_core.safety.payload:validate_servo_command | tests/test_safety_payload.py::test_validate_servo_command_rejects_bad_channel
SR-PAY-02 | edge_core.safety.payload:clamp_release_duration | tests/test_payload_servo.py::test_pump_deenergized_when_sleep_raises
SR-PAY-03 | edge_core.safety.payload:evaluate_release | tests/test_safety_payload.py::test_release_requires_prior_arm
SR-SEC-01 | edge_core.services.mavlink.commands:MavlinkCommands | tests/test_safety_command_surface.py::test_no_failsafe_disabling_commands
SR-SEC-02 | edge_core.api:create_app | tests/test_auth_middleware.py::test_no_key_blocks_remote
SR-SEC-03 | edge_core.api:create_app | tests/test_auth_middleware.py::test_command_path_requires_auth_even_on_loopback
```

## Verification roadmap

All SC requirements are verified. The geofence containment SITL scenario
([tests/sitl/geofence_containment.py](../../tests/sitl/geofence_containment.py),
`pixi run sitl-fence`) was executed against the `pixi run dev-up` stack on
2026-06-11 (commit f69a137, PASS) and now runs nightly in
[.github/workflows/sitl.yml](../../.github/workflows/sitl.yml) alongside the
velocity loop-closure scenario — so the loop-closure proof is continuously
re-checked rather than a one-off.
