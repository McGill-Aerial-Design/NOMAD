# NOMAD safety case

**Status:** Transitional evidence; C++ re-verification is required before the
current Python implementation is removed.

NOMAD commands ArduPilot but does not replace its flight-control safety system.
ArduPilot remains the inner safety layer. NOMAD's responsibility is to validate
commands, verify results, handle stale inputs, and relinquish control on doubt.

## Safety tiers

| Tier | Meaning | Expected rigor |
|---|---|---|
| SC | Can command aircraft motion or payload actuation | Requirements, fault tests, traceability, review, SITL |
| SR | Feeds SC decisions or operator safety picture | Typed, tested, reviewed, defined failure output |
| NC | UI, video, docs, tooling, non-command features | Normal engineering hygiene |

The target rule is:

> NC code may not command the vehicle directly. SC code may not depend on UI,
> ROS 2, Python tools, or other NC code.

The C++ core will enforce this through ownership and dependency boundaries. The
current Python partition is transitional.

## Hazards and mitigations

| ID | Hazard | Current mitigation | C++ status |
|---|---|---|---|
| H-01 | Excessive or wrong-axis velocity | Finite checks, clamps, frame conversion | Verified: C++ continuous watchdog path; SITL watchdog stop scenario passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) (2026-09-03) |
| H-02 | Stale VIO flyaway | Freshness, confidence, health gates, watchdog | Verified: C++ VIO gates, validator feed, and watchdog fault tests; SITL watchdog stop scenario passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| H-03 | Link loss while commanding | Heartbeat gate, command timeout, zero command | Verified: C++ watchdog/zero path and deterministic shutdown ordering tests; SITL link-loss injection passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| H-04 | Mode leaves GUIDED | Armed and mode gates from FC heartbeat | Verified: C++ watchdog path; SITL watchdog and command-flow scenarios passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| H-05 | Geofence breach | FC fence plus independent NOMAD containment check | Verified: C++ projected keep-in gate plus mission-fence upload/readback; SITL upload accepted (`MISSION_ACK: TYPE_FENCE: ACCEPTED`) and read back on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| H-06 | Unintended payload release | Range checks, duration clamp, interlock, de-energize cleanup | Verified: C++ validation and relay path; SITL relay acceptance passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04); hardware proof open |
| H-07 | Failsafe suppression | No failsafe-disabling surface; deny-list test | Verified: C++ command surface deny-list and Python scan test |
| H-08 | Unauthorized command | Authenticated command boundary and audit logging | Verified: C++ CLI key gate and audit lines (2026-09-03); adapter boundary docs updated; remote REST surface remains transitional |

The current Python implementation and its tests provide transition evidence. That
evidence does not automatically prove the C++ implementation.

## Safety requirements

Numbers are stable identifiers and must not be reused.

| ID | Requirement | Hazard | Target status |
|---|---|---|---|
| SR-VEL-01 | XY velocity setpoints are clamped to the reviewed limit. | H-01 | Implemented: C++ continuous path; SITL watchdog stop scenario passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| SR-VEL-02 | Vertical and yaw-rate setpoints are clamped to reviewed limits. | H-01 | Implemented: C++ continuous path; SITL watchdog stop scenario passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| SR-VEL-03 | Any non-finite velocity component rejects the complete command. | H-01 | Implemented: C++ path and deterministic tests |
| SR-VEL-04 | Input and MAVLink frames are converted explicitly and correctly. | H-01 | Implemented: C++ path; SITL watchdog stop scenario passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| SR-VEL-05 | Guided velocity requires armed state and GUIDED mode. | H-04 | Implemented: C++ gate and watchdog; SITL command/watchdog scenarios passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| SR-VEL-06 | Heartbeats are filtered to the commanded vehicle. | H-04 | Implemented: UDP transport; C++ unit proof present |
| SR-VIO-01 | Velocity is rejected for unhealthy, low-confidence, or stale VIO. | H-02 | Implemented: C++ gate and VioSourceValidator feed with unit tests; live sensor feed requires hardware |
| SR-VIO-02 | Stale VIO stops active velocity control within the watchdog interval. | H-02 | Implemented: C++ watchdog and deterministic fault test |
| SR-LNK-01 | Commands require a fresh FC heartbeat. | H-03 | Implemented: C++ gate and transport freshness; SITL link-loss injection passed on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04) |
| SR-LNK-02 | Missing velocity input causes a zero command within the timeout. | H-03 | Implemented: C++ watchdog and deterministic fault test |
| SR-LNK-03 | Shutdown sends a zero command before closing an active link. | H-03 | Implemented: Vehicle/UDP zero path with deterministic ordering tests; live transport-level zero-delivery proof remains open |
| SR-LNK-04 | The core announces itself with a standard GCS heartbeat so heartbeat-gated relay legs stream. | H-03 | Implemented: 1 Hz GCS heartbeat in the UDP wait path, pinned byte-for-byte against pymavlink (`nomad_codec_golden_tests`, `nomad_udp_tests`); SITL relay-gate scenario `core-sitl-gcs-heartbeat` wired into CI — Linux/CI evidence recording remains |
| SR-FEN-01 | The FC fence is uploaded, enabled, and verified before autonomous flight. | H-05 | Implemented: C++ mission-fence upload/download with golden wire tests; verification reads FENCE_ENABLE back as authoritative autopilot state and fails closed when disabled; SITL upload, enable read-back, and restore passed on Copter 4.7.1 (2026-09-04) |
| SR-FEN-02 | NOMAD rejects position targets outside the configured boundary. | H-05 | Implemented: C++ projected geofence and Vehicle test; drive-to-boundary SITL containment remains via the transitional scenario |
| SR-PAY-01 | Servo channel and PWM values are validated before actuation. | H-06 | Implemented: C++ validation and Vehicle test |
| SR-PAY-02 | Payload duration is bounded and outputs are de-energized on failure. | H-06 | Implemented: C++ bounded relay path; deterministic relay-on/off tests and SITL relay acceptance on Copter 4.7.0; re-verified on Copter 4.7.1 (2026-09-04); hardware proof open |
| SR-PAY-03 | Release requires an explicit operator interlock. | H-06 | Implemented: C++ consuming interlock and Vehicle test |
| SR-SEC-01 | NOMAD provides no command that disables FC failsafes. | H-07 | Implemented: C++ command surface deny-list and scan test |
| SR-SEC-02 | Command clients authenticate at the trust boundary. | H-08 | Implemented: C++ CLI actuation verbs require NOMAD_API_KEY before any socket work; contract tests (2026-09-03) |
| SR-SEC-03 | Command requests are authenticated and audit-logged. | H-08 | Implemented: C++ CLI audit lines for accepted and refused actuation attempts; contract tests (2026-09-03) |

## Verification policy

Each requirement needs:

1. a C++ core symbol;
2. unit tests for normal, invalid, and boundary inputs;
3. adapter tests proving the intended MAVLink command;
4. SITL evidence where flight behavior is involved;
5. a traceability row linking requirement, code, and test.

Transmission is not success. Critical commands must be acknowledged or verified
through a state change and must return a clear failure on timeout.

## Transition evidence

Until the C++ port is complete, the following current tests remain useful:

- `tests/test_safety_*.py` for current pure safety behavior;
- `tests/test_mavlink_*.py` for current command adapters;
- `tests/sitl/` for current loop closure;
- Mission Planner pure helper tests for payload and geofence behavior.

These tests are migration references. They do not authorize adding new Python
architecture. Port the requirement and test before deleting each old path.

## Traceability format

The machine-checked C++ block below records the current evidence. The retired
The Python transition block was removed on 2026-09-05 when `edge_core/safety/`,
`edge_core/services/mavlink/`, and the vehicle-command REST routes were deleted
(Phase 7 cutover, `docs/migration.md`). The C++ mapping below is the traceability
record: it covers the pure envelope, continuous watchdog, projected geofence,
env fence configuration, payload interlock, fence upload/status path, and
transmission paths. SITL and adapter evidence remain separate gates where stated
above. A missing or partial mapping is an open safety item, not a documentation
problem to hide.

```cpp_traceability
SR-VEL-01 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-02 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-03 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-04 | src/mavlink/protocol.cpp:encode_velocity_setpoint | tests/core_test.cpp::test_velocity_frame_uses_expected_wire_layout
SR-VEL-05 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-06 | src/mavlink/protocol.cpp:accepts_heartbeat | tests/core_test.cpp::test_heartbeat_filter_accepts_vehicle_only
SR-VIO-01 | src/safety/velocity.cpp:evaluate_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VIO-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss
SR-LNK-01 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_link_loss
SR-LNK-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_command_timeout
SR-LNK-03 | src/mavlink/udp_connection.cpp:send_velocity | tests/safety_test.cpp::test_vehicle_stop_velocity_sends_zero
SR-LNK-04 | src/mavlink/protocol.cpp:encode_gcs_heartbeat | tests/codec_golden_test.cpp::test_gcs_heartbeat_encoder_matches_mavlink_reference
SR-LNK-04 | src/mavlink/udp_connection.cpp:send_gcs_heartbeat_locked | tests/udp_connection_test.cpp::test_unlatched_connection_sends_gcs_heartbeats
SR-LNK-04 | src/mavlink/udp_connection.cpp:announcement_override | tests/udp_connection_test.cpp::test_relay_address_override_targets_prelatch_announcements
SR-FEN-01 | src/vehicle/vehicle.cpp:upload_fence | tests/safety_test.cpp::test_vehicle_upload_fence_validates_boundary
SR-FEN-01 | src/vehicle/vehicle.cpp:verify_fence_uploaded | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-01 | src/mavlink/fence.cpp:upload_fence_plan | tests/safety_test.cpp::test_vehicle_upload_fence_rejects_transport_failure
SR-FEN-01 | src/mavlink/fence.cpp:download_fence_plan | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-01 | src/mavlink/params.cpp:read_param | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-02 | src/safety/geofence.cpp:evaluate_global_position | tests/safety_test.cpp::test_vehicle_fence_rejects_target_before_transmission
SR-PAY-01 | src/safety/payload.cpp:validate_servo_command | tests/safety_test.cpp::test_vehicle_payload_commands_require_interlock_and_validate_ranges
SR-PAY-02 | src/safety/payload.cpp:clamp_release_duration | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-PAY-02 | src/vehicle/vehicle.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_on_failure_still_attempts_off
SR-PAY-02 | src/vehicle/vehicle.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_off_failure_is_reported
SR-PAY-03 | src/safety/payload.cpp:ReleaseInterlock::evaluate_release | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-SEC-01 | src/vehicle/vehicle.cpp:send_command | tests/core_test.cpp::test_command_frame_has_expected_header
SR-SEC-01 | src/main.cpp:run_command | tests/test_cpp_command_surface.py::test_cpp_command_surface_has_no_failsafe_controls
SR-SEC-02 | src/main.cpp:run_command | tests/test_core_client_contract.py::test_every_actuation_verb_refused_without_key_before_any_socket_work
SR-SEC-03 | src/main.cpp:audit_command | tests/test_core_client_contract.py::test_actuation_with_key_reaches_transport_and_audits
```

## Pre-flight rule

No real flight follows a documentation-only assumption. Before hardware:

- unit tests pass;
- SITL scenarios pass;
- command acknowledgements and state changes are observed;
- ArduPilot failsafes and an independent RC link are verified;
- fences, limits, modes, and payload interlocks are checked;
- real credentials and hosts remain outside source control.
