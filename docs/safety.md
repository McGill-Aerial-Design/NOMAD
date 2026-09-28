# Safety case

Baseline: CONOPS v1.0 reconciliation, 2026-09-10. This is a safety argument and verification backlog, not a
flight authorization. Existing requirement IDs remain stable. Current source and
test results are in [migration](migration.md); proposed requirements below are
not claimed as implemented.

## Safety argument and limits

NOMAD validates high-level requests and observes their outcomes. ArduPilot owns
stabilization, EKF, low-level navigation and its independent failsafes. Loss of
NOMAD, ROS, perception, video, ground compute or competition connectivity must
not suppress those failsafes.

There is no universally safe command for every aircraft state. A zero velocity
attempt can stop a Copter guided stream only when delivered and accepted; it is
not a fixed-wing abort maneuver. RTL or land must be appropriate to aircraft
type, navigation health, terrain, traffic, fence and remaining energy. Task 1's
proposed VTOL requires its own ArduPlane/QuadPlane safety evidence.

The release safety case needs reviewed operating limits, bounded fault response,
independent witnesses and residual-risk acceptance by the flight/safety lead.
Missing hardware, CONOPS detail or evidence keeps the relevant gate open.

## Existing hazards and mitigation coverage

| ID | Hazard | Implemented mitigation and evidence limit |
|---|---|---|
| H-01 | Excessive or wrong-axis velocity | C++ finite/clamp/frame checks; verify each axis through ROS to wire independently |
| H-02 | Stale/unhealthy VIO motion | C++ VIO/watchdog and ROS source/receipt-age gates; no real estimator/fusion qualification |
| H-03 | Link loss or shutdown while commanding | Heartbeat/watchdog/zero attempts and loopback tests; no guarantee across a severed link |
| H-04 | Motion after mode change/disarm | Copter mode/armed watchdog gates; aircraft-class support and authority handover incomplete |
| H-05 | Fence breach | Target polygon validation, upload/readback and enable check; not continuous trajectory/traffic/altitude containment |
| H-06 | Unintended payload action | Dedicated consuming release interlock and off attempt; raw outputs bypass it; hardware timeout/feedback open |
| H-07 | Failsafe suppression | Narrow command surface and structural scans; generic user commands/parameter writers require semantic review |
| H-08 | Unauthorized commands | Local CLI nonempty-key opt-in and admission audit; no authenticated remote boundary or inherited library audit |

Historical Copter SITL results support development but do not close current
release gates without artifact provenance. The zero-delivery loopback test keeps
the outbound path open when inbound heartbeat stops; it does not simulate total
bidirectional radio failure. A lost physical link requires an independent
autopilot timeout/failsafe and operator procedure.

## Competition safety obligations

These source requirements supplement, without renumbering or weakening, the SR
requirements below. They have no complete implementation mapping yet.

| Source IDs | Required safety argument | Falsification evidence / release blocker |
|---|---|---|
| AE27-OPS-015 through AE27-OPS-020/035 | Aircraft termination is available in every mode independently of ground core; failure of the termination/C2 path causes self-termination | Remove path/power/core under load in Copter, fixed-wing and transition states; observe actual state/output, five-second activation entry and approved rapid self-termination; G7 |
| AE27-OPS-016/017 | Fixed-wing motor-off/full surfaces differs from rotary vertical descent of at least 2 m/s to touchdown | Independently observe surface outputs and measured descent/touchdown; LAND dispatch or configured speed is not proof; Q02/G7 |
| AE27-OPS-005/006/020/037/038 | All-mode containment includes non-convex boundary and 100 m AGL | Verified hard polygon and internal inset per U-FEN-01, altitude datum/terrain validation, actual hard-breach and loss-of-navigation tests; G7 |
| AE27-NET-007/008 | Stay outside supplied traffic cylinders; stale traffic is unknown | Inject delayed/malformed tracks and prove operator response avoids intrusion; settle extent/datum/freshness Q04 before flight; G4/G7 |
| AE27-T2-005/006 | Exactly one tracker attachment and at least 100 m horizontal offset after withdrawal through rest of window | Wrong/duplicate tracker and target moving toward sampling/return path; prove detection and approved intervention before encroachment; G6/G7 |
| AE27-OPS-024/031 | Under-15-kg project margin and physical ground propeller inhibit | Independent weighing and props-safe inhibit fault tests; G7 |

The C++ watchdog's delivered zero command is not a competition termination
mechanism. The plugin's LAND-as-termination dispatch and descent-parameter
recipes have been removed. Its button and hard-boundary request now explicitly
report termination unavailable, with no aircraft command. The plugin's direct
vehicle-fence upload/clear writer is deleted because it could disable the fence
without qualified maintenance ownership or failure restoration. Visual Plan map
export remains; it sends no aircraft request. This removes a misleading independent writer, not a
safety mechanism that can satisfy AE27-OPS-015 through AE27-OPS-019. Aircraft
termination, hard-breach response and flight qualification remain blocked.
Preserve ArduPilot failsafes and qualify their interaction with the approved
termination mechanism; no substitute emergency recipe is approved.

Q01 is resolved by the project owner: hard-boundary violation triggers
termination; the soft boundary is an internal configurable inward margin from
the hard polygon, e.g. 5 m. Keep the existing plugin inset implementation as-is.
Crossing that internal margin alone is not a termination trigger. Appendix C's
inconsistent labels remain a source note; a second official polygon is not a
release dependency. Test inset geometry separately from hard-breach termination,
including concave/narrow shapes and infeasible margins, without changing the
plugin in this pass. Q02 still concerns QuadPlane transition termination and
fixed-wing surface behavior. The plugin's existing 122 m defaults/displays remain
a separate gap against the official 100 m AGL ceiling.

The original SR-PAY-03 explicit operator interlock remains binding project policy.
Do not remove it to pursue the sample-autonomy bonus. If preauthorization is
accepted and selected, propose a bounded sequence and intervention/abort contract
as a later reviewed change with no uncertain-action retries.

## D09 termination intent and independent activation

The [current project direction](prd.md#c2-and-termination-direction-2026-09-27)
defines ELRS primary C2, redundant LTE/MAVLink C2 and independent FPV awareness.
The Arduino HID red button and independent transmitter two-control chord both
request logical `TERMINATE`; exact RC/MAVLink mechanisms are not selected here.
CH5 arming must be checked against actual mode/auxiliary-channel mappings and
must not be reused as an assumed termination function.

Once valid termination is accepted it must latch for the current flight, inhibit
autonomous and manual movement overrides, invalidate stale work and survive
link restoration. Requested, transported, entered and physically completed must
be reported separately. The accepted implementation must define the safe reset;
an ACK or reconnection cannot supply it. These are project requirements, not
implemented or qualified behavior. A latch in one process alone cannot inhibit
external RC or native Mission Planner writers.

The [normal Copter 4.7.1 flight-termination handler](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/ArduCopter/GCS_MAVLink_Copter.cpp#L1140-L1152)
disarms motors; it is not a commanded controlled vertical descent. Do not equate
that command or existing plugin LAND dispatch with AE27-OPS-017 completion.
Q02 still requires accepted behavior throughout QuadPlane transitions; no
fixed-wing-to-VTOL termination substitution is approved.

Qualification must distinguish single-link faults from loss of all approved
C2/termination paths, include each manual activation with the other path absent,
and prove no stale/manual/mission request or recovered link cancels termination.
FPV-only loss is a separate operational awareness fault. Observe independent
aircraft state/output through completion; preserve ArduPilot failsafes.

### Manual takeover hardware qualification still required

The software UDP peer and router tests do not show that a flight controller
selects RC/ELRS over a MAVLink command. Before a hardware claim, record the
approved transmitter output map, receiver channel mapping, ArduPilot mode and
auxiliary parameters, firmware and NOMAD/router revisions, and the intended
pilot/native takeover signal. No channel or automatic loss response is assigned
by this procedure.

With propulsion made safe under the team's aircraft test procedure, use an
independent MAVLink observer and physical output observation. Establish one
admitted NOMAD generation and observe a valid command at the controller. Trigger
the approved external takeover; time-stamp the operator input, FC input/mode
state, NOMAD revoke response and last NOMAD frame. Hold an SDK command ACK to
force a retry opportunity, and verify no old-generation frame after revoke.
Demonstrate the external input's intended physical effect, then disconnect and
restore NOMAD's link. It must stay observation-only until explicit handback;
after handback a fresh command must work and an old request must stay rejected.
Repeat for each approved pilot path and supported flight phase, including
single-link faults. Retain synchronized wire, FC, RC input and physical-output
records. Failed or absent observations leave aircraft-wide takeover unqualified.

## Stable safety requirements

These retain their original obligations; partial coverage is not satisfaction.

| ID | Requirement | Current evidence / open scope |
|---|---|---|
| SR-VEL-01 | Clamp XY velocity to reviewed limits | C++ tests; per-axis limits, not a proven total horizontal-speed bound |
| SR-VEL-02 | Clamp vertical and yaw-rate velocity to reviewed limits | C++ tests; qualify per aircraft/profile |
| SR-VEL-03 | Reject the complete command for any non-finite component | C++ tests |
| SR-VEL-04 | Convert input and MAVLink frames explicitly and correctly | Core wire tests; end-to-end ROS/frame review open |
| SR-VEL-05 | Guided velocity requires armed state and GUIDED mode | Copter tests; never apply its numeric mode to Plane |
| SR-VEL-06 | Filter heartbeat to the commanded vehicle | MAVSDK autopilot-selection tests; authenticated source/target selection remains a security gate |
| SR-VIO-01 | Reject unhealthy, low-confidence, stale or unexpected-source VIO | Core/ROS tests; source timestamp, real sensor and fusion gates open |
| SR-VIO-02 | Stale VIO stops active velocity within watchdog interval | Deterministic tests; live estimator and full-load deadlines open |
| SR-LNK-01 | Commands require fresh FC heartbeat | Velocity gate/transport behavior tested; audit every discrete command path at G2 |
| SR-LNK-02 | Missing velocity input triggers a zero command within timeout | Watchdog tests; independent wire and FC observations required |
| SR-LNK-03 | Shutdown sends zero before closing an active link | Loopback ordering tests; live SITL and physical link evidence separate |
| SR-LNK-04 | Announce a standard GCS heartbeat for heartbeat-gated relays | MAVSDK `GroundStation` configuration announces at 1 Hz; the closed-gate `core-sitl-gcs-heartbeat` harness requires at least three measured intervals across four announcements at 0.9–1.3 s. Current-head full SITL qualification remains required |
| SR-FEN-01 | Upload, enable and verify FC fence before autonomous flight | Upload/readback/enable-reading tests; global preflight enforcement and all fence fields open |
| SR-FEN-02 | Reject position targets outside configured boundary | C++ target tests; live containment and full mission/velocity paths open |
| SR-PAY-01 | Validate servo channel and PWM before actuation | C++ generic range tests; board map and reserved payload channels open |
| SR-PAY-02 | Bound payload duration and de-energize outputs on failure | Dedicated release/off-failure tests; generic relay and physical power-loss behavior open |
| SR-PAY-03 | Release requires explicit operator interlock | Dedicated core release and UI tests; all raw output access must share authorization |
| SR-SEC-01 | No NOMAD command disables FC failsafes | Structural scan only; semantic allowlist and plugin parameter audit open |
| SR-SEC-02 | Authenticate command clients at trust boundary | Nonempty environment value is not authentication; production gate open |
| SR-SEC-03 | Authenticate and audit command requests | CLI admission logs only; final outcomes and all client/library entrypoints open |
| SR-TYP-02 | QuadPlane forward transition and VTOL takeoff stay bound to the admitted aircraft identity and session; transition completion remains armed/AUTO and takeoff requires a fresh post-ACK climb sample | Deterministic counterexamples and controls in `tests/vehicle/quadplane/quadplane_transition_test.cpp` and `tests/quadplane_vtol_takeoff_test.cpp`; full pinned QuadPlane chain through stable QLAND remains required |
| SR-LND-01 | Pinned QuadPlane landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope; ACK alone is never touchdown | C++ falsification and deterministic MAVSDK landed-state mapping pass; independent full-chain pinned SITL trace passed in [run 36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163) |

## Additional hazards and proposed obligations

The IDs below reserve new obligations for the target architecture. They are
proposed engineering requirements (R), not CONOPS rules. Add real code/test
mappings when implemented; do not invent entries in the existing checked block.

| Hazard | Proposed requirement | Mitigation / objective falsification test | Gate |
|---|---|---|---|
| H-09 Conflicting writers | SR-AUT-01: one active owner and explicit handover | Concurrent CLI/ROS/plugin commands, pilot takeover and reconnect cannot resume old action | G2 |
| H-10 Stale position with fresh heartbeat | SR-TEL-01: per-field age and clock validity | Freeze position while heartbeats flow; position-dependent actions fail closed — implemented as a configurable position-freshness gate (`position_freshness_timeout`, default 2000 ms) enforced by the core | G2 |
| H-11 Collision or missed traffic | SR-AIR-01: unknown/stale traffic never means clear | Crossing/head-on/reordered/expired tracks yield expected advisories with measured warning time | G4 |
| H-12 Wrong aircraft mode/transition | SR-TYP-01: validate aircraft class and state | Copter mode constants refused for Plane; failed/aborted VTOL transitions use reviewed response | G2/G7 |
| H-13 False identity or geolocation | SR-OBS-01: decisions retain evidence and uncertainty | Duplicate animals, occlusion, wrong datum and stale images cannot create unreviewed task actions | G5/G6 |
| H-14 Tracker confusion / duplicate payload | SR-TSK-01: bind task/target/action identity and expiry | Wrong tracker, reconnect or battery swap cannot retag/resample from replay | G6 |
| H-15 Payload jam/contact/power loss | SR-PAY-04: safe physical state and verified outcome | Interrupt power/link/feedback during actuation; independent measurement proves timeout and containment | G6/G7 |
| H-16 Mass, energy or navigation deficit | SR-OPS-01: qualify aircraft configuration and reserve | All-up weighing, endurance/transition energy and GNSS/RTK-loss evidence | G7 |
| H-17 Network/compute overload | SR-RES-01: bounded queues and safety execution | Video/server flood, dead worker, thermal throttling and disk-full cannot starve command deadlines | G3/G8 |
| H-18 Untrusted messages/replay | SR-SEC-04: authenticate, authorize and reject replay | Wrong/expired credentials, old session and malformed server/DDS/IPC input are refused and audited | G2/G8 |
| H-19 Unavailable or wrong actuation from a bad command identifier | SR-CMD-01: every actuation command carries an identifier the pinned dialect defines | An id no handler matches makes a capability unavailable in flight (C23: motor test sent 139, an undefined `MAV_CMD`, so ArduPilot answered `MAV_RESULT_UNSUPPORTED`; nothing exercised the verb, so it survived review), and a wrong-but-defined id could command something else entirely. Every hand-typed id now resolves against the pinned dialect definition (`tests/test_command_ids.py`); live acceptance evidence exists for motor test only | G2 |
| H-20 Fork-owned ArduPilot semantic is wrong or unverified | SR-CMD-02: every ArduPilot semantic comes from the pinned, tested fork | A wrong mode, altitude or frame interpretation inside the fork is treated as NOMAD code: each patch carries a test and an independent wire or SITL observation, and NOMAD still refuses to trust its acknowledgement | G-M |
| H-21 Route ACK or stale position is mistaken for QuadPlane route completion | SR-MIS-01: fixed-wing route success requires fresh post-ACK aircraft progress and state for every waypoint | Unsupported class and malformed route send nothing; stale position, a pre-ACK arrival without later progress, prior target location, intermediate point, ACK-only, interruption and timeout remain incomplete; final waypoint proximity is independently observed in pinned SITL ([workflow run 35818612311](https://github.com/YoussGm3o8/NOMAD/actions/runs/35818612311)) | G-M |
| H-22 Recovery tolerance is mistaken for transition readiness, or ACK/stale multicopter telemetry is mistaken for completed transition | SR-TYP-01: transition-to-VTOL uses the explicit recovery coordinates and altitude, measured pre-command altitude inside 15–25 m above home, a stabilized 55 m envelope and authoritative post-ACK state | Copter/Plane/Unknown, stale telemetry, wrong mode/state, outside-envelope, unstable position/speed samples, ACK-only, pre-ACK state, interruption and intermediate-only cases send nothing or fail; after command the aircraft remains above 15 m and independent state observation proves armed multicopter completion. The pre-command 25 m ceiling does not constrain transition climb. This does not prove a safe landing or pilot handover | G-M/G7 |
| H-23 QLAND ACK or stale/contradictory state is mistaken for QuadPlane touchdown | SR-LND-01: landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope | Unsupported classes, wrong profile, admission faults, stale/pre-ACK landed state, ACK-only, no-descent, interrupted link/session/mode, unstable touchdown and timeout remain failures; independent trace in [run 36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163) observed QLAND, descent, repeated `ON_GROUND`, disarm and a stable final envelope | G-M/G7 |

Traffic advisories and explicit payload authorization remain project scope.
CONOPS permits manual flight but requires actual traffic cylinder avoidance.
Task 2 no-intervention sample collection earns optional points; changing payload
permission policy requires D05/Q06 and separate safety evidence. Loss-of-traffic response and numeric limits need D07/D08;
do not silently choose hold/RTL/descent as an assessment rule.

## Required evidence by boundary

- Core: known state, one action, independent expected result; invalid inputs,
  boundaries, timeout, cancellation and failure paths.
- Transport: negative ACK, missing/wrong/duplicate ACK, wrong aircraft, stale
  fields, loss/reorder, actual stop wire delivery and independent FC outcome.
- QuadPlane fixed-wing route: each target needs a fresh post-ACK position
  observation at least 10 m closer than its captured ACK-boundary position, then
  within the reviewed horizontal/altitude tolerances; only the final target
  completes the route. The hosted observer checks
  ordered aircraft position samples independently from NOMAD's result.
- Library/fork: ArduPilot command, mode and telemetry semantics live in the
  pinned MAVSDK fork, so each patch needs a unit test, an independent wire or
  SITL observation against the selected firmware and a provenance pin; a fork
  acknowledgement is still not an aircraft outcome.
- ROS/perception: acquisition versus receive time, clock skew, reset counters,
  frame axes, delayed/replayed data, callback starvation and process failure.
- Payload: core permission plus physical timeout and attachment/sample evidence;
  ACK alone never proves delivery or collection.
- Flight: approved aircraft-specific procedure, independent pilot control,
  measured margins and recorded recovery. Fixed-wing and VTOL phases differ.
- Security: actual identity/authorization and outcome records at each boundary;
  a logged label saying auth=api-key is not proof of authentication.

The current traceability checker only resolves symbol/test names. It does not
test mapping completeness, semantic correctness, execution, or hazard coverage.
Missing coverage remains visible in the tables above.

## Existing machine-checked implementation mappings

Retained from the migration to avoid losing stable code/test references.
These mappings identify evidence locations; they do not assert full requirement
closure. The MAVSDK cutover remapped the transport rows to the MAVSDK symbols and
kept equivalent fault proof; the checker still only proves each reference
resolves.

```cpp_traceability
SR-VEL-01 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-01 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-02 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-02 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-03 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-04 | src/mavlink/mavsdk_velocity.cpp:queue_velocity_setpoint | tests/test_mavsdk_connection.py::test_velocity_reaches_the_wire_and_is_zeroed_on_disconnect
SR-VEL-05 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-06 | src/mavlink/mavsdk_system.cpp:select_expected_autopilot | tests/test_mavsdk_connection.py::test_wrong_autopilot_identity_is_refused
SR-VIO-01 | src/safety/velocity.cpp:evaluate_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VIO-01 | src/safety/vio_source.cpp:VioSourceValidator::validate | tests/vio_source_test.cpp::test_vio_source_validator_rejects_wrong_source
SR-VIO-01 | ros2/nomad_ros/src/node.cpp:on_velocity_command | tests/ros/test_nomad_ros_integration.py::test_vio_source_gate_blocks_mismatch
SR-VIO-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss
SR-LNK-01 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_link_loss
SR-LNK-01 | src/mavlink/mavsdk_mavlink_connection.cpp:wait_for_heartbeat | tests/test_mavsdk_connection.py::test_arm_acknowledgement_paths
SR-LNK-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_command_timeout
SR-LNK-03 | src/mavlink/mavsdk_mavlink_connection.cpp:send_velocity | tests/safety_test.cpp::test_vehicle_stop_velocity_sends_zero
SR-LNK-03 | src/mavlink/mavsdk_mavlink_connection.cpp:send_velocity | tests/test_mavsdk_connection.py::test_zero_delivery_reaches_the_wire_on_every_stop_path
SR-LNK-04 | src/mavlink/mavsdk_mavlink_connection.cpp:MavsdkMavlinkConnection | tests/test_mavsdk_connection.py::test_unlatched_link_announces_a_gcs_heartbeat
SR-MIS-01 | src/vehicle/vehicle_route.cpp:fixed_wing_route | tests/operation_capability_test.cpp::test_quadplane_supports_only_qualified_operations
SR-MIS-01 | src/vehicle/vehicle_route.cpp:wait_for_fixed_wing_waypoint | tests/vehicle/quadplane/quadplane_route_test.cpp::test_fixed_wing_route_sends_two_waypoints_and_verifies_position
SR-MIS-01 | src/vehicle/vehicle_route.cpp:wait_for_fixed_wing_waypoint | tests/vehicle/quadplane/quadplane_route_test.cpp::test_position_reached_before_ack_without_post_ack_progress_does_not_complete_route
SR-MIS-01 | src/mavlink/mavsdk_route.cpp:send_fixed_wing_waypoint | tests/test_mavsdk_connection.py::test_quadplane_fixed_wing_route_wire_protocol_and_completion
SR-MIS-01 | src/vehicle/vehicle_recovery.cpp:fixed_wing_recovery | tests/vehicle/quadplane/quadplane_recovery_test.cpp::test_capability_and_readiness_rejections
SR-MIS-01 | src/vehicle/vehicle_recovery.cpp:fixed_wing_recovery | tests/vehicle/quadplane/quadplane_recovery_test.cpp::test_ack_without_real_progress_cannot_complete
SR-MIS-01 | src/mavlink/mavsdk_route.cpp:send_fixed_wing_waypoint | tests/test_mavsdk_connection.py::test_quadplane_fixed_wing_recovery_wire_protocol_and_completion
SR-TYP-01 | src/vehicle/vehicle_quadplane_landing.cpp:quadplane_vtol_land | tests/vehicle/quadplane/quadplane_vtol_landing_test.cpp::test_initial_state_and_telemetry_fail_closed
SR-LND-01 | src/vehicle/vehicle_quadplane_landing.cpp:verify_quadplane_touchdown | tests/vehicle/quadplane/quadplane_vtol_landing_test.cpp::test_valid_landing_requires_command_and_physical_post_ack_evidence
SR-FEN-01 | src/vehicle/vehicle_fence.cpp:upload_fence | tests/safety_test.cpp::test_vehicle_upload_fence_validates_boundary
SR-FEN-01 | src/vehicle/vehicle_fence.cpp:verify_fence_uploaded | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-01 | src/vehicle/vehicle_fence.cpp:upload_fence | tests/safety_test.cpp::test_vehicle_upload_fence_rejects_transport_failure
SR-FEN-01 | src/mavlink/mavsdk_fence.cpp:upload_fence_plan | tests/test_mavsdk_connection.py::test_fence_uploads_reads_back_and_refuses_invalid_boundaries
SR-FEN-01 | src/mavlink/mavsdk_fence.cpp:download_fence_plan | tests/test_mavsdk_connection.py::test_fence_uploads_reads_back_and_refuses_invalid_boundaries
SR-FEN-01 | src/mavlink/mavsdk_mavlink_connection.cpp:read_param | tests/test_mavsdk_connection.py::test_disabled_fence_never_verifies
SR-FEN-02 | src/safety/geofence.cpp:evaluate_global_position | tests/safety_test.cpp::test_vehicle_fence_rejects_target_before_transmission
SR-FEN-02 | src/safety/geofence.cpp:evaluate_position | tests/fence_config_test.cpp::test_local_polygon_with_nonfinite_vertex_fails_closed
SR-PAY-01 | src/safety/payload.cpp:validate_servo_command | tests/safety_test.cpp::test_vehicle_payload_commands_require_interlock_and_validate_ranges
SR-PAY-02 | src/safety/payload.cpp:clamp_release_duration | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-PAY-02 | src/vehicle/vehicle_payload.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_on_failure_still_attempts_off
SR-PAY-02 | src/vehicle/vehicle_payload.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_off_failure_is_reported
SR-PAY-03 | src/safety/payload.cpp:ReleaseInterlock::evaluate_release | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-SEC-01 | src/vehicle/vehicle.cpp:send_command | tests/test_mavsdk_connection.py::test_command_wire_forms
SR-SEC-01 | src/main.cpp:run_command | tests/test_cpp_command_surface.py::test_cpp_command_surface_has_no_failsafe_controls
SR-SEC-02 | src/main.cpp:run_command | tests/test_core_client_contract.py::test_every_actuation_verb_refused_without_key_before_any_socket_work
SR-SEC-03 | src/main.cpp:audit_command | tests/test_core_client_contract.py::test_actuation_with_key_reaches_transport_and_audits
SR-TEL-01 | src/vehicle/vehicle.cpp:wait_for_location | tests/safety_test.cpp::test_vehicle_goto_location_rejects_stale_position
SR-CMD-01 | src/vehicle/output.cpp:motor_test | tests/output_command_test.cpp::test_vehicle_motor_test_validates_and_clamps_timeout
SR-CMD-01 | src/vehicle/output.cpp:make_command | tests/test_command_ids.py::test_command_id_matches_the_dialect
SR-CMD-01 | src/vehicle/output.cpp:motor_test | tests/test_mavsdk_connection.py::test_output_commands_reach_the_wire
```

## Operational release rule

No flight follows a documentation-only assumption. Before an authorized hardware
session, establish the correct aircraft/profile/firmware, safe physical setup,
current state and command owner, required capabilities and reviewed limits.
Verify autopilot failsafes and manual takeover independently.

Read back the fence, output mapping and required navigation state; invalidate
payload permissions before maintenance, battery swap or restart. On uncertainty,
report failed/unknown outcome and follow the reviewed procedure. After the
session, observe safe payload state, disarmed aircraft and restored configuration.
Full gate evidence and role ownership are in [migration](migration.md).
