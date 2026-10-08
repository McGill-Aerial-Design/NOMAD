# Safety case

NOMAD validates high-level requests and reports the available software evidence.
ArduPilot owns stabilization, EKF, navigation execution and native failsafes.
No NOMAD operation may disable those failsafes. This document defines obligations,
not flight authorization; [qualification](qualification.md) states the evidence limits.

## Safety argument and limits

Only the persistent runtime authorizes production NOMAD writes. Authentication,
request age/sequence checks, session and generation fencing, serialized operations,
final-send admission and a synchronized intent/outcome journal are necessary safety
mechanisms. Startup, reconnect and crash recovery do not restore command ownership.
Audit failure inhibits writes. Unknown or interrupted outcomes never authorize an
automatic retry. See the precise [IPC contract](runtime-ipc.md).

`revoke` inhibits NOMAD; `handback` explicitly returns software authority to NOMAD.
Neither establishes physical pilot control, blocks native GCS/RC sources, undoes an
accepted output or selects an FC mode. Aircraft-wide arbitration, total C2-loss
policy and physical takeover remain unqualified. A mode change alone does not
revoke runtime output authority.

Only `rejected` proves no eligible transmission. A negative ACK is a failed attempt;
success reports verified software/controller evidence, never guaranteed physical
effect. Configured actuator indicators have no physical feedback. Safe recovery
must be explicitly requested and succeed before hazardous activation resumes.
Pulse OFF failure latches recovery. Authentication, audit and authority still gate
safe requests. Configured outputs reject raw servo/relay access.

Velocity/watchdog zero attempts cannot cross a severed link and are not a universal
abort maneuver. Copter numeric modes must never be applied to Plane. QuadPlane
operations require the pinned class, fresh session/telemetry and phase-specific
verification. Position-target fence checks are not continuous containment.

Mission Planner polygons and altitude colors are advisory only. They do not install
an aircraft fence or implement the official 100 m AGL containment requirement.
Termination is unavailable: the plugin sends no substitute LAND command. Native
ArduPilot failsafes and an approved independent termination mechanism remain required.

## Competition safety obligations

These source requirements supplement, without renumbering or weakening, the SR
requirements below. They have no complete implementation mapping yet.

| Source IDs | Required safety argument | Falsification evidence / release blocker |
|---|---|---|
| AE27-OPS-015 through AE27-OPS-020/035 | Aircraft termination is available in every mode independently of ground core; failure of the termination/C2 path causes self-termination | Remove path/power/core under load in Copter, fixed-wing and transition states; observe actual state/output, five-second activation entry and approved rapid self-termination; G7 |
| AE27-OPS-016/017 | Fixed-wing motor-off/full surfaces differs from rotary vertical descent of at least 2 m/s to touchdown | Independently observe surface outputs and measured descent/touchdown; LAND dispatch or configured speed is not proof; Q02/G7 |
| AE27-OPS-005/006/020/037/038 | All-mode containment includes non-convex boundary and 100 m AGL | Runtime-owned boundary and altitude enforcement, validated datum/terrain inputs, actual breach and loss-of-navigation tests; Mission Planner's local outlines are visual advisory data only and do not satisfy this gate; G7 |
| AE27-NET-007/008 | Stay outside supplied traffic cylinders; stale traffic is unknown | Inject delayed/malformed tracks and prove operator response avoids intrusion; settle extent/datum/freshness Q04 before flight; G4/G7 |
| AE27-T2-005/006 | Exactly one tracker attachment and at least 100 m horizontal offset after withdrawal through rest of window | Wrong/duplicate tracker and target moving toward sampling/return path; prove detection and approved intervention before encroachment; G6/G7 |
| AE27-OPS-024/031 | Under-15-kg project margin and physical ground propeller inhibit | Independent weighing and props-safe inhibit fault tests; G7 |

The C++ watchdog's delivered zero command is not a competition termination
mechanism. The plugin's LAND-as-termination dispatch and descent-parameter
recipes have been removed. Its termination button reports termination
unavailable and sends no aircraft command. The plugin's direct vehicle-fence
upload/clear writer is deleted because it could disable the fence without
qualified maintenance ownership or failure restoration. Visual Plan map export
remains; it sends no aircraft request. Mission Planner's saved inner/outer
outlines and configurable altitude color threshold are local advisory display
data only. Protocol v1 has no boundary configuration, evaluation or status API;
the outlines are not installed on the runtime or aircraft and cannot satisfy a
containment requirement. No outline crossing requests return, termination or
another vehicle action. This removes misleading frontend policy without
providing a substitute safety mechanism. Aircraft termination, hard-breach
response and flight qualification remain blocked. Preserve ArduPilot failsafes
and qualify their interaction with an approved termination mechanism; no
substitute emergency recipe is approved.

Q01 is resolved by project direction: the plugin may derive an inner outline
from an outer outline for visual preview, but neither polygon is an authoritative
safety boundary. Crossing an internal margin is not a termination trigger.
Appendix C's inconsistent labels remain a source note; a second official polygon
is not a release dependency. Any future runtime enforcement needs a typed API,
runtime-owned policy and complete fault-path tests. Q02 still concerns QuadPlane
transition termination and fixed-wing surface behavior. The plugin's default
122 m altitude reference is configurable and display-only; it is not enforcement
of the official 100 m AGL ceiling, which remains a separate open requirement.

The original SR-PAY-03 explicit operator interlock remains binding project policy.
Configured hazardous actuator authorization is enforced by `nomad-runtime`, which
chooses output values, confirmation policy, sequencing and recovery. Names and
action labels are configuration data. The five concrete behaviors are
`ServoToggle`, `ServoPosition`, `ServoBidirectional`, `RelayToggle` and `RelayPulse`.

Hazardous configuration requires two or three confirmations within a total
500–5000 ms window, with real neutral between HID edges. Nonhazardous actions may
require zero to three confirmations. Sequences bind the actuator, operation,
normalized value, authority/session/generation, input source and physical slot.
Expiry, input loss, contradiction and authority changes fail closed. Startup,
configuration replacement and authority/session invalidation require an explicit
software-successful safe command before activation. Safe actions bypass hazardous
confirmation counts but retain ordinary authentication, authority and audit checks.

For a pulse, software-successful ON followed by any unsuccessful OFF latches
recovery. Further activation is blocked; an explicit safe OFF remains available.
Only a successful explicit safe request clears recovery. Initial definite ON
failure does not imply ON happened; unknown/interrupted results require recovery.
The backend never retries an uncertain mutation. Software command success,
request disposition and physical actuator state remain separate.

Configured outputs reject raw servo/relay requests, including requests from other
frontends. Primitives remain available for unconfigured outputs. This is a NOMAD
software boundary; external RC, native GCS and ArduPilot remain outside it.
Actuator configuration requires explicit output/safe values and reviewed bounds:
servo 1–16, relay 0–15, PWM 500–2500 us, pulse/motion wait 50–1500 ms. Before ON,
the original request must have enough validity for the 3000 ms ACK budget, wait
and 250 ms margin; OFF retains the same final-send authority token. These bounds
do not establish a maximum physical ON time or prove physical OFF. Hardware
feedback, mechanism calibration and ArduPilot failsafes require qualification.

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
| SR-VEL-04 | Convert input and MAVLink frames explicitly and correctly | Core wire tests; the ROS velocity input surface is removed |
| SR-VEL-05 | Guided velocity requires armed state and GUIDED mode | Copter tests; never apply its numeric mode to Plane |
| SR-VEL-06 | Filter heartbeat to the commanded vehicle | MAVSDK autopilot-selection tests; authenticated source/target selection remains a security gate |
| SR-VIO-01 | Reject unhealthy, low-confidence, stale or unexpected-source VIO | Unimplemented source admission; no estimator submission interface exists. Retained watchdog tests cover unhealthy/stale VIO state only |
| SR-VIO-02 | Stale VIO stops active velocity within watchdog interval | Deterministic tests; live estimator and full-load deadlines open |
| SR-LNK-01 | Commands require fresh FC heartbeat | Velocity gate/transport behavior tested; audit every discrete command path at G2 |
| SR-LNK-02 | Missing velocity input triggers a zero command within timeout | Watchdog tests; independent wire and FC observations required |
| SR-LNK-03 | Shutdown sends zero before closing an active link | Loopback ordering tests; live SITL and physical link evidence separate |
| SR-LNK-04 | Announce a standard GCS heartbeat for heartbeat-gated relays | MAVSDK `GroundStation` configuration announces at 1 Hz; the closed-gate `core-sitl-gcs-heartbeat` harness requires at least three measured intervals across four announcements at 0.9–1.3 s. Current evidence scope are recorded in [qualification status](qualification.md#sitl-qualification) |
| SR-FEN-01 | Upload, enable and verify FC fence before autonomous flight | Upload/readback/enable-reading tests; global preflight enforcement and all fence fields open |
| SR-FEN-02 | Reject position targets outside configured boundary | C++ target tests; Mission Planner has no navigation API, and the architecture guard forbids direct vehicle writers; unsupported runtime goto produces zero fake commands (`tests/runtime_ipc_test.cpp::test_protocol_errors`); live containment and full mission/velocity paths open |
| SR-PAY-01 | Validate servo channel and PWM before actuation | C++ generic range tests; board map and reserved payload channels open |
| SR-PAY-02 | Bound payload duration and de-energize outputs on failure | Dedicated release/off-failure tests; generic relay and physical power-loss behavior open |
| SR-PAY-03 | Authorize configured hazardous actuator actions behind runtime IPC | Backend confirmations, neutral readiness, expiry, authority invalidation, configured-output guards and explicit recovery are covered by actuator and IPC fault tests; software success is not physical proof |
| SR-SEC-01 | No NOMAD command disables FC failsafes | Structural scan only; semantic allowlist and plugin parameter audit open |
| SR-SEC-02 | Authenticate command clients at trust boundary | Per-client HMAC proofs authenticate configured local client identities; the nonempty deployment gate remains separate, and no human-user or remote identity is established |
| SR-SEC-03 | Authenticate and audit command requests | Runtime JSONL intent is synchronized before execution and observed outcomes afterward; authentication/audit faults fail closed, and recorded software evidence does not prove physical action; see [protocol policy](runtime-ipc.md#durable-runtime-command-evidence) |
| SR-TYP-02 | QuadPlane forward transition and VTOL takeoff stay bound to the admitted aircraft identity and session; transition completion remains armed/AUTO and takeoff requires a fresh post-ACK climb sample | Deterministic counterexamples and controls in `tests/vehicle/quadplane/quadplane_transition_test.cpp` and `tests/quadplane_vtol_takeoff_test.cpp`; the pinned profile chain through QLAND has SITL evidence at the SHA recorded in [qualification status](qualification.md#sitl-qualification), with hardware and other scenarios unqualified |
| SR-LND-01 | Pinned QuadPlane landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope; ACK alone is never touchdown | C++ falsification and deterministic MAVSDK landed-state mapping pass; pinned SITL evidence and exact revision are summarized in [qualification status](qualification.md#sitl-qualification) |


## Additional hazards and proposed obligations

The IDs below reserve new obligations for the target architecture. They are
proposed engineering requirements (R), not CONOPS rules. Add real code/test
mappings when implemented; do not invent entries in the existing checked block.

| Hazard | Proposed requirement | Mitigation / objective falsification test | Gate |
|---|---|---|---|
| H-09 Conflicting writers | SR-AUT-01: one active owner and explicit handover | Mission Planner's supported typed requests and the installed CLI use runtime IPC; authority lifecycle tests cover missing authority, revoke, replay and busy rejection, and MAVSDK peer checks cover the typed command path. Non-installed `nomad-qualification`, native Mission Planner controls, RC/pilot and maintenance tools remain outside aircraft-wide arbitration. No ROS provider ships. Physical handover is open | G2 |
| H-10 Stale position with fresh heartbeat | SR-TEL-01: per-field age and clock validity | Freeze position while heartbeats flow; position-dependent actions fail closed — implemented as a configurable position-freshness gate (`position_freshness_timeout`, default 2000 ms) enforced by the core | G2 |
| H-11 Collision or missed traffic | SR-AIR-01: unknown/stale traffic never means clear | Crossing/head-on/reordered/expired tracks yield expected advisories with measured warning time | G4 |
| H-12 Wrong aircraft mode/transition | SR-TYP-01: validate aircraft class and state | Copter mode constants refused for Plane; failed/aborted VTOL transitions use reviewed response | G2/G7 |
| H-13 False identity or geolocation | SR-OBS-01: decisions retain evidence and uncertainty | Duplicate animals, occlusion, wrong datum and stale images cannot create unreviewed task actions | G5/G6 |
| H-14 Tracker confusion / duplicate payload | SR-TSK-01: bind task/target/action identity and expiry | Wrong tracker, reconnect or battery swap cannot retag/resample from replay | G6 |
| H-15 Payload jam/contact/power loss | SR-PAY-04: safe physical state and verified outcome | Interrupt power/link/feedback during actuation; independent measurement proves timeout and containment | G6/G7 |
| H-16 Mass, energy or navigation deficit | SR-OPS-01: qualify aircraft configuration and reserve | All-up weighing, endurance/transition energy and GNSS/RTK-loss evidence | G7 |
| H-17 Network/compute overload | SR-RES-01: bounded queues and safety execution | Video/server flood, dead worker, thermal throttling and disk-full cannot starve command deadlines | G3/G8 |
| H-18 Untrusted messages/replay | SR-SEC-04: authenticate, authorize and reject replay | Wrong/expired credentials, old session and malformed server/DDS/IPC input are refused and audited | G2/G8 |
| H-19 Unavailable or wrong actuation from a bad command identifier | SR-CMD-01: every actuation command carries an identifier the pinned dialect defines | An id no handler matches makes a capability unavailable in flight (C23: motor test sent 139, an undefined `MAV_CMD`, so ArduPilot answered `MAV_RESULT_UNSUPPORTED`; nothing exercised the verb, so it survived review), and a wrong-but-defined id could command something else entirely. Every hand-typed id now resolves against the pinned dialect definition (`tests/test_command_ids.py`); peer acceptance checks cover motor test and the runtime-routed gimbal target, not physical mount motion | G2 |
| H-20 Fork-owned ArduPilot semantic is wrong or unverified | SR-CMD-02: every ArduPilot semantic comes from the pinned, tested fork | A wrong mode, altitude or frame interpretation inside the fork is treated as NOMAD code: each patch carries a test and an independent wire or SITL observation, and NOMAD still refuses to trust its acknowledgement | G-M |
| H-21 Route ACK or stale position is mistaken for QuadPlane route completion | SR-MIS-01: fixed-wing route success requires fresh post-ACK aircraft progress and state for every waypoint | Unsupported class and malformed route send nothing; stale position, a pre-ACK arrival without later progress, prior target location, intermediate point, ACK-only, interruption and timeout remain incomplete; final waypoint proximity requires independent SITL observation; dated evidence is in [qualification status](qualification.md#sitl-qualification) | G-M |
| H-22 Recovery tolerance is mistaken for transition readiness, or ACK/stale multicopter telemetry is mistaken for completed transition | SR-TYP-01: transition-to-VTOL uses the explicit recovery coordinates and altitude, measured pre-command altitude inside 15–25 m above home, a stabilized 55 m envelope and authoritative post-ACK state | Copter/Plane/Unknown, stale telemetry, wrong mode/state, outside-envelope, unstable position/speed samples, ACK-only, pre-ACK state, interruption and intermediate-only cases send nothing or fail; after command the aircraft remains above 15 m and independent state observation proves armed multicopter completion. The pre-command 25 m ceiling does not constrain transition climb. This does not prove a safe landing or pilot handover | G-M/G7 |
| H-23 QLAND ACK or stale/contradictory state is mistaken for QuadPlane touchdown | SR-LND-01: landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope | Unsupported classes, wrong profile, admission faults, stale/pre-ACK landed state, ACK-only, no-descent, interrupted link/session/mode, unstable touchdown and timeout remain failures; independent traces must observe QLAND, descent, repeated `ON_GROUND`, disarm and a stable final envelope; dated evidence is in [qualification status](qualification.md#sitl-qualification) | G-M/G7 |

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


## Pinned controller mechanisms

The table is an audit of the pinned ArduPlane source, not a production parameter
prescription. Read back the actual controller's complete parameter set for
every evidence record. Historical `SYSID_*` names must not replace the pinned
4.7.1 `MAV_*` names.

| Mechanism / parameters | Pinned behavior and qualification implication |
|---|---|
| `MAV_SYSID`, `MAV_GCS_SYSID`, `MAV_GCS_SYSID_HI`, `MAV_OPTIONS` | FC/source identities and allowed GCS range. Source defaults have primary GCS 255, high range 0, enforcement off (`MAV_OPTIONS=0`). With enforcement off, ordinary packets from other system IDs may pass; same-system and selected GCS IDs are accepted by the source check. This is acceptance filtering, not exclusive control. |
| Runtime source identity | Pinned MAVSDK GroundStation defaults are system 245/component 190. This differs from default primary GCS 255. An enforced GCS-ID profile must deliberately admit intended source IDs; never change it merely to make a test pass. |
| `SET_MODE`, `COMMAND_LONG`, `COMMAND_INT` | Packet/source checks, target addressing and command/mode validation determine acceptance. `SET_MODE` has no separate primary-GCS check in its handler. NOMAD's software generation is not transmitted as an FC ownership grant. |
| MAVLink signing, `MAVn_OPTIONS` and serial/UDP routes | Per-channel `MAVn_OPTIONS` defaults to 0; bits 0/1/2/3 permit unsigned MAVLink2 / disable forwarding / ignore streamrate / forward bad-CRC packets. This differs from global source-ID enforcement. Signing configuration and endpoint reachability require hardware records; transport priority is not sender arbitration. |
| `SERIALn_PROTOCOL`, `SERIALn_BAUD`, `SERIALn_OPTIONS` | Controller link/input-port selection and serial behavior. Protocol 23 is a possible serial RCIN route. Forwarding/streamrate options moved from serial flags to `MAVn_OPTIONS` in 4.7. Record the actual CRSF/ELRS receiver mode, port/wiring and readbacks; no port configuration is approved here. |
| `RC_CHANNELS_OVERRIDE`, `MANUAL_CONTROL` | Handlers require a selected GCS system ID, unlike ordinary mode traffic with default enforcement off. Runtime v1 has no override/manual-control request. Audit maintenance/native tools for such traffic. |
| `RC_OVERRIDE_TIME`, `RC_OPTIONS`, override-enable aux option 46 | Default override expiry is 3 s; 0 disables overrides, -1 prevents expiry. `RC_OPTIONS` bits 0/1 ignore receiver/overrides; bit 2 ignores the receiver failsafe bit; bit 10 enables multiple receivers; bit 13 selects the ELRS 420 kbaud option. Override enable state also gates acceptance; live overrides precede receiver data. |
| `RC_PROTOCOLS`, `RCMAP_*`, per-channel calibration and `RCn_OPTION` | Protocol selection, axis mapping/calibration and aux functions control FC input interpretation. `RC_PROTOCOLS=1` enables all; bit 9 selects CRSF. Axis defaults 1/2/3/4 are not an approved production ELRS map. Physical wiring/protocol must be measured. |
| `FLTMODE_CH`, `FLTMODE1..6`, `RCn_OPTION` | Switch input and configured mode slots/aux functions select modes. An external mode transition is not a runtime revoke or automatic physical handback. |
| `STICK_MIXING` | Pilot input can be mixed into automatic modes without changing mode; pinned Plane default is FBW stick mixing. Value 3 has QuadPlane yaw-only VTOL behavior. Actual control response requires bench/flight measurement. |
| `THR_FAILSAFE`, `THR_FS_VALUE`, `RC_FS_TIMEOUT` | Receiver/throttle loss handling; source defaults include enabled throttle failsafe, threshold 950 and 1 s RC timeout. `THR_FAILSAFE=2` ignores failed RC input without triggering the RC failsafe action. Real receiver hold/no-pulse/failsafe-bit behavior requires bench evidence. |
| `FS_SHORT_ACTN`, `FS_LONG_ACTN`, `FS_LONG_TIMEOUT`, `FS_GCS_ENABL` | Short/long loss actions and GCS monitoring. Source long timeout is 5 s and GCS failsafe is disabled by default. Configured GCS heartbeat monitoring does not imply every external source or NOMAD (default ID 245) is monitored. Long failsafe clears RC overrides; action and recovery still depend on mode/reason/profile. |
| `Q_OPTIONS` bits 5/20; current mode and failsafe reason | QuadPlane RC-loss selection can use QRTL/RTL instead of QLAND. In radio-failsafe recovery, saved entry mode can be restored when the control-mode reason remains radio failsafe. This is FC recovery, not NOMAD admission. |
| `Q_ENABLE`, `Q_TRANS_FAIL`, `Q_TRANS_FAIL_ACT` | Pinned SITL classification/transition profile (`Q_ENABLE=2`); transition-failure parameters belong to phase-specific qualification, not source arbitration. These do not establish a production C2 response. |
| `SIM_RC_FAIL` | Native simulator receiver fault injection only. Existing disarmed probe proves health loss/restoration; not physical RF failure or flight response. |

These failsafe names/actions describe **ArduPlane**. The Copter authority test
records its separate `FS_GCS_ENABLE` readback and does not qualify a failsafe
action; do not apply Plane's `FS_GCS_ENABL` or short/long action rules to Copter.

Pinned implementation references:
[GCS parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS.cpp#L38-L66),
[source acceptance](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L7180-L7201),
[mode handler](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L2878-L2904),
[override/manual-control handlers](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L4177-L4225),
[RC parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channels_VarInfo.h#L84-L113),
[RC input selection](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.cpp#L303-L310),
[Plane failsafe parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/Parameters.cpp#L406-L510),
and [radio recovery](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/events.cpp#L243-L251).

The pinned `FS_GCS_ENABL` description specifies monitoring after a first primary
GCS heartbeat: value 1 monitors heartbeat, 2 also monitors `RADIO_STATUS.remrssi`,
and 3 monitors heartbeat only in AUTO. Check actual heartbeat identity/range and
loss behavior; default runtime ID 245 is outside the default GCS ID 255.
Additional pinned references are
[channel options](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp#L200-L216)
and [serial configuration](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_SerialManager/AP_SerialManager.cpp#L185-L281).

## Required production RC / ELRS inputs

No approved production channel map or controller input-priority profile is
recorded. The [PRD](prd.md) mentions "CH5 currently arms" but requires verifying
actual mode/aux mapping; this is an unverified project assumption, not approval.
SITL defaults and deployment transport labels are not approval.
Keep this table unresolved until the aircraft owner supplies a reviewed record.

| Required decision | Required retained value and measurement | Current status |
|---|---|---|
| Executable runtime operation | Exact aircraft class, typed request and capability evidence before bench step 4 | Copter output tested in SITL; no executable QuadPlane v1 mutation |
| Receiver and FC wiring | Receiver model/firmware, ELRS settings, output protocol, wiring, FC port and protocol selection | Required; not qualified |
| Input calibration/map | Every control/aux channel, `RCMAP_*`, min/trim/max/reversal/deadzone; measured FC channel values | Required; do not derive from SITL |
| Pilot mode/takeover switch | Physical switch positions, `FLTMODE_CH`, `FLTMODE1..6`, `RCn_OPTION`, debounce and observed mode/time | Required; no approved production allocation |
| Override policy | Whether any source may emit RC overrides; allowed sender, enable option, timeout, release and physical-input precedence | Required; runtime has no RC-override request |
| GCS acceptance/identity | Runtime/native GCS/maintenance IDs, enforcement flags, signing policy and collision behavior | Required; IDs alone do not authenticate |
| Receiver-loss behavior | Receiver no-pulses/failsafe-bit/held-value behavior, FC timeout, short/long actions in each phase | Required; measure actual loss, not only disconnect a nominal link |
| GCS / total C2 loss | Selected monitored sources, timeout, action, redundant links and recovery/latching decision | Separate open C2 policy; not closed here |
| Pilot priority and handback | Approved response to simultaneous requests, explicit runtime revoke, physical takeover and re-admission conditions | Bench and flight qualification open |

## Controller bench and aircraft procedure

This is a procedure to run later, **not a passed qualification record**. A flight
and safety lead must approve the real configuration, expected transitions,
timing bounds and safe test outputs before execution. Begin with propellers
removed and power/output isolation verified. Measure output electrically or
with an approved unloaded actuator; do not use the simulator's channel 5 on an
aircraft without verifying its actual function. No automatic termination or
failsafe-disabling recipe is authorized here.

Use one test record per aircraft/profile/phase. For each row capture UTC and
monotonic transition time, active sources, runtime owner/generation/session
relationship, received RC channels, requested/observed FC mode, armed state,
ACK result, actual output, latency, expected outcome and actual outcome. An ACK
alone cannot pass an output or takeover row. Define expected outcomes and
maximum takeover/timeout latency **before** running the test. Any unexpected
source acceptance, continued old command, stale state, failure to recover, or
missing evidence fails the row and stops escalation to powered/flight tests.

| Step | Stimulus | Required observation / acceptance |
|---|---|---|
| 1. Inhibited startup | Start FC, router and runtime without admission | No runtime owner; typed mutation rejected; no corresponding command/output change |
| 2. Pilot input | Exercise approved physical transmitter controls and mode switch | Correct measured channels, FC mode and safe outputs; record every switch position |
| 3. Admit NOMAD | Explicit admission under approved safe preconditions | New owner/generation; pilot path remains available; no unsolicited output |
| 4. NOMAD command | Issue one approved bounded typed command | Delivery, FC acceptance and independently measured output agree; report failure separately |
| 5. Pilot takeover while NOMAD active | Operate takeover switch/control during a bounded NOMAD operation | Approved pilot mode/output wins within bound; determine whether continuing NOMAD output is accepted; do not infer runtime revocation from mode change |
| 6. Runtime revoke | Revoke with work queued/retrying; attempt fresh and old requests | No owner; fresh/old requests rejected; no covered command after completed fence transition; independently confirm pilot/native control |
| 7. Explicit handback | Confirm approved pilot release conditions, then runtime `handback` | New generation, only new request context executes; verify FC mode/input/output separately |
| 8. NOMAD link loss | Remove the selected NOMAD transport while independent pilot observer remains | Runtime inhibition and measured FC response/time; do not equate this with total C2 loss |
| 9. RC link loss | Remove physical RC RF path using approved isolation | Receiver/FC loss indication, approved short/long failsafe state/output/time; restore and measure recovery |
| 10. NOMAD reconnect | Restore transport; separately restart runtime | Recovery alone leaves NOMAD inhibited; old context rejected; explicit handback or new-start admission required |
| 11. Native GCS | Issue approved native MP action while NOMAD inhibited, then admitted | Record actual FC acceptance/mode/output; verify receive-only router consumer restriction separately |
| 12. Conflicting sources | Exercise approved bounded RC/native GCS/NOMAD/maintenance conflict pairs and order reversals | Observed winner, latency and output match preapproved matrix; IDs do not create an assumed priority |
| 13. FC restart | Safely restart isolated FC with links present | Safe boot outputs/input, fresh session, NOMAD inhibited, old context rejected; explicit recovery procedure |
| 14. Repeat in required phases | Only after bench pass and separate flight authorization, repeat approved cases in hover, fixed wing and transitions | Independent trajectory/control evidence within approved envelope; label physical-flight results separately |

Retain an immutable evidence bundle containing exact NOMAD SHA and build,
ArduPilot firmware SHA/version and FC build/board identity, complete before/after
parameter files, approved RC channel map, transmitter/receiver/ELRS firmware and
configuration excluding secrets, wiring/input protocol, both router configurations, system and
component IDs for every source, synchronized timestamp method, MAVLink tlogs,
FC DataFlash logs, runtime logs, observer measurements/video, initial state,
every expected/actual result, failed attempts and restoration results. Keep
signing-key material and ELRS binding secrets out of all retained logs; retain
only signing configuration/key presence or a nonsecret reference. Keep secrets
and machine-specific deployment identifiers out of this repository;
link a reviewed restricted evidence record instead. Record operator and safety
review approval, deviations and any residual limitation.

A controller bench pass still leaves RF independence, physical pilot handling,
airborne output/trajectory and phase-dependent takeover open. SITL must never
mark this procedure passed. Qualification records belong in
[qualification status](qualification.md); unresolved work stays in
[TODO](../TODO.md).


## Machine-checked evidence locations

These references identify code and tests, not full requirement closure.

```cpp_traceability
SR-VEL-01 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-01 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-02 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-02 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-03 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-04 | src/mavlink/mavsdk_velocity.cpp:queue_velocity_setpoint | tests/test_mavsdk_connection.py::test_velocity_reaches_the_wire_and_is_zeroed_on_disconnect
SR-VEL-05 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-06 | src/mavlink/mavsdk_mavlink_connection.cpp:select_expected_autopilot | tests/test_mavsdk_connection.py::test_wrong_autopilot_identity_is_refused
SR-VIO-01 | src/safety/velocity.cpp:evaluate_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VIO-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss
SR-LNK-01 | src/vehicle/vehicle_velocity.cpp:set_velocity | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_link_loss
SR-LNK-01 | src/mavlink/mavsdk_mavlink_connection.cpp:wait_for_heartbeat | tests/test_mavsdk_connection.py::test_arm_acknowledgement_paths
SR-LNK-01 | src/mavlink/mavsdk_connection_resources.cpp:close | tests/mavsdk_lifetime_test.cpp::test_command_retirement
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
SR-PAY-01 | src/safety/output.cpp:validate_servo_command | tests/safety_test.cpp::test_generic_servo_validation
SR-PAY-01 | src/runtime/actuator_config.cpp:validate_actuator_definitions | tests/actuator_test.cpp::test_configuration_validation_and_revision
SR-PAY-02 | src/runtime/actuator_sequence.cpp:run_actuator_sequence | tests/actuator_test.cpp::test_staged_recovery_matrix
SR-PAY-02 | src/runtime/actuator_state.cpp:ActuatorState::finish | tests/actuator_test.cpp::test_initial_failure_and_final_audit_failure
SR-PAY-02 | src/runtime/actuator_sequence.cpp:run_actuator_sequence | tests/actuator_test.cpp::test_staged_exceptions_leave_explicit_recovery_available
SR-PAY-03 | src/runtime/actuator_state.cpp:ActuatorState::confirm_locked | tests/actuator_test.cpp::test_hid_and_authority_bound_confirmation
SR-PAY-03 | src/runtime/runtime_actuators.cpp:execute_actuator_request | tests/runtime_actuator_cases.hpp::test_backend_actuator_authorization_and_raw_boundary
SR-PAY-03 | src/runtime/actuator_state.cpp:ActuatorState::finish | tests/runtime_actuator_cases.hpp::test_backend_pulse_failure_and_explicit_recovery
SR-PAY-03 | src/runtime/actuator_state.cpp:ActuatorState::release_input | tests/runtime_actuator_pending_cases.hpp::test_hid_bidirectional_release_preserves_confirmations_and_stops
SR-SEC-01 | src/vehicle/vehicle.cpp:send_command | tests/test_mavsdk_connection.py::test_command_wire_forms
SR-SEC-01 | src/qualification/main.cpp:run_command | tests/test_cpp_command_surface.py::test_cpp_command_surface_has_no_failsafe_controls
SR-SEC-02 | src/qualification/main.cpp:run_command | tests/test_qualification_cli.py::test_direct_actuation_refused_without_key_before_transport
SR-SEC-03 | src/qualification/main.cpp:audit_command | tests/test_qualification_cli.py::test_direct_actuation_with_key_is_audited
SR-TEL-01 | src/vehicle/vehicle.cpp:wait_for_location | tests/safety_test.cpp::test_vehicle_goto_location_rejects_stale_position
SR-CMD-01 | src/vehicle/output.cpp:motor_test | tests/output_command_test.cpp::test_vehicle_motor_test_validates_and_clamps_timeout
SR-CMD-01 | src/vehicle/output.cpp:make_command | tests/test_command_ids.py::test_command_id_matches_the_dialect
SR-CMD-01 | src/vehicle/output.cpp:motor_test | tests/test_mavsdk_connection.py::test_output_commands_reach_the_wire
```
