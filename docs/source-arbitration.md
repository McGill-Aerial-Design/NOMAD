# Command sources and takeover qualification

NOMAD admission arbitrates typed requests to one `nomad-runtime`. It does not
arbitrate the whole aircraft. ArduPilot accepts or rejects traffic according to
its firmware, parameters, input state and mode. A pilot or external GCS mode
change does **not** automatically revoke NOMAD output authority. Operators must
explicitly revoke it; a controller-side pilot-priority interlock remains an
unqualified requirement. Do not assume that changing mode blocks a later servo
command.

This qualification slice starts from NOMAD
`34d93335c41a000d78a323436e9027704bffc160` (after PR #52). It adds evidence,
not production termination, a C2-loss policy, or another command owner.

## Intended sources

| Source | Path | Can command? | NOMAD controls it? | Qualification status |
|---|---|---|---|---|
| `nomad-runtime` | runtime → MAVSDK → standalone ground router → aircraft router → FC | Yes, supported typed requests | Yes, this runtime's request admission and tested command-send fence | Software tests; disarmed pinned SITL scenario below |
| Mission Planner plugin typed requests | MP plugin → local runtime IPC | Yes, through runtime | Yes, same software boundary | Software-qualified request boundary; no independent vehicle fallback |
| Mission Planner native vehicle connection | Native/direct GCS path → FC | Potentially | No | External authority; bench interaction open |
| RC / ELRS pilot | Receiver → FC input | Yes, subject to configured mode/input handling | No | Production input/map unresolved; physical qualification required |
| Maintenance / independent MAVLink source | Separate source → router or FC | Potentially | No | External authority; isolated source-250 SITL mode test only |
| Qualification tooling | Isolated direct simulator/peer path | Test only | Separate from production admission | Non-installed direct driver; `NOMAD_INTEGRATED_FLIGHT=1` inhibits its actuation, but is not an aircraft access-control boundary |
| ROS 2 observer | Receive-only telemetry endpoint | No supported commands | Observation boundary only | Software-tested; not a control source |

The ground router's `mission_planner` consumer is receive-only for aircraft
command egress. This restriction is not FC authentication, does not block a
separate native MP connection, and does not apply to every aircraft-side
`mavlink-router` endpoint. Source system/component IDs identify MAVLink packets;
they are not a cryptographic grant of authority. See
[architecture](architecture.md), [runtime IPC](runtime-ipc.md), and the
[ground-router contract](../infra/transport/ground_router/README.md).

## Qualification levels and checks

| Check | Evidence level | Authoritative observation | Boundary it does not prove |
|---|---|---|---|
| Startup, admission, expiry/sequence, cache eviction, session and restart unit cases | Software test | Runtime response, fake transport calls and state | FC acceptance or physical action |
| Production-runtime retry/revoke/session fixture | Deterministic peer | Independently counted UDP commands; suppressed retry after revoke | ArduPilot arbitration |
| Queued `COMMAND_LONG` / `COMMAND_INT` final-send probe | Deterministic peer | UDP socket receives no old admitted work after fence transition | Every MAVLink message or other transports; queue proof uses a probe gate |
| Disarmed Copter runtime channel-5 command | SITL controller behavior | Runtime result plus fresh FC `SERVO_OUTPUT_RAW` change | QuadPlane v1 mutation support, physical servo movement, flight command support or payload safety |
| Independent source-250 mode request while NOMAD owns/is revoked | SITL controller behavior; simulated external-source takeover | Fresh FC heartbeat mode transition | RC/ELRS pilot priority, simultaneous conflict policy or physical takeover |
| NOMAD-path pause/recovery and runtime restart | Software plus SITL controller behavior | New session/incarnation, no owner, rejected old context, no command frames | Production C2-loss response, redundant radio failover or FC restart |
| `SIM_RC_FAIL` injection/restoration | SITL controller behavior | Disarmed receiver-health loss/recovery in `SYS_STATUS` | Physical receiver loss, aircraft failsafe action or pilot takeover |
| Approved transmitter/receiver input and source conflicts | Controller bench qualification | FC RC input, mode, command acceptance, measured output and timing | Airborne dynamics |
| Takeover in hover, fixed wing and transitions | Physical flight qualification | Independent mode/output/trajectory and pilot-control evidence | Untested aircraft, firmware, phases or faults |

The live runtime authority scenario uses the existing Copter-4.7.1 `quad`
stack, verified at firmware SHA
`dbe792162d06cab66c3475fd5556bf7a120f119e`, with the unchanged
[fence](../docker/sitl-fence.parm) and [stream](../docker/sitl-streams.parm)
files. It requires a dedicated native Docker simulator before sending anything
and reads back `SERVO5_FUNCTION=0`. Channel 5 here is a disabled simulator
output, **not** an approved production RC/payload channel. Protocol v1 has no
flight-mode request, so its runtime operation is an output change. The initial
disabled output can be zero; runtime PWM validation is preserved. After NOMAD
shutdown the isolated test GCS restores that initial output and observes fresh
FC readback. See the
[scenario procedure](../tests/sitl/README.md#runtime-authority-and-independent-source).

The attempted pinned QuadPlane runtime output scenario exposed an existing
capability limit: [`supports_operation`](../src/vehicle/operation.cpp) rejects
servo/relay/motor-test/gimbal requests for QuadPlane. Consequently **none of the
v1 runtime mutations is executable on QuadPlane**. Admission alone does not
qualify an operation. The capability gate remains intact; successful Copter
runtime-output evidence cannot be promoted to QuadPlane. Its pinned
[`quadplane-tilttri.parm`](../docker/quadplane-tilttri.parm) and existing
independent mode/RC-fault scenarios establish separate controller evidence.
Qualifying a future typed QuadPlane runtime operation remains open.

`handback` means explicitly returning software command authority **to NOMAD**
after revocation. `revoke` releases that authority. Neither operation switches
an FC mode, establishes pilot control, selects a receiver or silences an
external MAVLink source. Revoke prevents later covered sends; it does not undo
an already accepted FC output command or reset a latched output. Record the
actual output after revoke and select its approved safe restoration procedure.
After an admitted runtime loses a session, `admit`
cannot substitute for `handback`; a newly restarted inhibited runtime instead
requires a new `admit`. Link restoration alone does neither.

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
| `Q_ENABLE`, `Q_TRANS_FAIL`, `Q_TRANS_FAIL_ACT` | Pinned SITL classification/transition profile (`Q_ENABLE=2`); transition-failure parameters belong to phase-specific qualification, not source arbitration. This PR does not change them or establish a production C2 response. |
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
