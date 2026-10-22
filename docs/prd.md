# Product requirements

NOMAD is a Mission Planner-only operator plugin. It adds custom profiles, gimbal,
payload/HID, video, maps and link routing while reusing MP's normal flight functions.
ArduPilot remains the autopilot. The [source inventory](conops-requirements.md) retains
stable AE27 IDs, CONOPS page references, owner, status and evidence gates. User product
direction is distinct from organizer requirements; implementation is not qualification.

## Unresolved organizer questions

Q01 is resolved by project direction; Q02-Q09 remain questions to prepare for the organizers, not resolved specifications. Competition/safety leads own obtaining written answers and versioning them.

| ID | Missing or conflicting information | Consequence / required resolution |
|---|---|---|
| Q01 (resolved by project direction, 2026-10-05) | Appendix C labels remain inconsistent as a source note; no separate official soft polygon is needed for our design | Mission Planner stores inner/outer polygons only as local visual advisory outlines. No outline crossing dispatches an aircraft action. Authoritative containment and termination require an approved aircraft mechanism and separate qualification. |
| Q02 | Fixed-wing full-surface direction/magnitude and QuadPlane transition termination semantics are unspecified; lost-mechanism rapid response has no independent numeric definition | Safety lead obtains aircraft-specific acceptance and proves all flight phases without disabling failsafes |
| Q03 | Maximum 15 kg versus under 15 kg; Task 2 swaps not expressly permitted | Keep strict under 15 kg; plan Task 2 single-battery until ruling; validate swaps only as conditional recovery |
| Q04 | Server protocol/paths/token lifecycle, field keys/nulls, timestamp units/clock rules, mode precedence, accuracy interpretation, startup armed/GPS validity, latency penalty, outage/retry/events and Task 2 applicability unresolved | Version and test official contract; do not invent JSON, endpoint paths, disarmed rate or failover permission |
| Q05 | 10 m cluster radius does not define overlapping clusters or centre construction | Confirm scoring oracle before selecting clustering algorithm; retain operator review |
| Q06 | Whether pre-sequence operator authorization preserves autonomous sample credit is unspecified | Keep explicit authorization baseline; no autonomy-credit claim until accepted; no new flight behavior in this pass |
| Q07 | Tracker clock alignment/CSV name and destination, exact 5/15 m score boundary, shortened trajectory scoring, and dung 75% measurement method unspecified | Confirm deliverable/measurement conventions; use private ground truth and conservative design margins |
| Q08 | Insurance details and fees TBC; incorrect FRR cross-references; event-certificate versus AEAC SFOC signature wording | Obtain final administrative package, applicable regulatory review and judge approval before flight |
| Q09 | Speed ranking interpolation/ties/one-finisher cases and actual flight-window length not fully fixed | Keep configurable scoring/rehearsal oracle; no fixed 30-minute endurance assumption |

Q04 also includes traffic altitude datum, vertical keepaway half-height versus
full height, coordinate frame, track timestamps/sequence, exclusion-boundary
equality, freshness/expiry, velocity availability, prediction horizon, right-of-way,
required alerting and response to stale/missing feed. CONOPS supplies cylinders
and a duty to avoid them, not those detailed semantics.

## Confirmed product direction

| ID | Basis | Requirement | Gate |
|---|---|---|---|
| U-CORE-01 | U | Ship one Mission Planner-native C# plugin; MP owns flight APIs and ArduPilot owns execution/failsafes | G2 |
| U-AP-01 | U | Preserve ArduPilot control, EKF and native failsafes | G2/G7 |
| U-AP-02 | U | Support Copter/Plane aircraft across controllers/frames through per-aircraft profiles and native MP behavior; qualify actual integrations separately | G2/G7 |
| U-PY-01 | U | Python is development/documentation tooling; no independent Python flight owner ships | G1/G2 |
| U-ADAPT-01 | U | Mission Planner is the operator application; no separate UI/CLI/IPC framework | G2 |
| U-MOD-01 | U | Implement only required custom operator controls; competition integrations need separate reviewed scope and evidence | G2/G4 |
| U-PROF-01 | U | Optional onboard compute/video/perception remains a hardware choice outside this plugin's implemented flight controls | G3/G7 |
| U-PROF-02 | U | Optional ground GPU perception/video remains separately integrated and qualified | G3/G7 |
| U-PROF-03 | U | Plugin operation does not require a companion computer, perception provider or runtime service | G3 |
| U-TEST-01 | U | Test important behavior without hardware, including invalid/boundary/failure cases; retain physical acceptance separately | All |

Optional aircraft notes and custom output/input/link/video settings belong in saved aircraft profiles.
Labels do not establish live aircraft identity, hardware support or safe calibration.

## User direction recorded during review

- Task 1: proposed lightweight VTOL, no onboard Jetson, ground GPU compute for
  CV/video; Pi Zero for backup LTE and possibly video streaming.
- Task 2: quadcopter strictly below 15 kg all-up, with an onboard Jetson Orin
  Nano and a robotic arm intended to place the tracker and retrieve the egg,
  droppings and other permitted samples. This is project direction, not
  qualified hardware or payload capability.
- Walksnail FPV camera planned; additional camera TBD. No ZED dependency.
- Use the flight controller's IMU. Aircraft may use different ArduPilot controllers
  and frames; UART/PWM counts and each board integration need validation.
- Here4 GNSS with RTK base is intended; correction delivery and degraded-fix
  behavior require qualification.
- Custom tracker, possibly ESP32, remains undecided. Tagging and sample
  mechanisms/feedback are not selected.
- Begin with traffic advisories and explicit action authorization. Revisit
  automatic traffic/payload behavior only after reviewed evidence. CONOPS allows
  manual flight, requires actual traffic separation, and separately rewards autonomy.
- Current direction is Mission Planner only. MAVSDK was an implementation choice,
  not an organizer requirement. Reuse native host flight APIs rather than building
  a separate command service, CLI or general-purpose frontend framework.
- Keep custom aircraft profiles, payload/gimbal/HID, video and required link behavior.
  Native software results and physical completion remain separate.

These are user product directions or explicitly tentative hardware choices, not
additional organizer rules. Copter evidence cannot establish Plane/QuadPlane safety.

## C2 and termination direction — 2026-09-27

D09 is substantially resolved at the topology/interface level:

- ELRS is primary C2, carrying normal RC pilot controls and MAVLink telemetry/data.
  The conventional transmitter is the primary manual interface. The owner reports
  Cube Orange Plus, ELRS arming on RC input CH5 and flight modes on CH6 for one
  aircraft. These describe that aircraft's native MP/ArduPilot setup, not NOMAD
  profile settings or defaults for other aircraft. Exact frame, calibration and
  FC readbacks remain open.
- LTE/MAVLink is secondary/redundant C2 and data. The intended alternate pilot
  path is ground-station joystick → Mission Planner/NOMAD → LTE → aircraft.
  This is not an independent RC radio or a qualified takeover implementation.
- FPV uses a separate radio and provides awareness, not command authority.
- The Arduino USB HID ground controller provides the joystick and a physical
  red termination button. Its termination request should attempt every approved
  healthy MAVLink C2 route where deterministic safe delivery is supported.
- A deliberate two-control transmitter chord requests termination through a
  dedicated ELRS RC function, without the ground computer, Arduino, Mission
  Planner, NOMAD or LTE. Its exact channel/function must not overlap arming,
  mode selection, ordinary flight controls or payload functions.

Both human controls request one logical `TERMINATE` operation. Distinguish
requested, transported, aircraft behavior entered and physically completed;
neither an ACK nor a mode dispatch proves termination. Once valid termination
is accepted it is latched for the current flight: autonomous/manual motion,
stale requests and reconnect must not override it. The reviewed aircraft
mechanism must define an explicit safe reset; reconnection is not that reset.
Do not select generic `MAV_CMD_DO_FLIGHTTERMINATION` or in-air disarm as a
substitute for the required controlled rotary descent.

Source priority, takeover signal/mode, cancellation and explicit handback,
complete-C2-loss detection, exact termination mapping/mechanism and common-mode
failures remain D09 engineering work. LTE availability alone does not prove
valid pilot input or surviving termination authority. Safety/organizer acceptance
of the surviving LTE path remains required. Q02 stays open: neither fixed-wing
to VTOL termination descent nor fixed-wing surface behavior throughout transition
is approved by this direction. No recovered source may unexpectedly reclaim control.

No Jetson-originated flight-control path ships in this product. Any later companion
request needs its own reviewed integration. Flight and payload use remain separate
operator responsibilities even when transport is shared.
These decisions do not qualify any aircraft, link, takeover or termination behavior.


## Mission workflows

Task 1: ingest reviewed course, lap count, survey polygon and AGL limits;
check single-battery energy reserve; connect before arming as a project strategy
to avoid startup-armed penalties; emit required telemetry throughout armed time;
fly the complete approach, avoid traffic cylinders, count deer/clusters and read
tags/anomalies; export the labelled text with upload receipt; safely land and
clear the field before the flight window ends. Retain imagery internally for
verification; it is not a mandated Task 1 upload.

Task 2: identify the leg-banded deer; authorize and observe one tracker placement;
move at least 100 m away and maintain that moving-target offset through every
remaining action, including sampling and return; record the five-minute path
and submit CSV before window end. Collect selected egg/dung/droppings samples,
return intact to the marked pad, land safely and account for all parts except
the attached tracker. The geometry may make sampling and the 100 m offset
incompatible; detect that before flight rather than violate the exclusion.

Manual/semi-autonomous workflows remain eligible. Operator-authorized sample
manipulation cannot claim the 20 autonomous-collection bonus points without a
ruling on Q06 and continuous no-intervention evidence. Autonomous takeoff and
landing are separate five-point criteria. Box placement earns zero attachment
points but is a permitted tracking strategy. This review recommends reliable
manual/authorized completion first; team D05 chooses which bonus paths to pursue.

Task 2 swaps are a conditional engineering recovery path pending Q03: land/disarm,
make payload safe, preserve records, expire permissions, recheck state and obtain
explicit resume. Never replay uncertain physical actions. Restarting an attempt
forfeits previous points; exports and permissions must stay associated with the
selected attempt, not silently aggregate attempts.


## Performance and capability contract

AE27-NET-001 through AE27-NET-014 establish rates/scoring thresholds, not safety
margins. Numeric budgets for received-data age, traffic expiry, command/helper waits,
video latency, map/identity error, link capacity and endurance require D07/D08 evidence.
A functioning MP connection or a loaded profile is not competition task readiness.
Features not implemented or qualified must be presented as such; no generic runtime
capability/authority service ships.

## Decisions needing team input

Unanswered questions stay pending. Recommendations below may guide prototypes,
but cannot authorize autonomous actions, infrastructure changes, or release acceptance.

| ID | Decision | Recommendation and consequence | Needed before |
|---|---|---|---|
| D01 | Exact VTOL/quad firmware, hardware and Task 2 core/Jetson placement | Task 1 ground GPU and two aircraft types now directed; exact integrations remain TBD | G3/G7 |
| D02 | Product placement and groundstation | Resolved by current direction: Mission Planner plugin on Windows, host-owned connection and ordinary flight controls; optional companion work remains separate | G2/G3 |
| D03 | Transport choice | Resolved by current direction: native Mission Planner APIs; former MAVSDK implementation archived. MAVSDK is not an organizer mandate | G-M/G8 |
| D04 | Tracker/tagger/sample hardware and assessment interaction | Select mechanics and feedback before defining autonomous payload behavior; current generic outputs are insufficient | G6 |
| D05 | Final traffic, approach and payload autonomy level | Retain advisories with demonstrated operator avoidance and explicit authorization; choose Task 2 bonus targets after Q06 | G4/G6 |
| D06 | Is VIO for mapping/perception or required flight navigation? | GNSS/ArduPilot navigation baseline; external-navigation fusion only after end-to-end timing evidence | G3/G7 |
| D07 | Official server wire contract and traffic semantics (Q04) | Transcribe/version the official portal contract first; model confirmed fields/cylinders in isolated fixtures; no invented events | G4 |
| D08 | Accuracy, latency, stale-data, reserve and operating-environment budgets | Agree numeric acceptance thresholds before collecting gate evidence | G3–G7 |
| D09 | C2, manual authority and termination | Topology/interfaces directed on 2026-09-27: primary ELRS, redundant LTE/MAVLink, separate FPV, RC and ground joystick, red button and independent transmitter chord. Arbitration, handback, exact RC/ArduPilot mapping, automatic loss response, common-mode analysis and qualification remain open | G2/G3/G7 |
| D10 | Named engineering, payload, perception, safety and test owners; capacity and dates | Assign accountable people to gates before promising a schedule | G1 |
| D11 | Evidence storage, retention and team/server credentials | Access-controlled artifacts; sanitized manifests in repository; no private datasets or secrets committed | G4/G8 |

D02/D03 now follow the Mission Planner-only decision. D01/D04/D09 hardware details,
D05 autonomy choices and Q02-Q09 remain open. Stable IDs retain their source history;
retiring a backend does not resolve organizer acceptance or aircraft qualification.
Current evidence boundaries are in [qualification](qualification.md). Detailed plans
and handoffs belong in ignored local notes.
