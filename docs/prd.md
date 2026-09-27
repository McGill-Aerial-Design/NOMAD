# Product requirements

Baseline: official AEAC SUAS 2027 CONOPS v1.0, 2026-09-08; reconciliation
2026-09-10 against clean source commit `fab9f46` (including `6922371`).
NOMAD is a C++20 vehicle-control system with independent clients and optional
compute. This document is the product requirement authority.

## Requirement authority

| Class | Meaning |
|---|---|
| Confirmed CONOPS | AE27 IDs in the [source inventory](conops-requirements.md), with exact section/page and required evidence |
| Project decision (U) | Explicit project direction; not an organizer rule |
| Working assumption (R) | Engineering recommendation awaiting qualification |
| Unresolved (Q / TBD) | Insufficient or contradictory information; cannot close affected gate |
| Deferred design (D) | Team choice to make when dependencies and evidence are ready |

The complete supplied 36-page CONOPS is authoritative for this baseline. Its
identity, statement-by-statement inventory, scoring conditions, permissions and
interpretations are in the subordinate source inventory. That inventory is part
of this PRD. C means source-confirmed, not implemented. Later amendments must
be reconciled before release. The
[organizer page](https://www.aerialevolution.ca/2027-student-competition/) still described a preview
when checked during this review; it does not override the supplied v1.0 PDF.

## Competition requirements and superseded preview IDs

| Previous ID | Official source IDs | Reconciled requirement |
|---|---|---|
| P-T1-01 | AE27-T1-001 through AE27-T1-011 | Prescribed 3-5 km lap approach, deer totals, 10 m clusters, two-character tags, anomalous interactions and timed text report |
| P-T1-02 | AE27-NET-001 through AE27-NET-014 | Armed 1 Hz telemetry and 1 Hz simulated traffic; keep outside supplied cylindrical zones; no separate cooperation-event protocol specified |
| P-T1-03 | AE27-T1-012 | Safe flight-line landing by window end; field clear of UAS parts |
| P-T2-01 | AE27-T2-001 through AE27-T2-006 | Custom tracker under 250 g, maximum 8 cm per axis; attach one with hook-and-loop or use box; withdraw and maintain 100 m offset |
| P-T2-02 | AE27-T2-007 through AE27-T2-010; AE27-T2-018 | Five-minute path, chronological ISO8601 CSV before window end; 5/15 m accuracy scoring |
| P-T2-03 | AE27-T2-011 through AE27-T2-022 | Egg, cylindrical dung core and four droppings; intact return to 32-inch pad; autonomy is separately scored |
| P-AIR-01 | AE27-OPS-023 | One UAV per task; same or different designs/aircraft allowed |
| P-AIR-02 | AE27-OPS-024 | Maximum 15 kg wording conflicts with under-15-kg FRR; project uses strict under 15 kg with measurement margin |
| P-OPS-01 | AE27-T1-008; AE27-INT-001 | Task 1 no swaps confirmed; Task 2 permission unresolved, not an established rule |
| P-NET-01/02 | AE27-NET-001 through AE27-NET-014; AE27-INT-003 | Fields, units, rates and penalties specified; event requirement withdrawn pending official server contract |

Cross-task flight requirements now explicitly include 100 m AGL, continuous
per-aircraft GCS position/area display, all-mode containment and aircraft-specific
termination. ArduPilot and safety hardware must operate without ground C++
availability. Ground LAND dispatch and ordinary RTL are not termination evidence.
See [safety](safety.md), including the project boundary decision in Q01.

## Unresolved organizer questions

Q01 is resolved by project direction; Q02-Q09 remain questions to prepare for the organizers, not messages sent by this
review. Competition/safety leads own obtaining written answers and versioning them.

| ID | Missing or conflicting information | Consequence / required resolution |
|---|---|---|
| Q01 (resolved by project owner, 2026-09-10) | Appendix C labels remain inconsistent as a source note; no separate official soft polygon is needed for our design | Hard-boundary violation triggers termination. Soft boundary is internal, derived by a configurable inward distance from the same hard polygon, e.g. 5 m. Preserve the existing plugin implementation and settings; no plugin change requested |
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
and a duty to avoid them, not those detailed semantics. The named
[AEAC competition portal](https://aeac.mylonics.com/#/) responds as the AEAC
Student Competition Server, but its documentation is client-rendered and was not
exposed by the text-only retrieval available during this review. The verified
logical requirements and the still-unverified portal wire contract are tracked in
[AEAC 2027 integration](aeac-2027.md). No token was requested and no competition
telemetry was sent.

## Confirmed product direction

| ID | Basis | Requirement | Gate |
|---|---|---|---|
| U-CORE-01 | U | C++ owns vehicle behavior, safety validation, missions, telemetry models and MAVLink interaction | G2 |
| U-AP-01 | U | Preserve ArduPilot control, EKF and failsafes; NOMAD does not replace them | G2/G7 |
| U-PY-01 | U | Remove Edge Core and Python-owned vehicle decisions; retain Python for CV/ML/tools/tests | G1/G2 |
| U-ADAPT-01 | U | Mission Planner and ROS 2 are clients/adapters; no parallel vehicle policy | G2 |
| U-MOD-01 | U | Keep NOMAD general-purpose: competition/event-specific schemas, credentials, cadence and scoring behavior live in opt-in application modules with no core dependency on those details and no direct vehicle-command path | G2/G4 |
| U-PROF-01 | U | onboard_companion runs optional ROS 2, VIO, camera/video and perception on a Jetson/SBC aboard | G3/G7 |
| U-PROF-02 | U | groundstation_gpu runs those optional workloads on a GPU laptop with no Jetson aboard | G3/G7 |
| U-PROF-03 | U | groundstation_minimal works with direct MAVLink and C++ without companion/perception; missing features are explicit | G3 |
| U-TEST-01 | U | Important behavior is testable without hardware; safety needs invalid, boundary and failure cases | All |
| U-SCOPE-01 | U | This iteration reconciles plans and docs; no speculative implementation or destructive migration | G0 |

Board-independent MAVLink is the design boundary. Current verification is
Copter-oriented; this is not a claim that every ArduPilot vehicle type, board,
payload channel mapping, or transport has been qualified.

## User direction recorded during review

- Task 1: proposed lightweight VTOL, no onboard Jetson, groundstation_gpu for
  CV/video; Pi Zero for backup LTE and possibly video streaming.
- Task 2: quadcopter strictly below 15 kg all-up, with an onboard Jetson Orin
  Nano and a robotic arm intended to place the tracker and retrieve the egg,
  droppings and other permitted samples. This is project direction, not
  qualified hardware or payload capability.
- Walksnail FPV camera planned; additional camera TBD. No ZED dependency.
- Use the flight controller's IMU. Cube Orange or a custom ArduPilot controller
  is under consideration; UART/PWM counts and board integration need validation.
- Here4 GNSS with RTK base is intended; correction delivery and degraded-fix
  behavior require qualification.
- Custom tracker, possibly ESP32, remains undecided. Tagging and sample
  mechanisms/feedback are not selected.
- Begin with traffic advisories and explicit action authorization. Revisit
  automatic traffic/payload behavior only after reviewed evidence. CONOPS allows
  manual flight, requires actual traffic separation, and separately rewards autonomy.
- MAVSDK must be used at competition: early adoption, unit/integration testing
  and focused reviewable changes are required implementation work (D03 resolved).
  The pinned MAVSDK fork owns ArduPilot command, mode and telemetry semantics; NOMAD
  keeps safety policy, validation, verification of outcomes and the client
  contract — see [MAVSDK adoption](mavsdk-adoption.md).
- NOMAD remains general-purpose. AEAC 2027 and future event-specific behavior
  should be composed as optional modules at narrow generic boundaries rather than
  spread through the base core; minimize coupling and keep vehicle decisions in
  the core.

These are user product directions or explicitly tentative hardware choices,
not additional organizer rules. ArduPlane/QuadPlane support is necessary for
the proposed Task 1; Copter evidence cannot establish it.

## C2 and termination direction — 2026-09-27

D09 is substantially resolved at the topology/interface level:

- ELRS is primary C2, carrying normal RC pilot controls and MAVLink telemetry/data.
  The conventional transmitter is the primary manual interface. CH5 currently
  arms; verify actual flight-mode and auxiliary-channel mappings before use.
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

Jetson-originated aircraft requests pass through core authority/safety policy.
Flight and robotic-arm/payload authority are separate even when transport is shared.
These decisions do not qualify any aircraft, link, takeover or termination behavior.

## Internal soft boundary (project decision)

U-FEN-01, confirmed 2026-09-10: use the hard-boundary polygon as the authoritative
termination boundary. The soft boundary is an internal margin, generated by
insetting its sides by a configurable distance in metres (for example 5 m).
It is not a separately supplied competition polygon or a termination trigger
merely because the internal margin is crossed. Preserve the plugin's existing
SoftBoundaryFromHard / SoftBoundaryInsetMeters implementation as it is; the
current inset value defaults to 5.0 m. This decision does not change stored
settings, enable a mode, alter the 100 m AGL ceiling or qualify termination.
The PDF's contradictory soft/hard labels remain a provenance note, not a blocker
requiring a second official polygon.

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

AE27-NET-001 through AE27-NET-014 establish rates and scoring thresholds, not
engineering safety margins. A scoring penalty threshold is not a safe expiry limit.
Before implementation acceptance, choose measurable budgets for telemetry age,
traffic expiry and lookahead, command cancellation, inference latency, video age,
count/identity accuracy, map error, link capacity and endurance reserve (D07/D08).

Every optional feature reports one of: unconfigured, unavailable, starting,
ready, degraded, or failed, with reason and observation age. This is an R target
contract, not an implemented capability service. Readiness requires runtime
evidence, not GPU discovery or an environment boolean. Minimal operation supports
eligible non-perception missions; competition task readiness is evaluated
separately and can legitimately be unavailable.

## Decisions needing team input

Unanswered questions stay pending. Recommendations below may guide prototypes,
but cannot authorize autonomous actions, runtime changes, or release acceptance.

| ID | Decision | Recommendation and consequence | Needed before |
|---|---|---|---|
| D01 | Exact VTOL/quad firmware, hardware and Task 2 core/Jetson placement | Task 1 ground GPU and two aircraft types now directed; exact integrations remain TBD | G3/G7 |
| D02 | Core placement and client transport, groundstation OS | Persistent ground core for first integrated release; onboard authority only when required and remotely authenticated | G2/G3 |
| D03 | MAVSDK cutover before competition | Resolved and executed: MAVSDK is the only transport and the hand-written codec is deleted; the pinned fork owns ArduPilot semantics, and G-M still gates the firmware matrix and release | G-M/G8 |
| D04 | Tracker/tagger/sample hardware and assessment interaction | Select mechanics and feedback before defining autonomous payload behavior; current generic outputs are insufficient | G6 |
| D05 | Final traffic, approach and payload autonomy level | Retain advisories with demonstrated operator avoidance and explicit authorization; choose Task 2 bonus targets after Q06 | G4/G6 |
| D06 | Is VIO for mapping/perception or required flight navigation? | GNSS/ArduPilot navigation baseline; external-navigation fusion only after end-to-end timing evidence | G3/G7 |
| D07 | Official server wire contract and traffic semantics (Q04) | Transcribe/version the official portal contract first; model confirmed fields/cylinders in isolated fixtures; no invented events | G4 |
| D08 | Accuracy, latency, stale-data, reserve and operating-environment budgets | Agree numeric acceptance thresholds before collecting gate evidence | G3–G7 |
| D09 | C2, manual authority and termination | Topology/interfaces directed on 2026-09-27: primary ELRS, redundant LTE/MAVLink, separate FPV, RC and ground joystick, red button and independent transmitter chord. Arbitration, handback, exact RC/ArduPilot mapping, automatic loss response, common-mode analysis and qualification remain open | G2/G3/G7 |
| D10 | Named engineering, payload, perception, safety and test owners; capacity and dates | Assign accountable people to gates before promising a schedule | G1 |
| D11 | Evidence storage, retention and team/server credentials | Access-controlled artifacts; sanitized manifests in repository; no private datasets or secrets committed | G4/G8 |

User answers above resolve D03 and the initial D05 scope, establish U-MOD-01, and
partially resolve D01/D04 and D09. Other entries and Q02-Q09 remain open; the inventory
records v1.0 provenance, not organizer acceptance of our interpretations. Gate
ownership still needs named people.

## Scope boundary

NOMAD supports opt-in application modules at narrow public boundaries, but no
generic dynamic plugin registry, event bus, mission scripting language,
distributed workflow platform or replacement autopilot. Modules may translate
external protocols and observations; they do not become parallel command owners
or bypass the core to actuate a vehicle. One controlled aircraft per task does
not imply owning a multi-aircraft fleet; simulated traffic uses independent
track identities. Video display does not imply VIO navigation. Removing a Python
vehicle service does not remove supported onboard or ground GPU compute.

Implementation status and executable evidence live in [migration](migration.md);
safety requirements live in [safety](safety.md). Do not duplicate pass counts here.
