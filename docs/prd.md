# Product scope and AEAC tasks

NOMAD is a Mission Planner operator plugin. Current capabilities are saved aircraft
profiles, video/detection/region tracking, calibrated gimbal/payload/HID controls,
native fence draft/configuration and redundant telemetry routing. Mission Planner
owns ordinary flight controls and missions; ArduPilot owns flight execution/failsafes.
The optional native termination firmware has a separate commissioning boundary.

[CONOPS traceability](conops-requirements.md) preserves source version, stable AE27 IDs,
page references, owners and required evidence. Requirements describe what the team
must demonstrate; they do not claim that an implemented competition system exists.
[Brainstorming](brainstorming.md) holds proposed task approaches and hardware ideas.

## Current boundaries

- Use native MP APIs and the selected aircraft connection for custom commands.
  Keep ordinary flight UI, navigation and failsafes in MP/ArduPilot.
- Keep per-aircraft calibration, payloads, inputs, video and links in profiles.
  A profile label does not identify the connected physical aircraft.
- Detection/tracking steers only the calibrated gimbal. Generic servo/relay outputs
  do not establish tracker placement, sampling or physical payload feedback.
- Competition-server telemetry, traffic-feed ingestion/separation, task scoring
  and official submission exports are not implemented competition workflows.
  Define their official contracts before adding adapters.
- Python is development/simulation tooling. No independent flight owner ships.

## Mission workflows to support

**Task 1:** review course/laps, survey polygon, AGL limits and single-battery reserve;
connect before arming as the project strategy; maintain required armed-time server
telemetry; complete the approach while avoiding traffic cylinders; count deer/clusters
and read tags/anomalies; submit labeled text with receipt; land and clear the field
before the window ends. Imagery is internal verification, not a mandated Task 1 upload.

**Task 2:** identify the leg-banded deer; authorize/observe one tracker placement;
withdraw at least 100 m and retain that moving-target offset for all remaining
sampling and return actions; record the five-minute path and submit CSV before
window end; return permitted samples intact to the marked pad and account for parts.
Check whether sampling/return geometry conflicts with the offset before flight.
Box placement is a permitted tracking strategy but earns no attachment points.

Manual/semi-autonomous operation is eligible. Autonomous takeoff/landing and sample
bonuses need their own evidence; operator-authorized sampling cannot claim the
autonomous collection bonus until Q06 is resolved. D05 is the team's choice of
autonomy targets. Task 2 battery swaps remain conditional on Q03: land/disarm,
make payload safe, retain attempt records and require explicit reauthorization.
Never replay an uncertain physical action or combine forfeited attempts' scores.

## C2 and termination policy

Physical RC is primary; ELRS is the intended primary C2/data link and LTE the
redundant MAVLink route. FPV provides awareness. RC loss or loss of the dedicated
termination input requires onboard response in every mode; GCS loss alone must
not terminate an aircraft with healthy physical RC/FPV. Both human termination
controls request the same latched onboard operation, with explicit safe reset.

[Native termination](native-termination.md) documents the implemented optional policy
and guarded ground request. D09 covers remaining transmitter mixing, USB controller
integration, alternate pilot authority, takeover/handback and common-mode fault
acceptance. LTE availability alone does not prove pilot authority. The action-44
configuration ignores flight RC overrides; alternate joystick flight needs a
separately approved design. Stock flight termination/in-air disarm is not a
substitute for the required controlled rotary descent. Preserve native failsafes.

The safety lead must approve aircraft/phase-specific behavior, including Q02.
[Safety and qualification](safety.md) lists the remaining acceptance boundaries.

## Unresolved organizer questions

Q01 is resolved by project direction; Q02-Q09 remain questions to prepare for the organizers, not resolved specifications. Competition/safety leads own obtaining written answers and versioning them.

| ID | Missing or conflicting information | Consequence / required resolution |
|---|---|---|
| Q01 (native fence setup direction, 2026-10-21) | Appendix C labels remain inconsistent as a source note; no separate official soft polygon is needed for our design | NOMAD stages the published C1 polygon in native Mission Planner Fence mode for operator review and native Write/Read. It does not upload/enable the fence or define a second polygon. Aircraft containment, failsafes and termination require approved configuration and separate qualification. |
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
