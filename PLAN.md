# NOMAD delivery plan

Planning baseline: CONOPS v1.0 (2026-09-08), reconciled 2026-09-10.
This is the delivery summary for the official AEAC SUAS 2027 assessment;
it does not approve flight, hardware changes, or deployment.

## Read this plan in order

1. [Product requirements and decisions](docs/prd.md): requirement provenance,
   competition scope, assumptions, and the decisions requiring team input.
2. [Architecture](docs/architecture.md): ownership and data/control boundaries.
3. [Qualification status](docs/qualification.md): current evidence and its exact
   scope; [migration archive](docs/migration.md) preserves dated reasoning.
4. [Safety case](docs/safety.md) and [operations](docs/operations.md): failure
   behavior and supported deployment procedures.
5. [TODO](TODO.md): the single actionable ledger.

Each subject has one canonical owner above. Component READMEs explain local
interfaces. [MAVSDK adoption](docs/mavsdk-adoption.md) is a subordinate transport
decision record, not another product roadmap. Historical CHANGELOG entries are
history, not current release evidence.

## Recommended system

The persistent C++ runtime is the software owner for supported typed vehicle
requests. It does not arbitrate native GCS, RC/ELRS or pilot input. ArduPilot
owns stabilization, EKF, navigation execution and independent failsafes.
Mission Planner presents operator workflows; ROS 2 observes telemetry and
Python provides ground-side tools. Competition mission and server adapters are
future application work, not current runtime capabilities.

A small persistent C++ runtime is now justified by concurrent clients, the 1 Hz
competition exchange, traffic processing, cancellation, and payload state. The
installed CLI is a typed runtime-IPC client. Keep direct vehicle access in the
explicitly built, non-installed qualification tool for SITL and transport tests;
introduce no generic framework or Python vehicle service.

Support all three product profiles: onboard_companion, groundstation_gpu, and
groundstation_minimal. A profile describes compute placement, not control
authority, navigation validity, or competition readiness. Minimal operation must
work without ROS/perception, but does not by itself satisfy wildlife assessment.

## Delivery sequence

| Phase | Outcome | Exit gate |
|---|---|---|
| Planning | Reconciled requirements, source inventory, decisions explicitly pending | G0 |
| Baseline repair | Build/CI/tasks match the deleted Edge Core tree; retain supported compute and video | G1 |
| MAVSDK adoption | Tested ArduPilot transport parity, production cutover and focused reviewable changes | G-M |
| Core authority | Runtime request admission, explicit software handback, fresh state and truthful outcomes; aircraft-wide arbitration remains open | G2 |
| Profile qualification | Each profile runs its declared capabilities and exposes missing ones | G3 |
| Competition integration | Armed 1 Hz telemetry, inbound traffic cylinders and demonstrated operator avoidance | G4 |
| Task 1 | Survey evidence, count/identity review, simulated-UAV coordination, return and landing | G5 |
| Task 2 | Tracker association/path, authorized tagging and specific samples, conditional swap recovery | G6 |
| Hardware qualification | Mass/endurance/link/sensor/payload evidence for selected aircraft and profiles | G7 |
| Release | Full rehearsals, security/observability, operator acceptance and reproducible package | G8 |

G4 can use recorded/synthetic observations before G5/G6. Payload and sensor bench
development may proceed after the relevant software fault gates; it cannot close
G7 early. Autonomous conflict maneuvers and external-navigation fusion have their
own safety evidence inside G4/G7, even if advisory traffic or CV already works.

User decision (2026-09-08): MAVSDK must be used during the competition and is an
early prerequisite, including unit tests, integration evidence and focused,
reviewable implementation changes. Complete G-M after baseline repair and before
dependent integrated command work. Phases A-E have landed: MAVSDK is the only
transport and the hand-written codec is deleted, so G-M now covers the firmware
matrix and release evidence rather than the adoption itself. Isolated server/CV
prototypes may proceed independently. Never run two production command owners as
an adoption experiment.

Task 1 targets a lightweight VTOL with groundstation_gpu, ground CV/video and
Pi Zero LTE backup. Task 2 currently targets a quad below 15 kg with an onboard
Jetson Orin Nano and robotic arm; hardware integration remains unqualified.
Narrow pinned-profile QuadPlane operations are qualified in SITL; generic mode
selection, QuadPlane link-loss/manual takeover and hardware qualification remain open.
Traffic advisories and explicit payload authorization are the initial scope;
manual operation is allowed; Task 2 autonomous collection/takeoff/landing are
optional scored criteria. Team scoring priorities remain D05/Q06. Hardware
detail is recorded in PRD.

## Planning milestones

CONOPS v1.0 section 2 fixes team registration 2026-11-27, proposal
2027-01-15 1700 ET, flight selection starting 2027-03-02, member registration
2027-03-12, FRR documents 2027-04-28, presentation 2027-05-13 2359 ET,
demonstration May 14-16 and report May 26 1700 ET. Default deadlines are 2359 ET.
The [PRD inventory](docs/conops-requirements.md) owns format/eligibility/scoring
conditions. Obtain final insurance/fees and boundary/server clarifications.

Recommended internal sequence: close requirement and hardware choices after
the remaining Q02-Q09 clarification review; substantiate the proposal with measured
feasibility; complete
integration and authorized rehearsals before the May demonstration; preserve
evidence for the report. These are dependencies, not staffing estimates or
promised dates. Team owners and capacity remain D10.

## Working discipline

The planning baseline is recorded in commit `d31b0aa`. Focused implementation
follows the remaining items in TODO. Runtime/profile/setup repairs and MAVSDK
Phase A evidence are preserved in the migration archive. The CONOPS
reconciliation was prepared from clean main at `fab9f46`; subsequent mainline
commits added MAVSDK hardening, qualification and finally the Phase E cutover,
which replaced the production transport outright. Preserve unrelated work and
follow AGENTS for publication and deployment authorization.

Gate closure requires a recorded artifact/configuration and independent observed
outcome. A source file, a configured CI job, a mock response, or an old pass count
is not a current integration result. See the current [qualification status](docs/qualification.md)
and the dated [evidence archive](docs/migration.md).
