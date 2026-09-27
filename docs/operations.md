# Operations

Target procedures reconciled to CONOPS v1.0, 2026-09-10. Profile selection does not
authorize hardware use or prove readiness. [Current qualification status](migration.md#current-qualification-status)
owns evidence scope and gates;
[architecture](architecture.md) owns component boundaries.

## Task and deployment matrix

| Task/profile | Aircraft-side functions | Ground-side functions | Qualification status |
|---|---|---|---|
| Task 1 / groundstation_gpu | Proposed lightweight VTOL; Walksnail FPV camera; Pi Zero for backup LTE and possibly video; Here4 GNSS | GPU CV, video capture/display, C++ core, Mission Planner and server adapter | User direction; video capture, RF bandwidth, complete QuadPlane mission and endurance unqualified |
| Task 2 / onboard_companion direction | Under-15-kg quad; Jetson Orin Nano and intended robotic arm for tracker/egg/droppings | Mission Planner, operator approvals and tracker CSV export; server applicability Q04 | Project direction only; hardware, arm feedback and integrations unqualified |
| groundstation_minimal | ArduPilot, navigation and selected command/telemetry link | C++ core and clients; lightweight server exchange when implemented | Product requirement; no ROS/perception dependency |
| Development | Isolated Copter and pinned QuadPlane SITL | Core, fake/mock services and passive observers | See the [current qualification status](migration.md#current-qualification-status); hardware startup and broader profile matrix remain open |

Use a strict under-15-kg project ceiling including maximum task payload; the
CONOPS maximum-15-kg wording conflicts with its under-15-kg FRR wording (Q03).
Weigh every configuration and budget measurement margin. Task 1 has no battery
swap. Task 2 swap permission is unstated: plan single-battery completion until
organizer clarification, retaining swap recovery only as a conditional design.

A Cube Orange or custom ArduPilot controller is under consideration. UART/PWM
availability, power budget, timing, firmware support and channel mapping require
board-specific qualification. Here4 with an RTK base is the stated navigation
choice; verify correction transport, fix state and age, and behavior without
RTK corrections. Walksnail receiver capture/export interface and any second
camera remain to be selected/tested. The plan has no ZED dependency.

## Competition runbook requirements

The [PRD inventory](conops-requirements.md) owns exact rules and scores. This
runbook translates them into preparation, not a flight authorization.

1. Complete eligibility, registrations, insurance and per-aircraft FRR documents;
   obtain judge acceptance and schedule the props-off termination demonstration.
   Keep private certificates and receipts outside source control. Verify the hard polygon and
   resolve Q02 aircraft-phase termination before loading a competition fence.
2. Prepare the proposal by 2027-01-15 1700 ET, FRR by 2027-04-28, bilingual
   presentation by 2027-05-13 2359 ET, and mission report by 2027-05-26 1700 ET.
   Team/member registration dates are 2026-11-27 and 2027-03-12. Follow all format,
   page, heading and feedback-attestation conditions in AE27-ADM requirements.
3. Receive official task geometry/course by Thursday night; validate non-convex
   boundaries, AGL limits, route feasibility, payload search areas and Task 2
   100 m moving-target offset. Rehearse with variable windows; approximately
   30 minutes is not a guaranteed duration or an endurance budget.
4. Arrive with all equipment at least 30 minutes before window (scored rule).
   At most five flight crew at line; no communication with other team members
   during window. No field access without RSO permission. Use ground propeller
   inhibit, effective checklists and explicit per-aircraft pilot assignments.
5. Keep transmitters off until window or explicit FRR authorization. Obtain RSO
   permission before each takeoff. Display live position and competition area
   on a dedicated GCS display for each aircraft. Report GPS/link/RC/fence anomalies.
6. Task 1: establish server session before arming as the selected startup strategy;
   send required 1 Hz telemetry whenever armed; fly complete lap course then
   survey. Monitor cylinder separation and operator response deadlines. Submit
   labelled `<your_team_name>_task1_survey.txt` in Phase 2 Deliverables; preserve
   Drive receipt, with hard cutoff 15 minutes after window. Earlier submission
   earns the stated multiplier. Land safely and clear all UAS parts by window end.
7. Task 2: verify one permitted tracker placement, withdraw at least 100 m, then
   maintain that horizontal offset from the moving target through sample pickup
   and flight-line return. Record five-minute tracker path and submit chronological
   CSV before window ends. Return intact samples onto the blue 32-inch pad before
   cutoff. Manual unloading is allowed; record intervention honestly for scoring.
   Land safely, leaving only the one attached tracker in the field.
8. Stop transmissions at window end. A restart forfeits prior-attempt points;
   reconcile selected attempt and do not merge prior evidence silently. Do not
   publish task setup/perceived performance until every window finishes unless
   Chief Judge permits it. No on-site rehearsal flights without permission.

Use the existing plugin soft-from-hard feature unchanged: the soft boundary
is an internal configurable inward margin, such as 5 m, from the hard polygon
(U-FEN-01; Q01 resolved by the project owner). A soft-margin crossing is not itself
a termination trigger; hard-boundary violation requires termination. No separate
official soft polygon is required. Lost termination control must
cause independent aircraft self-termination; ordinary link failover cannot waive
that rule. Safety lead must approve the exact definition of surviving C2 and prove
it for the selected radio/LTE topology. RTK, video and competition-server outages
are distinct faults from loss of the termination path.

## Profile and configuration lifecycle

The profile files and scripts/profile.py exist. The three product names are
onboard_companion, groundstation_gpu and groundstation_minimal. dev, drone and
the older groundstation template are migration artifacts, not extra product
architectures. Run profile-list to inspect template names; profile-load writes
ignored runtime configuration and also changes Mission Planner configuration.
Do not load a profile just to read it.

The templates use explicit C++ UDP endpoints and contain no deployment key.
Invalid endpoints and mismatched profile identity fail before activation; profile
switches clear stale owned Mission Planner settings. Optional ROS, video and
container services remain disabled until their image, camera and estimator are
qualified. NOMAD_VIO_SOURCE_REQUIRED=false does not bypass the C++
VIO-conditioned velocity gate. A capability flag does not establish a healthy
sensor. G3 must validate effective configuration end to end.

Target activation: choose profile, aircraft/firmware identity, single core host,
command transport, navigation capability, video source, payload mapping and
server config independently; validate them; then activate while disarmed.
Reject invalid/missing required settings. Record sanitized configuration version
and capabilities; keep real settings/credentials in ignored local storage.

## Connection behavior

The persistent `nomad-runtime` and one-shot C++ CLI accept udp, udpin and udpout
endpoint schemes and hand them to the MAVSDK transport. In the ground-router
topology, the runtime default is `udpin:127.0.0.1:14601`; the router consumer
sends to that local MAVSDK endpoint from `127.0.0.1:14602`. Runtime IPC is a
separate TCP endpoint at `127.0.0.1:14611`. Set `NOMAD_RUNTIME_IPC_PORT` when
the runtime and clients need another loopback port. Native serial/TCP are
possible MAVSDK deployment capabilities but are not current CLI transports.
Routers bridge selected physical links to UDP. Core placement does not follow
automatically from compute placement.

## Persistent C++ runtime

Start one `nomad-runtime` process for clients that use persistent mode. It owns
one MAVSDK connection and one `Vehicle` until shutdown. It starts its local IPC
listener even while aircraft identity is unresolved; HELLO and STATUS remain
available during startup. Configure `NOMAD_MAVLINK_ENDPOINT`,
`NOMAD_RUNTIME_IPC_PORT`, `NOMAD_API_KEY`, and the existing fence/velocity
environment settings before starting it. The runtime retries the same MAVSDK
connection object after a link loss. A client disconnect does not recreate the
vehicle connection or cancel a command already executing.

Mission Planner selects `PersistentRuntime` in its NOMAD settings and uses the
configured loopback port. `LegacyOneShot` remains the compatibility default
until a deployment opts into the runtime. The C++ CLI can use
`nomad --runtime status`, `nomad --runtime servo <channel> <pwm_us>`, and other
protocol-v1 typed operations. Bare verbs and `nomad --direct` retain a one-shot
MAVSDK connection for exclusive debugging; use that mode only when no runtime
or other NOMAD client owns the same MAVSDK endpoint. Runtime mode accepts no
MAVLink endpoint override; configure the endpoint on `nomad-runtime` instead.

The protocol is versioned JSON Lines, limited to 64 KiB per message, and bound
to IPv4 loopback. Mutating requests run one at a time; a concurrent mutation
returns `busy`. A cached request ID returns the original response during the
runtime process lifetime. If the response is lost, the client reports unknown
outcome and must not automatically issue a fresh request. The cache is
in-memory, so restart clears it and does not resume work. Local machine access
is a trust boundary; `NOMAD_API_KEY` is only a nonempty actuation gate, not
client authentication. Native Mission Planner MAVLink, RC/pilot, ArduPilot,
ROS and maintenance tools remain independent authorities.

The heartbeat-gated SITL harness uses a `udpout:` endpoint so MAVSDK sends the
pre-latch GCS announcement to the relay and the relay can learn the ephemeral
source port. The transport emits 1 Hz GCS heartbeats while waiting/polling —
MAVSDK does that for its `GroundStation` configuration; after peer latching it
uses the learned peer. The removed `NOMAD_RELAY_ADDRESS` override is no longer
read by the MAVSDK transport.
is unrelated to the competition's required 1 Hz telemetry upload.

For remote core placement, a secure network and an authenticated client protocol
are both required; current IPC is local-only and does not authenticate clients.
Until global handover is implemented, use one selected NOMAD runtime owner.
Running the ROS node, legacy one-shot CLI or another direct MAVLink writer
against the same endpoint is not a qualified integration.

The [current D09 direction](prd.md#c2-and-termination-direction-2026-09-27)
uses ELRS as primary RC/MAVLink C2 and LTE/MAVLink as redundant command/data,
including an intended ground joystick manual path. FPV uses a separate radio;
video alone never restores C2. LTE may also serve the Task 2 Jetson. Keep command
latency, telemetry, RTK and optional video budgets separate; common power,
receiver, computer, antenna and router failures still require qualification.

CH5 currently arms. Before accepting a configuration, read back the actual
flight-mode channel, RC auxiliary functions and transmitter channel mapping;
prove the arming/termination/mode/payload controls are distinct. Do not assume
defaults: [Copter 4.7.1 defaults mode selection to CH5](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/ArduCopter/config.h#L565-L567),
while [pinned Plane defaults to CH8](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/config.h#L34-L45).
The repository profile omits `FLTMODE_CH` and `RC5_OPTION`, so production
CH5 compatibility is not verified by SITL or these documents.
The native `quadplane-tilttri` defaults loaded by the simulator select CH5:
the disarmed probe read back `FLTMODE_CH=5` and `RC5_OPTION=0`. That profile
does not implement the team's CH5 arming configuration. Reconcile the actual
aircraft and transmitter mappings before accepting CH5 arming; the compiled
Plane default above does not establish the effective channel after profiles load.

The red Arduino HID ground button and independent two-control transmitter chord
request the same latched termination intent. The chord must work with Mission
Planner, NOMAD, LTE and the ground computer unavailable. Dedicated RC function,
MAVLink mechanism, safe multi-route delivery and post-flight reset remain to be
qualified. The plugin has no operational termination dispatch: its monitored
button and hard-boundary request report unavailable and send no substitute
command. Descent-speed settings and LAND fence-action translation are removed;
the plugin's vehicle-fence uploader/clear writer is deleted. Export to the Plan
map changes only the visual outline; it is not aircraft containment evidence. A restored link must not reclaim pilot
authority or clear termination.

## Startup, degradation and recovery target

1. Establish physical safety, correct aircraft and firmware, configuration
   version, selected core owner and manual control authority.
2. Start routing and the core; verify ownship identity, fresh individual state,
   reviewed limits, fence readback and payload-safe state.
3. Start only selected adapters. Verify camera acquisition, clocks/calibration,
   CV/VIO/video health, tracker feedback and server connectivity as required.
4. Display capability reasons and ages; permit only missions whose prerequisites
   are met. Obtain explicit payload permission separately.
5. Record mission state and evidence; monitor data age, link margins and energy.

| Failure | Required behavior |
|---|---|
| Video/CV lost | Mark unavailable/stale; stop dependent task decisions; keep eligible flight/telemetry functions |
| VIO lost | Refuse/stop VIO-dependent control; use only separately qualified navigation/recovery |
| RTK corrections lost | Show changed fix quality/age; apply reviewed navigation accuracy policy |
| Traffic/server lost | Mark traffic unknown and delivery impaired; apply approved task loss-of-feed procedure |
| One link lost | Continue only with termination authority intact and measured surviving capacity; otherwise use approved self-termination behavior |
| LTE lost, ELRS healthy | Primary RC and intended independent ELRS termination path remain; degrade LTE-dependent services, not complete C2 |
| ELRS lost, LTE healthy | Alternate joystick/red-button C2 may survive only if the full path is healthy and approved; do not infer total C2 or competition compliance from LTE connectivity |
| NOMAD/MP/computer lost, ELRS healthy | RC pilot and transmitter termination path must remain independent; revoke stale automation and require explicit handback after recovery |
| ELRS and LTE lost, with or without FPV | Complete external C2-loss case; require approved onboard aircraft response. Video does not restore command authority |
| FPV lost, command links healthy | Separate awareness/video fault; apply reviewed flight procedures and visual-operation constraints, not a C2-loss classification |
| Termination/C2 path lost | Aircraft must self-terminate under the approved all-mode mechanism; a ground stop attempt or ordinary RTL is insufficient evidence (AE27-OPS-015/019) |
| Core/client restart | Reconcile authoritative aircraft/task state; expire permissions; no automatic motion resume |
| Payload outcome uncertain | Mark unknown, inhibit retry, inspect/reconcile physical state |
| Task 2 battery swap, only if permitted (Q03) | Land/disarm, make payload safe, preserve records, reset permissions, preflight and explicit resume |

These are target rules. Numeric deadlines and aircraft-specific abort actions
need D07/D08 and safety approval. A resumed heartbeat does not automatically
resume a mission, and fixed-wing motion cannot be made safe by a Copter zero
velocity command.

## Simulation and test operations

`pixi run core-sitl-quadplane-rc-loss-probe` is a disarmed simulator-only tool.
It requires the dedicated `nomad-quadplane-probe` Compose project and container
`nomad-quadplane-rc-probe`, the pinned image/profile and output ports 14680/14681.
It rejects a reused/default simulator, unexpected firmware, armed aircraft,
stale observations or missing parameter readback. Build the pinned image first
with `pixi run quadplane-sitl-build`, then start a fresh instance with:

```bash
docker compose -p nomad-quadplane-probe -f docker/docker-compose.quadplane.yml run \
  --rm --name nomad-quadplane-rc-probe -d --no-deps \
  -e SITL_UDP_OUTPUT_ADDRESS=udp:host.docker.internal:14680 \
  -e SITL_UDP_OBSERVER_ADDRESS=udp:host.docker.internal:14681 quadplane_sitl
```

After simulator warm-up, the tool reads failsafe/channel parameters, proves a
healthy no-injection control, sets only `SIM_RC_FAIL` to 1 (no pulses), observes
three fresh unhealthy RC-receiver samples, restores 0 and observes healthy
recovery. The separate observer sends no heartbeat or commands. Failure to
deliver or restore the fault fails the tool; cleanup errors do not hide the
original failure. Stop only the dedicated container after the attempt:
`docker stop nomad-quadplane-rc-probe`. Reset it before another attempt.

JSON evidence records source/firmware identity, dirty-source status, parameter
readbacks and monotonic health samples. Nightly/manual SITL retains two clean
attempts before the existing flight chain, with a three-minute step limit.
This proves simulator RC-fault delivery while the MAVLink observation/control
path remains available. It does not prove RF loss, airborne failsafe response,
complete ELRS/LTE loss, manual authority, termination or CH5 production safety.
Use approved aircraft-specific policy and independent live state evidence for
those gates; Q02 and the active loss/takeover ledger item remain open.

Read [development](development.md) before running tasks. Current known-good local
checks include core/Python tests, retained-package checks and daemon-free Compose
resolution. `dev` and `dev-build` build the C++ core; bootstrap and Jetson setup
use the product profile manager and retained systemd inventory. Remote setup
requires a pre-trusted SSH host key and does not open an HTTP API port. Live image,
SITL and ROS runs still require G1 qualification
before `dev-up` or `sitl` can be treated as verified quickstarts.

SITL runners already exist for status, command-flow, mission, watchdog, fence,
payload, link loss/recovery, GCS-heartbeat and zero-delivery. Run them serially
against an isolated identified simulation to qualify the repaired startup. Verify
disarmed/known state between scenarios. Fault injection must never target a
real aircraft by accidental endpoint reuse. A passive Mission Planner observer
may use the configured simulator TCP observer link; it must not issue commands.

Use a separate pinned QuadPlane SITL vehicle for transition, route,
return/recovery and landing qualification. Existing Copter runs do not qualify
those operations. Add mock competition telemetry/traffic and
recorded image/tracker feeds before demanding GPU simulation. Gazebo/Isaac are
optional when sensor/physics evidence requires them, not core build dependencies.

The bounded fixed-wing recovery verb is `fixed-wing-recovery <latitude>
<longitude> <relative_altitude_m>`. Supply a deliberate recovery point after
the qualified two-point route; the current core does not infer a home point.
The command requires armed QuadPlane fixed-wing GUIDED state, fresh position,
GPS, VTOL and heartbeat telemetry, and `Q_GUIDED_MODE=0` autopilot readback.
Success means at least 10 m of post-ACK progress and a new position within
45 m horizontally and 5 m of the requested
relative-home altitude. The 45 m radius is a fixed-wing SITL qualification
tolerance, not obstacle clearance. The aircraft remains armed in GUIDED and
circles the target. The subsequent transition-to-VTOL operation requires an
independent operator/test authority to establish AUTO first as described below.
VTOL landing is handled by a separate QLAND operation; it is not part of
transition-to-VTOL. The overall deadline is 180 s; the ACK wait is capped at
3 s. Runtime IPC v1 does not expose this verb.

The semantic direct CLI verb `transition-to-vtol <latitude> <longitude>
<relative_altitude_m>` qualifies the next handoff for the pinned QuadPlane
profile. ArduPlane 4.7.1 accepts `MAV_CMD_DO_VTOL_TRANSITION` only in AUTO, so
the recovered GUIDED aircraft is not ready by itself. An independent operator
or qualification authority must replace the completed route with one
`NAV_LOITER_UNLIM` item at the explicitly reviewed recovery coordinates and
establish AUTO. Fresh recovered altitude must be within 15–25 m above home.
The test authority keeps the explicit recovery-point altitude as the AUTO loiter
target. The pinned profile's `Q_ENABLE=2` AUTO entry starts in VTOL AUTO,
so the already-qualified NOMAD `transition-to-fixed-wing` operation restores
fixed-wing flight before readiness is measured. NOMAD still rejects arbitrary
QuadPlane `set_mode`, re-reads `Q_ENABLE=2`, and independently requires the
aircraft to remain armed, in AUTO, fixed-wing and inside the reviewed
transition-ready envelope. It also verifies the pinned frame and tilt profile
(`Q_FRAME_CLASS=7`, `Q_TILT_ENABLE=1`, `Q_TILT_MASK=3`, `Q_TILT_TYPE=0`,
`Q_TILT_RATE_UP=40`, and `Q_TILT_MAX=45`) before transmission.
`Q_GUIDED_MODE=0` controls the preceding GUIDED recovery reposition; it has no
effect on this AUTO-only transition handler.

That envelope is within 55 m horizontally of the explicit recovery coordinates,
with the requested and measured altitude both between 15 and 25 m above home
before transmission,
with at most 28 m/s groundspeed, at most 3 m/s groundspeed variation and 1 m/s
climb rate. The measured altitude may differ from the loiter target within this
band, but five distinct fresh position samples must span 2 s with altitude
variation within 1 m and radial-distance variation within 8 m. The
55 m boundary extends beyond the measured recovery completion point; it is
separate from the recovery completion tolerance of 45 m and 5 m. The live
observer reports the farthest sample in its stable readiness window. The dwell,
speed variation and climb limits reject samples that move through the region
without settling into a stable loiter.
Success requires an accepted
ACK followed by newer authoritative `VTOL_STATE=Multicopter` observations and
two seconds of stable fresh position/velocity, with the aircraft still armed
in AUTO. After transmission altitude must remain at least 15 m above home; the
pre-transition 25 m ceiling does not apply during transition climb. The command
does not land or disarm. Generic land/RTL/QRTL, link-loss/manual takeover and
hardware flight remain unqualified. VTOL landing is covered by the separate
QLAND operation below. Runtime IPC v1 does not expose
the operation.

Historical runs [35980310680](https://github.com/YoussGm3o8/NOMAD/actions/runs/35980310680)
and [36210548163](https://github.com/YoussGm3o8/NOMAD/actions/runs/36210548163)
record the pinned transition, Copter regression and full QuadPlane chain. Their
provenance and scope are in the [current qualification status](migration.md#current-qualification-status);
dated measured traces remain in the migration evidence. Hardware flight remains
unqualified.

The dedicated direct CLI verb `quadplane-vtol-land <latitude> <longitude>` is
restricted to the pinned ArduPlane 4.7.1 `quadplane-tilttri` profile. It admits
only an armed AUTO multicopter with fresh heartbeat, position, velocity, GPS,
VTOL and landed-state telemetry; exact firmware/profile parameter readback;
15–25 m altitude above home; at most 1 m/s groundspeed and 0.25 m/s absolute
climb; and five distinct fresh samples over 2 s within 5 m of the supplied
landing point. The point is only a bounded reference; QLAND holds position and
descends. NOMAD sends one `MAV_CMD_DO_SET_MODE` request for custom mode 20 and
does not expose arbitrary mode setting, generic landing, RTL/QRTL or mission
execution. The ACK proves command acceptance only. Completion requires newer
post-command QLAND telemetry, at least 5 m of descent, fresh `ON_GROUND`,
disarm and five stable final samples over 2 s within 5 m, altitude -1 to 1.5 m,
groundspeed at most 0.5 m/s and absolute climb at most 0.2 m/s. The historical
full-chain result and its independent observer trace are in the [current
qualification status](migration.md#current-qualification-status) and dated
migration evidence. Earlier harness failures stopped before QLAND and remain
recorded in the migration history. The operation is not available through
Runtime IPC v1.

## Observability and evidence

Display ownship and each field's age, aircraft type/mode, command owner/outcome,
mission progress/cancellation, navigation/RTK state, traffic feed age/advisories,
server delivery backlog, video age, tracker identity and payload safe/unknown
state. Log transitions with correlation IDs, configuration versions and clocks.

Measure update cadence/jitter, stale events, lost/reordered traffic, queue depth,
command latency, watchdog timing, CPU/GPU/memory/thermal headroom, dropped frames
and disk use. Logs rotate and disk-full has an explicit alert. Store imagery,
flight logs and tracker paths with access controls and retention chosen at D11;
repository evidence manifests are sanitized and reference private artifacts.

## Security and packaging

The current CLI/runtime accept any nonempty NOMAD_API_KEY: a local opt-in, not
identity verification. OS account/file/IPC permissions are the immediate trust
boundary. Runtime IPC has bounded parsing, request IDs and an in-memory dedupe
cache, but no authenticated identities, durable request records, authorization
policy or complete lifecycle audit. Enable and test DDS security for any exposed
ROS command surface; no such protection is implied by ROS domain naming.

Separate competition credentials from local command credentials. Validate TLS
and server identity for the selected official protocol. Restrict media HTTP,
RTSP, SSH and MAVLink endpoints to intended peers. The retained media HTTP server
lacks authentication and therefore rejects non-loopback binds. Never ship
development credentials as production configuration or commit real hosts/keys.

Release packages contain the tested core/plugin/adapters, dependency notices,
configuration templates and procedures. Qualify OS/architecture/GPU drivers and
firmware pairs; test clean install and rollback. Plugin build/install scripts
may overwrite an installed plugin, and profile-load changes local runtime state.
Perform those only as separately authorized operations. No runtime infrastructure
was changed by this planning review.

## Ground router process and endpoints

Use the [ground router configuration](https://github.com/YoussGm3o8/NOMAD/blob/main/infra/transport/ground_router/README.md)
for generic N-link deployment and the standalone host build/run commands. The
example reserves physical UDP `14560`, `14550` and `14570` for the router. MP uses
UDPCl to router-owned loopback `14600`; C++ alone binds loopback `14601`, receiving
from the separate router-owned `14602` socket. Do not bind C++ to the RadioMaster
physical port. Stop the embedded router before starting the standalone host with
the same endpoints. An absent consumer does not stop other consumers.

The standalone host can outlive Mission Planner. Its default management endpoint
is loopback TCP `127.0.0.1:14610`, using version-1 bounded UTF-8 JSON Lines. The
plugin's `RouterMode = Standalone` client reconnects and displays status, health,
failover events, and stale/unavailable state. It can only select an enabled link
or return to automatic selection; it cannot send raw MAVLink or flight commands.
Structural router settings require restarting the host. Keep the endpoint on
IPv4 loopback and treat local OS access as the trust boundary.

Select either embedded or standalone ownership explicitly and never run both with
the same physical/consumer endpoints. Multiple raw clients can issue MAVLink, so
integrated operation still needs command-authority handover/inhibition. Stateful
mission/fence/FTP exchanges have no router transaction coordinator and require
caller recovery on link changes.
