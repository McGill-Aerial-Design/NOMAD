# CONOPS requirement inventory

This is the source appendix of [PRD](prd.md), not a second product plan.
Source: **2027-AEAC-CONOPS-v1.0-2026-09-08.pdf**, version 1.0, 2026-09-08,
36 pages, supplied by the project team and read completely before edits.
SHA-256: `0097ad8029086e2163e613d4d234d4720ed151e90fdbde8c8889f1b748d93708`.
Page numbers below are printed PDF pages (also physical pages). No source PDF,
raw extraction, real deployment configuration or contact details are stored here.

Status C = confirmed statement in this version; I = interpretation requiring
approval; U = unresolved. Status describes provenance, never implementation
completion. Kind M = mandatory; S = scoring condition; P = permission; E =
environment/supplied input; R = recommendation. Permissions are not obligations
to exercise them. All IDs are permanent; retire rather than reuse them.
Safety H = directly safety relevant, M = supporting safety/evidence, L = mainly
administrative/scoring. Owner names are component or accountable team roles,
not a claim that software already exists. Every evidence cell is **required**
future acceptance evidence. Current implementation scope is in
[qualification status](qualification.md); dated run records are in the
[migration archive](migration.md).

## Administration and assessment

| ID | Source | Task/phase | Kind/status | Faithful requirement summary | Owner | Safety | Verification and evidence |
|---|---|---|---|---|---|---|---|
| AE27-ADM-001 | 1.2 p4 | All/eligibility | M/C | Submit Phase 1 to enter Phase 2; participate in Phase 2 to enter Phase 3. | Competition lead | L | Submission receipts and accepted participation record; G8 |
| AE27-ADM-002 | 1.3-1.5 p4 | All/assessment | M/C | Proposal covers specifications, development and mission strategy; demonstrate subscale UAS; report performance afterward. | Competition lead | M | Three reviewed deliverables; G8 |
| AE27-ADM-003 | 2 p5 | All/submission | M/C | Default deadlines are 2359 Eastern unless stated otherwise; all components must be complete on time. | Competition lead | L | Deadline checklist with timezone and receipt audit; G8 |
| AE27-ADM-004 | 2 p5 | All/submission | M/C | Upload deliverables to assigned shared Drive unless otherwise specified; Drive upload time is submission time. | Ground evidence tools | L | Upload receipt associated with final file/version; G8 |
| AE27-ADM-005 | 2.1 p5-6 | All/planning | E/C | CONOPS may change without notice; latest publication and team-contact notifications govern updates. | Competition lead | M | Version/hash comparison before each gate; G0/G8 |
| AE27-ADM-006 | 2.2 p6 | All/registration | M/C | Express interest with institution, team name, team contact and Google-compatible account; obtain shared folder. | Competition lead | L | Private registration acknowledgement; G8 |
| AE27-ADM-007 | 2.2 p6 | All/registration | M/C | Supply distinct captain contact, description, logo, photo and estimated Phase 2 attendance; pay team fee by 2026-11-27. | Competition lead | L | Completed registration checklist and payment receipt; G8 |
| AE27-ADM-008 | 2.3 p6; 5.1 p16 | All/proposal | M/C | Proposal due 2027-01-15 1700 ET; rankings announced by 2027-02-26; submissions become public after Phase 2. | Competition lead | L | Timestamped PDF receipt and publication-aware review; G8 |
| AE27-ADM-009 | 2.4 p6-7 | Both/scheduling | E/C | Flight selection starts 2027-03-02 in proposal rank order; choose morning/afternoon and order; actual schedule may change. | Flight lead | M | Confirm selected slot and organizer updates; G8 |
| AE27-ADM-010 | 2.4 p7 | Both/scheduling | E/C | Judges allocate unresponsive teams; loss of eligibility forfeits chosen window without changing other selections. | Competition lead | L | Eligibility and response checklist; G8 |
| AE27-ADM-011 | 2.5 p7 | All/registration | M/C | Submit Phase 2 member list, insurance proof and member payments by 2027-03-12. | Competition lead | M | Private submission/payment receipts; G8 |
| AE27-ADM-012 | 2.6 p7 | Both/FRR | M/C | Submit FRR documents by 2027-04-28; complete amendments and obtain judges' approval before flying. | Safety lead | H | Approved FRR package; G7/G8 |
| AE27-ADM-013 | 2.7 p7 | Both/FRR | M/C | Team schedules and performs safety demonstration before its first window; preference follows flight order. | Safety lead | H | Judge acceptance record; G7/G8 |
| AE27-ADM-014 | 2.8 p7-8 | Both/demonstration | E/C | Assessment is 2027-05-14 through 2027-05-16; one approximately 30-minute window per task, actual duration announced and variable. | Flight lead | M | Rehearsal with variable window cutoff; G8 |
| AE27-ADM-015 | 2.8 p7-8 Table 2 | Both/demonstration | E/C | Present Friday morning in flight order; Task 1 Friday afternoon/Saturday morning, Task 2 Saturday afternoon/Sunday morning, same order, roughly 24-hour gap. | Flight lead | M | Flight/team/equipment schedule; G8 |
| AE27-ADM-016 | 2.9 p8; 5.3 p27 | All/report | M/C | Mission report due 2027-05-26 1700 ET. | Competition lead | L | Final submission receipt; G8 |
| AE27-ADM-017 | 3.1 p9 | All/eligibility | M/C | Members enrolled part/full-time in Canadian college/university in fall 2026 and/or winter 2027; no member on multiple teams; same-institution proposals substantially different. | Competition lead | L | Private eligibility attestation; G8 |
| AE27-ADM-018 | 3.1-3.2 p9 | All/eligibility | P/C | Joint institutions and graduate/undergraduate members allowed; no overall team-size bound, but at most 10 on site. | Competition lead | L | On-site roster count; G8 |
| AE27-ADM-019 | 3.3 p9 | All/fees | M/U | Non-refundable team fee approximately CAD650+HST and member fee approximately CAD330+GST, both TBC; no Phase 3 fee. | Competition lead | L | Confirm final fees before payment; Q08/G8 |
| AE27-ADM-020 | 3.4 p9-10 | All/logistics | M/C | Team funds travel, excluded meals and transport among venues; communicate accommodation needs with roster. | Competition lead | M | Travel/accessibility/budget checklist; G8 |
| AE27-ADM-021 | 3.4 p9-10 | All/logistics | E/C | Included lodging Thursday-Monday, breakfast/lunch Friday-Sunday and Sunday banquet; event ends about 2200 Sunday; Monday departure advised. | Competition lead | M | Confirm organizer accommodation and safe-travel plan; G8 |
| AE27-ADM-022 | 3.5 p10 | All/eligibility | M/C | Only top 20 Phase 1 teams initially eligible for Phase 2; lower ranks waitlisted in order. | Competition lead | L | Acceptance/waitlist notice; G8 |
| AE27-ADM-023 | 3.6-3.7 p10 | Both/publication | M/C | Do not share task setup/perceived performance until all windows finish without Chief Judge permission; no cheating, falsified data, disobeying judges or sharing setup with unflown teams. | Team / evidence tools | M | Access/publication controls and crew briefing; G8 |
| AE27-ADM-024 | 5.1 p16 | All/proposal | M/C | English/French PDF in team Drive, maximum 15 pages including front matter/appendices except references; excess ignored; late penalty 10% per day or fraction. | Competition lead | L | PDF page/format/deadline preflight; G8 |
| AE27-ADM-025 | 5.1 p16 | All/proposal | M/C | Use required top-level headings: Introduction, Aircraft Design, Payload Design, Mission Strategy, System Testing, Project Management (or listed French equivalents); content outside them earns zero. | Competition lead | L | Heading/content audit against source; G8 |
| AE27-ADM-026 | 5.1 Table 3 p16-18 | All/proposal | S/C | Introduction 5; aircraft frame/performance and auxiliary systems 20; wildlife, sampling and tracker designs 30; strategy/environmental sensitivity 20. | Systems / payload leads | M | Rubric review with alternatives and measured design evidence; G8 |
| AE27-ADM-027 | 5.1 Table 3 p17 | All/proposal | S/C | Component and mission/system tests 20; risk severity/likelihood/mitigation, Gantt schedule and revenues/expenses budget 20. | Test / project leads | M | Test plan, risk register, schedule and budget review; G8 |
| AE27-ADM-028 | 5.1 Table 3 p17-18 | All/proposal | S/C | Justification, reputable references, Canadian content/IP/jobs 20; grammar, figures and professional client communication 15; total 150; criterion scale 0/4/7/10 before weighting. | Competition lead | L | Independent scored proposal review; G8 |
| AE27-ADM-029 | 5.2.2 p19 | Both/presentation | M/C | Pitch team expertise, design evolution/final design and each task; at most six minutes; English/French with at least one slide in the other language. | Competition lead | L | Timed bilingual rehearsal; G8 |
| AE27-ADM-030 | 5.2.2 p19 | Both/presentation | M/C | Upload PowerPoint or PDF by 2027-05-13 2359; native Google Slides not accepted. | Competition lead | L | File-format check and receipt; G8 |
| AE27-ADM-031 | 5.2.2 Table 4 p19 | Both/presentation | S/C | Organization/bilingual clarity 10, design evolution 15, execution 10, client confidence 5; total 40. | Competition lead | L | Rubric rehearsal; G8 |
| AE27-ADM-032 | 5.2.5 Table 8 p26-27 | Both/preflight | S/C | At flight line with equipment 30 minutes early and ready at window start; organization, easy setup and effective checklists; each 10 points. | Flight crew | M | Timed rehearsal and witness checklist; G8 |
| AE27-ADM-033 | 5.2.5 p26-27 | Both/scoring | S/C | Preparation uses lower score of the two windows for each criterion; Phase 2 total 325 = presentation 40 + Task 1 120 + Task 2 125 + preparation 40. | Competition lead | L | Independent rubric calculation; G8 |
| AE27-ADM-034 | 5.3 p27-28 | All/report | M/C | Report in English/French, at most five body pages; separate front matter excluded; excess unread; attest at least 50% of Phase 2 participants completed feedback survey. | Competition lead | L | Page/attestation audit and receipt; G8 |
| AE27-ADM-035 | 5.3 p27-28 Table 9 | All/report | S/C | Analysis, not flight success, is scored: mission performance 20, lessons/problems/improvements 25, writing/organization/feedback attestation 5; total 50. | Competition lead | L | Independent report rubric review; G8 |

## Aircraft, crew, containment and termination

| ID | Source | Task/phase | Kind/status | Faithful requirement summary | Owner | Safety | Verification and evidence |
|---|---|---|---|---|---|---|---|
| AE27-OPS-001 | 1 p1; 1.1 p4 | Both/all flight | E/C | Simulated BVLOS uses organizer spotters; no actual BVLOS and no real animals in mission. | Flight lead | H | Approved operating concept and crew briefing; G7 |
| AE27-OPS-002 | 4.1 p11 | Both/all flight/testing | M/C | Comply with CARs Part IX and CONOPS; obey RSO/Air Program Director, including pause/termination and lost window. | Flight lead | H | Current regulatory review and rehearsed command response; G7/G8 |
| AE27-OPS-003 | 4.2 p11 | Both/takeoff | M/C | PIC obtains RSO permission before every takeoff; airfield personnel follow RSO instructions. | Flight crew | H | Checklist and launch-permission witness; G7/G8 |
| AE27-OPS-004 | 4.2 p11 | Both/in flight | M/C | Each UAV has its own GCS display continuously showing real-time aircraft location and competition area. | Mission Planner / telemetry | H | Screen plus independent position/age replay and field witness; G3/G7 |
| AE27-OPS-005 | 4.2 p11 | Both/all modes | M/C | Remain in potentially non-convex polygon boundaries in manual, autonomous and return modes. | ArduPilot / C++ safety | H | Concave fence, edge, trajectory and all-mode tests; G2/G7 |
| AE27-OPS-006 | 4.2 p11 | Both/breach | M/C | Soft boundary including altitude: operator brings UAV inside or is ordered to terminate if unable; hard breach requires immediate termination. | ArduPilot / flight crew | H | Source wording retained; project U-FEN-01 distinguishes internal inset from hard termination; G7 |
| AE27-OPS-007 | 4.2 p11 | Both/anomaly | M/C | Report GPS, data-link, RC and boundary anomalies to Air Program Director. | Flight crew / diagnostics | H | Fault rehearsal with explicit reporting; G7 |
| AE27-OPS-008 | 4.2 p11-12 | Both/window / FRR | M/C | Transmitters on only in window, off after window; no C2 outside window except judge-authorized FRR transmissions. | Deployment / flight crew | H | RF/startup/shutdown rehearsal and authorization record; G7/G8 |
| AE27-OPS-009 | 4.2 p12 | Both/wildlife intrusion | M/C | Maintain regulatory separation from real wildlife including birds entering area. | Flight crew | H | Reviewed intrusion procedure and witness exercise; G7 |
| AE27-OPS-010 | 4.3 p12 | Both/flight window | M/C | Flight crew maximum five; only crew at flight line, no communications with other team members; crew stay at line unless RSO permits field access. | Flight crew | H | Roster and isolated-crew rehearsal; G8 |
| AE27-OPS-011 | 4.3 p12 | Both/vehicle movement | M/C | Separate pilot for each individual vehicle while flown or moved. | Flight lead | H | Movement/pilot assignment checklist; G7 |
| AE27-OPS-012 | 4.4 p12 | Both/FRR | M/C | Every pilot holds Advanced RPAS certificate, with recency evidence; provide certificates to Air Program Director. | Safety lead | H | Private certificate/recency review; G7 |
| AE27-OPS-013 | 4.4 p12; 4.8(b) p14 | Both/FRR | M/C | Register every RPAS under Part IX; FRR requires Canadian-citizen registration and matching model safety declaration. | Safety lead | H | Registration/model cross-check; Q08/G7 |
| AE27-OPS-014 | 4.4 p12; 4.9(c) p15 | Both/FRR | M/U | Organizer handles event approval; document says no team SFOC with Canadian-registered aircraft, yet FRR requires pilots to sign AEAC's SFOC. | Safety lead | H | Organizer confirms applicable instrument and signatures; Q08/G7 |
| AE27-OPS-015 | 4.5 p13 | Both/all modes | M/C | Every UAV has automatic or remotely activated safety termination, operational at all times; if termination mechanism unavailable, aircraft terminates itself rapidly. | ArduPilot / safety hardware | H | Independent mechanism-loss fault evidence in every mode; G7 |
| AE27-OPS-016 | 4.5 p13 | Both/fixed-wing termination | M/C | Shut engine down with full aileron, elevator up, full rudder and no motor; circling descent unacceptable. | ArduPilot / airframe | H | Approved surface-direction/authority bench evidence and authorized validation; Q02/G7 |
| AE27-OPS-017 | 4.5 p13 | Both/rotary-wing termination | M/C | Quick vertical descent at minimum 2 m/s through touchdown. | ArduPilot / airframe | H | Independently measured descent/touchdown, not parameter write or ACK; G7 |
| AE27-OPS-018 | 4.5 p13 | Both/termination trigger | M/C | Enter termination mode within five seconds of activation. | ArduPilot / safety hardware | H | Trigger-to-authoritative-mode timing under faults/load; G7 |
| AE27-OPS-019 | 4.5 p13 | Both/C2 loss | M/C | Never fly with crew unable to activate termination; losing C2 in AUTO must cause aircraft self-termination. | ArduPilot / safety hardware | H | Remove actual termination/C2 path with ground core unavailable; G7 |
| AE27-OPS-020 | 4.5 p13; 5.2.1 p19 | Both/breach | M/I | Automatic termination on flight-boundary crossing required; project owner resolves the trigger as the hard boundary (U-FEN-01). | ArduPilot / C++ safety | H | Verified hard polygon and all-mode hard-breach fault tests; internal soft inset preserved; G7 |
| AE27-OPS-021 | 4.5 p13 | Both/on-site testing | M/C | No rehearsal flights without judge authorization; test flights absent from event schedule. | Flight lead | H | Pre-arrival qualification and authorization checklist; G7/G8 |
| AE27-OPS-022 | 4.6 p13 | Both/attempt | P/C | May restart task any number of times within window, forfeiting points from previous attempt. | C++ mission / evidence | M | Attempt isolation, permission expiry and score reset replay; G5/G6 |
| AE27-OPS-023 | 4.6 p13 | Both/aircraft selection | M/C | One UAV per task; same or different aircraft/designs allowed; each must independently meet design/safety rules. | Systems lead | H | Aircraft/task roster and per-aircraft gate manifests; G7 |
| AE27-OPS-024 | 4.7(a) p13; 4.9(a) p15 | Both/maximum load | M/U | Design says maximum takeoff weight 15 kg including payload; FRR says under 15 kg with maximum task payload. | Airframe lead | H | Use strict under-15-kg project limit and calibrated margin; Q03/G7 |
| AE27-OPS-025 | 4.7(b,c) p13 | Both/design | M/C | No size restriction; electric propulsion only, including solar/batteries. | Airframe lead | H | Design/inspection checklist; G7 |
| AE27-OPS-026 | 4.7(d-f) p13-14 | Both/design | M/C | Termination system required; parachutes prohibited; provide loop attachment for spring-scale weighing. | Airframe lead | H | Physical inspection and weigh demonstration; G7 |
| AE27-OPS-027 | 4.7(g,j) p14 | Both/communications | M/C | Untethered data links may use radio/infrared/acoustic/other means; licensed RF bands require ISED licence supplied to judges. | Communications lead | H | Spectrum/licence/topology inspection; G7 |
| AE27-OPS-028 | 4.7(h) p14 | Both/task flight | P/C | Autonomous, semi-autonomous or manual operation allowed at bidder discretion. | C++ mission / operator | H | Declared autonomy and handover tests; G2/G6 |
| AE27-OPS-029 | 4.7(i) p14; 5.2.1 p19 | Both/weather | R/C | No weather pause within VMC; should operate in VMC including precipitation; expect possible 10 kt winds gusting 20 kt. | Airframe / flight lead | H | Measured operating envelope, go/no-go and weather rehearsal; G7 |
| AE27-OPS-030 | 4.7(k) p14 | Both/design | M/C | Complete off-the-shelf UAV prohibited; individual components allowed; airframe kits only without non-structural parts such as motors/wiring. | Airframe lead | M | BOM and design-origin inspection; G7 |
| AE27-OPS-031 | 4.7(l) p14 | Both/ground handling | M/C | Electrical/mechanical protection prevents accidental propeller spin until in takeoff position and ready. | Safety hardware / crew | H | Independent ground-inhibit inspection/fault test; G7 |
| AE27-OPS-032 | 4.8(a-g) p14 | Both/FRR documents | M/C | Per-model 922.04 safety declaration, per-aircraft registration, pilot/recency proof, applicable radio licence, general/aviation insurance, flight video and normal/emergency checklists. | Safety lead | H | Complete private FRR cross-index and judge approval; G7 |
| AE27-OPS-033 | 4.8 p14 | Both/proof flight | M/C | Video for each aircraft: takeoff, flyby/circle/hover if applicable, designed cruise, approach, full-stop landing. | Flight lead / evidence | H | Unambiguous per-aircraft recording and review; G7 |
| AE27-OPS-034 | 4.8 p15 | Both/insurance | M/U | Insurance details TBD and supplied separately; check institutional requirements. | Competition lead | H | Obtain and satisfy final insurance terms; Q08/G7 |
| AE27-OPS-035 | 4.9(b) p15 | Both/FRR demonstration | M/C | Props removed; demonstrate termination with motors running in every mode, datalink loss and programmed geofence breach, with temporary transmission permission. | Safety lead | H | Judge-witnessed props-off fault matrix; G7 |
| AE27-OPS-036 | 4.9(d) p15 | Both/inspection | M/C | Registration number physically present and compliant on every aircraft. | Airframe lead | M | Inspection record; G7 |
| AE27-OPS-037 | 5.2.1 p18-19 | Both/all flight | M/C | Altitude limit 100 m AGL; same site flight boundary for both tasks. | ArduPilot / C++ safety | H | Terrain/datum-aware altitude and hard-fence replay/readback; G7 |
| AE27-OPS-038 | Appendix C p31 | Both/fence loading | E/C | Prose calls C1 soft and C2 hard; only table C1 exists, titled hard, with six vertices. This is a source inconsistency, not a requirement for a second polygon. | Safety lead / fence loader | H | Verify hard polygon; project U-FEN-01 supplies internal configurable inset without changing plugin; G7 |

## Task 1 and competition exchange

| ID | Source | Task/phase | Kind/status | Faithful requirement summary | Owner | Safety | Verification and evidence |
|---|---|---|---|---|---|---|---|
| AE27-T1-001 | 5.2.3(1) p20 | T1/planning | E/C | By 2027-05-13 2359 receive enclosed search polygon, lap-course waypoints with AGL altitudes and required lap count. | C++ mission / operator | H | Validated input import and malformed/concave route tests; G5 |
| AE27-T1-002 | 5.2.3(5-6) p20-21 | T1/transit | S/C | Fly prescribed laps to survey; total course 3-5 km; time from takeoff to search-area boundary, as fast as possible. | C++ mission / ArduPilot | H | Independent full-course/lap/time/endurance witness; G5/G7 |
| AE27-T1-003 | 5.2.3(7) p21 | T1/survey | E/C | Plastic deer decoys, exact model variable, with/without antlers; some laminated ear tags with unique two-character alphanumeric code on both faces, font at least 85 pt (about 3 cm). | Perception / operator | M | Held-out decoy/tag imagery across lighting, range and occlusion; G5 |
| AE27-T1-004 | 5.2.3(8a) p21 | T1/report | M/C | Report total deer count. | Perception / C++ mission | L | Ground-truth count including duplicate views; G5 |
| AE27-T1-005 | 5.2.3(8b) p21 | T1/report | M/U | Report count in each 10 m-radius cluster; isolated deer are clusters of one; overlap/group construction unspecified. | Perception / C++ mission | M | Organizer-approved clustering oracle and boundary cases; Q05/G5 |
| AE27-T1-006 | 5.2.3(8c-d) p21 | T1/report | M/C | Report capitalized two-character tag codes and brief plain-language anomalous interactions with human artifacts. | Perception / operator | L | Tag/anomaly oracle with unknown/occluded review; G5 |
| AE27-T1-007 | 5.2.3(9) p21 | T1/submission | M/C | Clearly labelled plain-text report named `<your_team_name>_task1_survey.txt` in Phase 2 Deliverables Drive folder, no later than 15 minutes after window. | Ground evidence tools | L | Format/name/content validation and upload receipt; G5 |
| AE27-T1-008 | 5.2.3(10-11) p21-22 | T1/whole task | M/C | No battery swaps; one UAV. | Flight crew / mission | H | Full single-battery rehearsal with reserve and roster; G5/G7 |
| AE27-T1-009 | Table 5 p22 | T1/transit scoring | S/C | Completed-course speed score: fastest 30, slowest 15, other scores linearly by relative ranking; incomplete course excluded from scale. | Evidence tools | L | Ranked independent timing oracle; tie/one-finisher ruling Q09; G5 |
| AE27-T1-010 | Table 5 p22-23 | T1/survey scoring | S/C | Total count 5, clusters 10 divided among clusters, tags 20 divided among tags, anomalies 5; nominal total 40. | Evidence tools | L | Ground-truth rubric review; G5 |
| AE27-T1-011 | Table 5 p22-23 | T1/submission scoring | S/C | Survey multiplier 150% at/before five minutes before window end, 100% at 15 minutes after; linear between; zero later. Maximum survey score 60. | Ground evidence tools | L | Boundary-time oracle and real upload receipt; G5 |
| AE27-T1-012 | Table 5 p23 | T1/landing | S/C | Safely landed at flight line by window end with no UAS part left in field earns 5; Task 1 total 120. | ArduPilot / mission / crew | H | Authoritative landed/disarmed state and field-clearance witness; G5/G7 |
| AE27-NET-001 | 5.2.3(2-3a) p20 | T1/armed | M/C | Send competition telemetry whenever armed at 1 Hz; connection token and server documentation supplied through named competition portal. | Competition adapter | M | Official authenticated receipt over armed intervals; G4 |
| AE27-NET-002 | 5.2.3(3b i-iii) p20 | T1/telemetry | M/C | Include bidder-defined string UAV identifier, Unix timestamp and latitude/longitude in decimal degrees. | C++ telemetry / adapter | M | Independent schema, identity and clock oracle; G4 |
| AE27-NET-003 | 5.2.3(3b iv-v) p20 | T1/telemetry | M/C | Include altitude AGL and horizontal/vertical position accuracy, all in metres. | C++ telemetry / adapter | H | Terrain/datum and accuracy-source tests; no home-relative substitution; G4 |
| AE27-NET-004 | 5.2.3(3b vi) p20 | T1/telemetry | M/C | Include battery percentage 0-100. | C++ telemetry / adapter | M | Unknown/stale/range/scaling tests; G4 |
| AE27-NET-005 | 5.2.3(3b vii) p20 | T1/telemetry | M/C | Mode enum is off, idle, link-lost, failsafe, armed-pilot or armed-automatic. | C++ telemetry / adapter | H | Reviewed state-to-enum mapping including overlapping faults; Q04/G4 |
| AE27-NET-006 | 5.2.3(3b viii) p20 | T1/telemetry | M/C | RC and telemetry link status are bidder-defined floats 0-1, with 1 best quality. | Link adapter / telemetry | H | Document normalization and independently inject missing/degraded links; G4 |
| AE27-NET-007 | 5.2.3(4a) p20 | T1/in flight | M/C | React to 1 Hz server coordinates/altitudes of flying UAVs, including simulated UAVs; no concurrent real UAV mission required. | Competition adapter / core | H | Independent multi-track replay and official server run; G4 |
| AE27-NET-008 | 5.2.3(4b-d) p20 | T1/deconfliction | M/C | Avoid each other UAV's cylindrical radial/vertical exclusion zone; bidder need not publish own keepaway distances. | C++ safety / operator | H | Cylinder intrusion/boundary tests and timely operator response; Q04/G4 |
| AE27-NET-009 | Table 5 p22 | T1/scoring | S/C | At least 300 seconds armed telemetry earns 25 before deductions. | Competition adapter / evidence | M | Independent armed-duration and receipt evidence; G4 |
| AE27-NET-010 | Table 5 p22 | T1/scoring | S/C | Keepout intrusion costs 5 per occurrence; disconnect costs 5 at 30 seconds missing plus 1 per additional 15 seconds without duration cap. | Evidence tools | M | Timed outage/intrusion scoring oracle; Q04/G4 |
| AE27-NET-011 | Table 5 p22 | T1/scoring | S/C | Startup armed costs 2; invalid GPS armed costs 2 per occurrence, limited to one deduction per 15-second burst. | Competition adapter / evidence | M | Startup and GPS validity oracle; definition Q04; G4 |
| AE27-NET-012 | Table 5 p22 | T1/scoring | S/U | Latency text says >15-second timestamp delta between packets: 2 per occurrence, limited to one per 15-second window; actual timestamp/receipt interpretation unresolved. | Competition adapter / evidence | M | Clarified official clock/latency oracle; Q04/G4 |
| AE27-NET-013 | Table 5 p22 | T1/scoring | S/C | Slow rate >1.1 seconds or fast rate <0.4 seconds between packets costs 1 per packet, each rate-limited to one deduction per 15 seconds. | Competition adapter / evidence | M | Test threshold equality and both sides with independent receiver; G4 |
| AE27-NET-014 | Table 5 p22 | T1/scoring | S/C | Each penalty type limited to five occurrences; sixth onward no further penalties; criterion floor zero. | Evidence tools | L | Six-occurrence and long-disconnect oracle; Q04/G4 |

## Task 2

| ID | Source | Task/phase | Kind/status | Faithful requirement summary | Owner | Safety | Verification and evidence |
|---|---|---|---|---|---|---|---|
| AE27-T2-001 | 5.2.4(1) p23 | T2/planning | E/C | By 2027-05-13 2359 receive target search centre/radius 25 m and sample search centre/radius 15 m. | C++ mission | H | Coordinate/radius validation and reviewed task plan; G6 |
| AE27-T2-002 | 5.2.4(2) p23 | T2/identification | E/C | Target has brightly coloured bands on all four legs; only target is prepared for tracker. | Perception / operator | M | Distractor/occlusion/incorrect-target trials; G6 |
| AE27-T2-003 | 5.2.4(3a-c) p23 | T2/tracker design | M/C | Custom standalone untethered tracker, mass strictly less than 250 g, no axis exceeds 8 cm. | Tracker hardware | H | Calibrated complete-unit mass/dimension inspection; G6/G7 |
| AE27-T2-004 | 5.2.4(4a-c) p23 | T2/tagging | P/C | Attach using hook-and-loop on deer side (soft loop on animal) or drop in open box at least 30 by 30 cm within 2 m of animal. | Payload / C++ mission | H | Placement/attachment witness and failed-placement trials; G6 |
| AE27-T2-005 | 5.2.4(4d) p23 | T2/tagging | M/C | May carry multiple trackers but attach only one using allowed methods. | Payload / C++ mission | H | Duplicate-action/retry/reboot inhibition and physical count; G6 |
| AE27-T2-006 | 5.2.4(5) p24 | T2/after tagging | M/C | Move UAV at least 100 m horizontally away; volunteer then moves tracker; maintain 100 m offset for rest of window. | C++ mission / safety / operator | H | Independent moving-person trajectory and distance witness, including sampling/return; G6/G7 |
| AE27-T2-007 | 5.2.4(6); Appendix D p24,32 | T2/track export | M/C | CSV columns in order: ISO8601 timestamp, decimal-degree latitude, longitude; header required, any header text; chronological timestamps. | Tracker adapter / evidence tools | M | Independent CSV parser, time-order/unit and clock tests; G6 |
| AE27-T2-008 | 5.2.4(6a) p24 | T2/tracking | P/C | No mandated sample frequency or number; judge reconstructs path by linear interpolation. | Tracker adapter | M | Interpolated path versus independent ground truth including gaps; G6 |
| AE27-T2-009 | 5.2.4(6b-d) p24 | T2/tracking | E/C | Animal speed at most 5 m/s; track five-minute trajectory beginning when safe offset reached; motion stops after five minutes or window end, whichever first. | Tracker / mission | H | Timed independent path at speed bound and shortened window; G6 |
| AE27-T2-010 | 5.2.4(6e) p24 | T2/submission | M/C | Submit tracking CSV before window end. | Ground evidence tools | L | Upload receipt before cutoff; destination/name Q07; G6 |
| AE27-T2-011 | 5.2.4(7a-d) p24 | T2/collection | P/C | Multiple samples/attempts available on flat grass; landing for collection allowed; damage to uncollected source material unrestricted. | Payload / mission | H | Mechanism trials and reviewed landing/abort envelope; G6/G7 |
| AE27-T2-012 | Table 6 p25 | T2/egg | M/C | Large chicken egg surrogate: deliver one without cracking. | Payload hardware | M | Independent inspection after complete transport/deposit; G6/G7 |
| AE27-T2-013 | Table 6 p25 | T2/dung | M/C | Dough-like pile at least 10 cm wide/tall with pink outer 1 cm, green inside: return cylindrical core diameter 1-4 cm, any length, at least 75% green, cylinder preserved; clear container allowed. | Payload hardware | M | Measured core diameter/colour fraction and shape after delivery; fraction method Q07; G6/G7 |
| AE27-T2-014 | Table 6 p25 | T2/droppings | M/C | Hard blue spheres at most 3 cm diameter: return at least four individual spheres unsquished. | Payload hardware | M | Count and integrity inspection after delivery; G6/G7 |
| AE27-T2-015 | 5.2.4(8) p25 | T2/delivery | M/C | Return intact samples to blue 32-inch landing pad at flight line before window ends; unloading may be manual or automatic. | Payload / mission / operator | H | Pad/time/integrity witness; G6/G7 |
| AE27-T2-016 | 5.2.4(9) p25 | T2/whole task | M/C | Only one UAV. | Flight lead | H | Roster/configuration witness; G6/G7 |
| AE27-T2-017 | Table 7 p25 | T2/tag scoring | S/C | Hook-and-loop attachment 10; box placement zero attachment points; multiple/no trackers zero attachment points. | Evidence tools | L | Placement rubric and count witness; G6 |
| AE27-T2-018 | Table 7 p25 | T2/path scoring | S/C | Within 5 m of true path throughout five minutes earns 30; portions 5-15 m get half credit; worse than 15 m or outside submitted timestamp range zero for missing duration. | Tracker / evidence tools | M | Independent time-weighted interpolation error oracle; exact boundary Q07; G6 |
| AE27-T2-019 | Table 7 p25-26 | T2/sample scoring | S/C | One successful egg/dung/droppings return earns 20/15/10; further successes unnecessary; failed/damaged attempts incur no penalty. | Payload / evidence tools | L | Per-type success and no-duplicate-credit oracle; G6 |
| AE27-T2-020 | Table 7 p26 | T2/autonomy scoring | S/C | Autonomous takeoff 5 and landing 5; successful autonomous egg/dung/droppings collection earns 10/5/5. | C++ mission / ArduPilot | H | Continuous observer record of selected autonomous sequence; D05/G6 |
| AE27-T2-021 | Table 7 p26 | T2/autonomy scoring | S/C | For autonomous collection credit, all manipulation including pickup/extraction, flight to drop-off and depositing on pad without operator intervention; one successful demonstration per criterion suffices. | C++ mission / payload | H | Full-sequence intervention log plus physical success; approval timing Q06/G6 |
| AE27-T2-022 | Table 7 p26 | T2/landing | S/C | Safely landed at flight line by window end and no UAS part in field except one attached tracker earns 10; Task 2 total 125. | ArduPilot / mission / crew | H | Landed/disarmed and field-clearance witness; G6/G7 |

## Interpretation register and deliberate exclusions

| ID | Source | Task/phase | Kind/status | Faithful requirement summary | Owner | Safety | Verification and evidence |
|---|---|---|---|---|---|---|---|
| AE27-INT-001 | 5.2.3(10) p21; 5.2.4 p23-26 | T2/battery swap | P/U | Only Task 1 explicitly prohibits swaps; Task 2 swap permission is not stated. | Competition lead | H | Written ruling before including swap in scored strategy; Q03/G6 |
| AE27-INT-002 | 4.7(h) p14; 5.2.3(4) p20 | T1/traffic response | P/I | Manual deconfliction appears permitted, but advisory display alone cannot satisfy actual exclusion-zone avoidance. | Safety lead / operator | H | Organizer confirmation if needed and timely avoidance witness; Q04/G4 |
| AE27-INT-003 | 5.2.3(2-4) p20 | Both/server design | E/U | Token portal referenced; CONOPS does not specify wire protocol, endpoint paths, event schema, delivery/retry or outage policy; Task 2 server obligation not explicit. | Competition adapter | H | Versioned official contract and server acceptance; Q04/G4 |
| AE27-INT-004 | 5.2.3(8-9) p21 | T1/evidence | E/C | Required survey submission is text; no task imagery upload or bounding-box submission specified. Images remain project verification evidence. | Perception / evidence tools | L | Required text export plus separately retained internal imagery; G5 |

Appendices A/B are glossary/contact reference; F has no answered questions in
v1.0. Appendix E pp33-35 explicitly labels its advice non-binding, including
60-minute arrival and 10-minute-ready suggestions; these do not replace the
30-minute scored arrival criterion. Its project-management wording supports the
scored proposal plan, not a newly invented flight requirement. Table 9 is reused
for Phase 2 totals and Phase 3 scoring; citations include page numbers.

No CONOPS requirement selects MAVSDK, Jetson, ROS, LTE, Walksnail, a mission
camera, ZED, ESP32, Cube Orange, Here4, RTK or VIO. These remain project decisions
or deferred choices in PRD. Event-driven competition uploads from the preview
are not confirmed here. No prediction horizon, right-of-way algorithm, alert UI,
traffic freshness bound or mandated autonomous avoidance maneuver is specified.
Do not infer those from the cylindrical exclusion-zone obligation.
