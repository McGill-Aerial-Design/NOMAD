# Safety case

NOMAD's software tests establish bounded custom-control behavior, not aircraft safety.
Mission Planner owns ordinary flight/mission commands and native ACK/retry handling;
ArduPilot owns stabilization, navigation execution and native failsafes. Native MP,
RC and mission commands can act outside NOMAD's local interlocks. No aircraft-wide
command authority, durable backend audit or independent termination guarantee ships.

## Competition safety obligations

The source obligations remain binding acceptance work, not implementation claims.

| Source IDs | Required safety argument | Falsification evidence / release blocker |
|---|---|---|
| AE27-OPS-015 through AE27-OPS-020/035 | Aircraft termination is available in every mode independently of ground computer; failure of the termination/C2 path causes self-termination | Remove path/power/ground computer under load in Copter, fixed-wing and transition states; observe actual state/output, five-second activation entry and approved rapid self-termination; G7 |
| AE27-OPS-016/017 | Fixed-wing motor-off/full surfaces differs from rotary vertical descent of at least 2 m/s to touchdown | Independently observe surface outputs and measured descent/touchdown; LAND dispatch or configured speed is not proof; Q02/G7 |
| AE27-OPS-005/006/020/037/038 | All-mode containment includes non-convex boundary and 100 m AGL | Approved aircraft boundary and altitude enforcement, validated datum/terrain inputs, actual breach and loss-of-navigation tests; a staged native Fence draft does not establish an uploaded/enabled aircraft fence or satisfy this gate; G7 |
| AE27-NET-007/008 | Stay outside supplied traffic cylinders; stale traffic is unknown | Inject delayed/malformed tracks and prove operator response avoids intrusion; settle extent/datum/freshness Q04 before flight; G4/G7 |
| AE27-T2-005/006 | Exactly one tracker attachment and at least 100 m horizontal offset after withdrawal through rest of window | Wrong/duplicate tracker and target moving toward sampling/return path; prove detection and approved intervention before encroachment; G6/G7 |
| AE27-OPS-024/031 | Under-15-kg project margin and physical ground propeller inhibit | Independent weighing and props-safe inhibit fault tests; G7 |

## Current custom-control boundary

Custom commands capture connection/stream/aircraft IDs and profile generation.
Unsent stale input is refused; closed/read-only/playback contexts are unavailable.
HID invalid samples require a real center before motion; held input cannot replay
activation across profile/connection changes. Saved labels/channel roles do not prove
actual wiring. Review calibration for each physical aircraft.

The five payload behaviors are servo toggle/position/bidirectional and relay
toggle/pulse. Typed definitions validate channels, finite normalized values, PWM
bounds and configured Safe/neutral values. Hazardous actions require configured
distinct confirmations within a bounded window; HID requires real neutral-separated
edges. Input loss/conflicting pairs admit Safe for every original target. Native
host helpers serialize through the payload owner; a later sample must not discard
an already-admitted Safe for another output.

Timed servo commands read live captured-target `SERVOx_TRIM` and require it to match
the configured return value. Timed actions use ArduPilot's native repeat commands:
period seconds equal twice the active interval. The controller has one shared
repeat slot; NOMAD retains its reservation across retirement and uncertain outcomes.
External MP/mission commands can alter that slot, output values or observed state.
This is not a hardware interlock or an aircraft-wide output arbiter.

MP uses normal ACK waits and retries. An already-dispatched helper is not cancellable
by NOMAD; Stop waits for it, then attempts Safe on the same still-current target.
Unknown outcomes inhibit activation replay until explicit accepted Safe. ACK success
means software acceptance, not physical return, de-energization or completion.
Before disconnect/close, explicitly Safe/Stop and observe the result. Synchronous
unload only attempts best-effort Safe; helpers already dispatched can finish and
native onboard repeats retain controller return behavior. No post-unload NOMAD
command owner or delivery guarantee is claimed.

## Native fence setup, takeover and termination

Fence setup stages a validated inclusion polygon in native Mission Planner Plan,
only when Fence mode is already selected and the operator confirms replacing that
editor draft. It preserves aircraft caches and does not upload, enable or read back
a fence. Native Write and Read are explicit operator actions. Native failsafe
configuration remains in Mission Planner; no parameter writes occur from this page.
Legacy local advisory files are preserved on disk and are no longer loaded.

The published AEAC 2027 C1 coordinates are sourced from CONOPS v1.0, 2026-09-08,
Appendix C p31. NOMAD supplies no invented second polygon or boundary response. The
100 m AGL competition ceiling still needs approved enforcement and datum evidence.
NOMAD leaves normal LAND and QLAND settings unchanged. Competition termination is
a separate onboard mechanism for the aircraft and flight phase, not a global
landing-rate change. NOMAD termination is unavailable: LAND, in-air disarm or a
parameter recipe is not an approved replacement. ELRS/LTE takeover, RC handback,
independent transmitter termination and all-mode/loss-of-link response remain
aircraft-specific acceptance work under D09/Q02. Preserve native ArduPilot failsafes
and qualify interactions.

## Stable safety requirements

These retain their original obligations; partial coverage is not satisfaction.

| ID | Requirement | Current evidence / open scope |
|---|---|---|
| SR-VEL-01 | Clamp XY velocity to reviewed limits | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VEL-02 | Clamp vertical and yaw-rate velocity to reviewed limits | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VEL-03 | Reject the complete command for any non-finite component | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VEL-04 | Convert input and MAVLink frames explicitly and correctly | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VEL-05 | Guided velocity requires armed state and GUIDED mode | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VEL-06 | Filter heartbeat to the commanded vehicle | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-VIO-01 | Reject unhealthy, low-confidence, stale or unexpected-source VIO | No VIO/velocity input path ships; retain as an acceptance constraint for any later integration. |
| SR-VIO-02 | Stale VIO stops active velocity within watchdog interval | No VIO/velocity input path ships; retain as an acceptance constraint for any later integration. |
| SR-LNK-01 | Commands require fresh FC heartbeat | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-LNK-02 | Missing velocity input triggers a zero command within timeout | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-LNK-03 | Shutdown sends zero before closing an active link | Former C++ policy evidence archived. MP/ArduPilot own normal flight behavior; new product does not claim these removed velocity/watchdog APIs. |
| SR-LNK-04 | Announce a standard GCS heartbeat for heartbeat-gated relays | Mission Planner owns native GCS heartbeat behavior; qualify the installed host, physical links and aircraft failsafes. |
| SR-FEN-01 | Upload, enable and verify FC fence before autonomous flight | Native MP/ArduPilot flight responsibility; no NOMAD physical completion/containment guarantee. Aircraft-specific acceptance remains open. |
| SR-FEN-02 | Reject position targets outside configured boundary | Native MP/ArduPilot flight responsibility; no NOMAD physical completion/containment guarantee. Aircraft-specific acceptance remains open. |
| SR-PAY-01 | Validate servo channel and PWM before actuation | Native payload/input fault tests; MP ACK/retries, trim matching and shared-repeat limits. Physical outputs/calibration remain unqualified. |
| SR-PAY-02 | Bound payload duration and de-energize outputs on failure | Native payload/input fault tests; MP ACK/retries, trim matching and shared-repeat limits. Physical outputs/calibration remain unqualified. |
| SR-PAY-03 | Authorize hazardous custom outputs through local confirmation/neutral interlocks and captured native MP targets | Native payload/input fault tests; MP ACK/retries, trim matching and shared-repeat limits. Physical outputs/calibration remain unqualified. |
| SR-SEC-01 | No NOMAD command disables FC failsafes | No custom flight/failsafe-disable command path; controller setup and external MP/mission actions still need aircraft review. |
| SR-SEC-02 | Retired backend authentication/audit obligation; reassess any future remote trust boundary | No IPC client/service ships. MP and the local operator session own access; NOMAD does not claim HMAC authentication or durable command auditing. |
| SR-SEC-03 | Retired backend authentication/audit obligation; reassess any future remote trust boundary | No IPC client/service ships. MP and the local operator session own access; NOMAD does not claim HMAC authentication or durable command auditing. |
| SR-TYP-02 | QuadPlane forward transition and VTOL takeoff stay bound to the admitted aircraft identity and session; transition completion remains armed/AUTO and takeoff requires a fresh post-ACK climb sample | Native MP/ArduPilot flight responsibility; no NOMAD physical completion/containment guarantee. Aircraft-specific acceptance remains open. |
| SR-LND-01 | Pinned QuadPlane landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope; ACK alone is never touchdown | Native MP/ArduPilot flight responsibility; no NOMAD physical completion/containment guarantee. Aircraft-specific acceptance remains open. |



## Additional hazards and proposed obligations

IDs below retain their acceptance meaning after backend retirement. Their old
implementation/qualification traces are archived; no current physical acceptance
is inferred from them.

| Hazard | Proposed requirement | Mitigation / objective falsification test | Gate |
|---|---|---|---|
| H-09 Conflicting writers | SR-AUT-01: one active owner and explicit handover | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G2 |
| H-10 Stale position with fresh heartbeat | SR-TEL-01: per-field age and clock validity | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G2 |
| H-11 Collision or missed traffic | SR-AIR-01: unknown/stale traffic never means clear | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G4 |
| H-12 Wrong aircraft mode/transition | SR-TYP-01: validate aircraft class and state | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G2/G7 |
| H-13 False identity or geolocation | SR-OBS-01: decisions retain evidence and uncertainty | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G5/G6 |
| H-14 Tracker confusion / duplicate payload | SR-TSK-01: bind task/target/action identity and expiry | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G6 |
| H-15 Payload jam/contact/power loss | SR-PAY-04: safe physical state and verified outcome | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G6/G7 |
| H-16 Mass, energy or navigation deficit | SR-OPS-01: qualify aircraft configuration and reserve | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G7 |
| H-17 Network/compute overload | SR-RES-01: bounded queues and safety execution | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G3/G8 |
| H-18 Untrusted messages/replay | SR-SEC-04: authenticate, authorize and reject replay | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G2/G8 |
| H-19 Unavailable or wrong actuation from a bad command identifier | SR-CMD-01: every actuation command carries an identifier the pinned dialect defines | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G2 |
| H-20 Incorrect host/controller semantics | SR-CMD-02: verify reviewed MP APIs and aircraft behavior | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G-M |
| H-21 Route ACK or stale position is mistaken for QuadPlane route completion | SR-MIS-01: fixed-wing route success requires fresh post-ACK aircraft progress and state for every waypoint | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G-M |
| H-22 Recovery tolerance is mistaken for transition readiness, or ACK/stale multicopter telemetry is mistaken for completed transition | SR-TYP-01: transition-to-VTOL uses the explicit recovery coordinates and altitude, measured pre-command altitude inside 15–25 m above home, a stabilized 55 m envelope and authoritative post-ACK state | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G-M/G7 |
| H-23 QLAND ACK or stale/contradictory state is mistaken for QuadPlane touchdown | SR-LND-01: landing success requires fresh post-ACK descent, landed-state telemetry, disarm and a stable final envelope | Unqualified acceptance objective; use native host/controller observation and physical fault evidence before claiming safety. | G-M/G7 |

## Required evidence

Run the focused [software checks](development.md), review native host API arguments
and collect independent wire/controller/output evidence for the actual aircraft.
Exercise input loss, stale/wrong profiles, conflicting outputs, pending helpers,
unknown ACKs, link/power faults and MP exit. State passed/failed/skipped/unavailable
separately. Do not claim a landing, safe mechanical state, traffic avoidance or
termination from a software ACK. Remaining acceptance is in [qualification](qualification.md).
