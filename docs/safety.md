# Safety case

NOMAD's software tests establish bounded custom-control behavior, not aircraft safety.
Mission Planner owns ordinary flight/mission commands and native ACK/retry handling;
ArduPilot owns stabilization, navigation execution and native failsafes. Native MP,
RC and mission commands can act outside NOMAD's local interlocks. Aircraft-specific
qualification must cover those interactions.

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

The published AEAC 2027 C1 coordinates are sourced from CONOPS v1.0, 2026-09-08,
Appendix C p31. NOMAD supplies no invented second polygon or boundary response. The
100 m AGL competition ceiling still needs approved enforcement and datum evidence.
NOMAD leaves normal LAND and QLAND settings unchanged. Competition termination is
a separate onboard mechanism for the aircraft and flight phase, not a global
landing-rate change. The optional [native extension](native-termination.md) owns
the separate descent/aerodynamic policy, RC/fence triggers and reboot latch onboard.
NOMAD only checks the advertised capability and requests its native mode; it never
substitutes stock LAND, in-air disarm or a parameter recipe. ELRS/LTE takeover,
transmitter mixing, actual loss responses and physical termination remain aircraft
acceptance work under D09/Q02. Total estimator/controller failure is not guaranteed
controlled descent. Preserve native ArduPilot failsafes and qualify interactions.

## Software safety checks

| Boundary | Current validation |
|---|---|
| Selected aircraft and profile | Connection, stream, system/component and generation checks reject stale or unwritable targets |
| Custom input | Finite values, calibrated limits, neutral/confirmation edges and input-loss Safe |
| Timed payload actions | Live trim matching, bounded native repeats, shared-slot ownership and uncertain-outcome handling |
| Video and links | Bounded frames/queues, disposal, one command egress, deduplication and parameter pinning |
| Native fence draft | Complete polygon validation, selected Fence context, cache preservation and zero automatic uploads |
| Native termination | Advertised support, fresh parameter/heartbeat checks and separate ACK/mode observation; onboard entry, RC/fence, phase and latch tests |

The executable checks are listed in [development](development.md). Physical wiring,
surface/motor direction, actuator feedback, receiver failures and actual flight
behavior require the separate [aircraft acceptance](qualification.md#aircraft-acceptance-remains-open).

## Required evidence

Run the focused [software checks](development.md), review native host API arguments
and collect independent wire/controller/output evidence for the actual aircraft.
Exercise input loss, stale/wrong profiles, conflicting outputs, pending helpers,
unknown ACKs, link/power faults and MP exit. State passed/failed/skipped/unavailable
separately. Do not claim a landing, safe mechanical state, traffic avoidance or
termination from a software ACK. Remaining acceptance is in [qualification](qualification.md).
