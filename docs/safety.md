# Safety and qualification

Mission Planner owns ordinary flight/mission commands and native ACK/retry behavior.
ArduPilot owns stabilization, navigation and native failsafes. NOMAD's interlocks
apply to its own custom controls; MP, RC and mission commands can act outside them.
Software checks do not establish aircraft safety or physical completion.

## Custom-control contracts

Custom commands capture connection, stream, system/component and profile generation.
Closed, read-only, playback and retired targets refuse unsent input. Reconnect,
profile changes and input loss require fresh neutral/center and release edges.
Validate finite values, channel ranges and actual calibration for every aircraft.

Payloads support servo toggle/position/bidirectional and relay toggle/pulse.
Hazardous actions require configured distinct confirmations and neutral-separated
input edges. Conflicting inputs/loss request Safe for their original targets.
Timed servo actions require live `SERVOx_TRIM` to match the configured return value.
Native repeat period is twice the active interval; repeats share one controller
slot. NOMAD retains the reservation across retirement and uncertain outcomes.
External MP/mission actions can change that slot or output state.

Mission Planner owns retries. An already-dispatched helper may finish after Stop,
profile change or unload. Stop waits for pending payload work before attempting
Safe on the same current target. Unknown outcomes block activation replay until
explicit accepted Safe. Before disconnect/close, Safe/Stop and observe the result.
Unload attempts best-effort Safe; it cannot guarantee delivery or de-energization.
Onboard repeats retain native return behavior. An ACK is software acceptance.

Video tracking requires explicit activation, verified imagery, calibrated limits,
fresh selected-aircraft telemetry and one active gimbal owner. Searching/Outside
never authorize steering. Stop, manual input and stale imagery require explicit
rearming. Receipt time is not exposure time; upstream frozen pixels can look fresh.

## Native fence setup and termination

Fence staging validates one inclusion polygon and selects the native Fence editor.
Mission/Fence/Rally drafts remain separate during the plugin session. Staging
neither uploads nor enables a fence; native Write/Read remains explicit.
Save needed drafts before unload. Aircraft containment requires actual configuration.

Parameter Apply requires captured target identity, fresh disarmed state, confirmation
and independent readback. Writes are sequential: failure/uncertainty stops the
remainder, retains earlier writes and requires another Read. Closing the page
cannot cancel an already-dispatched native helper. Readback is configuration evidence.

The optional [native termination](native-termination.md) extension owns its separate
descent/aerodynamic policy and latch onboard. The plugin discovers advertised
support before enabling its guarded request/configuration. The supported subgroup
writes controlled action 44, bounded descent rate and supported AFS RC/fence settings;
it does not select an RC channel or alter normal LAND/QLAND rates. Aircraft
commissioning, transmitter independence, loss response and Q02 acceptance remain
required. Total estimator/controller failure cannot guarantee controlled descent.

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


## Software evidence

[Development checks](development.md) cover native command arguments, target/input
retirement, profile validation/persistence, failure handling, UI lifetime, routing,
video ownership and vision. Host doubles test NOMAD logic; loopback peers test
native MP API interactions. Optional owned SITL/rendered checks exercise a controlled
native-API cohort; installed-host checks exercise a separate application lifecycle.
Firmware gates separately test onboard termination policy and package identity.

These are available checks, not an evergreen record of passing releases. Each
report must bind exact source/local changes, binary hashes, reviewed host and actual
fixtures/configuration. State passed, failed, skipped and unavailable separately.
Keep raw/private artifacts and run histories outside published docs, under ignored
local evidence. Do not infer an aircraft output, touchdown or safe mechanism from
a command ACK, simulator PWM, synthetic image or UI screenshot.

## Aircraft acceptance remains open

- Qualify actual Copter/Plane/QuadPlane firmware, controllers, frames and flight phases.
- Measure gimbal/output direction, mechanical travel, trim, payload feedback and
  physical Safe under power/link faults; software target angles are not measurements.
- Measure camera latency, capture freezes and real-scene detection/tracking accuracy.
  Synthetic centering and model confidence do not qualify selected-deer tracking.
- Demonstrate RC/LTE takeover and handback, independent termination, all-mode
  containment, loss responses and failsafe interactions on the chosen aircraft.
- Verify mass/endurance, official server/traffic/task exports, full Task 1/2
  rehearsals, installation/operator acceptance and organizer approval.

Source obligations remain in [CONOPS traceability](conops-requirements.md), with
unresolved decisions in [product scope](prd.md). Required PR checks and reviews
must be verified before merge. Release, installation and physical tests are
separate authorized operations.
