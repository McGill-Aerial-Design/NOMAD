# Qualification status

Current evidence applies to the Mission Planner-native product and the exact reviewed
host target. Historical C++/MAVSDK/runtime/SITL evidence is preserved on
`MAVSDK-Archive`; it is not evidence that this implementation or a physical aircraft
passed qualification. Repository source describes checks, not a claim that hosted
runs completed on a particular commit. PR reports must record actual conclusions.

## Software evidence

[Development](development.md) lists executable gates. Host doubles exercise native
command arguments, profile/connection/stream/ID fences, invalid input, mode caching,
input generations, native helper failures, payload confirmation/neutral rules,
trim matching, Safe draining, shared repeat lifetime and uncertain outcomes.
Loopback router tests exercise link routing, deduplication, failover, parameter
pinning, disposal and replacement. WinForms tests cover isolated controls, profile
drafts/persistence failures, maps, router status and video lifetime.

The actual Release build references Mission Planner's reviewed assemblies; tests
never install the DLL or rewrite real operator profiles. Test doubles do not verify
host/controller implementation, aircraft output calibration or physical completion.
Mission Planner owns its normal command ACKs/retries; pending helpers can finish
past NOMAD Stop/unload. External MP/mission/RC actions are outside NOMAD's local
interlocks and may change output state or the controller's shared repeat slot.

The optional [native simulator check](development.md#manual-native-simulator-acceptance)
runs the actual installed plugin and host against pinned Copter and Plane SITL binaries.
Fresh PWM/relay telemetry verifies software output transitions for all five payload
behaviors, timed returns, Safe and gimbal positioning; native ACKs remain software
evidence. ReadOnly/stale generation rejection and normal host lifecycle are checked.
The same finite output checks pass with pinned Copter 4.8.0-dev and Plane 4.7.0-dev
simulators, using 10 Hz output/status streams. Positive and negative bidirectional
motion, native neutral return and explicit Safe are observed on both. Gimbal native
submission is checked separately from PWM response. This does not cover continuous
link-loss timing, reconnect, physical HID/video/map/audio, QuadPlane output behavior,
or real mechanisms. Existing focused fault/HID gates
remain necessary; simulator PWM is not measured electrical PWM or physical completion.

## Aircraft acceptance remains open

- Copter, Plane and QuadPlane native flight behavior across the chosen firmware,
  controllers, frames and every intended flight phase.
- Actual payload mechanisms, trim/calibration, feedback, link/power faults and
  physical safe state; accepted software commands do not prove physical outcomes.
- ELRS/LTE takeover, RC handback, loss responses, independent termination and
  all-mode containment. Native ArduPilot failsafes must remain enabled and reviewed.
- Official competition server contract, traffic separation, perception, task
  exports and complete Task 1/2 rehearsals from the [source inventory](conops-requirements.md).
- Installation/operator acceptance using a reviewed DLL and private deployment data.

Current source CI tests the plugin it ships. Obsolete external C++/MAVSDK/SITL status
contexts are not manufactured by replacement jobs. Repository protection is separate
external state: required names must match the actual product jobs before merge.
Source edits alone do not apply or establish that protection state.

`config/github-main-protection.json` proposes the two current job names while
preserving strict checks/reviews/admin controls. Applying that template requires
explicit authorization and live readback; record the actual alignment result with
the PR rather than treating this document as a live protection snapshot.
