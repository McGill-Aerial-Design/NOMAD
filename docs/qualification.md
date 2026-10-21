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

Current source CI tests the plugin it ships. Obsolete external C++/MAVSDK/SITL required
status contexts are not manufactured by replacement jobs. If repository protection
still requires them, merging remains blocked until separately authorized alignment;
changing this source does not alter protection.

`config/github-main-protection.json` proposes those two current job names while
preserving strict checks/reviews/admin controls. It is source data only: existing
GitHub protection still requires its old contexts until separately authorized application.
