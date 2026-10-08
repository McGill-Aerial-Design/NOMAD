# Remaining work

- Prove aircraft-wide source arbitration, physical pilot takeover and explicit
  handback with the reviewed RC/ELRS channel map and controller input priority.
- Select and qualify total C2-loss and independent termination behavior for Copter,
  fixed-wing and transitions, including latching/reset, power and external writers.
- Expose approved navigation/QuadPlane/boundary operations through typed runtime IPC
  only after their admission, audit, fault and recovery contracts are ready.
- Qualify real hardware/firmware/sensors, physical actuator feedback and release,
  actual deployment hosts, overload deadlines and privileged service registration.
- Implement AEAC telemetry/traffic from the official wire contract; resolve Q02–Q09
  in the [PRD](docs/prd.md) and assign accountable evidence/acceptance owners.
- Implement and qualify wildlife observation/export, Task 2 tracking/sample mechanics
  and complete Task 1/2 flights. Select compute/vision providers when needed.
- Complete system security, trusted release distribution and competition rehearsals
  with restricted evidence records and explicit safety acceptance.

The [qualification status](docs/qualification.md) records limits. Completed work
and historical implementation narratives remain recoverable from Git history.
