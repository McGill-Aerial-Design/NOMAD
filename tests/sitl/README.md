# SITL scenario suite

These tests drive isolated ArduPilot SITL with the non-installed
`nomad-qualification` executable and observe authoritative vehicle state. The
installed `nomad` CLI uses runtime IPC only; commands without a typed v1 request
report unavailable.
Normal pytest skips live scenarios without an explicitly configured simulation.
Nightly/on-demand SITL CI is configured, but a workflow file is not passed-run
evidence; merge requests must link a successful current-head live run.

## Local test responsibilities

- velocity_loop_closure.py: qualification CLI velocity command, observed motion/stop and
  mode-gate checks; Python is a test driver/observer.
- test_velocity_loop_closure.py: environment-gated pytest wrapper.
- scripts/dev/core_sitl_containment.py: in-fence movement, rejected out-of-fence
  target, authoritative position and cleanup (sitl-fence).
- scripts/dev/core_sitl_zero_delivery.py: independent wire zero plus observed
  vehicle stop; total physical link loss is a separate test.
- Other core_sitl_* runners cover status, command flow, mission, fence
  upload/readback, payload, link recovery and heartbeat relay behavior.
- scripts/dev/core_sitl_quadplane_observe.py checks the separately pinned
  ArduPlane 4.7.1 tilt-tricopter profile: real fixed-wing heartbeat plus the
  pinned `Q_ENABLE=2` classification, fresh position/GPS/attitude, and reported
  GUIDED/QLOITER/QRTL/RTL modes. It does not exercise flight primitives.
- scripts/dev/core_sitl_quadplane_vtol_takeoff.py drives the qualified NOMAD
  GUIDED arm + direct `MAV_CMD_NAV_TAKEOFF` path and verifies armed state,
  GUIDED mode and a fresh-position climb to the requested delta from the
  observed baseline, within a fixed 0.5 m completion margin.
- scripts/dev/core_sitl_quadplane_transition.py starts a fresh pinned profile,
  uses the independent pymavlink operator/test driver to establish `AUTO` with
  authoritative `EXTENDED_SYS_STATE` multicopter state,
  issues `MAV_CMD_DO_VTOL_TRANSITION` with `MAV_VTOL_STATE_FW`, and requires a
  newer authoritative `fixed_wing` state. Its forward waypoint supplies the
  tilt-tri airspeed condition; it is setup for this primitive, not route
  qualification. NOMAD deliberately rejects arbitrary QuadPlane `set_mode`, so
  this does not qualify a complete autonomous GUIDED -> AUTO -> transition
  sequence.
- scripts/dev/core_sitl_quadplane_route.py repeats the NOMAD VTOL takeoff and
  qualified forward transition on a fresh pinned profile, then sends a small
  two-point fixed-wing route through `MAV_CMD_DO_REPOSITION` in GUIDED. AUTO is
  set by the independent pymavlink test operator as transition setup. NOMAD
  waits for a new fresh position within 45 m and 5 m altitude of each target,
  at least 10 m closer than the captured ACK-boundary position; the independent UDP observer
  excludes samples before the route's armed GUIDED heartbeat, then records
  ordered waypoint proximity before reporting a pass. This does not
  qualify arbitrary modes, general missions, return/recovery, VTOL-back,
  landing, QuadPlane link-loss response or complete Task 1 execution. The first
  complete hosted run passed at implementation head `7f6206cbad51aade79ae86b20983d4e1fb818901`
  in [workflow run 35818612311](https://github.com/YoussGm3o8/NOMAD/actions/runs/35818612311);
  the PR description records the documentation-inclusive final-head rerun.

The obsolete sitl-gimbal task was removed with runtime wiring repair. No successful gimbal evidence is claimed.

Use the commands and safety discipline in
[development](../../docs/development.md) and
[operations](../../docs/operations.md). Run scenarios serially against a known
disarmed isolated simulator; never reuse a hardware endpoint for fault injection.
A configured passive observer link can feed Mission Planner without issuing
commands.

Scenarios share one vehicle, and two of them leave state that changes what a
later one can do. `core_sitl_geofence.py` uploads a polygon fence that stays
active even after it restores `FENCE_ENABLE`, and an active polygon makes
ArduPilot refuse guided targets outside it (observed: the same reposition
accepted with `FENCE_ENABLE=0` and rejected with the polygon loaded), so the
workflow runs guided-flight scenarios before the fence upload/readback step.
`velocity_loop_closure.py` waits for the RTL landing and authoritative disarm
before it returns, so a following scenario that requires a disarmed vehicle has
a deterministic handoff.

One unexplained failure is recorded here rather than explained away: on
2026-09-12 `core-sitl-link-recovery` failed on a fresh stack immediately after
`core-sitl-link-loss`, timing out on `wait_for_status({"armed": "true"})` while
its own `arm` command had exited zero, so the CLI believed it had verified the
armed state and the following 15 s of status reads did not agree. Re-running the
same pair, and the same four-scenario order, passed twice afterwards, so the
cause is not identified: treat a repeat as a real signal and capture the arm
step's full output before retrying.

Flight scenarios remain Copter-oriented except for the separately pinned
QuadPlane chain through return/recovery, VTOL-back and QLAND landing, plus the
disarmed receiver-fault delivery probe. Those slices do not qualify link loss,
authority, manual takeover, handback, termination or complete Task 1 flight. Required gate artifacts and historical/current distinctions live in
[migration](../../docs/migration.md); do not duplicate pass counts here.
