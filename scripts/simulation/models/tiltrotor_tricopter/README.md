# Tiltrotor tricopter fixture

This Gazebo model uses primitive visuals and collisions for a nominal 1.8 kg
ArduPlane vectored-yaw tilt-tricopter. Mass, inertia and surface areas follow the
supplied engineering model; positions, aerodynamic coefficients, blade geometry
and servo linkage remain approximate. It is an engineering fixture, not a measured
digital twin or an aircraft tune. No CAD mesh is required or included.

## Running

Prepare the existing ArduPilot and Gazebo builds using the
[simulation guide](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/simulation/README.md).
Select `scenarios/tiltrotor.json` for calm air or `tiltrotor-wind.json` for steady
4 m/s wind from the west. Both use the recorded ArduPlane executable and
`scenarios/tiltrotor.parm`.

The example scenarios select the NVIDIA WSLg adapter. On other hardware, change or
remove `wsl_graphics_adapter` in a local scenario copy, then restart the simulation.
Check the run's `gui-home/.gz/rendering/ogre2.log` for the actual renderer.

## Model and controls

- Body axes are forward/left/up. The IMU sensor is rolled 180 degrees into
  ArduPilot's forward/right/down axes; keep its link in the model frame because
  the native plugin transforms the pose separately from the sensor vectors.
- The front shafts point up at zero tilt and forward at +90 degrees about body
  +Y. The rear rotor stays upright. `Q_FRAME_CLASS=7`, `Q_TILT_TYPE=2` and
  `Q_TILT_MASK=3` select the vectored-yaw layout. Hover neutral is 15/105 of the
  configured -15..+90 degree servo travel.
- Positive aileron output lowers the left trailing edge and raises the right for
  right roll. Positive elevator output raises its trailing edge for nose-up pitch.
  The two aileron joint axes are mirrored.
- ArduPilot's force PIDs write joint forces directly. Keep `JointStatePublisher`:
  it supplies the position/velocity feedback even without a GUI or test observer.
  Its 30 Hz publication cap does not reduce the 1000 Hz physics feedback rate.
- Surface PIDs use `P=20`, `D=0.08` and passive hinge damping. Excessive derivative
  gain can cause numerical chatter on the low-inertia hinges; check actual joint
  tracking before changing flight-controller gains.

For simulator endpoint checks, QSTABILIZE commands upright tilt and disarmed
MANUAL commands forward tilt. An FBWA transition also depends on ArduPlane's
transition state and airspeed. These checks do not establish flight qualification.

## Wind and waypoint limits

Gazebo LiftDrag surfaces read the world's steady ENU wind. ArduPlane's `SIM_JSON`
link ignores `SIM_WIND_*`, so its synthetic sensors and wind estimate do not match
that world wind; do not tune wind estimation against this fixture.

`Q_GUIDED_MODE=1` hovers at GUIDED destinations, but ordinary GUIDED legs can still
use fixed-wing travel. An all-VTOL AUTO route needs `NAV_VTOL_TAKEOFF`,
`DO_VTOL_TRANSITION` state 3, waypoint legs and `NAV_VTOL_LAND`; state 4 explicitly
returns to fixed-wing travel.

The fixture uses `Q_A_ANG_YAW_P=2` and `Q_A_RAT_YAW_P=0.5` for slower heading demand
and stronger rate damping. `Q_TRANS_DECEL=0.7`, `Q_BACKTRANS_MS=10000` and
`WP_LOITER_RAD=180` start braking farther out and spread the nose-up pitch ramp
over ten seconds. The loiter radius also affects fixed-wing flight. Height gain
can remain during back-transition; these settings have limited simulator evidence.
