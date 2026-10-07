# nomad_ros

Read-only ROS 2 telemetry adapter. It publishes validated GPS and battery
observations received from MAVLink. The shipped node does not construct
`nomad::vehicle::Vehicle`, expose vehicle command services, or send MAVLink.

Runtime IPC protocol v1 reports telemetry validity and sample ages, but does not
return the measurements needed for ROS sensor messages. Until runtime status
exposes those source values with per-field timestamps, this adapter uses a
separate raw UDP receive socket with the generated ArduPilot MAVLink parser.
`nomad_mavlink_observation` exposes no send operation and the node does not link
the command-capable transport or `nomad_core`.

## Build

Build the package from the repository root in a ROS 2 Humble workspace:

```bash
colcon build --packages-select nomad_ros
```

The repository's `nomad-sim-ros` Docker image contains the toolchain for this
build and its integration suite.

## Run

```bash
ros2 launch nomad_ros nomad_vehicle.launch.py
```

`config/params.yaml` sets `observation_endpoint` to the MAVLink telemetry feed,
`expected_system_id` to select the autopilot, and `publish_rate_hz` for ROS
publication. These parameters configure telemetry reception only.

## Topics

Publishers:

- `/nomad/fix` — `sensor_msgs/NavSatFix`; WGS-84 latitude/longitude and MSL
  altitude. Published only with valid 3D GPS, valid coordinates, and fresh
  position and GPS samples.
- `/nomad/battery` — `sensor_msgs/BatteryState`; published only with a valid
  voltage and percentage from a fresh battery sample.
- `/nomad/connected` — `std_msgs/Bool`; fresh expected autopilot heartbeat.

Position and GPS source timestamps must advance, and values pass range and fix
checks. Battery validity requires finite, in-range voltage and percentage;
`SYS_STATUS` has no source timestamp, so freshness uses the incoming MAVLink
sequence and local receipt age. Position, GPS, and battery updates older than
1500 ms are omitted. Sensor message stamps approximate the oldest local receipt
time used to create each message. Missing, malformed, replayed, or stale source
data is not republished as healthy current data.

`/nomad/odom` is not published. The read-only telemetry contract used by this
adapter does not provide velocity or attitude, and the prior odometry message
mixed NED/FLU frame conventions. Re-enable odometry only after its source fields
and frame conversion have a tested contract.

## Commands and VIO

`/nomad/cmd_vel` and the `/nomad/arm`, `/nomad/disarm`, `/nomad/land`, and
`/nomad/rtl` services are not advertised. Protocol v1 has no typed requests for
these operations, so the ROS adapter reports them as unavailable by omission;
there is no direct fallback. The runtime's existing servo, relay, motor-test,
and gimbal requests do not correspond to these ROS APIs and are not exposed
here.

The node does not subscribe to VIO health, confidence, or source topics. Those
inputs previously gated only the removed direct velocity path. Protocol v1 has
no VIO observation-submission request, so this adapter does not synthesize or
submit VIO state.

## Transitional telemetry link

The separate MAVLink connection is a temporary observation-only bridge around
the runtime status contract's missing sensor values. It accepts only `udpin`
endpoints and its UDP socket only receives datagrams; it does not instantiate a
MAVSDK system, subscribe to command-capable plugins, or contain a send path. The
ROS binary links `nomad_mavlink_observation`, not the command-capable MAVLink
transport or `nomad_core`. The ROS integration suite checks that the observer
transmits no MAVLink frames and that former control inputs produce no vehicle
commands or velocity setpoints.

debt: one receive-only MAVLink telemetry connection per ROS adapter; revisit
when runtime status exposes source-stamped position, GPS, and battery values;
then publish those runtime-owned values and remove the ROS observation link,
endpoint parameters, router leg, and generated MAVLink header dependency.
