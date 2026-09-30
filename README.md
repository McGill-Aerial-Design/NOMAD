# NOMAD

NOMAD is a C++20 vehicle-control core with CLI, Mission Planner and ROS 2 clients,
prepared for the AEAC SUAS 2027 Wildlife Monitoring CONOPS v1.0. ArduPilot owns
stabilization, EKF, navigation execution and failsafes.

## Current status

The working migration tree contains a C++ core, MAVSDK MAVLink transport,
Copter operations, safety/watchdog/fence/payload primitives and adapters. Edge
Core source has been removed. Build and profile/service wiring repairs are
recorded in the migration gates, and setup/provisioning targets the C++ core.
The [current qualification status](docs/migration.md#current-qualification-status)
owns the exact aircraft profiles, implementation SHAs, test baseline, hosted SITL
evidence and remaining gates. Existing Copter scenarios and narrow QuadPlane
operations through QLAND are SITL evidence for their listed profiles. QuadPlane
link-loss/manual takeover, integrated Task 1 flight, competition-server
integration, command-authority/runtime hardening, generic landing/RTL/QRTL and
hardware qualification remain open.

Task 1 targets a lightweight VTOL with ground GPU vision. Task 2 currently
targets a quad below 15 kg with an onboard Jetson Orin Nano and robotic arm;
hardware integration remains unqualified. All three product profiles remain:
onboard_companion, groundstation_gpu and groundstation_minimal. See the canonical
documents for confirmed directions, confirmed source requirements and unresolved choices.

## Start here

- [Delivery plan](PLAN.md) and [working ledger](TODO.md)
- [Requirements and user decisions](docs/prd.md)
- [Architecture](docs/architecture.md)
- [Current qualification status](docs/migration.md#current-qualification-status)
- [Development](docs/development.md), [operations](docs/operations.md)
  and [safety case](docs/safety.md)

~~~sh
pixi run build-core
pixi run test-core
pixi run test-python
pixi run docs-build
~~~

`build-core` creates the production CLI and runtime in the build tree. It does
not install them; `package-core` creates release archives, and
`verify-core-staged-install` checks an isolated staged prefix. These checks do
not start hardware. The status matrix lists the scoped live SITL
evidence; broader image/profile and ROS startup gates at G1 remain open. Real
configuration stays in ignored local storage.

## Layout

C++ headers and implementation live in include/nomad and src; tests in tests.
ros2 contains adapters, python contains retained tools/perception work,
mission_planner contains the client, and config/docker/infra contain deployment
support. Public core headers do not depend on clients, ROS, Python or GPU SDKs.

## License

Apache 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
