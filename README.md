# NOMAD

NOMAD adds aircraft profiles, custom gimbal and payload controls, USB HID input,
video, local advisory maps and redundant-link routing to Mission Planner.
Mission Planner supplies the aircraft connection, setup, normal flight controls
and missions; ArduPilot flies the aircraft and owns its native failsafes.

The product is one .NET Framework 4.8 plugin DLL, built against the reviewed
Mission Planner target in [target metadata](scripts/release/mission-planner-target.json).
It supports saved aircraft-specific settings for Copter and Plane integrations;
board/frame/channel labels do not establish hardware compatibility or safety.

- [Development and checks](docs/development.md)
- [Installation, profiles and operation](docs/operations.md)
- [Architecture](docs/architecture.md)
- [Safety limits](docs/safety.md) and [qualification](docs/qualification.md)
- [Product requirements](docs/prd.md) and [organizer source inventory](docs/conops-requirements.md)

Builds write artifacts locally and never install them. Close Mission Planner and
follow the explicit manual installation procedure before copying a reviewed DLL.
The former C++/MAVSDK runtime, CLI, IPC and qualification implementation is preserved
on `MAVSDK-Archive`; its evidence does not qualify the native plugin.

Aircraft termination, manual takeover and full competition workflows still need
approved hardware integration and physical evidence. Command ACKs are software
evidence, never proof of actuator state, landing or termination.
