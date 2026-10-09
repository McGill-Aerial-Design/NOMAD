# NOMAD

NOMAD monitors and controls ArduPilot vehicles through a persistent C++20 runtime.
Mission Planner and the installed CLI send authenticated typed local requests;
the runtime owns authority, safety, actuator sequencing, audit and recovery and
uses one MAVSDK command transport. A standalone router handles physical links.
ArduPilot retains stabilization, navigation execution and native failsafes.

The delivered control surface is authority management, validated output/gimbal
operations and configured actuator actions. Mission Planner adds direct USB HID
input, live video, link status, advisory map outlines and bounded Copter LAND engagement.
LAND success means mode observed, not touchdown or termination. Other navigation/QuadPlane,
fence and velocity implementations remain in the non-installed qualification
driver; they are not production IPC features. Physical takeover, termination,
competition integration and full aircraft qualification remain open.

```sh
git submodule update --init --recursive
pixi install
pixi run build-core
pixi run test-core
pixi run test-python
```

See [development](docs/development.md) for prerequisites and complete checks,
[architecture](docs/architecture.md), [operations](docs/operations.md),
[safety](docs/safety.md), [qualification](docs/qualification.md),
[requirements](docs/prd.md) and [remaining work](TODO.md).
Contribution rules live in [AGENTS](AGENTS.md). Never use simulator actuation
against a live aircraft or interpret software/SITL evidence as flight authorization.
