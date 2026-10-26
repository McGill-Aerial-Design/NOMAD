# NOMAD for Mission Planner

Aircraft profiles, video, custom controls and redundant telemetry links in one
Mission Planner plugin. Mission Planner owns the aircraft connection, ordinary
flight controls, setup and missions. ArduPilot owns flight execution and failsafes.

![NOMAD dashboard](images/dashboard.png)

## Get started

1. Install the reviewed **Mission Planner 1.3.83** Windows host. The
   [target metadata](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/scripts/release/mission-planner-target.json)
   defines the exact build target.
2. Download the plugin setup from [NOMAD releases](https://github.com/McGill-Aerial-Design/NOMAD/releases).
   Close Mission Planner and follow [installation and rollback](operations.md#plugin-installation).
3. Open **NOMAD → Settings**, create or select the intended aircraft profile and
   review its links, video, payload calibration and input mappings. **Save** activates it.
4. Connect using Mission Planner and verify the live aircraft before using custom
   controls. Use Mission Planner for arm/disarm, modes, takeoff, landing and missions.

Source builds use the [development guide](development.md#build-without-installation).
Building the DLL does not install it.

## Systems

| System | What it does | Configure and use |
|---|---|---|
| Aircraft profiles | Saves separate controls, calibration, video and links for each aircraft | [Profiles](operations.md#saved-aircraft-profiles) |
| Video | Plays named RTSP or RTP/H.264 UDP inputs on Video, Dashboard and HUD | [Video inputs](video.md#video-inputs) |
| Detection and tracking | Highlights objects locally and follows a selected image region with the calibrated gimbal | [Tracking and observations](video.md#vision-and-gimbal-tracking) |
| Gimbal, payload and HID | Sends native mount, servo and relay commands with calibrated limits, input interlocks and Safe/Stop | [Custom controls](operations.md#gimbal-payload-and-input) |
| Link router | Routes Mission Planner traffic over configured UDP, TCP or serial links with deduplication and failover | [ELRS and LTE links](operations.md#elrs-and-lte-links) |
| Native fence setup | Stages an inclusion polygon in Mission Planner and reads/applies supported aircraft fence settings | [Fence setup](operations.md#native-fence-setup) |
| Optional termination | Requests a separate onboard mode provided by matching custom firmware | [Commissioning and operation](native-termination.md) |

Video tracking moves the gimbal; it does not navigate the aircraft or operate payloads.
Native command acceptance and displayed targets do not establish actuator motion,
touchdown or physical safety.

## Understand the system

The [architecture](architecture.md) explains ownership, command paths and lifecycle.
[Ecosystem integration](integrations.md) explains where ArduPilot mounts, camera
services and companion computers fit. Use [safety](safety.md) and
[qualification](qualification.md) to distinguish software checks from aircraft acceptance.
The [product requirements](prd.md) and [CONOPS inventory](conops-requirements.md)
record competition obligations and unresolved decisions.

## Future work

Research directions are collected under [brainstorming and future work](brainstorming.md),
with a [simulation planning-agent prompt](simulation-planning-prompt.md) for evaluating
rendered and headless tests. They are separate from current product capabilities.

## Support

For a reproducible issue, include the NOMAD source/version, Mission Planner version,
vehicle/firmware type, expected behavior and the smallest reproduction. Remove
private URLs, credentials, identifiers and location data from shared profiles or logs.
