# Operations

Use the reviewed Mission Planner version in
[`mission-planner-target.json`](../scripts/release/mission-planner-target.json).
Builds/tests do not install software, change operator settings or control aircraft.

## Manual plugin installation

1. Build and review `mission_planner/src/bin/Release/NOMADPlugin.dll` using
   [development checks](development.md). Record the checked source and test results.
2. Close Mission Planner. Preserve the existing plugin DLL and real operator profiles
   outside the replacement destination before any manual change.
3. Explicitly copy only the reviewed `NOMADPlugin.dll` into the `plugins` directory
   of the intended Mission Planner installation, then start that installation.
4. Verify the host version, plugin version/source identity and selected aircraft
   profile. Review mappings and calibration before enabling custom controls.

Host reference assemblies, binaries and installed dependencies are not copied into
NOMAD artifacts. Video requires the host's installed GStreamer support. The old
multi-component deployment/rollback framework and its installer wrapper are removed;
there is no automatic activation, service installation or rollback guarantee.

## Saved aircraft profiles

NOMAD stores profiles in `%LOCALAPPDATA%/Mission Planner/plugins/nomad_aircraft_profiles.json`.
Legacy single-profile loading remains supported; retired runtime port/credential and
management endpoint fields are discarded in memory. Loading never rewrites the file.
Explicit Save replaces atomically with a `.bak`; cancelled/failed saves preserve the
previous saved collection. Keep profiles private: connection URLs can contain sensitive
details even though runtime credentials are no longer stored/exported.

Each profile owns aircraft labels, RC roles, gimbal mode/rate, calibrated payload
outputs, HID mappings, links and video. Cube Orange Plus and ELRS CH5 arm/CH6 modes
are one aircraft's settings, not global defaults. New profiles start without inherited
payload mappings. Copy/import/export are drafts until Save.

Use Mission Planner for connection, arm/disarm, flight modes, takeoff, LAND, setup
and missions. Disconnect Mission Planner before applying a different aircraft profile
or changed physical routes. Saving unrelated settings does not restart those routes.
Reconnect explicitly and verify the live aircraft before using custom controls.

## ELRS and LTE links

Enable the profile's plugin router only when needed. Configure its physical links
and preferred link ID in the Multi-Link settings, using the reviewed ELRS primary
and LTE secondary topology. An empty link list opens no physical connection.
Connect Mission Planner with UDPCl to `127.0.0.1` and the profile's local router port.
The plugin owns sockets/serial handles and closes them on profile replacement/unload.
It does not start an external router process or stop an installed legacy service.
An operator must separately retire any old runtime/router deployment before installing
this replacement; this repository change performs no infrastructure transition.

Aircraft-side serial fan-out still uses separately reviewed deployment settings:
[router configuration](../infra/transport/mavlink_router/main.conf) and
[systemd unit](../infra/systemd/nomad-mavlink-router.service). Their addresses/device
paths are placeholders. This plugin does not install or alter that aircraft service.

## Gimbal, payload and input

Select explicit HID devices; missing devices are never replaced by unrelated ones.
After reconnect/input loss, center analog controls and release buttons before fresh
activation. Payload hazardous actions require their configured distinct confirmations
and neutral edges. Safe/Stop remains explicit. Review the chosen channel, PWM bounds,
neutral/safe values and actual aircraft wiring before use.

Timed servo actions read live captured-target `SERVOx_TRIM` and require it to match
the configured return value. Native repeats share one controller slot; the UI reports
busy/uncertain state. Mission Planner may retry commands and complete a dispatched
helper after Stop is requested. A Safe ACK is software evidence, not measured motion.
Unknown outcomes block activation replay until explicit accepted Safe. Do not assume
external MP/mission commands preserve NOMAD's repeat reservation or commanded state.

Use Safe/Stop and wait for the native result before disconnecting or closing MP.
Unload attempts best-effort Safe; it cannot guarantee return delivery, cancel a native
helper already executing, or prove physical de-energization. Onboard repeats retain
native controller return behavior. Keep physical emergency procedures and ArduPilot
failsafes independently qualified. NOMAD termination is unavailable: no LAND/disarm
recipe substitutes for an approved aircraft mechanism.
