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
Current-schema single-profile files at the legacy path remain supported; retired
runtime port/credential and management endpoint fields are discarded in memory. Older
schemas with unknown fields are rejected untouched and need explicit reviewed migration.
Loading never rewrites the file.
Explicit Save replaces atomically with a `.bak`; cancelled/failed saves preserve the
previous saved collection. Keep profiles private: connection URLs can contain sensitive
details even though runtime credentials are no longer stored/exported.

Each profile owns optional aircraft notes, gimbal mode/rate, calibrated payload
outputs, HID mappings, links and video. Aircraft notes are operator labels, not live
readbacks. Configure ordinary RC arming and flight modes through Mission Planner and
ArduPilot. Retired NOMAD RC-role fields load and validate in memory but are omitted
on the next explicit save/export; loading does not rewrite their source. New profiles
start without inherited payload mappings. Copy/import/export are drafts until Save.
Open settings from the NOMAD menu or sidebar. The persistent Editing label identifies
the draft aircraft; Save activates it. Profile actions contains reset, load and export.
Links, Video, Payloads and Inputs belong to that draft; Preferences contains alerts
and startup diagnostics.
Payloads and Links provide a list with Add, Edit and Remove. Their dialogs validate
the complete draft before accepting changes; Cancel leaves it untouched. Friendly
name changes preserve stable IDs and saved input bindings. Payload dialogs show
calibration, bounded timings and confirmation/neutral interlocks for the selected
behavior. Link dialogs distinguish UDP local listening ports from TCP remote server
ports; COM devices are entered explicitly and are never scanned or opened by setup.
Inputs pairs each of six switch actions with its physical button index. Actions show
payload names while retaining their IDs. Unavailable saved bindings stay visible for
correction; continuous position only lists eligible nonhazardous position payloads.

Use Mission Planner for connection, arm/disarm, flight modes, takeoff, LAND, setup
and missions. Disconnect Mission Planner before applying a different aircraft profile
or changed physical routes. Saving unrelated settings does not restart those routes.
Reconnect explicitly and verify the live aircraft before using custom controls.

## Native fence and LAND setup

Select Fence in native Plan first; NOMAD does not switch an existing mission draft.
In NOMAD → Fence Setup, choose the published polygon or paste/review coordinates,
then Stage. Review the native Plan fence draft, use native Write, then Read to verify
the aircraft fence. Configure fence enable/type and other failsafes through MP.

For the Copter landing preset, open native Config/Tuning → Full Parameter List and
clear filters so the required rows are visible. Keep that page active and choose
NOMAD → Tools → Stage Copter LAND settings. This requires a fresh, disarmed Copter
context and one complete recognized parameter layout. The preview uses current grid
values, including existing edits. It queues exactly three values for native review:

| Layout | Fence action | Initial/final landing rates |
|---|---|---|
| `LAND_SPD_MS` / `LAND_SPD_HIGH_MS` | `FENCE_ACTION=2` (Copter LAND) | Both `2` m/s |
| `LAND_SPEED` / `LAND_SPEED_HIGH` | `FENCE_ACTION=2` (Copter LAND) | Both `200` cm/s |

These global rates also affect ordinary landings. New-unit names and conversion are
documented in [pinned Copter source](https://github.com/ArduPilot/ardupilot/blob/b7ba8bca/ArduCopter/mode_land.cpp);
the old names/units are in [Copter 4.6.3](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/ArduCopter/Parameters.cpp).
Mixed/partial layouts, other firmware and incompatible heartbeat types are unavailable.
The preset does not enable a fence or change its type, radio-loss handling or any
other setting. It is not a separate termination mode or a compliance claim.

Staging uses MP's native edit callbacks and never writes to the aircraft. All three
values are queued again even if their display already matches, because MP can retain
a different queued value after refreshing its table. Other queued changes remain and
may be included in native Write Params. If an edit is rejected, a partial draft can
remain; review the reported result and native grid. Use Write Params, then Refresh
Params to read back aircraft parameter values; Refresh Table only reloads the cache.
This does not verify physical landing behavior or touchdown.

## Video inputs

Settings → Video stores named inputs for the selected aircraft. Add an input, enter
its name and source URL, and choose it from the dropdown. Save applies that selection
to the HUD, Dashboard preview and Video view; Cancel leaves the active configuration
unchanged. Remove the final input to leave video unconfigured. Existing `VideoUrl`
settings load as a named Default input without rewriting their file; the next save
uses the named-input format. Older plugin versions cannot read that new format.

RTSP negotiates the video codec supported by the installed GStreamer decoder. Select
TCP or UDP transport and a 0..5000 ms jitter buffer. UDP source URLs such as
`udp://@:5600` listen for RTP/H.264 on the explicit port; RTSP settings do not affect
those inputs. Camera and MediaMTX URLs, including addresses reachable through
Tailscale, are ordinary operator inputs; NOMAD does not set up their server or network.

The decoded-frame queue holds 1..120 frames. Drop old frames discards old queue
frames and bounds RTSP jitter buffering; disabling it lets the decode queue apply
backpressure. The display and native appsink still keep only the latest frame, so
this is not lossless recording. These receiver settings do not set end-to-end delay,
bitrate, resolution or encoded quality; configure those at the camera or encoder.
URLs can contain credentials: review profile exports before sharing them.

## ELRS and LTE links

Enable the profile's plugin router only when needed. Configure its physical links
and preferred link ID in the Links settings, using the reviewed ELRS primary
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
