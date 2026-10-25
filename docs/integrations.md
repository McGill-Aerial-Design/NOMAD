# Ecosystem integration

NOMAD runs inside Mission Planner. Aircraft hardware, camera servers and companion
computers remain separately configured systems. The links below describe their roles;
NOMAD does not install or manage these projects.

## Aircraft mounts and MAVLink

ArduPilot's [AP_Mount subsystem](https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html)
handles aircraft mount control and its configured hardware backend. NOMAD uses the
selected Mission Planner connection's native mount APIs, including explicit-target
mount configuration and angle commands. It does not open a second gimbal transport.

The [MAVLink Gimbal Protocol v2](https://mavlink.io/en/services/gimbal_v2.html) defines
gimbal manager/device roles, capability discovery, control ownership and reference
frames. Those protocol facilities are distinct from NOMAD's existing native mount
path. NOMAD does not implement a separate gimbal manager client or negotiate v2
capabilities. Do not assume a device accepts every axis or that yaw means the same
body-relative or earth-relative frame across mount configurations. Verify the actual
ArduPilot backend, Mission Planner behavior, axis directions and physical limits.
NOMAD's tracking calibration and fresh aircraft-heading compensation do not prove
measured gimbal orientation or v2 frame support.

### CADDX GM3

Use ArduPilot's existing CADDX mount driver rather than connecting the plugin to
the gimbal directly. The [official setup](https://ardupilot.org/copter/docs/common-caddx-gimbal.html)
requires ArduPilot 4.6 or later and compatible gimbal firmware 3.4 or later. For the
chosen serial port, set `SERIALx_PROTOCOL=8` and `SERIALx_BAUD=115` (115200 baud);
the mount instance uses `MNTx_TYPE=13`. Follow that guide for wiring, reboot and
gimbal configuration, then calibrate the actual camera and travel in NOMAD.
These values describe aircraft setup; NOMAD does not write them automatically.

The [4.7.2 CADDX driver](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/libraries/AP_Mount/AP_Mount_CADDX.cpp)
reports its requested angles as orientation because that interface supplies no
attitude feedback. A smart or internally stabilized mount therefore does not
necessarily expose measured heading to the ground station. The
[GM3 V2](https://www.caddxfpv.com/products/caddxfpv-gimbal-gm3-v2) advertises additional
interfaces; identify the actual generation, firmware and selected backend before
assuming equivalent telemetry.

Native earth-yaw lock can compensate aircraft rotation onboard, between ground
updates. NOMAD currently keeps body-relative targeting: an earth-lock target
continues after requests stop, and backend travel-limit enforcement differs.
For example, the cited CADDX send path converts earth heading to body yaw without
the servo backend's final yaw clamp. Capability flags alone do not establish
NOMAD's calibrated travel, target-rate or Stop behavior. Current tracking uses
fresh aircraft heading/rates and bounded image correction; it does not treat
reported target angles as physical measurements.

[AP_Periph](https://ardupilot.org/dev/docs/ap-peripheral-landing-page.html) is ArduPilot
firmware for peripheral hardware, including networked sensors and actuators. It is
not a ground-station plugin or a video server. Compatible peripheral selection,
firmware, wiring and autopilot configuration belong to aircraft commissioning;
NOMAD has no AP_Periph provisioning integration.

## Cameras and companion computers

| Project | Role | Relationship to NOMAD |
|---|---|---|
| [mavlink-camera-manager](https://github.com/mavlink/mavlink-camera-manager) | Camera service exposing video streams and MAVLink camera functionality | A separately configured supported RTSP stream can be entered as a named video input; NOMAD does not implement its camera discovery or management API |
| [BlueOS](https://github.com/bluerobotics/BlueOS) | Companion-computer platform for vehicle services and extensions | Can host services independently; NOMAD does not deploy extensions or administer BlueOS |
| [Rpanion-server](https://github.com/stephendade/Rpanion-server) | Web configuration for companion networking, telemetry and video services | Supplies independently configured endpoints; NOMAD does not configure the companion or synchronize its settings |

Compatibility is at the configured endpoint: NOMAD accepts RTSP with a codec
supported by Mission Planner's installed GStreamer decoder, or RTP/H.264 on a UDP
listening port. A service's presence does not establish that its selected stream,
codec, transport, authentication and network route are compatible. Enter and verify
the actual URL in [Settings → Video](video.md#video-inputs).
MAVLink camera discovery, start/stop recording, zoom and camera configuration are
not provided by NOMAD's video player.

Keep camera traffic and telemetry routing explicit. The optional NOMAD link router
owns only its configured physical telemetry links and one Mission Planner loopback
consumer. It does not discover companion routes or create a video network. A
companion service is unnecessary for local detection and region tracking: those
run in the plugin's existing decoder worker. See [architecture](architecture.md)
for the command path and [video](video.md) for calibration and delay limits.
