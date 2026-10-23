# Native termination

The optional ArduPilot extension provides a separate `TERMINATE` mode. Ordinary
`LAND` and `QLAND` rates remain unchanged. This is custom firmware, not a feature
of unmodified 4.7.2. The plugin remains usable with stock firmware; its termination
action stays unavailable without the advertised native capability.

Source is pinned to ArduPilot 4.7.2 commit
`fb773567896014da007bca58358704d4a0d8516f`. Both builds target **CubeOrangePlus**,
APJ board ID **1063**. CubeOrange is a different target; do not interchange images.
The patch, source identity and GPL notice are in
[`firmware/ardupilot`](../firmware/ardupilot/README.md).

## Onboard behavior

The private extension assigns mode **29**, AFS action **44**, and RC auxiliary
option **187**. These are not upstream reservations; review them on any upstream update.

- Copter uses native attitude/vertical controllers with a separately captured
  descent rate. It holds position when navigation is valid and falls back to level
  attitude after horizontal position loss.
- The front-two-motor tricopter tiltrotor uses native QuadPlane recovery, tilt
  slew, position control and landing detection. VTOL, assistance, partial tilt and
  transitions take the vertical path. Established fixed-wing flight with fully
  forward tilts uses native AFS motor-off/full-surface aerodynamic termination.
- RC loss in any flight mode, missing/invalid dedicated termination input and
  enabled native fence breach trigger the onboard response. GCS loss alone does
  not divert or terminate the aircraft while physical RC stays healthy.
- Accepted entry latches until flight-controller reboot. LOW/MIDDLE, another mode,
  reconnect, parameter edits and abort requests cannot release it. Vertical flight
  retains native contact/disarm; fixed-wing termination shuts propulsion down immediately.

`AFS_TERM_RATE` defaults to **2.2 m/s**, providing margin above 2 m/s. It must be
finite, at least 2 m/s and within native descent capability; its value is captured
once on entry. Actual descent and airframe tuning still require measurement.

## Aircraft setup

NOMAD → Fence Setup → Controlled termination can read/configure the supported
`AFS_*` settings below when the matching firmware advertises selectable `TERMINATE`
and freshly exposes the private `AFS_TERM_RATE` parameter. Stock firmware keeps
ordinary fence setup available and leaves this subgroup unavailable. NOMAD does not
choose or write an RC channel; remaining commissioning settings use native Mission
Planner parameters and fence tools. Configure/read back the actual aircraft with
props removed before separately authorized physical qualification.

The subgroup permits only controlled action **44**. Descent speed must be finite,
at least 2 m/s and no greater than a freshly read `WP_SPD_DN` for Copter or
`Q_WP_SPD_DN` for QuadPlane. It never changes either capability or normal LAND/QLAND
speed. RC loss timeout must be greater than zero and no more than 4.5 seconds;
the page offers 0.1–4.5 seconds.
Applying requires the captured current aircraft/profile and fresh disarmed heartbeats;
the confirmation names the aircraft and defaults to Cancel. Action/configuration is
written before enabling AFS, and each value is independently read back. Enabling
requires controlled action/rate, both RC/fence trigger parameters present and enabled,
and a present valid RC timeout. A failed or uncertain write stops subsequent changes
and requires another Read, preserving
earlier writes. The Dashboard action checks support again before a subsequent TERMINATE request.

| Native setting | Action-44 commissioning requirement |
|---|---|
| `AFS_ENABLE`, `AFS_TERM_ACTION` | `1`, `44` |
| `AFS_TERM_RATE` | Finite, at least 2 m/s, within native vertical controller capability |
| `AFS_RC`, `AFS_RC_FAIL_TIME` | Enabled; timeout greater than zero and no more than 4.5 seconds |
| `AFS_GEOFENCE`, native fence | Enabled, with a valid aircraft fence and reviewed altitude/datum |
| `RCx_OPTION` | Exactly one unused, transmitted channel assigned private option `187` |
| Termination input | Valid logical LOW before normal or forced arming; separate from roll, pitch, throttle, yaw and native mode channel |
| `FLTMODE1` through `FLTMODE6` | Ordinary modes; none assigned `29` |
| `RC_OPTIONS` | Ignore MAVLink RC overrides; do not ignore the receiver or receiver failsafe flags; preserve unrelated bits |
| Native RC failsafe | Enabled; receiver timeout no more than 4.5 seconds; fresh healthy physical receiver required |
| Copter descent capability | `WP_SPD_DN` at least `AFS_TERM_RATE` |
| QuadPlane descent capability | `Q_WP_SPD_DN` at least `AFS_TERM_RATE` |
| QuadPlane estimator | Native `AHRS_OPTIONS` VTOL DCM-fallback inhibition, bit 1/value 2; preserve other bits |

The transmitter implements the deliberate two-control mix on that dedicated input.
Logical HIGH activates termination. Firmware sees the mixed channel; it cannot
prove two controls were used or detect a broken switch still transmitting valid
LOW. Verify the mix, channel independence and receiver failsafe on the actual radio.
Tests use RC7; this is not an aircraft wiring recommendation. Stock MP metadata may
show private values numerically; use Full Parameter List and explicit write/readback.

## Ground operation

Select the live aircraft and open **NOMAD → Dashboard**. The TERMINATE widget's
**Check** button verifies the selectable native `TERMINATE` advertisement and
freshly reads `AFS_ENABLE`, `AFS_TERM_ACTION` and `AFS_TERM_RATE`. Hover over the
widget with the NOMAD window in the foreground, release both Shift keys, then
press left and right Shift within 750 ms of one another. Both must be down together.
The widget rechecks support; keep the pointer over it and the window focused, then
release both Shift keys to open the aircraft confirmation. Leaving the widget or
losing focus before that dialog cancels the review. Held keys cannot repeat it.

Confirmation names the captured aircraft and defaults to Cancel. Only explicit OK
requests the mode. A fresh armed heartbeat and the same connection, stream,
aircraft IDs and profile generation are required. Aircraft entry checks remain onboard.

The action uses MP's native `DO_SET_MODE` helper with the advertised numeric ID.
It has no command-185/disarm fallback or application retry. MP retains normal ACK
waits/retries; a dispatched helper can finish after profile change or unload.
Inspect an uncertain outcome before another deliberate request.

ACK acceptance and observed mode are separate. NOMAD names TERMINATE only after
a fresh matching heartbeat; the Dashboard widget then shows the latch/reboot requirement.
Neither proves descent, touchdown, stopped motors or physical safe state. The
radio path works without MP/NOMAD. USB ground-controller button integration,
transmitter mixing and wiring remain separate aircraft integration work.

## Build and reproduce

Use a Linux ArduPilot development environment, its normal prerequisites/Python test
dependencies and an ARM embedded toolchain. Hardware builds use GNU Arm Embedded
10.2.1. The plugin build neither installs that toolchain nor builds aircraft firmware.

At the pinned revision with its pinned recursive submodules, run `git apply --check`
and `git apply` on `firmware/ardupilot/controlled-termination.patch`. Alternatively,
extract the matching complete source archive, which includes revision metadata.

```sh
./waf configure --board sitl --out build \
  --enable-ADVANCED_FAILSAFE --enable-COPTER_ADVANCED_FAILSAFE --enable-QUADPLANE
./waf --out build copter plane -j4
python Tools/autotest/autotest.py --speedup 10 --timeout 600 test.Copter.Terminate
python Tools/autotest/autotest.py --speedup 10 --timeout 600 test.QuadPlane.Terminate

./waf configure --board CubeOrangePlus --out build-plus --toolchain arm-none-eabi \
  --no-submodule-update --enable-ADVANCED_FAILSAFE --enable-COPTER_ADVANCED_FAILSAFE \
  --enable-QUADPLANE --disable-scripting
./waf --out build-plus copter plane -j4
```

Name the output directory on every configure/build: Waf remembers its previous
configuration. Preserve qualified SITL binaries when building other variants.
These commands neither flash hardware nor build a bootloader.

## Build and release automation

The [firmware workflow](../.github/workflows/firmware.yml) runs the same Linux
[build script](../scripts/release/build_firmware.py) used for local qualification.
Relevant firmware pull requests build packages with read-only repository access.
Manual runs also accept an optional `release_tag`.

The environment is Ubuntu 24.04, Python 3.12 and GNU Arm Embedded 10.2.1. The
workflow verifies the toolchain archive checksum and installs the reviewed direct
Python dependencies from [firmware-requirements.txt](../scripts/release/firmware-requirements.txt).
These direct pins are not a complete transitive or bit-for-bit reproducibility
guarantee; the manifest records the actual installed versions and compiler identity.

With those prerequisites available, run from a Linux-readable NOMAD Git checkout:

```sh
python3 scripts/release/build_firmware.py --work /new/empty/firmware-work --jobs 4
```

The work path must not already exist. The script verifies the pinned upstream
revision, recursive submodules and patch hash, then freezes complete corresponding
source before building. It verifies an extraction and uses that source for builds.
All mandatory gates must pass: both focused termination suites, the five retained
stock regressions, three optional-feature compile configurations, both hardware
builds, required features, APJ board/image identity and source/binary stability.
Failure retains diagnostics and leaves the result unqualified. Only a fully
assembled package is exposed in `release/`; raw runtime logs stay in `diagnostics/`.

To prepare a GitHub release after the workflow is on the default branch:

1. Select **Actions → Firmware qualification → Run workflow** and the intended ref.
   Leave `release_tag` empty for a build-only run.
2. For a draft, first create a reviewed tag at exactly that NOMAD commit. Enter
   that existing tag as `release_tag`; a missing, moved or mismatched tag fails.
   The workflow refuses an existing release instead of replacing its assets.
3. Inspect the `firmware-release` artifact and results. A successful tagged manual
   run creates a **draft prerelease** with verified assets. Review the board,
   aircraft type, qualification scope and source before explicitly publishing it.

The package contains Copter/Plane APJs, BINs and ELFs, complete corresponding source,
source/patch/build identities, license notices, setup instructions, a manifest and
`SHA256SUMS`. GitHub's automatically generated NOMAD source ZIP is not the complete
ArduPilot corresponding-source archive; retain the separately supplied source asset.
No workflow stage installs a plugin, flashes a controller or configures an aircraft.

## Evidence limits

The focused suites cover commissioning/forced-arm rejection, independent RC
activation, RC/termination-input loss, fences, GCS-only loss, mode/latch behavior,
position loss, recovery, transitions, aerodynamic outputs, separate descent bands,
native contact/disarm and ordinary landing. Descent checks use signed velocity and
independent altitude/time samples. Recovery checks fresh native thrust-direction
tracking; native heading correction can settle later. The ground gate uses actual
MP assemblies and an owned synthetic loopback peer.

These are software checks. Physical mixing, surface direction, motor/tilt calibration,
actuator power loss, actual descent through touchdown and organizer acceptance remain
open. Total attitude/vertical estimate loss or stopped flight-controller execution
cannot be called guaranteed controlled descent. Transition policy requires aircraft
and organizer acceptance under [PRD Q02](prd.md#unresolved-organizer-questions).
