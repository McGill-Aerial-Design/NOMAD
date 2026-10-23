# Native ArduPilot termination firmware

This source carrier adds a separate onboard **TERMINATE** mode for Copter and the
supported front-two-motor tricopter tiltrotor QuadPlane. Ordinary LAND/QLAND rates
remain unchanged. Execution, RC/fence triggers and the reboot latch stay onboard;
the Mission Planner plugin discovers the capability and requests the native mode.

This is custom ArduPilot firmware, not a feature of unmodified 4.7.2. The plugin's
base features also work with stock firmware; native termination requires this extension.

![NOMAD native fence draft helper used before aircraft commissioning](../../docs/images/fence-setup.png)

## Supported build

| Item | Source authority |
|---|---|
| ArduPilot base | 4.7.2, pinned full revision in [source.json](source.json) |
| Board | **CubeOrangePlus**, APJ board ID **1063** |
| Vehicle images | `arducopter.apj` for Copter; `arduplane.apj` for the supported QuadPlane |
| Execution | Native controllers; scripting disabled |
| Native extension | Private mode 29, AFS action 44, RC auxiliary option 187 |

CubeOrange is a different target; images are not interchangeable. The private
values are not upstream reservations and must be reviewed on a firmware update.
Use the [native termination guide](../../docs/native-termination.md) for flight-phase
behavior, required parameters, the independent transmitter input and known limits.

## Source contents and reproduction

| File | Purpose |
|---|---|
| [source.json](source.json) | Upstream repository/revision, patch checksum, board, features and recursive submodule pins |
| [controlled-termination.patch](controlled-termination.patch) | Complete vehicle/shared changes and native regression fixtures, including both new mode files |
| [COPYING.txt](COPYING.txt) | Upstream GPL license notice |

Use the exact upstream revision and pinned recursive submodules. Verify the patch
checksum and `git apply --check` before applying it. A matching complete-source
archive already has the changes applied: do not apply the patch a second time.
Build commands, the reviewed ARM toolchain and native check commands are documented
in [Build and reproduce](../../docs/native-termination.md#build-and-reproduce).
Build/test does not flash hardware, install the plugin or configure an aircraft.

## Release requirements

The [firmware workflow](../../.github/workflows/firmware.yml) qualifies relevant pull
requests and supports manual builds. A manual run with an existing tag at the
selected NOMAD commit can create a **draft prerelease**; it never publishes that
draft automatically. The separate Windows workflow uploads the plugin DLL and offline setup.
The [release procedure](../../docs/native-termination.md#build-and-release-automation)
also covers the same local build script. Qualified packages retain:

- Clearly separated board-specific Copter and Plane images, with board ID verified.
- SHA-256 checksums and source/compiler/feature identities binding the binaries.
- Qualification results for the focused Copter/QuadPlane suites, retained stock
  regressions and optional-feature compile checks.
- Matching complete corresponding source, including the applied changes, pinned
  recursive submodules, revision metadata, build instructions and license notices.

The patch alone is not the complete corresponding source package. Software/SITL
results do not establish physical descent, surface direction, transmitter mixing
or competition acceptance; see [qualification](../../docs/qualification.md).

## License boundary

The carried ArduPilot changes are **GPL-3.0-or-later**, separately from the
Apache-2.0 Mission Planner plugin. The firmware is not linked into the plugin DLL.
Preserve upstream notices and provide matching corresponding source with firmware
distribution; see the root [NOTICE](../../NOTICE).
