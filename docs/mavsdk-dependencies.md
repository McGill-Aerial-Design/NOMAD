# Pinned MAVSDK dependency inventory

This inventory records the MAVSDK build inputs at the reviewed NOMAD gitlink. It
is an engineering and notice audit, not a release approval. The production core
uses this pinned MAVSDK checkout as its only vehicle transport.
The CONOPS v1.0 reconciliation did not change dependency pins or accepted evidence.
MAVSDK remains a project prerequisite, not an organizer-prescribed library.
Competition transport/traffic requirements do not justify adding speculative
network plugins: obtain the official server contract first. The selected build
includes Action, Geofence, Offboard, Param and Telemetry, and this table records the
reviewed fork gitlink.

| Component | Reviewed source | License found in fetched source |
|---|---|---|
| MAVSDK | NOMAD gitlink `900fb0fe7fec74608f1911331218557915cc501a` | BSD-3-Clause |
| MAVSDK proto | nested gitlink `5c81ecfeb6110cf74ba75ae50b78a1b265c05670` | Unknown: no separate license declaration found; server disabled |
| Asio | tag `asio-1-30-2` | Boost-1.0 |
| fmt | tag `12.1.0` | MIT |
| libevents | commit `840a88ea226d4eb0fd4c391ce860317422756435` | BSD-3-Clause |
| libmavlike | commit `90498b14262137ae10b633705810e81bdb85de9c` | BSD-3-Clause |
| MAVLink | commit `d6a7eeaf43319ce6da19a1973ca40180a4210643` | generator (L)GPL-3.0 with MIT output exception |
| nlohmann JSON | archive tag `v3.12.0`, SHA-256 `4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187` | MIT |
| PicoSHA2 | commit `1bf940d8a03bb752604fbb366d47b97b50b9e6ce` | MIT |
| tinyxml2 | tag `11.0.0` | Zlib |
| liblzma from XZ Utils | archive `5.4.5`, SHA-256 `135c90b934aee8fbc0d467de87a05cb70d627da36abe518c357a873709e5b7d6` | public domain for liblzma; package contains mixed licenses |

The connectivity-smoke build disables the MAVSDK server and curl, so their
optional dependency sets are outside this inventory. MAVSDK's patched MAVLink build uses
the pymavlink generator source nested in the pinned MAVLink checkout instead of
running a build-time `pip install`; generator packages are not linked into the
smoke executable.

The checker `pixi run verify-mavsdk-provenance` fails if reviewed gitlinks,
dependency references, archive hashes/timestamp handling, the pinned-generator
patch, NOTICE component names, or any bundled licence text changes without an
explicit audit update. The complete selected-build texts and their checked hashes
are in `licenses/mavsdk-phase-a/`. Recursive checkouts fetch MAVSDK-Proto, but it
is not compiled or linked while the server remains disabled; enabling it requires
a new audit.

## Command admission fork update - 2026-09-28

The NOMAD gitlink advances from `3f85f6f808b617c736316d7da5f51f3d3eba1737`
to `900fb0fe7fec74608f1911331218557915cc501a` in the project fork.
The [fork review](https://github.com/YoussGm3o8/MAVSDK/pull/2) adds
per-operation admission to the command sender and final UDP delivery for
`COMMAND_LONG` and `COMMAND_INT`; it does not change a selected
third-party dependency reference, archive hash, nested proto gitlink or license
text. The Windows MAVSDK Release target and hosted Linux/Windows MAVSDK
qualification pass at this pin; the fork's ArduCopter 4.7.1 SITL job also
passes. The peer-level authority qualification and its limits are recorded in
[migration](migration.md#command-retry-fencing-and-routed-writer-arbitration-2026-09-28).

## Rebase audit - 2026-09-19

The fork was rebased onto upstream
`d7043d3cafe8cd6250565fd211b966d8b455d561`. The selected production dependency
references, archive hashes and bundled license texts are unchanged. Enabling
Offboard adds no third-party library dependency. The nested proto revision
changed with upstream; its current tree contains no LICENSE or COPYING file,
so this inventory no longer infers BSD-3-Clause from MAVSDK's separate license.
Resolve that audit before enabling the server or distributing those inputs.

All platforms now apply the pinned MAVLink generator patch once, including iOS.
Debian 11 packaging uses signed, dated Debian and Debian Security snapshots
because its live indexes referenced unavailable packages. This reproduces the
build environment and does not promise continued Debian 11 security support.
Exact-pin qualification is recorded in the [dated compatibility evidence](migration.md#mavsdk-compatibility-evidence-2026-09-19).

## Historical hardening evidence

Before this rebase, the project MAVSDK fork was pinned at
`e0dada26a606ffa4f48e72841efa231177733d05`. The fork replaces the mutable
PicoSHA2 branch with an immutable commit, adds SHA-256 archive checks for
liblzma and nlohmann JSON, handles deterministic archive extraction on both
pre-3.24 and newer CMake, and removes MAVLink's build-time network package
resolution in favor of its pinned nested pymavlink generator source.NOMAD
pins that revision directly so a recursive clean checkout resolves the reviewed
graph.

The fork's intended scope is not limited to dependencies: since 2026-09-12 it
also owns ArduPilot command, mode and telemetry semantics, so the pinned
revision changes only when that work lands with its own tests and a
requalification — see [MAVSDK adoption](mavsdk-adoption.md). Commit
`e0dada26a606ffa4f48e72841efa231177733d05` contains the standalone compatibility layer, its regression tests,
and the dedicated ArduPilot workflow.

Recursive hosted test run `34535620056` passed the optional MAVSDK build,
provenance checker and deterministic expected/wrong/absent-peer fixture on both
Ubuntu and Windows, alongside the Python and C++ core suites. Selected ROS-image
run `34538394497` built the pinned graph with NOMAD_ENABLE_MAVSDK=ON and passed
the ROS adapter integration tests. Mainline SITL run `34538903820` built an
ArduPilot Copter 4.7.1 image and passed the live MAVSDK Phase A connect/status
smoke. These historical results establish clean-checkout and hosted build evidence
for that Phase A graph; they do not qualify later plugin additions,
production cutover or aircraft hardware.

The historical warm Windows build tree was 379,308,757 bytes and the smoke
executable was 2,032,128 bytes. These are one-machine diagnostics, not approved
budgets or clean-build benchmarks. Current build-tree, executable, runtime memory,
startup and CI-time measurements still need repeatable collection and explicit
budget approval before Phase A closes.

The build task now emits configure/build durations and footprint values together,
and the hosted Linux/Windows jobs retain the JSON record. Local collector checks
on 2026-09-10 used a dirty `34b417d4` vendor checkout and therefore remain
diagnostic; its provenance check failed the pinned-generator marker. Accepted
samples must come from the reviewed gitlink in a recursive clean checkout.
Live smoke metrics now report per-process-tree peak RSS instead of the previous
cumulative child-process maximum, and their hosted output is retained separately.

Clean local Windows verification on 2026-09-10 used NOMAD `610215f`, the reviewed
MAVSDK gitlink `9884f109533f564bc6250e5471e6301d3a62f4a7`, MAVSDK-Proto
`1fd0bc7a05c21336227b1eab266b8b610401cf38`, ArduPilot MAVLink `288b907c` and
pymavlink `ec06837a`. Provenance, build, deterministic peer cases, 10 CTests and
345 Python tests passed; three ROS/SITL environment cases skipped. The measured
warm build values are recorded in the adoption decision. Hosted resource samples,
live runtime metrics and approved thresholds remain open.

Hosted run `34550522657` at commit `219133e` then retained clean Linux and
Windows build records. Linux measured a 213,615,930-byte tree, 4,719,032-byte
executable, 1,552,468 selected-archive bytes and 43.020/172.291 s
configure/build time. Windows measured a 430,393,315-byte tree, 2,015,744-byte
executable, 12,051,864 selected-archive bytes and 187.318/326.074 s
configure/build time. Both provenance audits and deterministic peer suites
passed. Live Copter 4.7.1 SITL run `34550529273` retained 0.731 s connect and
2.534 s status measurements with 9,584,640 and 9,940,992-byte peak process-tree
RSS respectively. Budget approval, aircraft evidence and flight qualification
remain open. The enclosing full workflow failed twice afterward in the legacy
C++ zero-delivery observer with `wire=[]`; this does not invalidate the earlier
MAVSDK connect/status sample. The observer was repaired, and merged-main run
`35489428247` passed the complete Copter SITL matrix, including zero delivery and
the heartbeat gate. Resource approval, supported-aircraft and release evidence
remain open.

## Production resource composition

The resource collector audits the unchanged pinned graph through CMake's file
API. NOMAD selects Action, Geofence, Offboard, Param and Telemetry; the pinned
MAVSDK CMake also adds MAVLink Passthrough. All six compile into one static
`mavsdk` archive. Production consumes their APIs in the action, fence,
velocity, telemetry, parameter, QuadPlane and route transport implementations.
The runtime links that archive through `nomad_mavsdk_connection`; direct
qualification tools reuse the same transport. No selected plugin is present
solely for qualification. The CLI and connectivity observer have build-order
dependencies on MAVSDK but do not link the vehicle transport.

The child graph disables MAVSDK tests, the gRPC server and curl, uses the
ArduPilot dialect and static libraries, and enables the superbuild. NOMAD's
own Release qualification enables CTest separately. The superbuild includes
fmt, Asio, generated MAVLink, libevents, PicoSHA2, tinyxml2, libmavlike,
nlohmann JSON and liblzma. Header-only inputs and compiled archives are distinct
in the evidence. `runtime_link_archives` records actual linker inputs;
`built_but_not_runtime_link_inputs` exposes archives outside that link line.
Selected plugin source references establish production use, but archive inputs
do not measure individual archive-member inclusion or embedded object-code
bytes. No dynamic-versus-static counterfactual has been measured.

libmavlike supplies the installed `mav::mav` package and `mav.lib`/`libmav.a`,
which the SDK core links and uses through `LibmavReceiver`; the vendor project
name differs from its library name. It is required. PicoSHA2 is header-only and
remains a required MAVSDK package even though this audit found no direct source
include. Removing it needs a separate fork review; no compiled-plugin saving
is established. Core server/FTP/mission-transfer
translation units are also compiled internally even with the gRPC server off;
their names alone do not prove removable functionality or executable cost.
No dependency pin, plugin selection, library composition or safety check was
changed for resource numbers.

See [development](development.md#software-resource-qualification) for repeatable
production footprint, software runtime and per-phase time collection. Historical
smoke metrics above remain separate observations of older graphs. Current
baseline/budget definitions live in the versioned
[`core-resource-budgets.json`](../config/core-resource-budgets.json), while
per-run reports remain hosted artifacts. The core payload carries two
executables, headers, configuration/lifecycle templates and dependency notices;
it does not ship the separate MAVSDK or superbuild install trees, qualification
drivers, PDBs, or developer archives.

## Open release blockers

- Re-run the selected dependency and licence audit whenever production parity
  enables another plugin, server, curl or test dependency.
- Requalify the hosted Linux/Windows, selected ROS and live ArduPilot SITL matrix
  whenever the MAVSDK pin, selected dependency graph or smoke contract changes.

debt: resource budgets cover the recorded software workload and toolchains;
revisit when dependency/plugin selection, toolchain or workload changes; then
collect a comparable Release baseline and review the policy headroom.
