# Pinned MAVSDK dependency inventory

This inventory records the MAVSDK build inputs at the reviewed NOMAD gitlink. It
is an engineering and notice audit, not a release approval. The production core
uses this pinned MAVSDK checkout as its only vehicle transport.
MAVSDK remains a project prerequisite, not an organizer-prescribed library.
Competition transport/traffic requirements do not justify adding speculative
network plugins: obtain the official server contract first. The selected build
includes Action, Geofence, Offboard, Param and Telemetry, and this table records the
reviewed fork gitlink.

| Component | Reviewed source | License found in fetched source |
|---|---|---|
| MAVSDK | NOMAD gitlink `60ec4f2975c6d207c3ed1c6ff35d9295a5689340` | BSD-3-Clause |
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

NOMAD's `cmake/NomadJson.cmake` explicitly requires the reviewed nlohmann JSON
3.12.0 CMake package installed by that same superbuild. Runtime, CLI and their
JSON-consuming tests link `nlohmann_json::nlohmann_json`; they do not acquire JSON
through a MAVSDK include path or transport dependency. This adds no download
path and keeps the existing archive hash and MAVSDK pin unchanged.

## Fork contract and licensing

The reviewed fork must retain ArduPilot semantics, command-admission fencing and
subscription/connection lifetime guarantees. Any pin or enabled-plugin change requires
provenance verification plus deterministic and full aircraft qualification.

Windows uses system BCrypt; Linux links OpenSSL Crypto, whose deployment dependency
must be available on the target. Redistributors must retain NOMAD's LICENSE/NOTICE
and the complete bundled selected-dependency texts. The `mavsdk-phase-a` directory name
is retained only as the checked license-bundle path. No license bytes were removed.
Enabling the MAVSDK server or another dependency requires a new licensing audit.
