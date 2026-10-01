# Operations

This guide covers the supported ground deployment. For build and qualification
workflows, see [Development](development.md) and [Qualification status](qualification.md).
For the current component boundaries, see [Architecture](architecture.md).

## Processes and endpoints

Run one `nomad-runtime` process for each vehicle connection. It owns the
long-lived MAVSDK connection, vehicle policy, authority/request lifecycle, and
runtime IPC listener. The runtime does not load `config/nomad.env` itself and
does not have a checked-in systemd service. A local process supervisor must
start it with the deployment environment and restart policy. Do not start a
second runtime against the same vehicle endpoint.

The runtime and installed CLI accept these process-environment settings:

| Setting | Purpose |
| --- | --- |
| `NOMAD_MAVLINK_ENDPOINT` | MAVSDK vehicle endpoint, such as `udpin:127.0.0.1:14601` |
| `NOMAD_RUNTIME_IPC_PORT` | Loopback TCP port for installed CLI and typed clients; default `14611` |
| `NOMAD_API_KEY` | Nonempty deployment actuation enable gate; not identity authentication |
| `NOMAD_CLIENT_CREDENTIALS_FILE` | Protected JSON identity-to-token map, loaded once at startup |
| `NOMAD_AUDIT_DIRECTORY` | Private runtime journal directory; parent must exist |
| `NOMAD_CLIENT_CREDENTIAL` | CLI-only shared-secret credential; do not give clients the full runtime map |
| `NOMAD_CLIENT_ID` | CLI identity, default `nomad-cli` |
| `--system-id` | Optional runtime command system ID; defaults to `1` |

Use the example environment file as a template, then have the supervisor load
the operator's local `config/nomad.env`. Keep credentials local. Runtime IPC is
bound to loopback and is not authenticated as a general remote API; the API key
is an actuation gate, not a substitute for host access control. The example key
is blank; configure a nonempty secret value or runtime mutations remain unavailable.

After the supervisor has loaded the configured environment, a direct console
launch is:

```powershell
nomad-runtime --endpoint udpin:127.0.0.1:14601 --ipc-port 14611 --system-id 1
```

The runtime status endpoint can report that the process is available even when
the vehicle link is disconnected. Check vehicle/link state separately before
requesting an action. Installed `nomad` CLI commands such as `status`, `admit`,
`revoke`, and `handback` use runtime IPC; the CLI does not open its own flight
connection.

## Ground router

The standalone ground router is a separately supervised Windows/.NET Framework
4.8 process. It owns configured physical ground links and routes MAVLink among
its configured clients. Build and check it with:

```powershell
pixi run build-ground-router
pixi run test-ground-router
```

The checked-in template uses these local endpoints:

| Consumer | Router endpoint | Purpose |
| --- | --- | --- |
| Mission Planner | `127.0.0.1:14600` | Receive-only telemetry/status consumer |
| Runtime | Router output `14602` to runtime listener `14601` | Vehicle traffic for the sole NOMAD command path |
| Router management | `127.0.0.1:14610` | Local router management interface |
| Runtime clients | `127.0.0.1:14611` | Runtime IPC for CLI and typed clients |

Use `infra/transport/ground_router/example.json` as the router configuration
template to select physical serial or UDP links and client endpoints. The ground
router routes packets; it does not grant NOMAD authority or make flight decisions.
For a downloaded router package, start its executable from the directory that
contains the matching configuration; for example, `nomad-link-router.exe router.example.json`.
The aircraft-side `mavlink-router` is a different process, typically managed
on the aircraft by the optional `nomad.target` systemd setup. Enabling
`NOMAD_AUTOSTART_MAVLINK_ROUTER` concerns that aircraft-side service only.

## Mission Planner

Mission Planner provides UI, management, and status. The NOMAD plugin submits
only the typed requests currently supported by runtime IPC. Its router
telemetry consumer is receive-only. A Mission Planner native/direct vehicle
connection is external to NOMAD's software authority boundary and can provide
an independent control path; configure deployment wiring intentionally.

Configure the native telemetry connection as UDPCl to the ground router's
`127.0.0.1:14600` receive-only consumer. The plugin's router status panel uses
management TCP `127.0.0.1:14610`; its runtime client uses loopback IPC
`127.0.0.1:14611` by default. The panel reports stale or unavailable router
status when that separate process is stopped or disconnected.

For local development, use `pixi run build-plugin-only` and
`pixi run test-plugin-build-only`. Build the release archives through the
repository release workflow: a `v*` tag publishes release artifacts, while a
manual workflow run uploads downloadable artifacts. The plugin installation
script is `mission_planner/packaging/INSTALL.ps1`; inspect the generated
artifact and target installation before running it. `build-plugin-only` only
compiles the plugin and does not install it. See the
[plugin packaging guide](../mission_planner/packaging/README.md) for staging the
DLL beside the installer and installing the release ZIP.

## Product profiles

Profiles under `config/profiles/` are deployment templates. Inspect available
profiles and compare settings before applying one:

```powershell
pixi run profile-list
pixi run profile-show
pixi run profile-diff <profile>
pixi run profile-load <profile>
```

Loading a profile updates the local `config/nomad.env` and Mission Planner
configuration. It does not start processes, prove that hardware is present, or
qualify the resulting deployment. Review every endpoint and device setting.
The installed CLI/runtime IPC v1 does not expose mission upload, navigation,
geofence, or payload request paths; profile text must not be read as evidence
that those functions are usable through NOMAD.

## ROS observer

ROS 2 is optional and observation-only. It consumes MAVLink telemetry for ROS
applications; it does not issue vehicle commands or load NOMAD authority.
Profiles leave it disabled unless deliberately configured. The development
guide covers `sim-ros-build`, `test-ros-integration`, and `sim-ros-up`.

## Build, package, and install

Build and test the core before preparing an archive:

```powershell
pixi run build-core-release
pixi run test-core
pixi run package-core
pixi run verify-core-package
pixi run verify-core-staged-install
```

Packaging creates release archives. Package verification inspects those
archives; staged verification installs into `build/package/stage` only. Neither
step writes to the host install prefix. A real prefix install is an explicit
operation and requires a destination:

```powershell
pixi run install-core <prefix>
```

The core package includes `nomad` and `nomad-runtime`; qualification drivers are
not installed. Linux runtime/client binaries require the compatible system OpenSSL
Crypto shared library used by that build (Ubuntu builds use `libcrypto.so.3`);
Windows uses the OS BCrypt library. Mission Planner packaging is separate and handled by the release
workflow described above.

## Failure states and authority limits

Treat disconnected, stale, unavailable, invalid, busy, and unauthorized results
as failures. Check runtime status and fresh vehicle telemetry before retrying.
Admission expiry, sequence rejection, revocation, and reconnect do not restore
authority automatically; explicit handback is required by the software
lifecycle. Do not infer vehicle action from a request acceptance response.

NOMAD's guarantees cover its own software request path. They do not arbitrate
the flight controller's other MAVLink sources, RC/ELRS input, a native GCS, or
physical pilot control. Production C2-loss, termination, pilot takeover, and
hardware flight behavior remain unqualified; see [Qualification status](qualification.md).

## Local client credential deployment

Generate independent 32-byte random credentials (for example Python
`secrets.token_hex(32)`) for `nomad-cli` and `mission-planner`. Write only the
identity/token object into a local protected file outside tracked configuration;
never print tokens to shared logs or copy example/gate values as credentials.
On POSIX use an owner-only directory and mode 0600 file. On Windows restrict the
file/directory DACL to the runtime account, SYSTEM and administrators, removing
broad inherited access, and explicitly set the owner to the runtime account
(for example `icacls <credential-file> /setowner <runtime-account>` when provisioning
from an elevated shell). Provision only each client's token into its protected
environment (CLI) or `CoreClientCredential` plugin setting. Protect that plugin
JSON configuration and its `.bak`/`.tmp` siblings as credential stores with the
client account's ACL. Portable plugin exports omit the credential. These controls
do not protect against malware that can read the account's credentials.

Set the runtime's credential-file and audit-directory environment variables,
and independently set the nonempty `NOMAD_API_KEY` deployment gate. The runtime
does not automatically load `config/nomad.env`. Rotating the file while running
has no effect: stop the runtime, replace protected credentials, provision clients,
and restart; authority must be admitted again. Profile loading never substitutes
the API gate for the plugin credential. Existing `CoreApiKey` is discarded,
so upgrading the plugin requires explicit provisioning.

For audit startup failure, preserve the files and inspect permissions, space,
JSONL integrity and any competing directory owner. The journal is an operational
record, not a cryptographic tamper-evident ledger against its host administrator.
Do not delete evidence to
hide an error. Damaged history requires operator investigation and preservation
outside the active directory before starting a fresh journal. Intent without
outcome remains unknown; a restart must never replay it. Runtime audit failure
latches mutations off; status remains available. See the precise
[durability policy](runtime-ipc.md#durable-runtime-command-evidence).

Keep these states separate: actuation enabled; client authenticated; client
admitted as software authority; command eligible for final send; vehicle command
accepted by an observed response; physical outcome. None implies the next.
