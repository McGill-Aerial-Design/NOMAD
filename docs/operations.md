# Operations

## Release deployment audit (base `1efaa335`)

Before the versioned deployment slice, CPack generated core ZIP/TGZ archives
and the staged verifier checked contents and an offline CLI. The release
workflow published only separately assembled plugin/router ZIPs and a loose
plugin DLL. It did not aggregate core packages, bind package digests to one
source/tag identity, or refuse a partial component set. CMake/runtime used a
fixed `0.1.0`; plugin/router implementation metadata did not establish the
same authoritative release identity. A manual workflow run could borrow its
branch name as a package label.

PR56 systemd/SCM lifecycle provisioning keeps protected configuration,
credentials and audit journals external, and restart invalidates software
authority. PR57 records reviewed dependency/build/resource provenance. These
are useful foundations, but neither is an installation activation journal or
rollback qualification. Prefix installation could overwrite an existing
prefix. The plugin installer overwrote `NOMADPlugin.dll` and deleted a legacy
AppData copy without retaining an immutable previous payload. Router packages
contained an example configuration; deployed topology remained operator-owned
but had no versioned activation procedure. No component had a durable pending
activation record, failed-candidate recovery, or exact previous-version check.

The executable disproof check for this slice is an A → B → A transition through
the deployment engine, including failed B health, corrupt packages, interrupted
activation, retained operator state, and real runtime/router processes. A failed
restoration must remain pending/failed and must never report successful rollback.

## Versioned release deployment

Use Python 3.11 or newer and the reviewed deployment tools from the same release
set. Verify the downloaded tooling ZIP against `SHA256SUMS` before extracting it;
verify the manifest checksum too. Obtain both through a trusted operator channel.
SHA-256 correlates bytes and detects corruption; it does **not** authenticate a
publisher. No signing-key distribution or release signing is implemented.

`release-manifest.json` identifies one release set, not a distributed transaction.
It binds the tag/dev identity, full NOMAD/MAVSDK source SHAs, component versions,
platform/architecture, package hashes, required contents and supported protocols.
The supported set contains Linux x86-64 core, Windows x86-64 core, Windows
standalone router and an AnyCPU plugin targeting Mission Planner 1.3.83. Runtime
IPC v1 and router management v1 remain distinct compatibility contracts. This
slice permits their reviewed v1 combinations; operators should deploy matching
release sets and record each host's actual component status. There is no mixed
version dependency solver or cross-host atomicity claim.

Each component has its own protected deployment root:

```text
<root>/<core|router|plugin>/
    deployment.json
    releases/<release_version>/
        package             # original verified archive, retained for rollback
        record.json         # identity and exact extracted file hashes
        payload/            # immutable release files
```

`deployment.json` records active/previous releases, source/package identity,
paths, timestamps and pending activation intent. It contains no credentials and
is separate from the runtime command audit. POSIX records are mode 0600; release
files are read-only. Windows tooling protects the root ACL for the operator,
Administrators and SYSTEM, with LocalService read/execute, and rejects unsafe
existing ACLs and reparse points. Provision this root beneath an administrator
controlled parent; ordinary users must not be able to rename its parent.

The common command is `python -m scripts.release.deploy` from the extracted
tooling directory. Every action takes `--root <absolute-root> --component
<core|router|plugin>`. These examples use placeholders, not production settings:

```sh
python -m scripts.release.deploy verify --root <root> --component core --manifest <manifest> --package <core-package>
python -m scripts.release.deploy stage --root <root> --component core --manifest <manifest> --package <core-package>
python -m scripts.release.deploy status --root <root> --component core
python -m scripts.release.deploy activate --root <root> --component core --release <release> --adapter systemd --config <config>
python -m scripts.release.deploy rollback --root <root> --component core --adapter systemd --config <config>
python -m scripts.release.deploy recover --root <root> --component core --adapter systemd --config <config>
python -m scripts.release.deploy cleanup --root <root> --component core --release <unused-release>
```

`verify` has no deployment side effects. `stage` verifies the full manifest,
native platform, digest, embedded identity and required files before publishing
a new release directory. It rejects traversal, links, ambiguous Windows names,
duplicate names, oversized archives, operator state and qualification executables.
A version already present with different bytes fails. Staging never stops an
active process. Cleanup is explicit and refuses active, previous and pending
versions. It never traverses operator state or deletes audit evidence.

### Linux core

Keep `/etc/nomad/runtime.json`, its credential map and `/var/lib/nomad` audit
history outside `<root>`. Provision the existing PR56 systemd unit using
`install_systemd.py install --executable <root>/core/current/bin/nomad-runtime
--config <config> --state <audit-parent> --user nomad`. The service account must
be able to traverse the protected program directories. Registration does not
start or enable the service. Stage A, provision the unit, then explicitly
activate A. For an existing versioned pointer, stage the exact matching release
and use `adopt` after checking its running version and health.

Activation stops systemd and verifies its stopped state, atomically replaces
`core/current` with a symlink to the complete candidate payload, starts systemd
and checks read-only IPC hello/status, implementation version, readiness and no
authority owner. PR56 SIGTERM closes final-send admission. Upgrade and rollback
each start a fresh incarnation; old contexts fail and fresh authentication and
explicit admission are required. The service never receives authority from the
deployment tool.

### Windows core

Use `--adapter scm`, with the same stage/status/activate/rollback/recover/cleanup
actions. No symlink privilege is required. SCM must first be provisioned through
PR56 `Manage-NomadRuntime.ps1` to point at staged A's absolute
`payload/bin/nomad-runtime.exe` and external protected configuration. Explicitly
start that service and `adopt --release <A>` to establish the initial verified
record. Adoption refuses a different SCM executable/config path. Activation
stops and waits for SCM STOPPED, changes only the executable command to the
candidate's complete versioned path, starts and verifies read-only IPC health.
Service account/recovery policy and external settings are preserved. Rollback
restores the exact previous executable path and verifies the restarted runtime.

### Router and plugin

The [router deployment guide](../infra/transport/ground_router/README.md) describes
the independent foreground supervisor and Windows scheduled-task start command.
Use `--component router --config <external-router.json>
--router-start-command <operator-owned-argv.json>` for activate/rollback/recover.
Before stopping the deployed router, candidate preflight runs a separate router
against loopback-only endpoints and requires hello, management status, expected
version/protocol and safe shutdown. This test router grants no authority and
does not connect to physical links. Authoritative endpoints, consumer ports,
preferred link and topology stay in the unchanged external configuration.

Use `--component plugin --mission-planner <installation>` for activation,
rollback and recovery. The [plugin packaging guide](../mission_planner/packaging/README.md)
gives the PowerShell wrapper commands. Mission Planner must be closed; the tool
never kills it. Only `plugins/NOMADPlugin.dll` is atomically replaced. The target
Mission Planner version and PE DLL are checked before replacement, and the final
DLL embedded version/source and final hash are checked. Settings, `CoreClientCredential`, Mission Planner
configuration, other plugins and legacy AppData files remain untouched. Plugin
health qualifies replacement bytes, not GUI startup.

On a common Windows groundstation, stage and verify all candidates first. Close
Mission Planner deliberately, activate core, then router, then plugin, verifying
each before proceeding. Core can be ready/degraded without a router or vehicle.
Reopen Mission Planner only after the component statuses match the intended set;
fresh authentication/admission remains mandatory. On failure, stop the sequence
and restore affected components individually. Across machines, coordinate this
same procedure per host and retain each host record; no all-or-nothing operation
is promised.

### Failure, interruption and host acceptance

The engine durably writes `activating` with candidate and previous records before
stopping anything. Any ordinary failure after that boundary attempts restoration:
write `rollback_pending`, stop candidate, validate retained previous bytes,
restore its pointer/payload, start and health-check previous, then record
`rolled_back`. If restoration or its record write fails, pending intent remains
and `recover` deterministically restores the recorded previous version. A missing
or corrupt previous version fails clearly; restore its reviewed archive before
retrying. An initial activation failure leaves no committed active release.
Never interpret a nonzero activation command as success even if A was recovered.

After a host/tool crash during a pending transition, run `status`, then `recover`
before another activation. Supervision may have restarted a component during
that interruption; recovery stops it and restores the journal's exact previous
release. Temporary `.stage-*` trees left by a staging crash are never active and
may be removed by an operator after confirming no staging tool is running.

Directory publication, POSIX symlink switches and individual state replacements
use same-filesystem atomic rename. Windows deployment records use `ReplaceFileW`
with readers permitting delete sharing. A per-record Windows kernel mutex guards
recovery checks, existence probes, snapshot opening and replacement/backup cleanup
across processes and sessions; readers release it once their snapshot is open.
A stalled writer causes a clear failure after a 30-second guard wait; no record
read or repair is retried. Process exit releases ownership, so a reader then
checks the records left on disk. A failed replacement attempts to restore
its retained old record before reporting failure. If that restoration is blocked,
the deterministic `.deployment.json.previous` backup makes commands fail closed.
Any retained backup blocks mutations; reads require a present, valid primary and
never substitute the backup. A missing primary with a backup requires repair,
including when checking whether this is a new deployment.
Repair storage permissions, stop the affected deployment tool/supervisor, preserve
both records as evidence, and restore the backup to `deployment.json` if the target
is absent; if both exist, inspect them and retain the record with pending intent.
Remove the backup only after recording that repair, then run `status` and `recover`.
The same procedure applies to other `.NAME.previous` record backups.
POSIX writes synchronize files and parent
directories. Windows uses flushed file writes and atomic replacement, but cannot
promise directory/power-loss durability equivalent to POSIX fsync. SCM changes,
process lifecycle and plugin replacement are separate steps guarded by the
pending record; the overall transaction is recoverable, not one filesystem
atomic operation. Multi-process and multi-host deployment is not atomic.

The router supervisor retains its lifetime lock after a record-write failure until
its owned child has exited and the final stopped/failed record can be written.
If storage remains unavailable or shutdown is refused, activation/rollback times
out with a pending journal. Repair storage or shutdown on that host, then run
`recover`; a stale running marker is never accepted as proof of process exit.

Hosted/unprivileged fixtures prove byte identity, state preservation, real child
runtime/router transitions, authority reset and recovery failures. Privileged
host acceptance still requires actual systemd/SCM permissions, account access,
Windows task supervision, protected parent ACLs, stop timeouts, locked DLLs,
power-loss recovery and operator restart procedures. Software rollback evidence
does not establish aircraft readiness or physical flight safety.

This guide covers the supported ground deployment. For build and qualification
workflows, see [Development](development.md) and [Qualification status](qualification.md).
For the current component boundaries, see [Architecture](architecture.md).

## Processes and endpoints

### Supported deployment matrix

| Profile / placement | Runtime host and supervisor | Client / router boundary |
| --- | --- | --- |
| `groundstation_minimal` | Windows groundstation: native SCM service | Mission Planner requires runtime on the same Windows host; standalone ground router remains separate |
| `groundstation_gpu` | Same ground runtime placement as minimal | Optional GPU/ROS workloads do not own commands or supervise runtime |
| `onboard_companion` | Linux onboard CLI deployment: systemd; Windows Mission Planner deployment retains a ground runtime | Loopback IPC cannot reach an onboard runtime from Mission Planner; do not run both command owners for one vehicle |
| Development / deterministic peers | Foreground console, either OS | No privileged service installation required |

Profiles describe optional compute and endpoint defaults, not runtime placement
discovery. The onboard profile's wildcard UDP endpoint must be reviewed for the
chosen host; a ground runtime uses the standalone router's loopback endpoint.
The checked-in standalone ground router requires Windows; a Linux groundstation
profile is not currently supported. Linux supervision supports the onboard
local-client placement and software-only peers. Optional compute profiles do not
constitute OS or hardware qualification.
The existing `infra/systemd/install.sh` manages optional aircraft router/media/ROS
units from a checkout, and `scripts/setup/setup_service.sh` delegates to it.
Neither currently supervises the production runtime. Keep that optional setup
separate from the packaged runtime service and from the Windows ground router.

Run one `nomad-runtime` process for each vehicle connection. It owns the
long-lived MAVSDK connection, vehicle policy, authority/request lifecycle, and
runtime IPC listener. The runtime does not load `config/nomad.env` itself.
Use the packaged systemd or native Windows SCM deployment below. Do not start a
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

Use the example environment file for console development. Services use a
protected external JSON configuration, loaded with `--config`. Keep credentials local. Runtime IPC is
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

Loading a profile identifies the intended file paths and reports a final result
for each target before the overall completion message:

| Target result | Meaning |
|---|---|
| `[APPLIED] env` | `config/nomad.env` was atomically replaced with the selected template and canonical MAVLink endpoint; existing `NOMAD_API_KEY` and `NOMAD_CLIENT_CREDENTIAL` values were preserved. |
| `[APPLIED] mission_planner` | Profile-owned MP settings were synced: `ActiveProfile`, `VideoUrl`, and supported legacy migration. Unrelated settings and the separately provisioned `CoreClientCredential` remain. Retired settings, including `CoreApiKey`, are removed. |
| `[SKIPPED] mission_planner: config path unavailable` | MP sync is optional when neither `NOMAD_MP_CONFIG` nor `LOCALAPPDATA` supplies a path. Only env was applied; exit status is 0. Set `NOMAD_MP_CONFIG` to require a specific MP target. |
| `[SKIPPED] ... unchanged` or `... rolled back` | That target was not applied because another target failed, or its change was restored. |
| `[FAILED] ...` | The requested load failed; exit status is 1 and no overall success message is printed. Read both target results before using the configuration. |

A known MP path is an intended target even if the file does not exist yet; the
loader creates its parent directory and config. Unreadable, malformed/non-object
JSON and unsupported legacy settings fail preflight instead of silently skipping
MP. Both outputs and the original env restoration copy are staged in private,
unique sibling files before either config changes. Existing permission bits are
retained. A timestamped `nomad.env.bak.*` backup is kept before env replacement.
If env replacement fails, MP remains unchanged. If MP replacement fails after
env replacement, env is restored atomically (or removed if it was newly created).
Temporary files are cleaned up on handled failures. Backups contain credentials;
keep them private as you would `nomad.env`.

For example, an env-only load reports `[APPLIED] env`, `[SKIPPED] mission_planner`,
then `[OK] Profile load completed` and exits 0. Invalid MP JSON reports
`[SKIPPED] env: Mission Planner preflight failed; unchanged`,
`[FAILED] mission_planner`, then `[FAILED] Profile load` and exits 1. An MP commit
failure reports `[SKIPPED] env: rolled back` and `[FAILED] mission_planner`.
If rollback itself fails, `[FAILED] env: changed; rollback failed` explicitly
requires restoring the reported backup before use (or removing a newly created
env manually). The env backup remains available.

Run `load` while configuration editors and other loaders are stopped. The two
files do not form a crash-atomic transaction: process termination, power loss,
concurrent edits, or a second filesystem failure during rollback can require
manual recovery. Newly created parent directories may remain after a failed
load. Replacement files belong to the invoking user and inherit directory ACLs;
the loader preserves permission bits but does not provision ownership or ACLs.

Loading does not start processes, prove that hardware is present, or
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
not installed. Templates, explicit registration helpers, a blank service JSON
example and this guide are in `share/nomad/`. Package/prefix installation never
registers, starts or enables a service, or overwrites operator secrets/state.
Linux runtime/client binaries require the compatible system OpenSSL
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

Mission Planner reports vehicle mutations using the
[runtime outcome contract](runtime-ipc.md#vehicle-mutation-outcomes).
Rejected means no eligible vehicle send; failed means a definite unsuccessful
attempt; interrupted and unknown mean the final vehicle effect is uncertain.
Do not retry an uncertain release or retract automatically. Observe the payload
and follow the reviewed procedure before deciding on another action. Payload
indicators record commanded state only; successful release/retract commands do
not verify physical release or retraction.

## Managed runtime configuration

Copy the packaged `share/nomad/lifecycle/runtime.example.json` outside the
installation tree. Values are strings: set the endpoint, IPC port, absolute
credential-file and audit-directory paths, and independent API gate. The protected
`--config` loader rejects unknown/duplicate keys, non-string values and missing
required settings. It replaces inherited runtime settings, so a missing API gate
stays disabled. Optional fence/velocity settings use their console environment
names. JSON is data, never shell code. Keep per-client tokens in the separate
protected identity/token map; both files load once per process.

The stable service account must own configuration, credential file and audit
directory. Linux files require mode 0600 and state directories 0700. Windows
uses `NT AUTHORITY\LocalService`, with owner/DACL restricted to that account,
SYSTEM and administrators. Protect parent directories against untrusted replacement
as well. Keep binaries administrator-owned but readable/executable by the service
account, and keep configuration/state outside the package prefix. Use local
persistent storage with durable flush support, not network shares. Provision the
audit parent directory first. Runtime startup verifies credential and audit
protection; registration alone is not deployment readiness.

## Linux systemd procedure

Use a Linux systemd host, Python 3 and the build's OpenSSL Crypto runtime library.
These conventional paths are operator choices, not compiled-in paths:

```sh
sudo useradd --system --user-group --home-dir /nonexistent --shell /usr/sbin/nologin nomad
sudo install -d -o nomad -g nomad -m 0700 /etc/nomad /var/lib/nomad
sudo install -o nomad -g nomad -m 0600 /opt/nomad/share/nomad/lifecycle/runtime.example.json /etc/nomad/runtime.json
```

Generate a separate `/etc/nomad/clients.json` identity/token map using independent
`secrets.token_hex(32)` credentials in a secure provisioning tool/editor, without
printing them. Set owner `nomad:nomad` and mode 0600. Provision each client's token
separately. Edit the protected runtime JSON to reference that file and
`/var/lib/nomad/audit`. For the onboard profile, review its
`NOMAD_MAVLINK_ENDPOINT` (`udpin:0.0.0.0:14550`) and the separately supervised
aircraft-side router's output; the example's ground loopback port is not an
onboard deployment default. Keep IPC loopback-only and deliberately set the API gate if
actuation is wanted. The account must traverse all these paths. `ProtectHome=yes`
intentionally excludes home directories. Render for review, then register:

```sh
python3 /opt/nomad/share/nomad/lifecycle/install_systemd.py render --executable /opt/nomad/bin/nomad-runtime --config /etc/nomad/runtime.json --state /var/lib/nomad --user nomad
sudo python3 /opt/nomad/share/nomad/lifecycle/install_systemd.py install --executable /opt/nomad/bin/nomad-runtime --config /etc/nomad/runtime.json --state /var/lib/nomad --user nomad
sudo systemctl start nomad-runtime.service
systemctl status nomad-runtime.service
journalctl -u nomad-runtime.service
/opt/nomad/bin/nomad status
sudo systemctl stop nomad-runtime.service
sudo systemctl restart nomad-runtime.service
```

Configure the client's IPC port when it differs from default. The foreground unit
has no router dependency or network-online readiness assertion. Hardening removes
capabilities, isolates temporary files/devices and restricts writes to `--state`.
Put audit beneath that path, with credentials elsewhere. Custom users require a
matching primary group. Registration does not enable boot startup; explicitly run
`sudo systemctl enable nomad-runtime.service` only after provisioning if desired.

Upgrade by stopping, preserving secrets/evidence, installing the new package and
re-registering if paths changed, then starting. Never overwrite `/etc/nomad` or
`/var/lib/nomad`. Unregister with:

```sh
sudo python3 /opt/nomad/share/nomad/lifecycle/install_systemd.py uninstall
```

This disables/stops and removes only the unit, retaining secrets and audit history.
Remove the package prefix separately if wanted.

## Windows native service procedure

Mission Planner needs the Windows core package on the same host.
`nomad-runtime --service --config <absolute-path>` connects to native SCM;
without `--service`, console mode remains available. RUNNING is reported only
after protected config, credentials, audit and IPC binding initialize. The binary
never registers itself; no third-party service wrapper is needed.

In elevated PowerShell create `C:\ProgramData\NOMAD`, copy the packaged
`runtime.example.json` there, and provision a separate `clients.json` map with
independent random tokens. Edit the JSON with absolute credential/audit paths,
loopback endpoint/IPC configuration and the deliberately configured API gate.
Choose an administrator-controlled binary prefix such as `C:\NOMAD`:

```powershell
$helper = 'C:\NOMAD\share\nomad\lifecycle\Manage-NomadRuntime.ps1'
& $helper -Action Protect -Executable 'C:\NOMAD\bin\nomad-runtime.exe' -Config 'C:\ProgramData\NOMAD\runtime.json'
& $helper -Action Plan -Executable 'C:\NOMAD\bin\nomad-runtime.exe' -Config 'C:\ProgramData\NOMAD\runtime.json'
& $helper -Action Install -Executable 'C:\NOMAD\bin\nomad-runtime.exe' -Config 'C:\ProgramData\NOMAD\runtime.json'
Start-Service nomad-runtime
Get-Service nomad-runtime
sc.exe query nomad-runtime
Get-WinEvent -FilterHashtable @{LogName='Application'; ProviderName='nomad-runtime'} -MaxEvents 10
& 'C:\NOMAD\bin\nomad.exe' status
Stop-Service nomad-runtime
Restart-Service nomad-runtime
```

`Protect` explicitly provisions owner/DACL for config, credentials and audit
directory, refusing reparse points. Run it before first start. It does not
recursively rewrite prior journals; retain the same identity on upgrade.
Give LocalService read/execute access to the binary prefix; avoid user-profile
paths. Configure Mission Planner's `CoreClientCredential` separately under the
operator account. Do not share LocalService's full map with clients.

Registration passes only executable/config paths on argv, never tokens or the
API gate. Startup errors report service-specific code 78 and safe categorical
Application events. Console startup under the service identity provides detailed
initialization diagnostics. Audit failure is also visible through status. STOP
and SHUTDOWN immediately close the existing authority/final-send gate, then report
STOP_PENDING while the main thread drains (30-second budget). The callback never
calls concurrent `Runtime::stop()` operations or requests a flight maneuver.

Upgrade by stopping, retaining operator state, replacing the package, updating
registration if needed, then starting. Install refuses a running service.
Unregister with elevated `& $helper -Action Uninstall`; it waits for stop and
deletes only the SCM registration, preserving credentials, journals and event
evidence. Default startup is demand/manual; explicitly use
`sc.exe config nomad-runtime start= delayed-auto` after provisioning if wanted.

## Lifecycle, recovery and health

| Lifecycle | Meaning / observation |
| --- | --- |
| configured | Operator files/registration exist; not a readiness claim |
| starting | Credential, audit and IPC initialization; SCM START_PENDING / systemd process startup |
| ready | IPC/audit healthy, transport open and fresh vehicle heartbeat; derived `status.lifecycle` |
| degraded | IPC available but audit, transport or fresh vehicle state unavailable |
| stopping | Existing gate closed, owner cleared, generation invalidated; workers drain |
| stopped / failed | Process exited; manager exit status/journal distinguish clean stop from failure |

Lifecycle projects existing state; it is not another authority state machine.
Manager running means process availability, even when runtime is degraded.
Inspect read-only `nomad status`: `runtime_ready` (IPC), `audit_healthy`,
`mavsdk_connection_open`, `vehicle_session_established`, `vehicle_connected`,
`telemetry.heartbeat_fresh`, `actuation_enabled` and `authority_owner` separately.
HMAC authentication is request-bound, not a connected-client lease: a fresh HELLO
server proof establishes client credential availability; status does not fabricate
an authenticated-client count. Process health never proves aircraft controllability
or physical flight safety.

| Failure | Process/recovery policy | Authority/mutation result |
| --- | --- | --- |
| Invalid/inaccessible config/credentials, audit history/permissions/lock failure, IPC conflict | Startup exits 78, no automatic retry | No vehicle mutation |
| Unexpected crash | systemd waits 5 s; at most 3 starts/120 s. SCM waits 5 s then 30 s, then no action; reset after 24 h | New incarnation, no owner; fresh authentication/admission |
| Clean service stop | No recovery restart | Closed gate and durable shutdown evidence where possible |
| Shutdown evidence unavailable | Exit 74; systemd may retry before audit validation refuses; SCM reports failure without recovery | Closed gate; preserve evidence/investigate |
| Router/endpoint unavailable, aircraft absent/lost/returns | Stay running/degraded; worker retries | Loss revokes/inhibits; reconnect never restores owner |
| Audit failure while running | IPC stays available/degraded; no process restart | Latched mutation inhibition, final-send admission denied |

SCM `failureflag 0` applies recovery to unexpected process death, not a reported
STOPPED failure; permanent startup errors stay failed. Further crashes repeat the
last no-action entry. systemd excludes 78 with RestartPreventExitStatus; after
repairing configuration/start-limit failure, reset-failed and explicitly start.
Recovery is unrelated to flight policy. See [systemd semantics](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html)
and [SCM failure actions](https://learn.microsoft.com/en-us/windows/win32/api/winsvc/ns-winsvc-service_failure_actions_flag).

Orderly stop closes mutation/final-send admission under the existing authority
lock, clears owner and advances generation, stops IPC acceptance/drains clients,
joins the connection worker, disconnects MAVSDK, appends/flushes runtime_shutdown
where possible, releases audit lock and exits. Client sockets close during drain
before transport disconnect; the send fence precedes both. A send admitted before
the fence may complete; queued/retrying sends cannot cross the closed gate.
Runtime v1 exposes no active velocity stream: stop adds no zero/LAND/RTL command
and never bypasses transport fencing.

Each replacement validates preserved history and creates a new incarnation
journal, never restoring owners/caches or replaying unmatched intent. Vehicle
session must be established anew; old incarnation/session/generation/request
context is invalid. Supervisor restart is neither handback nor admission.
Credential rotation requires stop, preserved evidence, protected file replacement
and separate client provisioning, then restart and fresh authentication/admission.

The standalone ground router retains separate operator/supervisor ownership.
These helpers supervise only runtime, have no router-first ordering and never
restart the router. Optional aircraft router units remain separate. Neither
router nor runtime restart grants authority. Software qualification uses peer
traffic absence/return; privileged host registration, boot recovery and deployed
upgrade/rollback remain operator acceptance checks.
