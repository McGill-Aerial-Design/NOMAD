# Operations

Operate only an approved aircraft/firmware/link profile with separate physical
safety acceptance. [Safety](safety.md) and [qualification](qualification.md) define
what software evidence cannot guarantee. Never disable ArduPilot failsafes.

## Processes and endpoints

| Process | Default local endpoint | Owner |
|---|---|---|
| `nomad-runtime` | IPC TCP 127.0.0.1:14611; MAVLink UDP 127.0.0.1:14601 | Vehicle commands, authority, actuator policy, audit/recovery |
| Ground router | Management TCP 127.0.0.1:14610 | Physical links, routing, health, deduplication |
| Mission Planner | Router telemetry UDP 14600 | UI/input/presentation; receive-only router consumer |

Use [ground router configuration](../infra/transport/ground_router/example.json)
and its [protocol README](../infra/transport/ground_router/README.md). Do not have
two processes bind a physical-link or consumer port. Runtime/CLI clients remain
loopback-only. Consumer IDs and routing priority do not authenticate a writer or
establish physical flight authority.

## Current configuration

Managed services read protected [runtime JSON](../infra/runtime/runtime.example.json)
with `nomad-runtime --config <absolute-path>`. Unknown keys, relative protected paths,
unsafe permissions/reparse points and invalid values fail closed. Console mode may
use the retained [environment template](../config/nomad.env.example). Do not mix
competing sources of defaults. The runtime alone receives the aircraft endpoint.

Provision independent random 32-byte client credentials, mapped to `nomad-cli` and
`mission-planner`, in a protected local JSON object. Tokens must never be logged or
committed. POSIX protected files are owned by the service user with mode 0600 and
trusted parents; use the Windows helper to protect/validate ACLs. Keep the nonempty
actuation-enable gate distinct from authentication. Clients receive only their own
credential. Credential rotation requires stop, protected replacement and fresh
client authentication/admission after restart.

The durable audit directory and actuator JSON belong under the service's writable
state root; credentials/configuration live outside it. Use current
[actuator schema](../config/actuators.example.json) and validate under the service
account with `nomad-runtime --validate-actuators <file>`. Configuration replacement
through IPC requires protected storage, revision fencing and safe output recovery.
Never replace operator data from a sample or an old settings export automatically.

Mission Planner uses current JSON only. Unknown/retired fields, duplicate properties
and invalid input mappings are rejected. Invalid primary settings stop plugin loading;
they are not rewritten or replaced by defaults/backup. Defaults apply only when no
saved settings exist; the backup is used only when the primary is missing. Preserve
unsupported files, review them and create current settings explicitly. Portable
exports omit the client credential. Advisory map files/presets also reject old policy
fields; outlines never enforce an aircraft boundary. Mission Planner's native tools
own flight-log analysis.

## Copter LAND engagement

With the runtime provisioned and an explicitly admitted client, `nomad land` requests
Copter LAND mode through authenticated IPC. For a fresh ownerless runtime:

```sh
nomad status
nomad admit
nomad land
```

Use explicit handback after prior revocation/session loss. Mission Planner provides
**Engage Copter LAND** in Settings > Core beside its existing authority controls;
save settings and admit that client's authority first. Neither client admits itself.

Success says **LAND mode observed; touchdown not verified**. Continue observing the
aircraft independently; this action is not termination and does not qualify hardware.
A three-second engagement timeout can leave ArduPilot executing LAND. Inspect fresh
telemetry after an unknown/interrupted result and never replay blindly. Revoke inhibits
new NOMAD sends; it does not undo an accepted LAND or establish physical pilot control.
The [IPC contract](runtime-ipc.md#copter-land-engagement) defines exact outcomes and limits.

## Aircraft serial router

When an onboard host fans FC serial traffic onto the network, use standard
`mavlink-routerd` with a reviewed explicit serial device and destination.
[router.conf template](../infra/transport/mavlink_router/main.conf) and the
[systemd unit](../infra/systemd/nomad-mavlink-router.service) are independent of the
ground router. Provision `/etc/nomad/router.conf`, device permissions and the `nomad`
service account; install/enable/start the reviewed unit with standard systemd commands.
It runs the daemon in the foreground and retries through systemd. Missing/renamed
serial devices require the operator's stable device path; there is no automatic peer
or serial-device selection. Use standard Tailscale tooling if that network is chosen.
There is no NOMAD service dispatcher or profile generator.

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
`/var/lib/nomad/audit`. For actuator deployments, review the current actuator schema,
then provision it without changing the private Mission Planner settings export:

```sh
sudo install -d -o nomad -g nomad -m 0700 /var/lib/nomad/actuators
sudo install -o nomad -g nomad -m 0600 <reviewed-backend.json> /var/lib/nomad/actuators/actuators.json
```

Set `NOMAD_ACTUATORS_FILE` to `/var/lib/nomad/actuators/actuators.json`. Do not put
it in `/etc/nomad`: the hardened service cannot replace files there. Validate the
protected file under the service account with `sudo -u nomad /opt/nomad/bin/nomad-runtime
--validate-actuators /var/lib/nomad/actuators/actuators.json`.
For an onboard placement, explicitly configure its
MAVLink endpoint and the separately supervised
aircraft-side router's output; the example's ground loopback port is not an
onboard deployment default. Keep IPC loopback-only and deliberately set the API gate if
actuation is wanted. The account must traverse all these paths. `ProtectHome=yes`
intentionally excludes home directories. Render for review, then register:

```sh
sudo -u nomad python3 /opt/nomad/share/nomad/lifecycle/install_systemd.py render --executable /opt/nomad/bin/nomad-runtime --config /etc/nomad/runtime.json --state /var/lib/nomad --user nomad
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
Put audit and actuator state beneath that path, with credentials elsewhere. The renderer
and installer reject writable state paths outside `--state` and symbolic links. Custom users require a
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

For actuator deployments, create a dedicated directory such as
`C:\ProgramData\NOMAD\actuators`, put only the reviewed backend JSON there, and set
`NOMAD_ACTUATORS_FILE` to its absolute path in the runtime JSON. Keep private Mission Planner settings under the operator account. Leave the runtime field
blank only when no configured outputs are wanted.
Use fully qualified paths; drive-relative forms such as `C:runtime.json` or
`\NOMAD\runtime.json` depend on the caller's current drive/directory and are rejected.

`Protect` explicitly provisions owner/DACL for config, credentials, audit directory,
and the optional actuator file and its dedicated parent. It refuses reparse points,
including in ancestor directories, and refuses a shared actuator directory. This
transfers the reviewed backend file from the operator to LocalService and grants the
create/delete access required for atomic runtime replacement. Run it before first start. It does not
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
