# Security policy

## Current trust boundary

Production clients use loopback typed IPC to `nomad-runtime`, which owns the single
NOMAD vehicle-writing path. Per-client shared secrets authenticate HMAC request
proofs and runtime hello proofs; request identity and source must match the
authenticated configured identity. `NOMAD_API_KEY` remains only an explicit
deployment actuation-enable gate. Authentication alone does not admit authority.
Authority generation, vehicle session, expiry and sequence fence final-send admission.
The runtime durably journals intent before command execution and observed outcomes
afterward. Audit failure inhibits further mutations; software evidence does not
prove physical action. Core library and qualification calls are outside this
installed-client boundary.

This protects against ordinary local processes without the client's credential,
including identity claims and a rogue listener trying to harvest raw credentials.
It does not protect against administrator/root, kernel, credential-reading malware
or physical host compromise. Credential files and local client settings require
OS access protection. Remote authentication and exposed ROS/media endpoint protection
remain outside this slice.
A VPN is an optional network control; it does not authorize commands by itself.
The retained video tool's HTTP controls currently lack authentication.

See [operations](docs/operations.md), [architecture](docs/architecture.md) and
[the safety case](docs/safety.md) for canonical controls and open evidence.
Do not deploy template development credentials or infer security from a config
flag. Independent manual control must be qualified for the selected aircraft;
this plan prescribes no airborne motor-kill action.

## Reporting

Report command injection, authentication bypass, credential leakage, unsafe
replay or remote execution privately to repository maintainers using their
configured private security reporting channel. Avoid public disclosure of
exploitable details, credentials, aircraft locations or private datasets.
Do not invent an issue type or publish a vulnerability if a private channel
is unavailable.

## Operator responsibilities

Restrict command, MAVLink, DDS, media and maintenance endpoints to intended
principals. Protect/rotate actual secrets outside source control, validate
server identity and keep dependency/firmware pairs pinned and qualified.
Preserve sanitized evidence of security failures without recording secrets.
Current development support does not imply a flight-qualified release.
