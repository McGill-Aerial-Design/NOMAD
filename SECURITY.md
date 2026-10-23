# Security policy

## Current trust boundary

NOMAD runs inside the local Mission Planner operator process and uses its selected
connection and native APIs. Its custom controls validate the captured target and
local input lifetime. The plugin-owned router exposes its MP consumer on loopback; physical
telemetry links and host/controller configuration need separate review.

Protect the ground computer, plugin installation, operator profiles, firmware and
actual network endpoints. Saved video/connection URLs may contain sensitive details;
review exports before sharing. A VPN provides network access control, not permission
to control an aircraft. Native MP/mission/RC actions are outside NOMAD's local payload
interlocks. Software test results and ACKs do not establish physical safety.

See [operations](docs/operations.md), [architecture](docs/architecture.md) and
[safety](docs/safety.md) for implemented controls and limits. Keep secrets, raw
transcripts and deployment identifiers outside source control and public reports.

## Reporting

Report command injection, credential leakage, unsafe replay or remote execution
privately to repository maintainers through their configured private security
reporting channel. Avoid public disclosure of exploitable details, credentials,
aircraft locations or private datasets. If no private channel is available, ask
maintainers for one without publishing the vulnerability.
