# Remaining work

This is the actionable branch-tip ledger. Current evidence and its exact limits
are in [Qualification status](docs/qualification.md); product decisions live in
the [PRD](docs/prd.md). Completed implementation slices and their dates remain
in the [historical migration archive](docs/migration.md).

## Aircraft authority and fault response

- [~] Prove aircraft-wide writer arbitration and physical pilot takeover/handback.
  Software/isolated SITL qualification and a concrete
  [source model / bench procedure](docs/source-arbitration.md) are tracked
  separately from physical completion. The software/disarmed Copter slice
  passed at `aff6d152`; its exact evidence and limits are in qualification status.
  The new runtime scenario checks
  inhibited ownership, output delivery/acceptance, external mode control and
  explicit recovery on Copter; it cannot qualify a physical pilot or decide RC
  priority. Current runtime v1 mutations are unavailable for QuadPlane; qualify
  a future typed QuadPlane operation without widening capabilities for a test.
  Set and verify the production RC/ELRS channel map and FC input priority; test
  native GCS, RC, runtime revoke, lost links and reconnects against the actual
  controller. Runtime generations and retry fencing prove only a software
  request boundary, not which source the aircraft accepts.
- [ ] Define and qualify the complete C2-loss policy across supported aircraft
  and phases. Keep communication loss, pilot takeover, and termination as
  separate fault paths with observed FC state and physical outcomes.
- [x] Resolve the hosted GCS-heartbeat cadence failure in run `36672382568`.
  Verified PR53 final-head run `36744321862`: the heartbeat scenario and all
  downstream velocity/geofence checks passed, as did both full aircraft jobs.
  See the exact head and evidence limits in [qualification](docs/qualification.md).
- [ ] Select, implement and qualify production termination behavior. Include
  activation, latching, reset, external-source interaction and every QuadPlane
  phase required by unresolved Q02. The Mission Planner termination control
  currently reports unavailable and sends no substitute command.
- [x] Establish a durable command/outcome audit record and an authenticated
  client trust boundary. Per-client HMAC proofs bind identity, and the runtime
  synchronizes JSONL intent before execution and outcomes afterward, failing
  closed on audit faults. The nonempty API-key setting remains an actuation gate.
  See [protocol and recovery semantics](docs/runtime-ipc.md#authenticated-local-clients).

## Aircraft and mission qualification

- [ ] Complete authorized hardware/flight qualification for the selected
  aircraft, firmware, sensors, RC/ELRS/LTE links, mass, endurance, failsafes and
  operating procedures. No product profile is hardware-qualified.
- [ ] Repeat and extend the pinned QuadPlane SITL evidence for all required
  transition/recovery, fault and Q02 termination cases, then qualify approved
  behavior on hardware. The existing chain through QLAND is limited to its
  pinned SITL profile and recorded scenarios.
- [ ] Integrate and qualify the complete Task 1 flight; separately establish
  Task 2 tracking, payload interlocks, physical attachment and verified release
  outcomes. A tested command or ACK does not prove physical payload/gimbal action.

## Runtime deployment and release

- [x] Define supported runtime process supervision and lifecycle: packaged
  systemd foreground deployment, native Windows SCM, protected configuration,
  bounded recovery, health separation and fenced shutdown. See
  [operations](docs/operations.md#lifecycle-recovery-and-health) and the
  software-only persistent-history restart qualification.
- [x] Approve measurable build-size, memory, startup and CI-time budgets for
  the pinned MAVSDK dependency and retain comparable qualification artifacts.
  See the [software resource evidence and limits](docs/qualification.md#software-resource-budgets),
  versioned policy and separate Linux/Windows hosted artifacts.
- [x] Qualify software-only versioned installation, activation and rollback
  for the core, standalone ground router and Mission Planner plugin. Exact
  A → B → A binaries, failed candidates, persistent operator state and fresh
  runtime authority are exercised without hardware. See
  [release lifecycle evidence and limits](docs/qualification.md#versioned-release-lifecycle).
  Privileged systemd/SCM registration and actual groundstation acceptance remain
  part of deployment-profile qualification below.
- [ ] Qualify each intended deployment profile with its actual host, endpoint,
  hardware, optional workloads and failure states. Profile tests validate
  templates only. GPU/Jetson images require compatible hardware and base images.

## Product modules and competition evidence

- [ ] Resolve open organizer decisions Q02â€“Q09 and assign accountable owners
  for requirements, implementation, safety acceptance and retained evidence.
- [ ] Implement the required AEAC 2027 server/traffic contract and demonstrate
  telemetry, stale-data behavior, traffic separation and outage/replay handling.
- [ ] Build the required mission application modules only after their contracts
  and safety gates are approved. The competition task workflows are not part of
  the current installed runtime protocol.
- [ ] Complete system-level security, overload/resource, observability and
  competition release rehearsals with exact source, firmware, profile and
  retained evidence.
