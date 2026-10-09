# NOMAD contributor rules

NOMAD is C++20. Production: Mission Planner/installed CLI -> authenticated typed IPC
-> persistent runtime -> vehicle/safety policy -> one MAVSDK command transport ->
router -> ArduPilot. Frontends own input/presentation, never reusable policy.
ArduPilot owns stabilization, navigation execution and native failsafes.

- Read applicable instructions. Trace the owning implementation, callers and focused
  tests before editing; state material assumptions and an executable falsification check.
- Prefer deletion and standard-library/local code. No speculative interfaces, factories,
  registries, service locators, fallback chains or parallel vehicle command paths.
- Write for a first-year engineering student: one job per named helper, early returns,
  shallow indentation, explicit multiline bodies; aim for 40 logical lines/function,
  split responsibilities around 500 lines/file, use 120-column source lines.
- Use RAII and explicit ownership. Keep public headers in `include/nomad`, implementation
  in `src`; keep MAVLink packing in the transport. No global mutable ownership.
- Validate external input. Preserve fail-closed behavior, fresh telemetry, authentication,
  replay/session/generation fencing, command admission, concurrency and durable audit.
  An ACK is not physical completion; uncertain mutations must never be retried blindly.
- Tests must establish known state, perform one action, observe authoritative state and
  assert independent conditions with deadlines and diagnostic messages. Retain invalid,
  boundary and fault cases. Hardware-free tests cannot qualify physical outcomes.
- Parallelize independent reads only; serialize edits/shared-state commands. Keep at most
  one active checklist item and report material findings. Review the final diff.
- Run the focused check, then all relevant [development checks](docs/development.md).
  Keep full Copter/QuadPlane, authority/replay, actuator, router and release-safety gates.
- Never expose/persist secrets, raw transcripts, temporary infrastructure state or real
  deployment identifiers. Use placeholders; `config/nomad.env` is ignored.
- No commit, push, deployment, flash, erase/reset or runtime infrastructure changes without
  explicit user authorization. Build/test must not deploy; staged installs stay in build/.
- Use short unprefixed branches; never `codex/`. Commit subjects: `[part,sub-part] Imperative
  description`, under 72 characters. One logical change per PR; do not merge without request.
- Keep guidance here. Canonical docs: PRD/source requirements, architecture, development,
  operations, safety, qualification, IPC and dependency provenance; TODO is outstanding work.
  Do not retain completed-phase reports or duplicate component setup/architecture guides.
- Record deliberate debt as `debt: <ceiling>; revisit when <measurable trigger>; then <upgrade path>`.
