# NOMAD contributor rules

NOMAD is a C# Mission Planner plugin targeting the reviewed host in
`scripts/release/mission-planner-target.json`. Mission Planner owns the aircraft
connection, ordinary flight controls, setup and missions. ArduPilot owns flight
execution and native failsafes. NOMAD adds saved profiles, custom gimbal/payload/HID,
video, native fence draft setup and the plugin-owned link router.

- Verify HEAD, local changes and applicable instructions. Read the owning symbol,
  callers and closest tests before editing. State a concrete falsifying check.
- Plan small: name files, checks, pass/fail criteria and non-goals. Reuse existing
  patterns; no speculative frameworks, extra dependencies or broad cleanup.
- Keep each change useful and narrow. Prefer deleting superseded code to wrappers.
  Write explicit, shallow, human-readable C#; one job per helper, about 120 columns.
- Use Mission Planner's actual reviewed APIs. Do not add an independent transport,
  CLI, service, IPC compatibility layer or duplicate ordinary flight UI.
- Bind custom controls to the selected connection, stream, aircraft IDs and profile
  generation. Fence stale inputs. Keep finite/range/calibration checks, confirmation
  interlocks, input-loss Safe, bounded native repeats and truthful outcomes.
- Native MP commands use host ACK/retry semantics. Already-dispatched helpers may
  finish after Stop/unload; do not claim cancellation or physical completion.
  Preserve ArduPilot failsafes. Do not invent a termination mechanism.
- Tests establish known state, perform one action and observe independent results
  with deadlines and diagnostic messages. Retain meaningful invalid/boundary/fault
  cases; hardware-free tests do not qualify physical outcomes.
- Run focused checks and relevant [development checks](docs/development.md). Report
  passed, failed, skipped and unavailable separately. Review the final diff.
- Parallelize independent reads only; serialize edits/builds/shared-state commands.
  Keep one active checklist item and report material findings.
- Preserve unrelated edits, worktrees, archives, ignored evidence and real operator
  settings. Never expose secrets, raw transcripts or deployment identifiers.
- No commit, push, merge, deployment, installation, flash/reset or infrastructure
  changes without explicit authorization. Build/test never installs the plugin.
- Use short unprefixed branches; never `codex/`. Commit subjects use
  `[part,sub-part] Imperative description`, under 72 characters.
- Keep current contracts in docs, legal notices intact, and plans/TODOs/handoffs/run
  histories in ignored `local/`. Do not accumulate obsolete guides.
- Record deliberate debt as `debt: <ceiling>; revisit when <measurable trigger>; then <upgrade path>`.
