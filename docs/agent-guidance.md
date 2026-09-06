# Agent and engineering guidance

This document records the repository guidance adopted from the external
`agent-knowledge` collection. It complements `AGENTS.md`; it does not replace the
canonical product, architecture, migration, or safety documents.

## Scope and authority

- `AGENTS.md` is the primary repository instruction file.
- Deeper instruction files apply to files below their directory when they exist.
- Project-local rules override reusable external guidance.
- Stable product and safety decisions belong in `docs/prd.md`,
  `docs/architecture.md`, `docs/migration.md`, and `docs/safety.md`.
- The working plan and task ledger are `PLAN.md` and `TODO.md`.

## Agent operations

Agents begin with the concrete request, failure, owning symbol, or nearest test.
They trace the controlling implementation, callers, and focused tests before
editing. They state material assumptions and define a check that could disprove
the proposed fix.

Read-only research and review remain separate from implementation. Edits and
stateful commands are serialized. Independent reads may be parallelized. Agents
must not expose or persist secrets, raw transcripts, temporary infrastructure
state, or machine-specific identifiers.

No agent may push, publish, deploy, flash, erase, reset, or perform another
potentially destructive operation without explicit authorization.

## Practical engineering

Use the smallest sound solution in this order:

1. Do nothing when the behavior already exists or the need is speculative.
2. Reuse an existing local helper or pattern.
3. Use the standard library.
4. Use a native platform capability.
5. Use an already-installed dependency.
6. Write the minimum new code required.

Fix the root cause at the narrowest owning boundary. Preserve public interfaces
and test contracts unless the task explicitly changes them. Do not weaken tests,
add speculative flags or abstractions, or perform unrelated cleanup.

Run the cheapest executable check after the first substantive edit. Broaden
validation according to risk: focused tests for local behavior, affected unit and
integration tests for shared contracts, and end-to-end or authoritative endpoint
checks for cross-layer and hardware behavior.

## Human-readable implementation

Write code for reading under pressure, not for minimum line count.

- Keep control flow top-to-bottom with guard clauses and shallow indentation.
- Put one logical operation on each line.
- Name intermediate values and meaningful operations.
- Use helpers only for real operations, coherent responsibilities, or meaningful
  duplication removal.
- Prefer plain data and ordinary standard-library types over unnecessary class
  hierarchies, factories, wrappers, or registries.
- Name functions for the object and outcome, such as `read_connection_state` or
  `validate_servo_command`.
- Keep query functions free of side effects and make mutating names reveal their
  effect.
- Keep comments and docstrings short; explain why a non-obvious constraint exists.

For polling, use explicit bounded deadlines and named state variables. Do not add
fallback chains or defensive handling for states that required dependencies cannot
produce.

## Readable tests

A test file is diagnostic documentation. Each important test should read as:

1. establish known state;
2. perform one meaningful action;
3. observe authoritative state;
4. assert independent conditions;
5. restore state when required.

Use narrow fixtures, named helpers, explicit deadlines, and assertion messages
that identify expected and observed values. Prefer authoritative read-back over a
sender success message or command acknowledgement. Do not hide policy, branching,
or unrelated work inside a generic harness.

Retry only genuinely probabilistic operations, with a bounded documented reason.
Do not wrap deterministic failures in retries or arbitrary sleeps when a readiness
condition exists.

## Evidence-driven debugging

When a failure crosses layers, define the expected state sequence and an
independent witness for each transition. Establish a known-good baseline, change
one variable at a time, sample timing-sensitive state more than once, and find
the first boundary where observed behavior diverges.

Prefer evidence in this order:

1. authoritative endpoint state or read-back;
2. independent liveness or progress counters;
3. protocol traces and exact-artifact debugger state;
4. component logs;
5. acknowledgements and local success messages.

Record disproved hypotheses and remaining uncertainty. An acknowledgement proves
receipt, not completion.

## Hardware and SITL

Before changing a stateful endpoint, confirm target identity, power and topology,
current ownership, and a baseline. Serialize access. After reset, flashing, power
changes, or mode transitions, close stale handles, wait for re-enumeration, resolve
current identities, and re-establish invalidated protocol state.

Use existing build and deployment tasks. Verify the exact artifact and endpoint,
then read back authoritative state with an independent progress witness where
possible. Keep credentials, hostnames, device paths, and calibration values in
local ignored configuration.

## Technical debt

A deliberate simplification must identify its ceiling, measurable revisit trigger,
and upgrade path:

```text
debt: <current ceiling>; revisit when <measurable trigger>; then <upgrade path>
```

Do not use an unexplained TODO as a substitute for an owned decision. Review
entries during migration milestones and remove them when the replacement exists.
