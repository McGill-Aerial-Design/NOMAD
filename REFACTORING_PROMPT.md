# Reusable AI-Agent Prompt — Refactoring NOMAD

> Paste this file (or point your agent at it) whenever an AI agent refactors,
> extends, or cleans up this codebase. It encodes the house rules. It
> complements [AGENTS.md](AGENTS.md) (project layout, workflows, commit style)
> and defers to [docs/safety/](docs/safety/) (the living safety case) on
> anything flight-related.

---

You are refactoring **NOMAD**, a drone companion-computer (Python, `edge_core/`)
and ground-control-station plugin (C#, `mission_planner/src/`) baseline used by
a student team for the AEAC competition. Parts of this code can **move a real
aircraft and fire a real payload**. Your job is to make the code shorter,
clearer, and more modular **without ever weakening a safety property**.

## Step 0 — Classify before you touch

Every file belongs to one tier, defined in
[docs/safety/partition.md](docs/safety/partition.md):

- **SC (safety-critical)** — can command aircraft motion, payload actuation, or
  suppress a failsafe. Includes `edge_core/safety/`,
  `edge_core/ros_http_bridge/mavlink_velocity.py`,
  `edge_core/services/mavlink/commands.py`, `edge_core/modules/payload/servo.py`,
  and the C# `Control/`, `Payload/`, `Geofence/` command paths.
- **SR (safety-related)** — feeds SC code or the operator's safety picture
  (auth middleware, health monitor, MAVLink connection, VIO freshness, time).
- **NC (non-critical)** — everything else (UI chrome, video, 3D, terminal, docs).

Before editing, state which tier each file you will touch is in. The tier sets
your rules of engagement:

| Tier | You may | You must |
|------|---------|----------|
| NC | Refactor freely, delete dead code, simplify aggressively. | Keep lint/types/tests green. Never add a call into SC except via its public API. |
| SR | Refactor with tests. | Preserve defined failure outputs; keep existing tests passing. |
| SC | Only behavior-preserving refactors, or changes backed by a requirement. | Follow the SC protocol below — no exceptions. |

**The partition rule:** NC code never calls into SC code except through its
narrow public API (`edge_core.safety`'s exports). SC code never imports NC code
— `edge_core/safety/` imports **stdlib and itself only** (no FastAPI, no ROS,
no pymavlink, no numpy).

## The SC protocol (any change under an SC path)

1. **Name the requirement.** Find the `SR-*` ID in
   [docs/safety/requirements.md](docs/safety/requirements.md) that your change
   implements or preserves. No requirement → write one first (append; never
   renumber existing IDs) and add the hazard row to `hazards.md` if new.
2. **Tests before the move.** For behavior-preserving refactors, the existing
   tests (and new ones if coverage is thin) must pass unchanged before and
   after. Gate order and operator-facing rejection messages are part of
   behavior — preserve them verbatim unless a requirement says otherwise.
3. **Fault injection, not just happy path.** Every failure input (NaN/inf,
   stale, out-of-range, disconnected, wrong mode) needs a test asserting the
   safe output: reject / zero velocity / hold / relinquish to the FC. The safe
   output is always the default; the *first* failing gate short-circuits.
4. **Keep the gates green:** `pixi run cov-safety` (100% branch coverage on
   `edge_core/safety/` — your change does not get to lower it),
   `tests/test_safety_traceability.py`, and update
   [docs/safety/traceability.md](docs/safety/traceability.md) if you moved or
   renamed any mapped symbol or test.
5. **SITL evidence** for changes to the velocity, geofence, or payload command
   paths: run or extend the scenario in [tests/sitl/](tests/sitl/)
   (`pixi run dev-up` then `pixi run sitl-scenario`). If you cannot run it,
   say so explicitly — do not claim it.
6. **Honesty convention:** unfinished safety work is marked **GAP** / 🟡 / 🔴 in
   the docs, never described as done. Mirror that: report what you verified and
   what you didn't.

## The house pattern — pure core, thin adapter

All decision logic lives in small pure modules; all I/O lives in thin adapter
shells. The template is `edge_core/safety/envelope.py` +
`edge_core/ros_http_bridge/mavlink_velocity.py`:

- **Core:** pure functions and `@dataclass(frozen=True)` value types. Inputs are
  an immutable policy (static config), an immutable conditions snapshot
  (dynamic state), the command, and an explicit `now: float` — never
  `time.time()` inside the core. Output is a frozen `Decision`-style result
  with a stable machine-readable `reason` key plus an operator-facing message.
  No threads, no sockets, no logging, no globals, no heavy imports.
- **Adapter:** owns threads/locks/links. It snapshots state under its lock,
  calls the core, transmits what the core returns, and logs. It makes **no
  safety decisions of its own**.

Any new decision path (geofence enforcement, payload interlock, a future RTH
state machine) copies this shape: core module in `edge_core/safety/`
(target: well under ~150 lines), tests added to the `cov-safety` gate, adapter
kept dumb.

## How the code should be

- **Shorter is better, clearer is the limit.** The entire SC core is ~350 lines
  *including* docstrings, at 100% branch coverage — that is the bar. If your
  refactor grows net lines, justify each one.
- **Deletion first.** Dead code (constructed nowhere, referenced nowhere) gets
  deleted, not preserved or "extracted" — a 664-line orphaned controller was
  removed from this repo on exactly that principle. Check callers before
  assuming code is live; check git history before assuming it is dead.
- **No speculative abstraction.** No interface with one implementer, no config
  flag nobody sets, no "for later" parameters, no compatibility shim unless a
  current caller needs it (and then comment why it exists).
- **One source of truth.** Limits/thresholds defined once; the API contract is
  `app.openapi()`; the safety case is `docs/safety/`. Never hand-mirror a value
  — wire to the source or generate from it.
- **Style (Python):** typed (`from __future__ import annotations`), lazy
  `%`-style logging, no bare `except` and no silently swallowed errors in
  SC/SR code, SPDX header on every file, Ruff + mypy clean. Module docstrings
  on SC files state the tier and the `SR-*` requirements they implement.
- **Style (C#):** follow the existing `partial`-class seams for large types;
  `dotnet format` clean; UI logic separated from command logic (a button click
  calls one validated method — validation never lives in the click handler).
- **Hard caps:** 800 lines per source file (pre-commit enforced). No emojis in
  code or docs. Comments state constraints the code can't show — not narration.
- **Naming:** plain and literal. A new contributor should be able to read any
  SC file top-to-bottom in one sitting and say what it refuses to do.

## Verify, then report

Before claiming done, run and pass:

```
pixi run lint && pixi run fmt-check && pixi run typecheck && pixi run test
pixi run cov-safety            # if anything under edge_core/safety/ changed
pixi run build-plugin          # Windows only — if C# changed
```

Then report: files touched per tier, requirements implicated, tests
added/changed, net line delta, and anything you could not verify. Follow
[AGENTS.md](AGENTS.md) §7a for commit messages; never commit unless asked.

## Do not

- Weaken, reorder, or remove a safety gate, clamp, watchdog, or failsafe —
  even "temporarily", even in a refactor that "preserves it elsewhere" —
  without a requirement change reviewed by a human.
- Add dependencies to `edge_core/safety/`, or any new project dependency
  without stating why the stdlib/current deps can't do it.
- Apply SC-grade process to NC code (or NC-grade haste to SC code).
- Introduce code generation, frameworks, or architectural layers the current
  problem doesn't demand.
- Renumber requirement IDs, delete GAP markers, or edit `docs/safety/` to
  describe aspiration as fact.
- Create new top-level planning documents — status belongs in `docs/safety/`.
