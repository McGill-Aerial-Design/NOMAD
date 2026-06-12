# NOMAD Next Steps

Status: **active plan** (2026-06-11). This is the only live planning document.
It supersedes `NOMAD_REFACTOR_ROADMAP.md` (fully implemented in PR #8),
`BASELINE_POLISH_PLAN.md`, and `REFACTORING_PROMPT.md` — all three are deleted
by item 3 below. Delete this file too once every item is closed.

The refactor era is over: the largest source file is 575 lines, route/UI
handlers are thin, adapters are isolated, and the maintenance rules live in
`CONTRIBUTING.md`. **No further restructuring work should be planned** — the
remaining leverage is safety evidence, one unverified feature, and building
competition capability on the clean base.

---

## 1. Close the geofence SITL evidence gap

**Why:** every safety requirement has unit tests at 100% branch coverage, but
SR-FEN-02 (geofence containment) has never been demonstrated against a live
autopilot since the geofence wiring landed. It is the one missing piece of SC
evidence; `docs/safety/hazards.md` marks other hazards "SITL-proven" but not
this one.

**Effort:** ~30 minutes. Needs Docker; no hardware.

- [x] `pixi run dev-up` (Edge Core + ArduPilot SITL stack)
- [x] `pixi run sitl-fence` — geofence containment scenario — **PASSED**
      2026-06-11 at commit f69a137
- [x] If it fails: file an issue with the transcript; treat as release-blocking
      (N/A — passed)
- [x] If it passes: record the run (date, commit, result) in
      `docs/safety/hazards.md` next to the fence hazard row, mirroring the
      existing "SITL-proven" entries
- [x] While the stack is up, re-run `pixi run sitl-scenario` (velocity loop
      closure) — **PASSED** same day/commit
- [x] `pixi run dev-down`

**Acceptance:** both SITL scenarios pass on current `main` and hazards.md
cites the runs.

---

## 2. Ratchet the coverage floor

**Why:** overall coverage is ~49% but `pixi.toml` still gates at
`--cov-fail-under=25`. The floor is documented as a ratchet ("raise it as more
of the codebase gets tested") — leaving it at 25 lets the refactor's gains
erode silently.

**Effort:** ~10 minutes.

- [x] In `pixi.toml`, change the `test` task to `--cov-fail-under=45`
      (just below the current ~49% so flaky environments don't trip it)
- [x] Run `pixi run test` to confirm the gate passes (48.68% ≥ 45%)
- [x] Keep the ratchet comment; update it if the wording references the old value

**Acceptance:** CI fails if coverage drops below 45%.

---

## 3. Delete completed planning documents

**Why:** three large planning files sit at the repo root. All are finished
working documents; a new contributor cannot tell which (if any) is current.
The durable content has already been folded into the right homes (maintenance
rules → `CONTRIBUTING.md`; architecture → `docs/`).

**Effort:** ~30 minutes.

- [x] Skim `BASELINE_POLISH_PLAN.md` one last time; fold anything still
      load-bearing into `docs/` (confirmed: nothing — P0s spot-checked done)
- [x] Skim `NOMAD_REFACTOR_ROADMAP.md`; the size-limits table moved into
      `CONTRIBUTING.md`
- [x] Update `CONTRIBUTING.md` §"Safe refactor checklist" — now self-contained
- [x] `git rm BASELINE_POLISH_PLAN.md NOMAD_REFACTOR_ROADMAP.md REFACTORING_PROMPT.md`
- [x] Grep for remaining references — none outside this file

**Acceptance:** repo root contains no historical planning documents;
`CONTRIBUTING.md` stands alone; no dangling references.

---

## 4. Verify (and if needed restore) the boundary feature end-to-end

**Why:** the open-source cleanup gutted the C# plugin and the boundary feature
was repaired from the `AEAC2026` branch on 2026-06-06, with a note that "a few
boundary methods remain empty stubs." `BoundaryManager.cs` now looks
implemented, but nothing automated exercises the UI path — geofence
display/upload is a keep feature and a safety-adjacent one (SC dirs:
`Geofence/`).

**Effort:** one focused session on Windows with Mission Planner installed.

- [x] Audit `NOMADBoundaryView*.cs`, `BoundaryManager.cs`, `MPFenceUploader.cs`,
      `MapOverlayManager*.cs` for empty/no-op method bodies; diff suspicious
      ones against `origin/AEAC2026` (found: 7 empty stubs + gutted import
      handlers, dead fence push, disabled return-point/preset persistence,
      and no `BoundaryMonitor` ever constructed)
- [x] Restore any missing bodies from `AEAC2026` (competition-only code stays
      out — boundary is a keep feature; restored against `GeofenceConfig`,
      Task fields dropped)
- [x] `pixi run build-plugin && pixi run lint-plugin` — both pass
- [ ] Manual verification in Mission Planner against the SITL stack
      (`pixi run dev-up`, connect MP to TCP 5760):
  - [ ] Draw/load a boundary; soft (yellow) and hard (red) polygons render on the map
  - [ ] Fence upload to the (SITL) autopilot succeeds
  - [ ] Fence export round-trips
  - [ ] Boundary violation raises a notification (move SITL vehicle outside)
- [ ] Record what was verified in the PR description; add geofence-validation
      unit tests for any pure helper logic touched

**Acceptance:** boundary display, upload, export, and violation alerts all
demonstrated working; no stub bodies remain in `Geofence/` or the boundary views.

---

## 5. Small hygiene (batch into any of the above PRs)

- [x] Migrate the two Pydantic V1 `@validator`s in `edge_core/api_models.py`
      to `@field_validator` (deprecation warnings in every test run; V1 style
      is removed in Pydantic V3)
- [x] `.vscode/mcp-settings.json` / `settings.json` — already fixed for the
      tailscale flatten; spot-checked: no stale paths after item 3's deletions

---

## 6. Then: competition capability, as modules

With items 1–4 closed, the baseline is clean, evidenced, and guarded. New
season work follows the onboarding ladder in `CONTRIBUTING.md` §8:

- Task/competition features are built as `NomadModule`s (Python entry points,
  C# `INomadModule`) — never wired into the baseline directly
- Pull prior implementations from the `AEAC2026` branch as references, but
  re-enter them through the module SDK
- SC-touching work (velocity, fence, payload command paths) carries an `SR-*`
  requirement, tests with fault inputs, and SITL evidence per the PR template

**Guardrail:** if a feature PR needs to modify `edge_core/safety/` or grow a
file past the caps, that is a design smell — bring it back to the module
boundary first.

---

## Sequencing

1 and 2 are independent and immediate (same day). 3 follows any time. 4 needs
a Windows + Mission Planner session and can run in parallel with 1–3. 6 starts
whenever 1 and 4 are done — those two are the only items gating flight-adjacent
feature work.
