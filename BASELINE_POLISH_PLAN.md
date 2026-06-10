# NOMAD Baseline Polish Plan

> Goal: take NOMAD from "works, mostly clean" to a **polished, reusable drone-control
> baseline** that is modular, easy to understand, simple, and stable.
>
> Status snapshot (audit date 2026-06-08): the repo is in good shape. `pixi run lint`,
> `fmt-check`, `typecheck`, and `test` (17 tests) all pass. The module SDK
> (`edge_core/core/`) is clean, typed, and well-documented. CI covers test + lint +
> docs + docker. The work below is about **finishing the gutting**, removing drift,
> tightening consistency, and raising test coverage — not a rewrite.

---

## How to read this

Items are tagged by priority:

- **P0 — blocking**: must be done for the repo to be a credible, clean baseline.
- **P1 — should**: clear quality/consistency wins; do before calling it "polished".
- **P2 — nice**: polish and future-proofing.

Each item lists the concrete change and the files involved.

---

## 1. Working tree & repo hygiene

The git history is clean (346 tracked files, only 3 intentionally-vendored DLLs).
The problems are in the **untracked working tree** and a few ignore gaps.

- [ ] **P0 — Commit or stash the in-flight refactor.** The tree is dirty with a
  coherent but uncommitted change set: the new `edge_core/api_routes/vio.py`
  module (+ its `pyproject.toml` entry point), sim compose `--profile` fixes in
  `pixi.toml` / `docker-compose.sim.yml`, `servo_controller` wired onto
  `app.state` (`payload_module.py`), and additions to `calibration.py`,
  `health_monitor.py`, `time_manager.py`, `Dockerfile.isaac_sim`. A baseline must
  not ship with a dirty tree. Finish + test (see §6) + commit it, or stash it.
- [ ] **P0 — Add `.kilo/` to `.gitignore`.** It is an editor/agent vendored
  `node_modules` tree (~57 MB) sitting untracked and unignored — one `git add -A`
  away from being committed. (`mission_planner/src/bin/`, `libvlc-windows/`, and
  `config/*.env.bak.*` are already correctly ignored.)
- [ ] **P1 — Normalize line endings.** `git diff` warns LF→CRLF on
  `.toml`/`.json`/`.py` because `.gitattributes` only covers `.sh`/`.bash`/`.ps1`.
  Add `* text=auto eol=lf` (keeping the existing `*.ps1 eol=crlf` exception) so
  cross-platform contributors don't churn line endings. Then run
  `git add --renormalize .` once.
- [ ] **P2 — Prune local build bloat from clones' docs.** `mission_planner` is
  ~349 MB on disk (untracked libvlc + bin/Debug + bin/Release, each carrying a
  full VLC distribution). It's correctly gitignored, but the
  `packaging/fetch-libvlc.ps1` flow means three copies can accumulate. Document
  in `mission_planner/README.md` that `bin/` and `libvlc-windows/` are
  disposable, and have `copy-libvlc.ps1` reuse one source instead of duplicating.

---

## 2. Finish the gutting (competition cruft → clean baseline)

This branch is the public baseline gutted from `AEAC2026`. The gutting is ~90%
done; these leftovers betray its origin and should be neutralized.

- [ ] **P0 — Fix broken doc links in `AGENTS.md`.** Lines ~283–286 reference docs
  that do not exist in this branch: `docs/NVBLOX_VISUALIZATION.md`,
  `docs/TASK1_COMPETITION_GUIDE.md`, `docs/TASK2_MANUAL_POSITIONING.md`. Remove
  the rows or point them at the real docs.
- [ ] **P1 — Neutralize competition framing in code.** Replace "Task 1 / Task 2 /
  competition" wording with generic capability language in:
  - `edge_core/api.py:7` and `:77` — API title/description says
    "Task 1 & Task 2 Operations".
  - `edge_core/services/geospatial.py:4` — "NOMAD Task 1 (Recon Mission)".
  - `edge_core/services/logging_service.py:6` — "evidence logging for competition tasks".
  - `edge_core/services/models.py:102` — "Task 2 approach logic" comment.
  (`nvblox` is a real subsystem name — keep it.)
- [ ] **P0 — Deal with `tests/test_p3_7_debounce.py`.** It's competition-specific
  ("P3.7") and is silently skipped via `addopts = "... --ignore=tests/test_p3_7_debounce.py"`
  in `pyproject.toml`. Either fix it and remove the ignore, or delete the file and
  the ignore hack. A baseline should not carry a permanently-disabled test.
- [ ] **P1 — Triage `scripts/dev/` (~60 one-off scripts).** It's full of hardware
  bring-up/debug throwaways (`check_*`, `diag_zed_*`, `spy_*`, `test_zed_*`,
  `test_usb*`, `patch_depth_conf*`, `probe_mesh_*`, …) that make the repo look
  like a lab notebook. Keep the genuinely reusable ones (`test_api_endpoints.py`,
  `run_dev.sh/.ps1`, `gdrive_auth.py`); move the rest to a clearly-labeled
  `scripts/dev/archive/` or delete. Same review for `tests/run_jetson_tests.sh`.
- [ ] **P2 — Audit `scripts/` README** so it documents only what survives the
  triage above.

---

## 3. Code quality & consistency (edge_core)

The architecture is sound; these are specific, contained fixes.

- [ ] **P1 — Migrate off deprecated FastAPI `on_event`.** `edge_core/core/__init__.py:106-112`
  uses `@app.on_event("startup"/"shutdown")`, which emits `DeprecationWarning`
  (visible in the test run) and will be removed in a future FastAPI. Switch to a
  `lifespan` async context manager that calls `registry.start_all()` /
  `stop_all()`.
- [ ] **P1 — Consolidate the two module-wiring paths.** `wire_modules()` in
  `core/__init__.py` reaches into the registry's privates (`registry._modules`,
  `registry._order`) and re-implements logic that already exists as
  `ModuleRegistry.configure_all()` / `register_routes()` / `wire()`. There are now
  two overlapping ways to wire modules. Add one public, per-module-fault-isolated
  method on `ModuleRegistry` (e.g. `wire_safe(ctx, app)`) and have `wire_modules`
  call it — no private access, one source of truth.
- [ ] **P1 — Pick one logging style.** `edge_core/main.py` uses f-string logging
  (`logger.error(f"...")`); `registry.py` uses lazy `%`-style
  (`logger.error("...: %s", exc)`). Standardize on lazy `%` (the conventional,
  lint-enforceable choice) and consider enabling Ruff's `G`/`LOG` rules to keep it
  that way.
- [ ] **P2 — Guard the module-level app singleton.** `edge_core/main.py:55`
  runs `app = get_app()` at import time. It's needed for uvicorn's import string,
  but add a short comment explaining why, so it isn't "cleaned up" later.
- [ ] **P2 — Reconsider swallowing wiring failures.** `main.py:112-113` catches
  `wire_modules` exceptions and continues, which can boot a half-wired server. For
  a "stable" baseline, log at `critical` and consider failing fast outside sim
  mode.
- [ ] **P2 — Centralize CLI→env config.** `main.py` translates `--sim`,
  `--no-vision`, `--servo-mode` into `os.environ` mutations that modules read back
  implicitly. It works, but document this contract (or route it through a single
  settings object) so the config flow is discoverable.

---

## 4. Mission Planner plugin (C#)

- [ ] **P1 — Split the two files that violate the repo's own 800-line limit.** The
  `file-line-count` pre-commit hook caps source files at 800 lines, but
  `mission_planner/src/Geofence/MapOverlayManager.cs` (896) and
  `src/Panels/LinkHealthPanel.cs` (878) exceed it — they'd block any future edit
  to those files. Split along existing `partial`-class seams (the codebase already
  uses that pattern heavily) or raise the limit deliberately.
- [ ] **P2 — Verify the plugin still builds from a clean checkout** on Windows via
  `pixi run build-plugin`, since this is the GCS half of the baseline and CI can't
  build it (Windows-only MP DLLs). Document the exact prerequisites in
  `mission_planner/README.md`.

---

## 5. Tooling, CI & config drift

- [ ] **P1 — Unify Ruff version across pre-commit, dev dep, and CI.** Pre-commit
  pins `ruff-pre-commit` at `v0.7.4` (`.pre-commit-config.yaml`), `pyproject`
  declares `ruff>=0.7`, and the local cache shows `0.15.16` was actually used.
  Formatting output can differ across Ruff versions → "passes locally, fails in
  CI". Pin all three to the same minor.
- [ ] **P1 — Unify `setup-pixi` action version.** `test.yml`/`lint.yml` use
  `prefix-dev/setup-pixi@v0.8.1`; `static.yml` uses `@v0.8.8`. Pick one.
- [ ] **P1 — Resolve the line-length contradiction.** `pyproject.toml` sets
  `line-length = 120` but then `ignore = ["E501"]` disables line-length
  enforcement entirely. Either drop the `E501` ignore (enforce 120) or drop the
  `line-length` setting. Right now it reads as confused intent.
- [ ] **P1 — Fix the stale comment in `pixi.toml`.** The header (line ~5) points
  readers to `docker/Dockerfile.jetson`, which doesn't exist; the real images are
  `Dockerfile.dev` and `Dockerfile.isaac_sim`.
- [ ] **P2 — Extend mypy scope or document why not.** `[tool.mypy] files =
  ["edge_core"]`, but `infra.tailscale` and `infra.transport` are shipped as
  packages (`[tool.setuptools.packages.find]`). Either add them to mypy or note
  they're intentionally untyped.
- [ ] **P2 — Avoid the docs/dev port clash.** `pixi run docs` and `pixi run dev`
  both default to `:8000`, and the VS Code "Docs: Serve locally" task even says so
  in its detail text. Move docs to `:8001` to allow running both.

---

## 6. Testing (the biggest stability gap)

Only ~3 test files / 17 tests for a multi-service codebase. The module SDK is
well-covered; almost nothing else is.

- [ ] **P0 — Add a `create_app` boot smoke test.** Instantiate the real app via
  `create_app(StateManager.instance())`, hit `/health` and `/docs`, and assert a
  sample module's route mounts. This is the single highest-value test for
  "does it work".
- [ ] **P1 — Unit-test the pure-logic services** (no hardware, fast, high value):
  - `edge_core/ros_http_bridge/coordinate_math.py`
  - `edge_core/services/geospatial.py`
  - `edge_core/ros_http_bridge/mesh_packer.py`
  - `edge_core/ros_http_bridge/mavlink_velocity.py` (frame conversion)
- [ ] **P1 — Test the API-key auth middleware** in `api.py`: exempt paths,
  loopback-dev fallback, `NOMAD_ALLOW_INSECURE_REMOTE`, and the internal-bridge
  token (incl. the min-length disable path). This is security-sensitive logic.
- [ ] **P1 — Add a test for the new `vio` module** before committing it (see §1).
- [ ] **P2 — Wire coverage reporting** (`pytest-cov`) into the `test` task and CI,
  with a modest floor (e.g. fail under 50%) so coverage trends up, not down.

---

## 7. VS Code tasks

The task set is comprehensive and genuinely useful; these are cleanups.

- [ ] **P1 — Make tasks call `pixi run …`, not raw docker.** The "Isaac Sim
  (Docker)" and "Docker (dev)" task blocks shell out to `docker compose …` /
  `docker build …` directly, duplicating the `sim-*` and `dev-*` tasks already
  defined in `pixi.toml`. They've **already drifted** (the in-flight `pixi.toml`
  diff added `--profile` flags the tasks lack). Point each task at the pixi task
  so commands live in one place.
- [ ] **P2 — Normalize indentation.** The Isaac Sim block (lines ~161–224) uses
  tabs; the rest of `tasks.json` uses 2-space. Reformat to one style.
- [ ] **P2 — Reconsider the default build task.** `"C#: Build Mission Planner
  Plugin"` is `isDefault: true` for the build group, but it's Windows-only. For a
  cross-platform baseline, a safer default (`Edge Core: Lint` or
  `Edge Core: Dev server`) won't fail for Linux/macOS contributors who press
  Ctrl+Shift+B.

---

## 8. Documentation

Docs exist and are reasonable (getting_started, architecture, writing_a_module,
deployment, configuration, api_reference) — they just need accuracy passes.

- [ ] **P1 — Verify the README quick-start literally works.** It says
  `pip install pixi` then `pixi run dev`. Pixi is normally installed via its own
  installer; confirm `pip install pixi` actually yields a working `pixi` on a
  clean machine, or correct the instruction.
- [ ] **P1 — Keep `AGENTS.md` / README structure lists in sync** with the code
  after the `vio` module lands and after the `scripts/dev` triage (§2).
- [ ] **P2 — Prevent `api_reference.md` drift.** It's hand-maintained and will
  fall out of step with the routers. Add a small test asserting documented paths
  exist in `app.openapi()["paths"]`, or generate the reference from the OpenAPI
  schema.
- [ ] **P2 — Add a `CHANGELOG.md`** (Keep a Changelog format) — the project is at
  `0.1.0` and a baseline others fork from benefits from a visible change history.
- [ ] **P2 — Add a one-screen architecture diagram** (even ASCII) to
  `docs/architecture.md` showing Edge Core ↔ ROS-HTTP bridge ↔ MAVLink ↔ GCS
  plugin and the module lifecycle.

---

## 9. Suggested execution order

1. **Land the baseline cleanly (P0):** commit/stash in-flight work (§1), fix
   `AGENTS.md` broken links (§2), resolve `test_p3_7_debounce` (§2), add `.kilo/`
   ignore (§1), add the `create_app` smoke test (§6).
2. **Remove drift (P1):** Ruff/`setup-pixi`/line-length unification (§5), tasks →
   `pixi run` (§7), line-ending normalization (§1), competition-wording pass (§2).
3. **Tighten the code (P1):** lifespan migration + wiring consolidation +
   logging style (§3), C# file splits (§4).
4. **Raise the floor (P1):** pure-logic + auth + vio tests (§6).
5. **Polish (P2):** coverage gate, CHANGELOG, docs accuracy, architecture
   diagram, remaining nits.

---

## Appendix — What's already good (don't touch)

- `edge_core/core/` module SDK: clean Protocol-based interface, topological
  dependency resolution, fault isolation, excellent docstrings.
- `api.py` auth design: API key + loopback-dev fallback + explicit insecure-remote
  opt-in + length-checked internal bridge token + exempt paths. Thoughtful.
- CI: test (incl. docker image smoke test), lint (ruff + format + mypy +
  shellcheck + advisory dotnet-format), and Pages docs deploy.
- Pre-commit: SPDX header check, 800-line cap, large-file guard, private-key
  detection, ruff, shellcheck.
- Secrets hygiene in `.gitignore` is thorough; no hardcoded hosts/keys in-repo.
- Module sizing: largest Python file is 732 lines; good separation of concerns.
