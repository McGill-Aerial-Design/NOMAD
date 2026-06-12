# Contributing to NOMAD

## 1. Forking Workflow

NOMAD uses the **forking workflow** to keep the main repository clean.
Developers never push branches directly to `McGill-Aerial-Design/NOMAD`.
Instead, each contributor works in their own fork and submits changes via
**merge requests** (pull requests).

```
Upstream (main repo)          Your fork
  McGill-Aerial-Design/NOMAD     <your-username>/NOMAD
           │                           │
           └──────── merge request ────┘
```

## 2. Getting started

1. **Fork** the repository on GitHub to your own account.
2. **Clone your fork** and add the upstream as a remote:

   ```bash
   git clone https://github.com/<your-username>/NOMAD.git
   cd NOMAD
   git remote add upstream https://github.com/McGill-Aerial-Design/NOMAD.git
   ```

3. **Sync your fork** before starting any work:

   ```bash
   git fetch upstream
   git switch main
   git merge --ff-only upstream/main
   git push origin main
   ```

4. **Create a feature branch** in your fork (never work on `main`):

   ```bash
   git switch -c <category>/<short-description>
   ```

5. **Set up your environment:**

   ```bash
   pixi run bootstrap
   ```

## 3. Branch naming

```
<category>/<short-description>
```

| Category   | Use for                     | Example                                    |
|------------|-----------------------------|--------------------------------------------|
| `edge`     | Edge Core (Python API)      | `edge/fix-mavlink-reconnect`               |
| `plugin`   | Mission Planner plugin (C#) | `plugin/add-connection-health`             |
| `docker`   | Container definitions       | `docker/fix-dev-image-entrypoint`          |
| `infra`    | Systemd, transport, scripts | `infra/consolidate-service-units`          |
| `ci`       | CI / workflow changes       | `ci/add-lint-stage`                        |
| `docs`     | Documentation only          | `docs/rewrite-contributing-guide`          |
| `refactor` | Behavior-preserving restructure | `refactor/extract-module-registry`     |
| `chore`    | Tooling, deps, maintenance  | `chore/bump-ruff-to-0.9`                  |

## 4. Commit messages

Maximum width is 72 characters. One commit per logical change. Style fixes
must be in their own commits — never mix them with feature work.

```
[part,sub-part] Short description in imperative

Longer explanation if needed. Skip a line between the subject
and body. Explain what and why, not how.
```

- Prefix (always lowercase) names the part and optional sub-part in brackets.
- Start the description with a capital letter and an imperative verb.

Examples:

```
[edge_core] Fix MAVLink reconnect on serial drop
[mission_planner] Fix WASD key release on panel exit
[docs] Rewrite contributing guide for fork-based workflow
```

## 5. Merge request process

Once your feature or fix is complete:

1. **Push your branch** to your fork:

   ```bash
   git push -u origin <category>/<short-description>
   ```

2. **Open a merge request** on GitHub from your fork's branch into
   `McGill-Aerial-Design/NOMAD:main`.

3. **MR title format:**

   ```
   <category>: <short-description>
   ```

   Examples:
   ```
   edge: Fix MAVLink reconnect on serial drop
   plugin: Add connection health indicator
   docs: Rewrite contributing guide
   ```

4. **MR description:** Fill in the [merge request template](.github/PULL_REQUEST_TEMPLATE.md).

5. **Labels:** Add a label matching the category (`edge`, `plugin`, `docker`,
   `infra`, `ci`, `docs`, `refactor`, `chore`).

6. **Address review feedback** by pushing additional commits to the same branch.
   Squashing is handled at merge time — do not rebase after opening the MR.

> Never force-push to a branch that has an open merge request.

## 6. Local development

```bash
pixi run dev           # Edge Core sim on http://localhost:8000
pixi run test          # pytest
pixi run lint          # ruff check
pixi run fmt           # ruff format
pixi run line-report   # largest files and longest source lines
pixi run docs          # serve docs site locally
pixi run build-plugin  # build Mission Planner DLL (Windows)
```

Install pre-commit hooks:

```bash
pixi run precommit
```

## 7. Before opening a merge request

- [ ] **No secrets or PII** — no real IPs, API keys, emails, or absolute
      user paths. Use env vars or placeholders like `<jetson-ip>`.
- [ ] `pixi run lint` and `pixi run fmt-check` pass.
- [ ] `pixi run test` passes (or is N/A for your change).
- [ ] `pixi run line-report` shows no touched source file over 800 lines.
- [ ] Docs are updated if behavior or configuration changed.
- [ ] The MR is focused — one feature/fix per MR.
- [ ] Branch is up to date with `upstream/main`.

The line report scans source/docs files only. It excludes generated or bulky
non-source output such as `.git/`, Pixi/Ruff/Pytest/Mypy caches, `site/`, package
metadata, `pixi.lock`, and vendored Mission Planner third-party binaries under
`mission_planner/third_party/`.

### Safe refactor checklist

For changes that touch source code:

- [ ] Deleted unused code before adding new abstractions.
- [ ] Decision logic stays separate from API/UI/MAVLink/ROS adapters.
- [ ] Route handlers and UI event handlers stay thin (convert + delegate).
- [ ] No new file near the 800-line cap; split by responsibility instead
      (C#: `partial` files; Python: focused modules).
- [ ] Environment parsing goes through `edge_core/env.py` helpers.
- [ ] New abstractions have more than one real caller.
- [ ] The important logic can be tested without the drone.

Do not merge if a route handler contains hardware command logic directly, a UI
click handler contains a full command workflow, or a safety decision depends on
UI state.

### Size limits

| Item | Target |
|---|---|
| Python line length | 120 characters (enforced by Ruff E501) |
| C# line length | ~120 characters (manual) |
| Normal source file length | Aim for 400–600 lines |
| Hard source file cap | 800 lines (CI-enforced via `pixi run line-report`) |
| Safety-core files (`edge_core/safety/`) | Aim for under 150 lines |
| Generated and lock files | Excluded from line-count checks |

Reduce long lines by naming intermediate boolean expressions, extracting
helper variables, and splitting call arguments — not by disabling the check.

## 8. Where to start — the onboarding ladder

The repo is partitioned by safety tier (see
[docs/safety/partition.md](docs/safety/partition.md)). The tiers are not
bureaucracy — they are *why* most of the repo is fast to change: heavy process
applies only to the thin slice that can move the aircraft or fire the payload.
Competition-specific code stays in modules so the baseline survives to the next
season.

1. **Start in modules.** Competition features are `NomadModule`s on the SDK —
   isolated, fault-contained, and unable to touch safety-critical code. See
   [examples/sample_module/](examples/sample_module/) and
   [docs/writing_a_module.md](docs/writing_a_module.md). This is where new
   recruits build task code from day one.
2. **NC (non-critical) next.** Panels, views, video, docs, scripts — normal
   review, fast merges. Look for `good-first-issue` labels.
3. **SR/SC last, and paired.** Safety-related/safety-critical paths (listed in
   [.github/CODEOWNERS](.github/CODEOWNERS) and `partition.md`) require:
   reading [docs/safety/README.md](docs/safety/README.md) (one page), a named
   `SR-*` requirement, a test (fault inputs included), the SC checklist in the
   PR template, and a CODEOWNER review. CI enforces 100% branch coverage on
   `edge_core/safety/` (`pixi run cov-safety`), the requirement→code→test
   traceability sync, the SC/NC import partition, and the C# client↔API
   contract.

## 9. Reporting bugs / requesting features

Open an issue using the templates under **New issue**. Redact any secrets in
logs and reproduction steps.

By contributing, you agree that your contributions will be licensed under
[Apache 2.0](LICENSE).
