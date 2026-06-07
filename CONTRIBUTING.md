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
- [ ] Docs are updated if behavior or configuration changed.
- [ ] The MR is focused — one feature/fix per MR.
- [ ] Branch is up to date with `upstream/main`.

## 8. Reporting bugs / requesting features

Open an issue using the templates under **New issue**. Redact any secrets in
logs and reproduction steps.

By contributing, you agree that your contributions will be licensed under
[Apache 2.0](LICENSE).
