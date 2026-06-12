# NOMAD Codebase Refactor Roadmap

## Purpose

This roadmap defines the next cleanup and simplification work for NOMAD. The goal is not to make the codebase smaller at all costs. The goal is to make the codebase easier to read, easier to test, safer to modify, and easier for new team members to understand.

The main design rule for this refactor is:

> Keep decision logic small, pure, and testable. Keep API, UI, MAVLink, ROS, and hardware-facing code as thin adapters.

This roadmap focuses on:

- Reducing oversized files.
- Reducing very long individual lines.
- Simplifying control flow.
- Splitting files by responsibility.
- Making safety-critical logic testable without hardware.
- Keeping the architecture obvious for future contributors.

---

## Refactor Principles

### 1. Pure Core, Thin Adapter

Decision logic should live in small, pure modules.

Adapters should only handle:

- FastAPI request/response conversion.
- Mission Planner UI events.
- MAVLink transmission.
- ROS message conversion.
- Logging.
- Threading.
- Locks.
- External process management.
- Hardware access.

Core logic should avoid:

- Threads.
- Sockets.
- Global mutable state.
- Hardware dependencies.
- Heavy imports.
- Direct logging.
- Direct UI/API/MAVLink calls.

Recommended shape:

```text
feature/
  core.py          # pure decision logic
  adapter_api.py   # FastAPI wrapper
  adapter_ui.cs    # Mission Planner wrapper
  adapter_mav.py   # MAVLink wrapper
  tests/
```

---

### 2. Delete Before Abstracting

Before adding an abstraction, check whether code can be deleted first.

Avoid:

- Interfaces with only one implementation.
- Compatibility shims that nothing uses.
- "Future-proof" flags with no current caller.
- Helper layers that only move complexity somewhere else.

Ask:

- Is this code still used?
- Does this abstraction have more than one real caller?
- Would a new member understand this faster?
- Can this logic be tested without the drone?

---

### 3. Split Files by Responsibility

Do not split files randomly just to reduce line count. Split files when one file has multiple reasons to change.

Good Python split:

```text
edge_core/
  api.py                 # create_app only
  api_auth.py            # authentication settings and middleware
  api_state.py           # app.state initialization
  api_route_registry.py  # route registration
```

Good C# split:

```text
MissionPlannerPanel.cs
MissionPlannerPanel.Layout.cs
MissionPlannerPanel.Events.cs
MissionPlannerPanel.Commands.cs
MissionPlannerPanel.State.cs
MissionPlannerPanel.Rendering.cs
```

---

### 4. Make Routes and UI Boring

API route files should mostly convert HTTP requests into service calls.

They should not contain:

- Environment parsing.
- MAVLink details.
- Safety decisions.
- Thread management.
- Long validation blocks.
- Large inline command logic.

Mission Planner button handlers should mostly call presenter/service methods.

Bad:

```csharp
btnRelease.Click += async (s, e) =>
{
    // validate
    // build payload
    // call API
    // parse result
    // update UI
    // handle error
};
```

Better:

```csharp
private async void OnReleaseClicked(object? sender, EventArgs e)
{
    await _payloadPresenter.ReleaseAsync();
}
```

---

## Line Size Rules

### Target Limits

| Item | Target |
|---|---|
| Python line length | 100 or 120 characters, but enforce it |
| C# line length | Around 120 characters manually |
| Normal source file length | Aim for 400-600 lines |
| Hard source file cap | 800 lines |
| Safety-core files | Aim for under 150 lines |
| Generated files | Exclude from line-count checks |
| Lock files | Exclude from line-count checks |

---

### Preferred Ways to Reduce Long Lines

#### Break Boolean Expressions

Before:

```python
if not provided_key or not hmac.compare_digest(provided_key, _NOMAD_API_KEY):
    ...
```

After:

```python
has_valid_key = bool(provided_key) and hmac.compare_digest(
    provided_key,
    settings.api_key,
)

if not has_valid_key:
    ...
```

---

#### Move Environment Parsing Into Helpers

Before:

```python
_ALLOW_INSECURE_REMOTE = (os.environ.get("NOMAD_ALLOW_INSECURE_REMOTE") or "").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
```

After:

```python
_ALLOW_INSECURE_REMOTE = env_bool("NOMAD_ALLOW_INSECURE_REMOTE")
```

Helper:

```python
def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)

    if value is None:
        return default

    return value.strip().lower() in {"1", "true", "yes", "on"}
```

---

#### Move Large Constants Out of Functions

Before:

```python
def create_app(...):
    _AUTH_EXEMPT_PATHS = {"/", "/health", "/docs", "/redoc", "/openapi.json"}
    _COMMAND_PATH_PREFIXES = ("/api/servo/", "/api/spray/")
```

After:

```python
AUTH_EXEMPT_PATHS = frozenset({
    "/",
    "/health",
    "/docs",
    "/redoc",
    "/openapi.json",
})

COMMAND_PATH_PREFIXES = (
    "/api/servo/",
    "/api/spray/",
)
```

---

#### Replace Comments With Named Functions When Possible

Before:

```python
# Command endpoints never ride the unauthenticated loopback fallback
if _is_command_path(request_path):
    ...
```

After:

```python
if command_requires_real_auth(request_path):
    ...
```

---

## Phase 1 — Tooling and Safety Net

### Goal

Make sure the codebase has a reliable baseline before major refactoring starts.

### Checklist

- [ ] Confirm the project runs in development/simulation mode.
- [ ] Confirm Edge Core starts successfully.
- [ ] Confirm `/health` responds.
- [ ] Confirm `/docs` loads or fails only due to expected authentication.
- [ ] Run the current Python tests.
- [ ] Run the current linting tools.
- [ ] Confirm the Mission Planner plugin still builds on Windows.
- [ ] Add a FastAPI `create_app` boot smoke test.
- [ ] Add a simple line-count report script.
- [ ] Add CI output for largest files and longest lines.
- [ ] Decide on the enforced Python line length: 100 or 120.
- [ ] Update Ruff configuration so line length is actually enforced.
- [ ] Stop ignoring `E501` if line length is meant to matter.
- [ ] Pin the same Ruff version in all relevant tooling.
- [ ] Document which files are excluded from line-count checks.

### Suggested `create_app` Smoke Test

```python
def test_create_app_boots():
    app = create_app(StateManager.instance())
    client = TestClient(app)

    assert client.get("/health").status_code == 200
    assert client.get("/docs").status_code in {200, 401}
```

### Acceptance Criteria

- [ ] A new developer can run the project locally in sim mode.
- [ ] CI reports formatting/linting consistently.
- [ ] The app boot path has at least one automated test.
- [ ] Large-file and long-line problems are visible in CI or a local script.

---

## Phase 2 — Python API Cleanup

### Goal

Make `edge_core/api.py` small and focused.

Current problem:

`api.py` is responsible for too many things:

- Creating the FastAPI app.
- Configuring CORS.
- Reading authentication environment variables.
- Defining API-key middleware.
- Handling internal bridge token behavior.
- Initializing `app.state`.
- Registering routes.

### Target Structure

```text
edge_core/
  api.py                 # create_app only
  api_auth.py            # AuthSettings + APIKeyMiddleware
  api_state.py           # initialize_app_state(app, state_manager)
  api_route_registry.py  # register_all_routes(app, ctx)
```

### Target `create_app` Shape

```python
def create_app(state_manager: StateManager) -> FastAPI:
    app = build_fastapi_app()
    configure_cors(app, CorsSettings.from_env())

    auth = AuthSettings.from_env()
    app.add_middleware(APIKeyMiddleware, settings=auth)

    initialize_app_state(app, state_manager)
    register_all_routes(app, build_route_context(auth))

    return app
```

### Checklist

- [ ] Create `edge_core/api_auth.py`.
- [ ] Move auth constants out of `create_app`.
- [ ] Create an `AuthSettings` dataclass or Pydantic settings model.
- [ ] Move API-key middleware into `api_auth.py`.
- [ ] Add tests for missing API key behavior.
- [ ] Add tests for invalid API key behavior.
- [ ] Add tests for command endpoints requiring real authentication.
- [ ] Add tests for internal bridge token behavior.
- [ ] Create `edge_core/api_state.py`.
- [ ] Move `app.state` initialization into `initialize_app_state`.
- [ ] Create `edge_core/api_route_registry.py`.
- [ ] Move route imports and registration into `register_all_routes`.
- [ ] Keep `api.py` as the app factory only.
- [ ] Run tests after each extraction.
- [ ] Confirm `/health`, `/docs`, and command endpoints behave the same as before.

### Acceptance Criteria

- [ ] `edge_core/api.py` is short and easy to scan.
- [ ] Authentication logic has its own tests.
- [ ] App-state initialization is isolated.
- [ ] Route registration is isolated.
- [ ] No endpoint behavior changes unintentionally.

---

## Phase 3 — Python Runtime Cleanup

### Goal

Make `edge_core/main.py` smaller by separating CLI, runtime startup, cleanup, signal handling, and platform-specific workarounds.

### Target Structure

```text
edge_core/
  main.py              # module-level app and minimal entry point
  cli.py               # argparse only
  runtime.py           # run(), cleanup(), signal handling
  platform/
    jetson.py          # local library preload / Jetson-specific helpers
```

### Checklist

- [ ] Create `edge_core/runtime.py`.
- [ ] Move `run()` into `runtime.py`.
- [ ] Move cleanup handling into `runtime.py`.
- [ ] Move signal handling into `runtime.py`.
- [ ] Create `edge_core/cli.py`.
- [ ] Move argparse logic into `cli.py`.
- [ ] Create `edge_core/platform/jetson.py`.
- [ ] Move local library preload workaround into `platform/jetson.py`.
- [ ] Keep module-level `app = get_app()` available for Uvicorn.
- [ ] Confirm `python -m edge_core.main --sim` still works.
- [ ] Confirm Uvicorn import string still works.
- [ ] Confirm cleanup still runs on SIGINT/SIGTERM.
- [ ] Add or update tests for CLI parsing if practical.

### Acceptance Criteria

- [ ] `main.py` only exposes app creation and entry point behavior.
- [ ] Runtime lifecycle code is isolated.
- [ ] CLI parsing is isolated.
- [ ] Jetson-specific workaround code is not mixed into generic startup code.

---

## Phase 4 — C# Mission Planner UI Cleanup

### Goal

Make Mission Planner plugin files easier to navigate by splitting layout, events, state, rendering, and command logic.

### Target Split Pattern

```text
PanelName.cs
PanelName.Layout.cs
PanelName.Events.cs
PanelName.Commands.cs
PanelName.State.cs
PanelName.Rendering.cs
```

### General Checklist

- [ ] Identify the largest C# files.
- [ ] Split files using existing `partial` class seams.
- [ ] Move layout/control construction into `.Layout.cs`.
- [ ] Move event hookup and event handlers into `.Events.cs`.
- [ ] Move API or command calls into `.Commands.cs` or service classes.
- [ ] Move drawing/map/rendering code into `.Rendering.cs`.
- [ ] Move state synchronization into `.State.cs`.
- [ ] Replace long inline lambdas with named methods.
- [ ] Replace duplicated UI update code with small helper methods.
- [ ] Run `dotnet format`.
- [ ] Build the plugin after each major split.

### Boundary / Geofence UI Checklist

- [ ] Separate geofence layout from geofence upload/export logic.
- [ ] Move map drawing into a rendering-specific file.
- [ ] Move polygon validation into a testable helper.
- [ ] Move upload/export command construction into a service.
- [ ] Add tests for geofence validation if possible.
- [ ] Confirm geofence display still matches the previous behavior.
- [ ] Confirm upload/export still works.

### Payload Panel Checklist

- [ ] Separate payload layout from payload actions.
- [ ] Move arm/release state into a small presenter or state class.
- [ ] Move API calls into `PayloadClient` or equivalent.
- [ ] Keep button handlers short.
- [ ] Make error messages consistent.
- [ ] Confirm camera tilt behavior still works.
- [ ] Confirm payload release command behavior still works.
- [ ] Confirm unsafe states remain blocked.

### Dashboard / Health View Checklist

- [ ] Separate dashboard layout from status polling.
- [ ] Move status parsing into a service or model.
- [ ] Move repeated UI update logic into helpers.
- [ ] Keep periodic update handlers short.
- [ ] Confirm degraded/offline states render clearly.
- [ ] Confirm stale data is handled visibly.

### Terminal Panel Checklist

- [ ] Separate terminal layout from command execution.
- [ ] Move command presets into a dedicated file or class.
- [ ] Move API execution into a client class.
- [ ] Move output formatting into a helper.
- [ ] Keep command submission handler short.
- [ ] Confirm whitelisted commands still work.
- [ ] Confirm blocked commands still fail safely.

### Video / Link UI Checklist

- [ ] Separate video layout from pipeline construction.
- [ ] Move stream URL construction into a helper.
- [ ] Move connection/link status handling into a service.
- [ ] Avoid duplicated RTSP/GStreamer string construction.
- [ ] Confirm primary stream still works.
- [ ] Confirm secondary stream still works.
- [ ] Confirm link status display still works.

### Acceptance Criteria

- [ ] Largest C# UI files are smaller and easier to navigate.
- [ ] Button handlers are mostly one to five lines.
- [ ] API/command logic is no longer buried in layout code.
- [ ] Plugin builds successfully.
- [ ] No UI behavior changes unintentionally.

---

## Phase 5 — Service and Adapter Cleanup

### Goal

Make service boundaries clearer and keep external I/O out of core logic.

### Checklist

- [ ] Identify services that mix decision logic with I/O.
- [ ] Extract pure validation logic into small modules.
- [ ] Extract MAVLink send/receive code into adapter classes.
- [ ] Extract ROS message conversion into adapter helpers.
- [ ] Extract HTTP request construction into client classes.
- [ ] Keep service methods short and named after real actions.
- [ ] Remove duplicate environment parsing.
- [ ] Use common env helpers where appropriate.
- [ ] Remove unused legacy scripts or archive them clearly.
- [ ] Remove stale comments that describe old behavior.

### Suggested Feature Structure

```text
edge_core/
  services/
    payload/
      core.py       # validate command, state transitions
      mavlink.py    # send servo/relay commands
      routes.py     # HTTP wrapper if needed
      models.py     # request/response models
      tests/
```

### Acceptance Criteria

- [ ] Safety decisions can be unit-tested without hardware.
- [ ] External I/O code is easy to find.
- [ ] Services have clear ownership.
- [ ] There is less duplicated config/env parsing.

---

## Phase 6 — Documentation and Config Cleanup

### Goal

Make docs and config easier to maintain by keeping one source of truth.

### Checklist

- [ ] Confirm `config/nomad.env` is the canonical runtime config.
- [ ] Remove references to obsolete `.env` or `jetson.env` flows if they are no longer used.
- [ ] Keep `README.md` short and link to detailed docs.
- [ ] Move long setup explanations into dedicated docs files.
- [ ] Remove duplicated service lists.
- [ ] Make service ownership clear.
- [ ] Add a short architecture overview diagram or text section.
- [ ] Add a "new developer quick start" section.
- [ ] Add a "safe refactor checklist" section.
- [ ] Add a "hardware required vs sim-safe" section.
- [ ] Update docs after each completed phase.

### Acceptance Criteria

- [ ] New members know how to run the project.
- [ ] New members know where major systems live.
- [ ] Runtime config has one clear source of truth.
- [ ] Docs do not duplicate large blocks of information unnecessarily.

---

## Phase 7 — Testing Expansion

### Goal

Increase confidence before changing safety-critical drone behavior.

### Priority Tests

- [ ] FastAPI app boot smoke test.
- [ ] Route registration test.
- [ ] Auth middleware tests.
- [ ] Command endpoint authentication tests.
- [ ] Payload command validation tests.
- [ ] Geofence validation tests.
- [ ] Operational mode transition tests.
- [ ] MAVLink command construction tests.
- [ ] Mission Planner API client tests where practical.
- [ ] SITL tests for behavior that affects aircraft movement.

### SITL / Integration Scenarios

Add or preserve tests for:

- [ ] Geofence containment.
- [ ] Return point behavior.
- [ ] Payload command safety gating.
- [ ] Velocity command loop closure.
- [ ] Link loss or degraded communication behavior.
- [ ] Startup in sim mode.
- [ ] Startup with missing optional hardware.
- [ ] Failure when required real-hardware config is missing.

### Acceptance Criteria

- [ ] Refactors can be reviewed with automated safety checks.
- [ ] Core logic has fast unit tests.
- [ ] Hardware-facing behavior has at least smoke/integration coverage.
- [ ] Safety-critical command paths are not only manually tested.

---

## Phase 8 — Ongoing Maintenance Rules

### Pull Request Checklist

Every PR that touches source code should answer:

- [ ] Did I delete unused code before adding new abstractions?
- [ ] Did I keep decision logic separate from API/UI/MAVLink/ROS adapters?
- [ ] Did I reduce or avoid long lines?
- [ ] Did I avoid adding giant files?
- [ ] Did I add or update tests for changed behavior?
- [ ] Did I keep route handlers thin?
- [ ] Did I keep UI event handlers thin?
- [ ] Did I avoid duplicated environment parsing?
- [ ] Did I update docs if behavior or setup changed?
- [ ] Did I run formatting and tests?

### File Review Checklist

For each file touched:

- [ ] Is the file still focused on one responsibility?
- [ ] Is there a clear reason this file exists?
- [ ] Are long blocks split into named helpers?
- [ ] Are names specific and understandable?
- [ ] Is hardware/API/UI code separated from decision logic?
- [ ] Can the important logic be tested without the drone?
- [ ] Would a new member know where to look next?

### Do Not Merge If

- [ ] A route handler contains hardware command logic directly.
- [ ] A UI click handler contains a full command workflow.
- [ ] A safety decision depends on UI state directly.
- [ ] A new abstraction has only one speculative use.
- [ ] A file grows past the agreed hard line-count cap without justification.
- [ ] Long-line checks are bypassed without explanation.
- [ ] Tests are skipped for safety-critical behavior.

---

## Suggested Tracking Labels

For GitHub issues or project boards:

- `refactor`
- `cleanup`
- `testing`
- `docs`
- `python`
- `csharp`
- `mission-planner`
- `edge-core`
- `safety-critical`
- `good-first-issue`

---

## Suggested Milestones

### Milestone 1 — Baseline Safety

- Tooling fixed.
- App boot test added.
- CI reports line/file size.
- Dev/sim startup confirmed.

### Milestone 2 — Edge Core Split

- `api.py` split.
- `main.py` split.
- Auth tests added.
- Route registration isolated.

### Milestone 3 — Mission Planner Split

- Largest UI files split.
- Button handlers simplified.
- API clients separated.
- Plugin still builds.

### Milestone 4 — Core Logic Tests

- Payload validation tested.
- Geofence validation tested.
- Operational modes tested.
- MAVLink command construction tested.

### Milestone 5 — Documentation Cleanup

- Config docs cleaned.
- Setup docs updated.
- New contributor guide added.
- Architecture overview added.

---

## Final Target State

The codebase should feel like this:

- New members can find the right file quickly.
- Most files have one clear job.
- Route handlers are thin.
- UI event handlers are thin.
- Safety decisions are pure and tested.
- External systems are isolated behind adapters.
- Long lines are rare and intentional.
- Large files are split by responsibility.
- CI catches formatting, boot, and basic safety regressions.

The goal is not "minimum number of lines."

The goal is:

> Small files, short lines, obvious names, and logic that can be tested without the drone.
