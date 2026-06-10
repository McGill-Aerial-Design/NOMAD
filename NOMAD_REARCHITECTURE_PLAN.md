# NOMAD Re-Architecture & Safety Plan

> Goal: a baseline whose **structure is legible**, whose **safety-critical
> surface is isolated and proven**, and which **new AEAC 2027 recruits can
> contribute to easily and safely**. Aviation practice is borrowed only where it
> buys real safety — proportionate to a small UAS, never certification theatre.
>
> This plan sits above [BASELINE_POLISH_PLAN.md](BASELINE_POLISH_PLAN.md)
> (tactical hygiene, done). The **safety case itself lives in
> [docs/safety/](docs/safety/)** — partition, hazards, requirements,
> traceability. This file is only the roadmap; when they disagree,
> `docs/safety/` wins.

---

## 0. Where we actually are (status ledger)

| Phase | What | Status |
|-------|------|--------|
| 0 | Baseline polish P0s (clean tree, smoke test, single wiring path via `ModuleRegistry.wire_safe`, toolchain pinned) | ✅ done |
| 1 | Safety case written: [docs/safety/](docs/safety/) — SC/SR/NC partition, FHA-lite hazards H-01..H-08, requirements SR-*, traceability | ✅ done |
| 2 | SC core extracted: [edge_core/safety/](edge_core/safety/) (`limits`, `gates`, `watchdog`, `envelope`, `geofence` — ~350 lines, pure, zero heavy deps); `mavlink_velocity.py` reduced to a thin I/O adapter | ✅ done |
| 3 | SC core proven (velocity path): 100% branch-coverage CI gate (`pixi run cov-safety`), traceability CI gate, ArduPilot SITL loop-closure scenario ([tests/sitl/](tests/sitl/)) — which caught a real defect (GCS heartbeat flipping the armed/mode gate → SR-VEL-06) | ✅ done |
| — | `GuidedRthLandingController.cs` (664 lines, orphaned — constructed nowhere) | ✅ **deleted**, not extracted. RTH relies on ArduPilot RTL/LAND until a need is proven. |
| 4 | Finish the safety case: geofence wiring, payload interlock, security-as-safety | ◻ next — §4 |
| 5 | Contract seam: client↔API drift gate | ◻ §5 |
| 6 | Partition enforcement + opportunistic C# cleanup | ◻ §6 |
| 7 | Lock-in + onboarding for AEAC 2027 recruits | ◻ §7 |

> **P0 before anything else: commit the in-flight tree.** Phases 1–3 sit
> uncommitted in the working tree (new `edge_core/safety/`, `docs/safety/`,
> `tests/sitl/`, six `test_safety_*` files, the RTH deletion, CI gates). Land it
> as one or a few reviewed commits; a safety case that exists only in a dirty
> tree is not a safety case.

---

## 1. Guiding principles

1. **Criticality drives rigor.** Heavy process only on the thin slice that can
   move the aircraft or fire the payload. Everything else gets good modern
   hygiene. (This is DAL allocation in spirit, and it keeps the effort sane.)
2. **Pure core, thin adapter — the house pattern.** Safety *decisions* are pure
   functions / small frozen dataclasses with zero I/O deps
   (`edge_core/safety/`); I/O shells (threads, sockets, MAVLink, HTTP) snapshot
   state, ask the core, and transmit what it returns. `envelope.evaluate` +
   `mavlink_velocity.py` is the template. **Every future SC path (geofence,
   payload, any RTH revival) copies this shape.**
3. **Determinism at the edges.** Every actuator path has a defined output for
   every failure (stale input, lost link, bad value, mode change), and the
   default is the safe one: zero velocity / hold / relinquish to the FC, which
   remains the certified inner layer.
4. **Deletion is the best refactor.** The orphaned RTH controller was deleted,
   not "extracted" — 664 lines of risk gone. Prefer removing code to
   restructuring it; prefer not writing it to writing it. No speculative
   abstraction, no code for futures that may not come.
5. **Shortest clear code wins.** The SC core is ~350 lines *including*
   docstrings and is 100% branch-covered — that is the size/clarity bar. Small
   pure modules are what make exhaustive testing cheap and what new members can
   actually read.
6. **Single source of truth.** The safety case lives in `docs/safety/` (CI
   keeps traceability honest); limits live in one place in code; the API
   contract is `app.openapi()`. Hand-mirrored copies are defects waiting.
7. **The partition is also the onboarding rail.** SC/SR/NC (see
   [docs/safety/partition.md](docs/safety/partition.md)) tells a recruit
   exactly where they can move fast (NC, modules) and where changes need a
   requirement + test + review (SC). Safety rigor and easy onboarding are the
   same mechanism.

---

## 2. Safety classification

Done — and **maintained in [docs/safety/partition.md](docs/safety/partition.md)**,
not duplicated here. The one rule worth restating, because tooling will enforce
it in §6:

> **NC code may never call into SC code except through a narrow, validated SC
> API. SC code may never import NC code.**

---

## 3. Target structure — what changed since v1 of this plan

- **§3.1 SC core extraction: done** (`edge_core/safety/`). Locked in as the
  house pattern (§1.2); not revisited.
- **§3.4 single wiring path: done** (`ModuleRegistry.wire_safe`, no private
  reaching).
- **RTH state-machine extraction: dropped.** The controller was dead code and
  was deleted. If AEAC 2027 needs a NOMAD-side RTH/landing sequence, build it
  fresh as a pure state machine in the house pattern (decision core +
  thin Form), with requirements in `docs/safety/` *first*. Until then,
  ArduPilot RTL/LAND owns it.
- **Big MVP decomposition of UI hubs: demoted to opportunistic.** Every C# file
  is now under the 800-line cap; `DualLinkSender` is already split into
  partials (`.cs` / `.Http.cs` / `.Ssh.cs`). Pay the remaining debt (the
  `SetStaticConfig` / `SetStaticModuleHost` statics, panel/logic mixing) **when
  touching those files**, not as a project. NC chrome does not earn a
  re-architecture sprint.
- **OpenAPI C# codegen: demoted from default to fallback** — see §5.

What remains is §§4–7 below, in priority order.

---

## 4. Phase 4 — finish the safety case (the real remaining safety work)

Every open item is already named as a GAP / 🟡 / 🔴 in
[docs/safety/requirements.md](docs/safety/requirements.md). In order:

### 4.1 Wire the geofence (SR-FEN-02, hazard H-05 — highest residual risk)
The pure containment primitive exists and is tested
(`safety/geofence.py::is_contained`, point-in-polygon + keep-in margin). Wire it:
- Add the optional boundary polygon (+ margin) to `EnvelopePolicy`; have the
  envelope (or a sibling `evaluate_position`) reject position targets outside
  it. Projection from lat/lon to the planar frame happens in the adapter using
  `services/geospatial.py` — the SC core stays frame-agnostic and pure.
- Enforcement points: `services/mavlink/commands.py::send_global_position_target`
  / `send_position_target` (and velocity-toward-boundary if cheap; the FC fence
  remains the independent backstop).
- Prove it with a SITL scenario (boundary approach → command rejected →
  vehicle contained), added to [tests/sitl/](tests/sitl/) next to the velocity
  loop-closure script.

### 4.2 Extract + interlock the payload path (SR-PAY-01/02/03, H-06)
`ServoController.trigger_water_shooter` / `set_channel_pwm` hold the rules
in-line today. Apply the house pattern:
- `edge_core/safety/payload.py`: pure decision — channel 1–16, PWM 500–2500 µs,
  duration clamp 0.05–5.0 s, and the **arm→confirm interlock** as a small
  explicit state machine (release requires a prior arm within a short window).
  Target: well under 100 lines, like its siblings.
- `servo.py` becomes the adapter; keep the de-energize-in-`finally` guarantee
  there and **add the missing fault-injection test for it** (the pump must
  never stay energized on an exception path).
- C# side: `PayloadControlPanel` release button calls one validated SC method
  (confirm + interlock), separated from panel chrome — a contained change, not
  a panel rewrite.
- Add `safety/payload.py` tests to the `cov-safety` 100%-branch gate.

### 4.3 Security as safety (SR-SEC-01/03, H-07/H-08)
- **SR-SEC-01:** one test asserting the MAVLink command surface
  (`services/mavlink/commands.py`) contains no failsafe-disabling command
  (deny-list of MAV_CMDs / param writes). Cheap, makes "by construction"
  checkable.
- **SR-SEC-03:** command-path endpoints (velocity/payload/mode) require auth
  **even on loopback**, and every SC command is logged with operator identity
  for post-flight audit. Extend `tests/test_auth_middleware.py`.

Exit criteria for Phase 4: zero 🔴 rows in `requirements.md`; every hazard row
in `hazards.md` names a passing test or SITL scenario; `cov-safety` still 100%.

---

## 5. Phase 5 — contract seam, the simple way

`DualLinkSender`'s ~30 hand-written HTTP methods can silently drift from the
FastAPI routes. The hazard is **drift**, not hand-writing — so kill drift with
a test, not a toolchain:

- **Primary: a CI contract test.** Extend the existing
  `tests/test_api_reference_sync.py` approach to the client surface: extract
  the route literals from `DualLinkSender.Http.cs` (one regex over the file in
  a Python test) and assert each exists in `app.openapi()["paths"]`. No dead
  client methods, enforced on every PR, ~50 lines total.
- **Fallback only:** if the endpoint surface keeps growing, generate the client
  (NSwag) into `Connectivity/Generated/`. Until then, codegen is net negative
  for this team — a new toolchain plus thousands of generated lines that would
  themselves need review, versus a stable hand surface that is already split by
  concern (transport/failover · HTTP calls · SSH).

---

## 6. Phase 6 — enforce the partition with tooling

Make the §2 rule a failing test instead of a stated intention, proportionately:

- **Python (cheap, do it):** `tests/test_safety_partition.py` —
  walk `edge_core/safety/*.py` ASTs and assert imports are stdlib +
  `edge_core.safety` only (this also stops FastAPI/ROS/pymavlink ever leaking
  into the core); assert `edge_core.safety` is imported elsewhere only via its
  public API. ~40 lines, no new dependency (`import-linter` optional later).
- **Python SC strictness:** `mypy --strict` scoped to `edge_core/safety/`;
  Ruff `B`, `S`, `G`/`LOG`, `RUF` rule sets on SC files. No `Any`, no bare
  `except` in the core.
- **C# (proportionate):** no analyzer crusade. SC files are few and named in
  `partition.md`; enforce by CODEOWNERS + the SC PR checklist (§7), plus
  `dotnet-format` promoted from advisory to blocking on SC paths.
- **Opportunistic statics cleanup:** when an SC/SR C# file is next touched,
  route its dependencies through the existing context object instead of
  `SetStaticConfig`-style statics. Tracked as debt, not scheduled as a sprint.

---

## 7. Phase 7 — lock-in & AEAC 2027 onboarding

The point of the whole plan: new members contribute on day one without being
able to hurt the aircraft.

**Change control (lock-in):**
- CODEOWNERS rules on SC paths (`edge_core/safety/`, `mavlink_velocity.py`,
  `services/mavlink/commands.py`, `modules/payload/`, `mission_planner/src/Control/`,
  `Payload/`, `Geofence/`) — the file exists; replace the commented stubs with
  real area owners.
- An **SC PR checklist** in the PR template, triggered by those paths: which
  requirement, which test, SITL evidence if the velocity/payload/fence path
  changed. Keep it to five lines — a checklist nobody fills is theatre.
- CI already gates: tests, `cov-safety` 100% branch, traceability sync. Add the
  Phase 5/6 gates as they land. SITL loop-closure stays runnable on demand
  (`pixi run sitl-scenario`) and required (by checklist) for SC changes.

**Onboarding ladder (write once in CONTRIBUTING.md, ~a page):**
1. **Start in modules.** Competition features are modules on the SDK
   (`examples/sample_module/`, `docs/writing_a_module.md`, C# module SDK) —
   isolated, fault-contained, can't touch SC. This is where recruits build
   AEAC 2027 task code.
2. **NC next.** Panels, views, video, docs — normal review, fast merges. Label
   `good-first-issue` here.
3. **SR/SC last, paired.** Requires reading `docs/safety/README.md` (one page),
   a named requirement, a test, and a CODEOWNER review.

Rationale to make explicit to recruits: the tiers are not bureaucracy — they
are *why* most of the repo is fast to change. Competition-specific code stays
in modules so the baseline survives to AEAC 2028.

---

## 8. What *not* to do

- **No DO-178C certification cosplay.** No auditor exists. Borrow practices
  (already done: partition, FHA, traceability-with-CI, robustness testing,
  branch coverage, SITL verification), skip the paperwork.
- **No rewrites of working subsystems** — module SDK, auth middleware, video
  pipeline are good. No MVP re-architecture sprint on NC chrome (§3).
- **No codegen toolchain until drift actually outpaces the contract test** (§5).
- **No rebuilding RTH speculatively** (§3) — ArduPilot RTL/LAND until a real
  AEAC 2027 requirement demands more.
- **No SC rigor on NC code.** 100% branch coverage on the mesh renderer is how
  you make a student team resent safety. The partition exists precisely so you
  don't have to.
- **No new planning documents.** The living safety case is `docs/safety/`;
  status lives there. This file shrinks as phases complete — when Phases 4–7
  land, fold what's left into `docs/safety/README.md` and delete this plan.

---

## Appendix — practice → aviation analogue

| Practice (status) | Analogue |
|---|---|
| SC/SR/NC partition (✅, enforcement §6) | DAL allocation + partitioning (ARP4754A, DO-178C) |
| FHA-lite hazard table (✅) | Functional Hazard Assessment (ARP4761) |
| Requirement→code→test + CI sync (✅) | Bidirectional traceability (DO-178C §6) |
| Fault-injection tests on the SC core (✅) | Robustness / abnormal-range testing (DO-178C §6.4.2) |
| 100% branch coverage on SC core (✅) | Structural coverage, MC/DC-adjacent (DO-178C §6.4.4) |
| SITL loop-closure scenarios (✅ velocity; ◻ fence) | SW/HW integration verification |
| Auth-as-safety + operator audit log (◻ §4.3) | Airworthiness security (DO-326A) |
| Pinned toolchain, reviewed generated output (✅/n.a.) | Tool qualification (DO-330) |
| CODEOWNERS + SC PR checklist (◻ §7) | Configuration management (DO-178C §7) |
