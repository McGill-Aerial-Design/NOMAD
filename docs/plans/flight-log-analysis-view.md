# Implementation Plan — Flight Log Analysis View

**Status:** implemented (2026-06-14) · **Scope:** new Mission Planner plugin view · **Author:** NOMAD
**Goal:** a "Log Analysis" page that (a) opens DataFlash logs straight off the drone
or from a local `.bin`/`.log` file, (b) auto-computes a *smart* post-flight summary
(flight time, endurance, current draw, speed/distance, vibration, tune quality,
anomalies), and (c) shows *real-time* tuning telemetry during flight to spot issues
as they happen.

## Implementation outcome

All four phases are implemented on `feature/flight-log-analysis-view`:

- Local `.bin`, `.log`, and `.tlog` import with background analysis.
- Mission Planner's autopilot log downloader plus configurable Jetson SSH/SCP retrieval.
- Streaming, constant-memory DataFlash analysis and bounded native plot series.
- Summary cards, issue list, mode timeline, field browser, zoom/pan/hover plot, MP Log Browse handoff,
  Markdown export, PNG export, and a built-in sample flight.
- Live vibration, attitude, flight, speed, power, GPS/EKF, and RC panels; shared issue rules;
  notifications; optional HUD injection; and CSV recording.
- A Log Analysis settings tab for directories, thresholds, live buffer size, and HUD behavior.
- Pure analyzer/rule tests plus a runtime `DFLogBuffer` smoke test using the committed DataFlash fixture.

---

## 1. Why this is cheap to build well

Mission Planner already ships the exact engine we need, and it is reachable from the
plugin (the assemblies are referenced with full `HintPath`s in
[NOMADPlugin.csproj](../../mission_planner/src/NOMADPlugin.csproj)). Verified by
reflection against the installed `MissionPlanner.Utilities.dll`:

| Type | What it gives us |
|---|---|
| `MissionPlanner.Utilities.DFLogBuffer` | Random-access reader over a `.bin` **or** `.log` (sniffs format). `new DFLogBuffer(string path)` / `(Stream)`, `.Count`, indexer → `DFItem`, `.SeenMessageTypes`, `.FMT`/`.FMTU`/`.Unit`/`.Mult` metadata, `.Dispose()`. |
| `…DFLogBuffer.GetEnumeratorType("VIBE")` | **Typed** iteration over just one message type (or `string[]`), so we never full-scan a 300 MB log to plot one field. Also `GetEnumeratorTypeAll()`. |
| `…DFLogBuffer.GetUnit(msg, field)` | Units → axis labels. |
| `MissionPlanner.Utilities.DFLog.DFItem` | One row: `.msgtype`, `.time` (DateTime), `.timems`, `.instance`, `.items` (string[]), indexer `item["VibeZ"]` (string), `GetRaw<double>("VibeZ")`, `ToDictionary()`. |
| `MissionPlanner.Utilities.BinaryLog` | `ConvertBin(in, out, progress)` (.bin → .log text) and `ReadMessage(Stream, pos)` if we ever need raw access. |
| `MissionPlanner.Log.LogBrowse` | MP's full-featured log grapher — our "open in MP" power-user escape hatch. |
| `MissionPlanner.Log.LogDownloadMavLink` / `LogDownloadscp` | MP's own log-download flows (autopilot SD over MAVLink / SCP) — candidate for the "download from drone" button. |

This is the same parser MP's own Log Browse uses, so we get correct multi-firmware,
multi-instance handling for free. We write the **analysis + UI**, not a log parser.

For the live tab, `MissionPlanner.CurrentState` (in `MissionPlanner.ArduPilot`, the
type behind `MainV2.comPort.MAV.cs`) exposes every field we need, confirmed by
reflection: `vibex/vibey/vibez`, `vibeclip0/1/2`, `roll/pitch/yaw`,
`nav_roll/nav_pitch/nav_bearing/target_bearing`, `groundspeed/airspeed/climbrate/
verticalspeed`, `battery_voltage/current/watts/battery_usedmah`, `ch{1..16}in` /
`ch{1..32}out`, `rssi/remrssi/noise`, `ekfstatus/ekfposhor/ekfposvert/ekfvelv/
ekfcompv`, `wp_dist/wpno`, `sonarrange/rangefinder{1..10}`, `satcount/gpsstatus/hdop`.
We already read these by reflection in
[BatteryHealth.GetCsDouble](../../mission_planner/src/Telemetry/BatteryHealth.cs) —
that helper is the template for a config-driven field reader.

---

## 2. Where it plugs in (mirrors the SLAM3D wiring already in the tree)

A new `NOMADLogView : NOMADViewBase, IUpdatableView`, lazily created and switched
exactly like the other pages. Five touch-points, identical to how `Slam3D` was wired:

1. [NOMADMainScreen.Layout.cs](../../mission_planner/src/Plugin/NOMADMainScreen.Layout.cs) —
   add a `CreateSeparatorLabel("ANALYSIS")` + a `_btnLogs = CreateSidebarButton("Log Analysis")`
   with `Click → ShowView("Logs")`.
2. [NOMADMainScreen.cs](../../mission_planner/src/Plugin/NOMADMainScreen.cs) — add
   `_btnLogs` field, `_logView` field, a `case "Logs":` in `ShowView`, the entry in the
   `UpdateSidebarButtonState` button array + switch, and a `_logView?.Dispose()` in `Dispose`.

The screen's existing update timer already calls `IUpdatableView.UpdateData()` on the
visible view at `HealthPollInterval` — that drives the live tab with **no new timer**.

The page itself is a `TabControl` (same idiom as
[NOMADHealthView](../../mission_planner/src/Views/NOMADHealthView.cs)):

- **Tab 1 — Post-Flight** (offline analysis of a loaded log)
- **Tab 2 — Live Tuning** (streaming telemetry while connected)

Files (partial-class split, matching the plugin's convention):

```
mission_planner/src/Views/NOMADLogView.cs            // shell, tab host, dispose
mission_planner/src/Views/NOMADLogView.Import.cs     // file dialog + drone/Jetson download
mission_planner/src/Views/NOMADLogView.Summary.cs    // quick-analysis cards UI
mission_planner/src/Views/NOMADLogView.Plot.cs       // post-flight field tree + plot host
mission_planner/src/Views/NOMADLogView.Realtime.cs   // live tab (UpdateData tap)

mission_planner/src/Logs/DFLogModel.cs               // DFLogBuffer wrapper (load, types, rows, units)
mission_planner/src/Logs/LogAnalysis.cs              // PURE metric computation → LogSummary
mission_planner/src/Logs/LogSummary.cs               // result DTOs (Metric, Verdict, Anomaly, ModeSpan)
mission_planner/src/Logs/IssueRules.cs               // shared rule engine (post-flight + live)
mission_planner/src/Controls/TimeSeriesPlot.cs       // reusable GDI+ multi-series plot (live + offline)
```

`DFLogModel`/`TimeSeriesPlot` are written as **reusable components** (same posture as
[Controls/](../../mission_planner/src/Controls/README.md)): the plot serves both tabs
and any future telemetry page; `LogAnalysis`/`IssueRules` are pure and host-free.

---

## 3. Loading logs

### 3a. Import a file (Phase 1)
`OpenFileDialog` filtered to `*.bin;*.log;*.tlog`. `.bin`/`.log` → `DFLogModel.Load`
wraps `new DFLogBuffer(path)`. `.tlog` is a raw MAVLink stream (different parser:
`MAVLink.MavlinkParse.ReadPacket`); support it in a later phase or convert on import.

Parsing runs on a background `Task` with a progress bar (logs can be 100s of MB);
`DFLogBuffer` is lazy + random-access so memory stays bounded. UI updates marshal
through [UiAsync](../../mission_planner/src/UI/UiAsync.cs). The buffer is disposed on
reload and on view dispose. One reader thread only (`DFLogBuffer` isn't concurrent).

### 3b. Download "from the drone directly" (Phase 2)
Two real sources — offer both, enabled by what's connected:

- **Autopilot SD card over MAVLink** (`LOG_REQUEST_LIST` → pick latest → `LOG_REQUEST_DATA`
  → save `.bin` → load). First choice: reuse `MissionPlanner.Log.LogDownloadMavLink`
  against `MainV2.comPort`. **Risk:** that class may be UI-coupled — verify; fallback is
  to issue the MAVLink log commands directly, or simply deep-link to MP's built-in
  Download screen and let the user Import the result.
- **From the Jetson** if logs are offloaded to the companion: reuse the plugin's
  existing SSH/SCP ([DualLinkSender.Ssh](../../mission_planner/src/Connectivity/DualLinkSender.Ssh.cs))
  or HTTP ([JetsonApiService](../../mission_planner/src/Connectivity/JetsonApiService.cs))
  to pull a `.bin`/`.log` into temp, then load it through the same path as 3a.

Empty-state when nothing is loaded, with a **"Load sample log"** button (a tiny
synthetic fixture, mirroring `BuildingViewer3D.LoadSampleBuilding`) so the page is
explorable with no hardware.

---

## 4. Smart post-flight summary (the headline feature)

`LogAnalysis.Analyze(DFLogModel) → LogSummary` walks the log **once per needed message
type** via typed enumerators and emits a list of `Metric { Label, Value, Unit, Verdict,
Detail }` plus `Anomaly[]` and a `ModeSpan[]` timeline. Rendered as colored cards
(reuse [NOMADViewBase.CreateCard](../../mission_planner/src/UI/NOMADViewBase.cs)) under
an overall verdict banner ("⚠ 3 issues: high Z vibration, 2 EKF resets, GPS glitch").
Everything degrades gracefully off `SeenMessageTypes` — missing messages just drop
their cards (Copter vs Plane vs firmware differences).

| Card | Source msg → fields | Computed |
|---|---|---|
| **Flight time** | `EV` (Id 10 armed / 11 disarmed), `MODE`, first/last `GPS` | total log span, **armed time**, est. airborne time (alt/throttle gated), takeoff count |
| **Battery & endurance** | `BAT` (`Volt`,`Curr`,`CurrTot`,`Temp`); `PARM` `BATT_CAPACITY` | start/end V, sag, **total mAh**, avg & peak A, avg W, Wh, mAh/min, **est. endurance = capacity ÷ avg draw → "~X min"** |
| **Distance travelled** | `GPS` (`Spd`,`Alt`,`NSats`,`HDop`), `POS` | total horizontal GPS path (Σ haversine between fixes), max & avg groundspeed, altitude span, total climb/descent |
| **Vibration** | `VIBE` (`VibeX/Y/Z`,`Clip0/1/2`) | peak & mean per axis, **clip counts** (any clip = red), % time over 30 / 60 m/s² advisory |
| **Tune quality** | `ATT` (`DesRoll`/`Roll`, `DesPitch`/`Pitch`, `DesYaw`/`Yaw`), `RATE` | RMS attitude/rate tracking error per axis → a "tune score" |
| **GPS / EKF health** | `GPS` (`NSats`,`HDop`), `EKF*`/`NKF*`, `GPA` | min sats, max HDop, EKF variance/innovation peaks, glitch/reset count |
| **Throttle headroom** | `CTUN` (`ThO`), `MOT`/`RCOU` | remaining throttle at peak demand, with average and peak throttle shown explicitly |
| **Anomalies** | `ERR` (subsystem+code), `MSG` text | decoded failsafes, EKF resets, crashes, etc. — the "identify issues" automation |
| **Mode timeline** | `MODE` | time spent per flight mode (mini bar) |

Verdicts use thresholds (defaulted in config, see §7). Battery thresholds reuse the
vehicle-param approach already in
[BatteryHealth](../../mission_planner/src/Telemetry/BatteryHealth.cs) where a live
vehicle is connected; otherwise fall back to `PARM` values baked into the log.

**Numeric extraction note (verify against a known log):** `DFItem.GetRaw<double>(field)`
returns the field value; whether DataFlash multipliers (`.Mult`/`FMTU`) are pre-applied
must be confirmed so engineering units (e.g. `Curr` in A, `Volt` in V) come out right —
apply `.Mult`/`GetUnit` if not. Bake a fixture log into the analyzer tests to pin this.

---

## 5. Interactive plot (post-flight, Phase 1)

`TimeSeriesPlot` extends the GDI+ approach already proven in
[SparklinePanel](../../mission_planner/src/Panels/SparklinePanel.cs) and
[EnhancedHealthDashboard.GraphAndAlerts.cs](../../mission_planner/src/Panels/EnhancedHealthDashboard.GraphAndAlerts.cs)
(no external charting dependency):

- Left: a `TreeView` of `SeenMessageTypes` → fields (from `.FMT`), each a checkbox.
- Checking a field adds a series via `GetEnumeratorType(msg)` (lazy, typed).
- Multiple series with auto-scale or normalized overlay; per-series color + units in
  the legend (`GetUnit`); X axis = `DFItem.time`/`timems`.
- Zoom/pan, hover readout, vertical time cursor shared with the summary.
- **"Open in MP Log Browse"** button hands the file to `MissionPlanner.Log.LogBrowse`
  for users who want the full tool — cheap, and sets expectations for v1's native plot.

---

## 6. Live tuning tab (Phase 3)

Driven by the existing screen timer through `IUpdatableView.UpdateData()` — on each
tick, read `MainV2.comPort.MAV.cs` (reflection reader cloned from `BatteryHealth.
GetCsDouble`), append to rolling ring buffers, and repaint `TimeSeriesPlot` strips.
Empty-state when `MainV2.comPort.BaseStream?.IsOpen != true`.

Live panels (all from confirmed `CurrentState` fields):

- **Vibration** — `vibex/y/z` + `vibeclip0/1/2` counters, warn/crit lines at 30 / 60.
- **Attitude tracking** — `roll` vs `nav_roll`, `pitch` vs `nav_pitch` (live tune feel).
- **Rates / throttle / climb** — `climbrate`, `verticalspeed`, `ch3out` (throttle).
- **Speed** — `groundspeed`, `airspeed`.
- **Power** — `battery_voltage`, `current`, `watts`, `battery_usedmah`; **live endurance estimate**.
- **EKF / GPS** — `ekfposhor/posvert/velv/compv` bars, `satcount`, `hdop`.
- **RC in/out** — `ch{1..N}in` / `ch{1..N}out` mini-bars.

**Live issue detector:** `IssueRules` (the *same* rule engine as the post-flight
anomaly card) runs each tick; trips raise a themed toast via the existing
[NotificationService](../../mission_planner/src/Notifications/NotificationService.cs)
and optionally inject a HUD `STATUSTEXT` via
[TelemetryInjector](../../mission_planner/src/Telemetry/TelemetryInjector.cs)
(e.g. "VIBE Z 45 m/s² — high"). A **"Record session"** toggle dumps the live stream to
CSV for later review.

---

## 7. Config & reuse

- Add a `NOMADConfig` section (see [NOMADConfig.cs](../../mission_planner/src/Config/NOMADConfig.cs)):
  default log directory, vibration/HDOP/tune thresholds, live-tab buffer length,
  auto-inject-to-HUD toggle. Surface in a new Settings tab if it grows.
- `LogAnalysis`, `LogSummary`, `IssueRules` are **pure C#** (no MP types) → directly
  unit-testable and reusable; the live tab and the offline tab share `IssueRules` and
  `TimeSeriesPlot`, so the rule logic and rendering exist once.
- `DFLogModel` isolates the one `DFLogBuffer` dependency behind a small surface
  (`Load`, `MessageTypes`, `Fields(type)`, `Rows(type)`, `Unit(type,field)`), so a
  future swap or a reflection fallback is contained.

---

## 8. Testing & CI

- **`LogAnalysis` / `IssueRules` (pure)** → `csc`-harness tests under
  `mission_planner/tests/logs/`, fitting the existing **"test plugin SC logic
  (Windows)"** gate idiom. Fixture = a small hand-written DataFlash **text** `.log`
  (FMT lines + data rows) committed to the repo. Cases: endurance from capacity+draw,
  distance via haversine, vibe-clip detection, arm/disarm flight time, `ERR` decoding,
  graceful handling of an absent message type.
- **`DFLogModel` + UI** are MP-runtime-bound → covered by the **"build MP plugin
  (Windows)"** compile gate plus a manual pass on a real `.bin` and `.tlog`.
- Keep the SC partition gate green: nothing here imports `edge_core.safety`; this is
  GCS-side analysis only (consistent with the "modelling/operator-facing → groundstation"
  placement principle).

---

## 9. Phasing

| Phase | Status | Deliverable |
|---|---|---|
| **1** | Complete | View shell, imports, summary, native plot, and sample flight. |
| **2** | Complete | Autopilot downloader, Jetson SCP, `.tlog`, and MP Log Browse handoff. |
| **3** | Complete | Live tuning, shared issue rules, notifications/HUD, and CSV recording. |
| **4** | Complete | Settings, mode timeline, tune score, Markdown export, and PNG export. |

---

## 10. Resolved decisions

1. `DFLogModel` reads `DFItem`'s formatted field indexer. The committed fixture smoke test pins
   timestamps, numeric values, strings, parameters, and summary output against the real `DFLogBuffer`.
2. `LogDownloadMavLink` is UI-coupled, so NOMAD opens Mission Planner's authoritative downloader and
   loads the resulting file when it appears in the configured log directory.
3. Both drone sources are supported: autopilot SD over Mission Planner's MAVLink downloader and the
   latest companion-computer log over configured SSH/SCP.
4. The native plot is intentionally lightweight and bounded. DataFlash logs retain an "Open in MP"
   escape hatch for Mission Planner's full Log Browse tool.
