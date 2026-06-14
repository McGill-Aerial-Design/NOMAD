<!--
SPDX-License-Identifier: Apache-2.0
Copyright 2026 The NOMAD Authors
-->
# NOMAD Mission Planner — Reusable Controls

Self-contained WinForms controls meant to be **embedded by pages**, not pages
themselves. They carry no NOMAD wiring of their own: a host view constructs one,
docks it, feeds it data through public methods, and subscribes to its events.

## `BuildingViewer3D`

An OpenTK (OpenGL) `UserControl` that renders a building footprint extruded to a
configured height, a surround search circle, target-marker spheres, and an
optional drone model. Built for the competition task of **building a 3D model of
a structure to locate items/targets on or around it** — but it has no competition
logic baked in; it only knows corners, targets, a building height, and a pose.

Mouse: left-drag orbit, wheel zoom, right-drag pan. Hovering a marker fires
`TargetHovered`; clicking fires `TargetClicked`. With `PlacementMode = true`, a
click on a surface fires `PlacementClicked` with a resolved `Placement`.

### Public API

| Member | Purpose |
| --- | --- |
| `SetCorners(IList<Corner>, centerLat?, centerLon?)` | Set the footprint. Corners are given as lat/lon; the control projects them to a local East/North metre frame and re-centres on the centroid. |
| `SetBuildingHeight(double m)` | Extrusion height of the footprint (min 0.5 m). |
| `SetTargets(IList<Target>)` | Replace the marker set. Targets are local East/North/Up metres in the same frame as the re-centred footprint. `Color` is a name (`red`/`green`/`blue`/`yellow`/`orange`/`purple`/`white`/`black`). |
| `SetDronePoseGps(lat, lon, altAglM, yaw, pitch, roll)` | Position the drone model (needs a footprint first, for the projection origin). |
| `SetHighlightedTarget(string id)` | Highlight a marker (e.g. to mirror a selection in a host grid). |
| `TryGetLocalFromGps(lat, lon, out e, out n)` / `CreatePlacementFromLocal(...)` | GPS↔local helpers for hosts that place markers from map clicks. |
| `DronePovEnabled`, `PlacementMode`, `BuildingHeightM`, `HighlightedTargetId` | View-state properties. |
| events `TargetHovered` / `TargetClicked` / `PlacementClicked` | Host hooks for hover/select/place. |
| `LoadSampleBuilding()` | Populate a synthetic 20×15 m building + markers + pose. Demo/dev only — drop the control in a page and call this to smoke-test rendering without a live data source. |

### Embedding it in a page

```csharp
var viewer = new BuildingViewer3D { Dock = DockStyle.Fill };
viewer.TargetClicked += id => Log.Info($"target {id}");
host.Controls.Add(viewer);

viewer.SetBuildingHeight(6.0);
viewer.SetCorners(corners);          // corners from the boundary/footprint source
viewer.SetTargets(targets);          // markers from detections
// — or, to verify the embed with no data wired yet:
viewer.LoadSampleBuilding();
```

Via the module SDK (see [`../Core`](../Core/README.md)) a feature module can
surface a page hosting it without editing `NOMADMainScreen`:

```csharp
yield return NomadViewDescriptor.View(
    "Building3D", "Building 3D", "Building Model", "TOOLS",
    () => { var v = new BuildingViewer3D { Dock = DockStyle.Fill }; v.LoadSampleBuilding(); return v; });
```

### Dependencies

OpenTK + OpenTK.GLControl (already referenced in `NOMADPlugin.csproj`; they ship
with Mission Planner) and the shared `SLAM3D.Rendering.DroneRenderer`. No NOMAD
services, config, or network clients — feed it data and it renders.

## `SnapshotManager`

A self-contained image-gallery `UserControl`: scans a snapshot directory, shows
thumbnails, and supports preview / open / delete. Useful for any task that
captures stills (target capture, inspection photos, payload-drop evidence).

```csharp
var gallery = new SnapshotManager(config) { Dock = DockStyle.Fill };
host.Controls.Add(gallery);
gallery.LoadSnapshots();          // populate from the configured snapshot dir
gallery.AddSnapshot(@"C:\shot.jpg"); // copy a new still in + refresh
```

`SnapshotInfo` (`FilePath` / `FileName` / `CaptureTime`) is the record type it
surfaces. Pass a `NOMADConfig` so it resolves the snapshot directory; `null` is
tolerated (falls back to defaults).

## `JetsonHealthTab`

A compact CPU/GPU load + temperature + status poller (`UserControl`) that reads
the Jetson `/health` API on a timer. A lightweight, embeddable alternative to the
full `Panels/EnhancedHealthDashboard` — drop it into the corner of a task page
when you just want an at-a-glance health readout.

```csharp
var health = new JetsonHealthTab(pollIntervalMs: 2000) { Dock = DockStyle.Fill };
host.Controls.Add(health);   // self-polls JetsonApiService.BaseUrl
```

## Reusable utilities (live outside this folder)

These are page-agnostic too, but they are not controls so they stay in their
domain folders:

- **`Telemetry/TelemetryInjector`** — pushes short status strings to the Mission
  Planner HUD message log (`SendVisionStatus`, `SendTargetStatus`, `SendStatus`,
  `SendCustomStatus`). Construct one and call it from any task that wants operator-
  visible status (e.g. `injector.SendTargetStatus(isLocked: true, "ArUco 12")`).
- **`UI/SafeDispose`** — `SafeDispose.Dispose(obj)` / `StopAndDispose(timer)` null-safe
  disposal helpers for control teardown.
