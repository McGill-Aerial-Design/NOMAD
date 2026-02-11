# Mission Planner 3D SLAM Visualization Implementation Plan

**Feature**: Real-time 3D mapping visualization in Mission Planner plugin  
**Status**: Design Phase  
**Priority**: High (improves Task 2 situational awareness)  
**Date**: February 2, 2026

---

## Problem Statement

**Current Setup**:
- nvblox 3D mapping runs on Jetson in ROS2
- Visualization via RViz on Jetson X11 display (not accessible from Windows)
- Not integrated with Mission Planner operator interface

**Requirements**:
1. Real-time 3D map visualization in Mission Planner plugin
2. First-person view (drone perspective)
3. Third-person view (external camera)
4. Acceptable performance (target 10+ Hz)
5. Integrated with existing NOMADPlugin UI

---

## Architecture Overview

### Data Flow
```
Jetson:
  ZED Camera → Isaac ROS Container → nvblox Node → ROS2 Topics
                                          ↓
                                    HTTP Bridge (NEW)
                                          ↓
                                    REST API Endpoint
                                          ↓
Windows Ground Station:                  ↓
  Mission Planner Plugin ←────────── HTTP/WebSocket
           ↓
    3D Viewport Render (Helix Toolkit WPF)
           ↓
    Display (FPV / 3rd Person Camera)
```

### Key Components

**Jetson Side** (Python):
1. ROS2-to-HTTP bridge for mesh streaming
2. FastAPI endpoint `/api/task/2/slam/mesh`
3. Mesh compression (optional: zlib, msgpack)

**Mission Planner Side** (C#):
1. 3D viewport control (Helix Toolkit)
2. HTTP client for mesh data
3. First-person / third-person camera modes
4. Drone position marker

---

## Technology Choices

### C# 3D Rendering Library

| Library | Pros | Cons | Verdict |
|---------|------|------|---------|
| **Helix Toolkit** | WPF 3D, easy WinForms embed, good mesh support | Moderate performance | ⭐ RECOMMENDED |
| OpenTK | OpenGL, best performance | Complex, requires OpenGL knowledge | Future optimization |
| Unity embedded | Professional, beautiful | Overkill, complex integration | Not needed |
| SharpDX | DirectX, best Windows perf | Complex API, low-level | Not needed |

**Decision**: Start with **Helix Toolkit** for MVP (minimum viable product).

**Why**:
- Easy integration with WinForms (Mission Planner uses WinForms)
- Built-in camera controls, lighting, mesh rendering
- Performance adequate for 7-10Hz updates
- Well-documented, active community
- NuGet: `HelixToolkit.Wpf` or `HelixToolkit.SharpDX.Wpf`

---

## Implementation Plan

### Phase 1: ROS2-to-HTTP Bridge (Jetson - Python)

**File**: `edge_core/ros_mesh_bridge.py` (NEW)

```python
import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import Mesh
from fastapi import FastAPI
import asyncio
import json

class MeshBridge(Node):
    def __init__(self):
        super().__init__('mesh_bridge')
        self.subscription = self.create_subscription(
            Mesh,
            '/nvblox_node/mesh',
            self.mesh_callback,
            10)
        self.latest_mesh = None
    
    def mesh_callback(self, msg):
        # Convert ROS Mesh to simplified format
        self.latest_mesh = {
            'vertices': [[v.x, v.y, v.z] for v in msg.vertices],
            'triangles': [[t.vertex_index_1, t.vertex_index_2, t.vertex_index_3] 
                          for t in msg.triangles],
            'colors': [[c.r, c.g, c.b] for c in msg.vertex_colors] if msg.vertex_colors else None
        }
```

**API Endpoint** in `edge_core/api.py`:
```python
@app.get("/api/task/2/slam/mesh", tags=["Task 2"])
async def get_slam_mesh():
    """Stream current 3D SLAM mesh from nvblox."""
    if not ros_mesh_bridge or not ros_mesh_bridge.latest_mesh:
        raise HTTPException(status_code=503, detail="Mesh not available")
    
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mesh": ros_mesh_bridge.latest_mesh,
        "drone_position": {
            "x": state.position.x,
            "y": state.position.y,
            "z": state.position.z
        },
        "drone_attitude": {
            "roll": state.attitude.roll,
            "pitch": state.attitude.pitch,
            "yaw": state.attitude.yaw
        }
    }
```

### Phase 2: Mission Planner 3D Viewport (C# - WPF)

**File**: `mission_planner/src/SLAM3DView.cs` (NEW)

```csharp
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms.Integration;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

public class SLAM3DView : UserControl
{
    private HelixViewport3D viewport;
    private MeshGeometry3D slamMesh;
    private Model3DGroup modelGroup;
    private PerspectiveCamera fpvCamera;  // First-person
    private PerspectiveCamera tpvCamera;  // Third-person
    private bool isFPV = true;
    
    public SLAM3DView()
    {
        InitializeViewport();
        InitializeCameras();
        StartMeshUpdateLoop();
    }
    
    private void InitializeViewport()
    {
        viewport = new HelixViewport3D();
        viewport.ShowCoordinateSystem = true;
        viewport.ShowViewCube = true;
        
        modelGroup = new Model3DGroup();
        viewport.Children.Add(new ModelVisual3D { Content = modelGroup });
        
        Content = viewport;
    }
    
    private void InitializeCameras()
    {
        // FPV: Camera at drone position, looking forward
        fpvCamera = new PerspectiveCamera();
        
        // TPV: Camera behind and above drone
        tpvCamera = new PerspectiveCamera();
        tpvCamera.Position = new Point3D(0, -5, 3);
        tpvCamera.LookDirection = new Vector3D(0, 1, -0.5);
        
        viewport.Camera = tpvCamera;  // Start with third-person
    }
    
    public async void StartMeshUpdateLoop()
    {
        while (true)
        {
            try
            {
                await UpdateMeshFromJetson();
                await Task.Delay(100);  // 10 Hz update rate
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }
    }
    
    private async Task UpdateMeshFromJetson()
    {
        var response = await httpClient.GetAsync("http://100.85.121.98:8000/api/task/2/slam/mesh");
        var data = await response.Content.ReadAsAsync<SLAMMeshData>();
        
        // Update mesh geometry
        UpdateMeshGeometry(data.Mesh);
        
        // Update drone marker
        UpdateDroneMarker(data.DronePosition, data.DroneAttitude);
        
        // Update FPV camera if active
        if (isFPV)
        {
            UpdateFPVCamera(data.DronePosition, data.DroneAttitude);
        }
    }
    
    private void UpdateMeshGeometry(MeshData mesh)
    {
        var geometry = new MeshGeometry3D();
        
        // Add vertices
        foreach (var v in mesh.Vertices)
        {
            geometry.Positions.Add(new Point3D(v[0], v[1], v[2]));
        }
        
        // Add triangles
        foreach (var t in mesh.Triangles)
        {
            geometry.TriangleIndices.Add(t[0]);
            geometry.TriangleIndices.Add(t[1]);
            geometry.TriangleIndices.Add(t[2]);
        }
        
        // Add colors if available
        if (mesh.Colors != null)
        {
            // Convert to texture coordinates or material colors
        }
        
        // Replace old mesh
        modelGroup.Children.Clear();
        var material = new DiffuseMaterial(new SolidColorBrush(Colors.Gray));
        var model = new GeometryModel3D(geometry, material);
        modelGroup.Children.Add(model);
    }
    
    private void UpdateFPVCamera(DronePosition pos, DroneAttitude att)
    {
        fpvCamera.Position = new Point3D(pos.X, pos.Y, pos.Z);
        
        // Calculate look direction from yaw/pitch
        var yaw = att.Yaw * Math.PI / 180;
        var pitch = att.Pitch * Math.PI / 180;
        fpvCamera.LookDirection = new Vector3D(
            Math.Cos(yaw) * Math.Cos(pitch),
            Math.Sin(yaw) * Math.Cos(pitch),
            -Math.Sin(pitch)
        );
    }
    
    public void ToggleCamera()
    {
        isFPV = !isFPV;
        viewport.Camera = isFPV ? fpvCamera : tpvCamera;
    }
}
```

### Phase 3: Integration with NOMADPlugin

**File**: `mission_planner/src/NOMADViews.cs` (MODIFY)

Add to Task 2 View:
```csharp
// In CreateTask2View()
var slam3DView = new SLAM3DView();
var cameraToggle = new Button { Text = "Toggle FPV/TPV" };
cameraToggle.Click += (s, e) => slam3DView.ToggleCamera();

task2Panel.Controls.Add(WpfHostControl(slam3DView));
task2Panel.Controls.Add(cameraToggle);
```

---

## Performance Optimization

### Mesh Data Compression (Phase 4 - Optional)

1. **Delta compression**: Only send changed triangles
2. **LOD (Level of Detail)**: Reduce mesh resolution for distant objects
3. **Octree culling**: Only send visible mesh sections
4. **msgpack**: Binary serialization instead of JSON

**Estimated sizes**:
- Full mesh: ~500KB-2MB (depends on map size)
- Delta updates: ~50-200KB
- Target: <100ms transfer time over Tailscale

### Rendering Optimization

1. **Frustum culling**: Only render mesh in camera view
2. **Mesh simplification**: Reduce triangle count
3. **GPU acceleration**: Use Helix Toolkit SharpDX backend
4. **Update throttling**: Only redraw when mesh actually changes

---

## Testing Plan

### Unit Tests
- ROS2 mesh subscriber (Python)
- Mesh format conversion
- HTTP endpoint response

### Integration Tests
- End-to-end mesh streaming (Jetson → Windows)
- Camera view switching
- Drone marker updates

### Performance Tests
- Measure latency: ROS2 publish → Mission Planner display
- Profile render FPS
- Monitor CPU/memory usage
- Test with various map sizes

### User Acceptance
- Operator can see real-time 3D map
- FPV view matches drone perspective
- TPV view provides good situational awareness
- No lag or stuttering

---

## Dependencies

### Python (Jetson)
```
rclpy>=3.3.0              # ROS2 client
nvblox-msgs               # Mesh message types
msgpack>=1.0.0            # Binary serialization (optional)
```

### C# (Mission Planner)
```xml
<PackageReference Include="HelixToolkit.Wpf" Version="2.24.0" />
<!-- OR for better performance: -->
<PackageReference Include="HelixToolkit.SharpDX.Wpf" Version="2.24.0" />
<PackageReference Include="Newtonsoft.Json" Version="13.0.3" />
```

---

## Milestones

### MVP (Minimum Viable Product) - 2 days
- [x] Design document (this file)
- [x] ROS2-to-HTTP bridge implementation (`edge_core/ros_mesh_bridge.py`)
- [x] Basic Helix Toolkit viewport (`mission_planner/src/SLAM3DView.cs`)
- [x] Mesh streaming and rendering
- [x] Third-person view only

### v2 - First-Person View - 1 day
- [x] FPV camera implementation
- [x] Camera toggle button (FPV/TPV/Orbit)
- [x] Drone marker in 3D space

### v3 - Performance Optimization - 2 days
- [x] Delta updates support
- [ ] Mesh compression
- [ ] Frustum culling
- [ ] Target 10+ Hz

### v4 - Polish - 1 day
- [ ] Better materials/lighting
- [x] Grid overlay
- [ ] Distance measurements
- [ ] Screenshot/recording capability

---

## Alternative Approaches Considered

### 1. RViz Web Bridge
**Idea**: Use rosbridge_suite to forward ROS2 topics to web browser  
**Why not**: Still relies on web technologies, doesn't integrate with Mission Planner native UI

### 2. VNC to Jetson
**Idea**: Stream Jetson X11 display with RViz to Windows  
**Why not**: High bandwidth, introduces compression artifacts, not integrated

### 3. Unity WebGL embedded
**Idea**: Render in Unity, embed WebGL in Mission Planner  
**Why not**: Overkill, complex build pipeline, large runtime

### 4. Direct ROS2 subscription from Windows
**Idea**: Run ROS2 on Windows, subscribe to topics directly  
**Why not**: Heavy dependencies, ROS2 Windows support spotty, HTTP is simpler

---

## Risks and Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Mesh too large for real-time transfer | High | Implement compression, LOD, culling |
| WPF performance poor | Medium | Switch to HelixToolkit.SharpDX backend |
| Mission Planner crashes | High | Separate process for 3D view, error handling |
| ROS2 bridge unstable | Medium | Retry logic, fallback to last known mesh |
| Helix Toolkit learning curve | Low | Start with examples, iterate |

---

## Success Criteria

1. ✓ 3D SLAM map visible in Mission Planner
2. ✓ Update rate ≥10 Hz
3. ✓ FPV and TPV camera modes working
4. ✓ Latency <500ms from ROS2 publish to display
5. ✓ Stable operation for >30 minutes
6. ✓ Operator prefers Mission Planner over external tools

---

## Future Enhancements

- [ ] Path planning overlay (show Nav2 planned path)
- [ ] Waypoint placement in 3D view
- [ ] Target highlighting (fire detected locations)
- [ ] Thermal overlay (if thermal camera added)
- [ ] Recording/playback of 3D mission
- [ ] Multi-drone support (show other drones in map)

---

## References

- [Helix Toolkit Documentation](https://helix-toolkit.github.io/)
- [nvblox ROS2 Package](https://github.com/nvidia-isaac/nvblox)
- [Mission Planner Plugin Development](https://ardupilot.org/planner/docs/common-mp-plugins.html)
- [NOMAD docs/NVBLOX_VISUALIZATION.md](NVBLOX_VISUALIZATION.md)

---

*NOMAD - McGill Aerial Design AEAC 2026*
