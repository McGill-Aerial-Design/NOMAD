# Nav2 Integration Architecture Plan
## NOMAD Task 2 - Full ROS2 Navigation Stack

> ⚠️ **HISTORICAL / NOT USED IN COMPETITION (May 2026).**
> Nav2 was removed from the flight plan. Task 2 is flown manually
> indoors, with one autonomous spray sequence on an outdoor target.
> Positioning is GPS + barometer + optical flow — no VIO, no Nav2.
> Document retained for design history only.

**Version**: 1.0  
**Date**: January 29, 2026  
**Status**: Historical (superseded May 2026)

---

## Executive Summary

This document outlines the migration from custom Python navigation to a full ROS2 Nav2 stack with direct MAVLink integration. The key change is **eliminating the nav_controller.py middleware** and using **mavros2** for direct ROS-to-MAVLink communication.

### Key Benefits
- ✅ **Microsecond latency** - No Python API serialization overhead
- ✅ **Battle-tested algorithms** - DWB controller, Smac planner, nvblox integration
- ✅ **Standard tooling** - ROS2 ecosystem support
- ✅ **Minimal custom code** - Focus on mission logic, not control loops
- ✅ **Professional autonomy** - Obstacle avoidance, dynamic replanning, recovery behaviors

---

## System Architecture

### High-Level Overview

```
┌────────────────────────────────────────────────────────────────┐
│  ROS2 NETWORK (Jetson Orin Nano)                              │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  ┌──────────────┐         ┌──────────────┐                   │
│  │ ZED Wrapper  │         │ Isaac ROS    │                   │
│  │              │────────→│ nvblox       │                   │
│  │ - Odom       │ depth   │              │                   │
│  │ - Depth      │ +odom   │ - 3D mapping │                   │
│  │ - RGB        │         │ - Costmap    │                   │
│  │ - YOLOv26    │         └──────┬───────┘                   │
│  └──────┬───────┘                │                            │
│         │                        │ /global_costmap            │
│         │ /odom                  │                            │
│         │                        ▼                            │
│         │         ┌──────────────────────────┐               │
│         └────────→│ Nav2 Stack               │               │
│                   │                          │               │
│                   │ - Controller (DWB)       │               │
│                   │ - Planner (Smac)         │               │
│                   │ - Costmap Layers         │               │
│                   │ - Behavior Tree          │               │
│                   │ - Recovery Behaviors     │               │
│                   └──────────┬───────────────┘               │
│                              │                                │
│                              │ /cmd_vel (Twist)               │
│                              ▼                                │
│         ┌─────────────────────────────────┐                  │
│         │ mavros2 (MAVLink-ROS2 Bridge)   │                  │
│         │                                 │                  │
│         │ - /cmd_vel subscriber           │                  │
│         │ - SET_POSITION_TARGET_LOCAL_NED │                  │
│         │ - VISION_POSITION_ESTIMATE      │                  │
│         └─────────────┬───────────────────┘                  │
│                       │                                       │
│  ┌────────────────────┴────────────────────┐                │
│  │ Edge Core (Reduced Scope)               │                │
│  │                                          │                │
│  │ - HTTP API (FastAPI)                    │                │
│  │ - WASD velocity publisher               │                │
│  │ - Mission orchestrator                  │                │
│  │ - Health monitoring                     │                │
│  │ - NOT navigation control                │                │
│  └──────────────────────────────────────────┘               │
│                                                                │
└────────────────────────────────────────────────────────────────┘
                         │ MAVLink (UART)
                         ▼
                ┌─────────────────┐
                │ Cube Orange     │
                │ ArduPilot       │
                │ GUIDED mode     │
                └─────────────────┘
```

---

## Component Breakdown

### 1. ZED ROS Wrapper

**Package**: `zed-ros2-wrapper`  
**Running**: Docker container or native on Jetson

**Topics Published**:
- `/zed/zed_node/odom` (nav_msgs/Odometry) - VIO odometry for nav2 + nvblox
- `/zed/zed_node/depth/depth_registered` - Depth for nvblox 3D mapping
- `/zed/zed_node/rgb/image_rect_color` - Rectified RGB for visualization
- `/zed/zed_node/obj_det/objects` (zed_interfaces/ObjectsStamped) - YOLOv26 detections
- `/zed/zed_node/point_cloud/cloud_registered` - Point cloud (optional)

**Configuration**:
```yaml
# zed_config.yaml
general:
  camera_model: 'zed2i'
  
pos_tracking:
  pos_tracking_enabled: true
  imu_fusion: true
  area_memory: false  # Task 2 indoor - single session
  
object_detection:
  od_enabled: true
  model: 'CUSTOM'  # YOLOv26
  confidence_threshold: 0.5
  
depth:
  depth_mode: 'NEURAL'  # Best quality for indoor
  depth_stabilization: true
```

---

### 2. Isaac ROS nvblox

**Package**: `nvblox_nav2`  
**Running**: Isaac ROS Docker container

**Topics Subscribed**:
- `/zed/zed_node/odom` - Odometry for localization
- `/zed/zed_node/depth/depth_registered` - Depth for 3D reconstruction

**Topics Published**:
- `/nvblox_node/map` (nvblox_msgs/DistanceMapSlice) - 3D occupancy map
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid) - 2D costmap for Nav2
- `/nvblox_node/mesh` (nvblox_msgs/Mesh) - Visualization mesh

**Configuration**:
```yaml
# nvblox_performance.yaml
nvblox_node:
  ros__parameters:
    voxel_size: 0.15  # 15cm - revised from 0.05 for Orin Nano performance
    esdf_mode: '3D'
    mapping_type: 'static_tsdf'  # Indoor static environment
    max_integration_distance: 8.0
    max_obstacle_distance: 2.0  # Important for costmap
```

---

### 3. Nav2 Stack

**Package**: `nav2_bringup`  
**Running**: Native on Jetson

#### 3.1 Controller Server (DWB Local Planner)

**Purpose**: Generate velocity commands from global plan

**Configuration**:
```yaml
# controller_server.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Hz
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 1.0  # Conservative for indoor
      max_vel_y: 0.5  # Holonomic support
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 1.0
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.5
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: -0.5
      decel_lim_theta: -1.0
      vx_samples: 20
      vy_samples: 10
      vtheta_samples: 20
      sim_time: 1.5
      
      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "BaseObstacle"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"
```

#### 3.2 Planner Server (Smac Planner)

**Purpose**: Generate global path from start to goal

**Configuration**:
```yaml
# planner_server.yaml
planner_server:
  ros__parameters:
    planner_frequency: 1.0  # Hz - replan rate
    
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.1  # Goal tolerance (meters)
      downsample_costmap: false  # Use full resolution
      allow_unknown: false  # Don't plan through unknown space
      max_planning_time: 5.0
      motion_model_for_search: "REEDS_SHEPP"  # Ackermann-like
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      smooth_path: true
```

#### 3.3 Behavior Tree Navigator

**Purpose**: High-level mission logic

**Configuration**:
```yaml
# bt_navigator.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /zed/zed_node/odom
    
    default_nav_to_pose_bt_xml: "$(find nomad_behaviors)/behavior_trees/nav_to_pose.xml"
    default_nav_through_poses_bt_xml: "$(find nomad_behaviors)/behavior_trees/nav_through_poses.xml"
    
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nomad_engage_target_action_bt_node  # Custom plugin
      - nomad_check_exclusion_bt_node  # Custom plugin
```

---

### 4. mavros2 (MAVLink-ROS2 Bridge)

**Package**: `mavros`  
**Running**: Native on Jetson

**Topics Subscribed**:
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands from Nav2

**Topics Published**:
- `/mavros/state` (mavros_msgs/State) - ArduPilot state
- `/mavros/local_position/odom` (nav_msgs/Odometry) - ArduPilot's position estimate

**MAVLink Messages Sent**:
- `SET_POSITION_TARGET_LOCAL_NED` - Velocity commands to ArduPilot
- `VISION_POSITION_ESTIMATE` - VIO data to ArduPilot EKF

**Configuration**:
```yaml
# mavros_config.yaml
mavros:
  ros__parameters:
    fcu_url: "/dev/ttyACM0:921600"  # Serial to Cube Orange
    gcs_url: "udp://:14560@100.76.127.17:14560"  # Tailscale to GCS
    target_system_id: 1
    target_component_id: 1
    
    setpoint_velocity:
      mav_frame: "BODY_NED"  # Body frame for velocity control
      
    vision_pose:
      tf:
        frame_id: "map"
        child_frame_id: "base_link"
      vision_position_estimate:
        enabled: true
```

**Key Functions**:
1. Subscribe to `/cmd_vel` (Twist messages from Nav2)
2. Convert Twist → `SET_POSITION_TARGET_LOCAL_NED` MAVLink
3. Forward ZED odometry → `VISION_POSITION_ESTIMATE` MAVLink
4. Publish ArduPilot state to ROS

---

### 5. Edge Core (Reduced Scope)

**Language**: Python 3.13  
**Framework**: FastAPI  
**Running**: Native on Jetson

**New Responsibilities** (ONLY):
1. HTTP API for ground station
2. WASD velocity publisher (HTTP → ROS /cmd_vel)
3. Mission orchestrator (start/stop Nav2 actions)
4. Health monitoring (ROS nodes, system metrics)
5. VIO watchdog (restart ZED wrapper if hung)

**Removed Responsibilities**:
- ❌ Navigation command translation (mavros2 handles this)
- ❌ Velocity command safety limits (Nav2 handles this)
- ❌ Command timeouts (Nav2 handles this)
- ❌ VIO frame transformations (TF2 handles this)

#### 5.1 New Edge Core Structure

```python
# edge_core/main.py
from fastapi import FastAPI
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

app = FastAPI()

class EdgeCoreOrchestrator:
    """Lightweight mission orchestrator - NOT navigation controller."""
    
    def __init__(self):
        # ROS2 node for publishing/subscribing
        rclpy.init()
        self.node = rclpy.create_node('edge_core_orchestrator')
        
        # WASD control publisher (overrides Nav2 temporarily)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Nav2 action client for high-level commands
        self.nav2_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        # Health monitoring
        self.health_monitor = HealthMonitor()
    
    def publish_wasd_velocity(self, vx: float, vy: float, vyaw: float):
        """Publish WASD velocity command directly to /cmd_vel."""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vyaw
        self.cmd_vel_pub.publish(twist)
    
    def start_autonomous_search(self):
        """High-level: Start Nav2 autonomous navigation."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = 5.0
        goal.pose.pose.position.y = 0.0
        self.nav2_client.send_goal_async(goal)
    
    def stop_navigation(self):
        """Cancel current Nav2 goal."""
        self.nav2_client.cancel_goal()

# HTTP API
@app.post("/api/nav/velocity")
async def wasd_velocity(vx: float, vy: float, vyaw: float):
    """WASD nudge control from ground station."""
    orchestrator.publish_wasd_velocity(vx, vy, vyaw)
    return {"status": "sent"}

@app.post("/api/task2/start")
async def start_task2():
    """Start autonomous Task 2 navigation."""
    orchestrator.start_autonomous_search()
    return {"status": "started"}
```

---

## Data Flow Diagrams

### Autonomous Navigation Flow

```
1. Mission Start
   ↓
2. Ground Station → HTTP POST /api/task2/start
   ↓
3. Edge Core → Nav2 Action: NavigateToPose(search_pattern)
   ↓
4. Nav2 Planner → Generate global path from costmap
   ↓
5. Nav2 Controller → /cmd_vel (Twist) @ 20Hz
   ↓
6. mavros2 → SET_POSITION_TARGET_LOCAL_NED @ 20Hz
   ↓
7. ArduPilot → Attitude control, motor mixing
   ↓
8. Drone moves
```

### WASD Nudge Control Flow

```
1. Ground Station → HTTP POST /api/nav/velocity {vx, vy, vyaw}
   ↓
2. Edge Core → Publish to /cmd_vel
   ↓
3. mavros2 → SET_POSITION_TARGET_LOCAL_NED
   ↓
4. ArduPilot → Execute velocity command
   ↓
5. Drone moves (Nav2 temporarily overridden)
```

**Note**: When WASD publishes to `/cmd_vel`, Nav2's output is effectively overridden since both publish to the same topic. Priority is "last publisher wins."

### Target Engagement Flow

```
1. ZED YOLOv26 → /zed/zed_node/obj_det/objects
   ↓
2. Custom Behavior Plugin → Detect target
   ↓
3. Check Exclusion Map → Not previously engaged?
   ↓
4. Behavior Tree → Navigate closer + center target
   ↓
5. Aiming solution met → Trigger water pump (MAVLink)
   ↓
6. Add to Exclusion Map → Costmap inflation layer
   ↓
7. Continue search
```

---

## Frame Transformations (TF2)

**Critical**: Proper frame transformations required for Nav2.

```
map (VIO origin)
  └─ odom (moving frame)
       └─ base_link (drone body)
            └─ zed_camera_link (ZED camera)
```

**Who Publishes What**:
- `map → odom`: ZED ROS wrapper (VIO localization)
- `odom → base_link`: ZED ROS wrapper (odometry)
- `base_link → zed_camera_link`: Static transform (launch file)

**Static Transform**:
```xml
<!-- Base link to ZED camera -->
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="0.1 0 0.05 0 0 0 base_link zed_camera_link"/>
```

---

## Custom Behavior Tree Plugins

### 1. Target Engagement Plugin

```cpp
// nomad_behaviors/src/engage_target_action.cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"

class EngageTargetAction : public nav2_behavior_tree::BtActionNode<EngageTargetAction>
{
public:
  EngageTargetAction(const std::string & name, const BT::NodeConfiguration & config)
    : BtActionNode<EngageTargetAction>(name, config)
  {
    // Subscribe to YOLOv26 detections
    detection_sub_ = node_->create_subscription<zed_interfaces::msg::ObjectsStamped>(
      "/zed/zed_node/obj_det/objects", 10,
      std::bind(&EngageTargetAction::detection_callback, this, std::placeholders::_1)
    );
  }

  void detection_callback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
  {
    for (const auto & obj : msg->objects) {
      if (obj.label == "Purple" && !is_excluded(obj.position)) {
        current_target_ = obj;
        target_detected_ = true;
      }
    }
  }

  BT::NodeStatus tick() override
  {
    if (!target_detected_) {
      return BT::NodeStatus::FAILURE;  // No target found
    }

    // Approach target
    if (!is_in_range(current_target_)) {
      publish_approach_velocity();
      return BT::NodeStatus::RUNNING;
    }

    // Center target (visual servoing)
    if (!is_centered(current_target_)) {
      publish_centering_velocity();
      return BT::NodeStatus::RUNNING;
    }

    // Fire water pump
    trigger_payload();
    add_to_exclusion_map(current_target_.position);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr detection_sub_;
  zed_interfaces::msg::Object current_target_;
  bool target_detected_ = false;
};
```

### 2. Exclusion Map Service

```cpp
// nomad_behaviors/src/exclusion_map_service.cpp
#include "rclcpp/rclcpp.hpp"
#include "nomad_msgs/srv/add_exclusion.hpp"

class ExclusionMapService : public rclcpp::Node
{
public:
  ExclusionMapService() : Node("exclusion_map_service")
  {
    service_ = this->create_service<nomad_msgs::srv::AddExclusion>(
      "add_exclusion",
      std::bind(&ExclusionMapService::handle_add_exclusion, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Publisher to add obstacle to costmap
    costmap_update_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap_updates", 10
    );
  }

private:
  void handle_add_exclusion(
    const std::shared_ptr<nomad_msgs::srv::AddExclusion::Request> request,
    std::shared_ptr<nomad_msgs::srv::AddExclusion::Response> response)
  {
    // Add position to exclusion list
    excluded_positions_.push_back(request->position);

    // Inflate costmap around engaged target (0.5m radius)
    inflate_costmap_at(request->position, 0.5);

    response->success = true;
  }

  void inflate_costmap_at(const geometry_msgs::msg::Point & pos, double radius)
  {
    // Create temporary obstacle in local costmap
    // This prevents Nav2 from planning paths through engaged targets
    // Implementation: publish OccupancyGrid update with lethal obstacle
  }

  std::vector<geometry_msgs::msg::Point> excluded_positions_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_update_pub_;
  rclcpp::Service<nomad_msgs::srv::AddExclusion>::SharedPtr service_;
};
```

---

## Launch Configuration

### Master Launch File

```python
# nomad/launch/task2_autonomous.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # 1. ZED Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed2i.launch.py'
            ]),
            launch_arguments={
                'config_path': './config/zed_config.yaml'
            }.items()
        ),

        # 2. Isaac ROS nvblox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nvblox_nav2'),
                '/launch/nvblox_nav2.launch.py'
            ]),
            launch_arguments={
                'config': './config/nvblox_performance.yaml'
            }.items()
        ),

        # 3. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'params_file': './config/nav2_params.yaml'
            }.items()
        ),

        # 4. mavros2
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[
                './config/mavros_config.yaml'
            ],
            remappings=[
                ('/mavros/setpoint_velocity/cmd_vel_unstamped', '/cmd_vel')
            ]
        ),

        # 5. Custom Behavior Tree Plugins
        Node(
            package='nomad_behaviors',
            executable='exclusion_map_service',
            name='exclusion_map_service'
        ),

        # 6. Static TF: base_link → zed_camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'zed_camera_link']
        ),

        # 7. Edge Core (FastAPI - separate systemd service)
        # Started via: systemctl start nomad-edge-core
    ])
```

---

## Migration Plan

### Phase 1: Setup ROS2 Environment (Week 1)

**Tasks**:
1. Install Nav2 packages
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. Install mavros2
   ```bash
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
   ```

3. Install Isaac ROS (Docker)
   ```bash
   cd ~/workspaces/isaac_ros-dev
   ./scripts/dev/run_dev.sh
   ```

4. Install ZED ROS2 wrapper
   ```bash
   cd ~/ros2_ws/src
   git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
   cd ~/ros2_ws && colcon build --packages-select zed_wrapper
   ```

### Phase 2: Configuration (Week 1-2)

**Tasks**:
1. Configure ZED wrapper for VIO + YOLOv26
2. Configure nvblox for indoor 3D mapping
3. Configure Nav2 parameters (DWB, Smac, Costmap)
4. Configure mavros2 for ArduPilot GUIDED mode
5. Set up TF tree (static transforms)

### Phase 3: Custom Plugins (Week 2-3)

**Tasks**:
1. Create `nomad_behaviors` ROS2 package
2. Implement `EngageTargetAction` behavior tree node
3. Implement `ExclusionMapService` ROS2 service
4. Create custom behavior tree XML
5. Test plugins in simulation

### Phase 4: Edge Core Refactor (Week 3)

**Tasks**:
1. Remove `nav_controller.py` (no longer needed)
2. Simplify `mavlink_interface.py` (only health broadcast, mode changes)
3. Create `ros_orchestrator.py` (high-level Nav2 action client)
4. Implement WASD velocity publisher (HTTP → /cmd_vel)
5. Update HTTP API endpoints

**Files to Modify**:
- `edge_core/main.py` - Integrate ROS2 node
- `edge_core/api.py` - Update endpoints for Nav2 actions
- **KEEP**: `edge_core/nav_controller.py` (still in use)
- **SIMPLIFY**: `edge_core/mavlink_interface.py`
- **NEW**: `edge_core/ros_orchestrator.py` (NOT YET CREATED)

### Phase 5: Integration Testing (Week 4)

**Test Sequence**:
1. **VIO Test**: Verify VISION_POSITION_ESTIMATE sent to ArduPilot
2. **Nav2 Test**: Send NavigateToPose goal, verify /cmd_vel output
3. **mavros2 Test**: Verify SET_POSITION_TARGET_LOCAL_NED sent to ArduPilot
4. **WASD Test**: HTTP → /cmd_vel → mavros2 → ArduPilot
5. **Target Engagement**: Detect → Approach → Center → Fire → Exclude
6. **Full Mission**: Start → Search → Engage 3 targets → RTL

### Phase 6: Competition Deployment (Week 5)

**Pre-Flight Checklist**:
- [ ] All ROS2 nodes launching successfully
- [ ] TF tree valid (no transform errors)
- [ ] VIO streaming to ArduPilot EKF
- [ ] ArduPilot in GUIDED mode
- [ ] Nav2 receiving costmap from nvblox
- [ ] WASD controls responding < 100ms latency
- [ ] YOLOv26 detecting targets at 10+ FPS
- [ ] Exclusion map service running
- [ ] Ground station RTSP video streaming

---

## ArduPilot Configuration

### Required Parameters

```
# Source Set 2 (Indoor VIO)
EK3_SRC2_POSXY = 6    # ExternalNav (VISION_POSITION_ESTIMATE)
EK3_SRC2_POSZ = 6     # ExternalNav
EK3_SRC2_VELXY = 6    # ExternalNav
EK3_SRC2_VELZ = 6     # ExternalNav
EK3_SRC2_YAW = 6      # ExternalNav (VIO yaw)

# Disable magnetometer indoors
EK3_SRC2_MAG = 0      # None

# GUIDED mode velocity limits
WPNAV_SPEED = 100     # 1.0 m/s
WPNAV_ACCEL = 50      # 0.5 m/s^2

# Failsafe: Switch to Source Set 1 (Baro/IMU) if VIO fails
EK3_SRC_OPTIONS = 1   # Enable source switching

# VIO variance thresholds for failsafe
EK3_VELNE_M_NSE = 0.5
EK3_VELD_M_NSE = 0.7
EK3_POSNE_M_NSE = 1.0
```

---

## WASD Control Implementation

### Ground Station Plugin (C#)

```csharp
// mission_planner/src/EnhancedWASDControl.cs (existing file - adapt this pattern)
public class WASDNavControl : Form
{
    private HttpClient _httpClient = new HttpClient();
    private string _jetsonApiUrl = "http://100.85.121.98:8000";
    
    protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
    {
        float vx = 0, vy = 0, vyaw = 0;
        float speed = 0.5f;  // m/s

        switch (keyData)
        {
            case Keys.W: vx = speed; break;   // Forward
            case Keys.S: vx = -speed; break;  // Backward
            case Keys.A: vy = -speed; break;  // Left
            case Keys.D: vy = speed; break;   // Right
            case Keys.Q: vyaw = 0.5f; break;  // Yaw left
            case Keys.E: vyaw = -0.5f; break; // Yaw right
            case Keys.Space: SendStopCommand(); return true;
            default: return base.ProcessCmdKey(ref msg, keyData);
        }

        SendVelocityCommand(vx, vy, vyaw);
        return true;
    }

    private async void SendVelocityCommand(float vx, float vy, float vyaw)
    {
        var payload = new { vx, vy, vyaw };
        var json = JsonConvert.SerializeObject(payload);
        var content = new StringContent(json, Encoding.UTF8, "application/json");
        
        await _httpClient.PostAsync($"{_jetsonApiUrl}/api/nav/velocity", content);
    }

    private async void SendStopCommand()
    {
        await SendVelocityCommand(0, 0, 0);
    }
}
```

### Edge Core ROS Publisher

```python
# edge_core/ros_orchestrator.py
import rclpy
from geometry_msgs.msg import Twist

class ROSOrchestrator:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('edge_core_orchestrator')
        
        # Publisher to /cmd_vel (same topic Nav2 publishes to)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Watchdog: Stop publishing after 500ms if no new command
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.node.create_timer(0.1, self.watchdog_callback)
    
    def publish_velocity(self, vx: float, vy: float, vyaw: float):
        """
        Publish WASD velocity command.
        
        This OVERRIDES Nav2 output temporarily since both publish to /cmd_vel.
        Nav2 will resume control when WASD stops sending commands.
        """
        twist = Twist()
        twist.linear.x = vx    # Forward
        twist.linear.y = vy    # Lateral
        twist.angular.z = vyaw  # Yaw rate
        
        self.cmd_vel_pub.publish(twist)
        self.last_cmd_time = time.time()
    
    def watchdog_callback(self):
        """Stop robot if no WASD command for 500ms."""
        if time.time() - self.last_cmd_time > 0.5:
            self.publish_velocity(0, 0, 0)
```

**Behavior**:
- WASD publishes to `/cmd_vel` → overrides Nav2 temporarily
- When WASD stops → Nav2 resumes publishing to `/cmd_vel`
- mavros2 always subscribes to `/cmd_vel` regardless of source

---

## Testing & Verification

### Unit Tests

1. **TF Tree Test**
   ```bash
   ros2 run tf2_tools view_frames
   # Verify: map → odom → base_link → zed_camera_link
   ```

2. **Topic Test**
   ```bash
   ros2 topic echo /cmd_vel
   # Verify Nav2 publishing Twist messages at 20Hz
   ```

3. **MAVLink Test**
   ```bash
   rostopic echo /mavros/setpoint_raw/target_local
   # Verify SET_POSITION_TARGET_LOCAL_NED being sent
   ```

### Integration Tests

1. **VIO to ArduPilot**
   - Move drone manually
   - Monitor Mission Planner HUD position
   - Verify EKF using ExternalNav (no GPS)

2. **Nav2 Navigation**
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}}"
   ```
   - Verify drone moves to target

3. **WASD Override**
   - Start autonomous navigation
   - Press W key on ground station
   - Verify drone responds immediately
   - Release W key
   - Verify Nav2 resumes control

4. **Target Engagement**
   - Place purple target 2m in front of drone
   - Start autonomous mode
   - Verify: Detect → Approach → Center → Fire → Exclude → Continue

---

## Rollback Plan

If Nav2 integration fails during competition:

**Fallback**: Revert to original nav_controller.py

1. `git checkout main` (before Nav2 branch)
2. Restart Edge Core with old nav_controller
3. Use WASD controls directly via nav_controller
4. Manual target engagement via HTTP API

**Time to Rollback**: < 5 minutes

---

## Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| **VIO to ArduPilot** | 30 Hz | `ros2 topic hz /mavros/vision_pose/pose` |
| **Nav2 /cmd_vel Rate** | 20 Hz | `ros2 topic hz /cmd_vel` |
| **MAVLink Latency** | < 10ms | Timestamp diff in SET_POSITION_TARGET |
| **WASD Response** | < 100ms | HTTP POST → Movement observed |
| **Target Detection** | 10 FPS | ZED YOLOv26 inference rate |
| **Costmap Update** | 5 Hz | nvblox publish rate |
| **CPU Usage** | < 80% | `htop` on Jetson |
| **GPU Usage** | < 90% | `tegrastats` |

---

## Dependencies

### ROS2 Packages

```bash
# Core Nav2
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-nav2-bt-navigator
ros-humble-nav2-controller
ros-humble-nav2-planner
ros-humble-nav2-costmap-2d

# MAVLink
ros-humble-mavros
ros-humble-mavros-extras
ros-humble-mavlink

# ZED
zed-ros2-wrapper (from GitHub)

# Isaac ROS (Docker)
isaac_ros_nvblox
isaac_ros_visual_slam

# Utilities
ros-humble-tf2-ros
ros-humble-tf2-tools
ros-humble-robot-localization
```

### Python Packages

```bash
# Edge Core
fastapi
uvicorn
rclpy  # ROS2 Python client
```

---

## Questions & Answers

### Q: What happens if 4G/Tailscale fails?

**A**: ELRS backup control remains active. Pilot can take manual RC control via ELRS, switch to `ALT_HOLD` or `RTL` mode.

### Q: What if VIO fails indoors?

**A**: ArduPilot automatically switches to Source Set 1 (Baro + IMU). Flight mode changes to `ALT_HOLD`. Pilot alerted for manual takeover.

### Q: Can we still use WASD if Nav2 is running?

**A**: Yes! WASD publishes to `/cmd_vel` → overrides Nav2 temporarily. Nav2 resumes when WASD stops.

### Q: How does exclusion map work?

**A**: When target engaged:
1. 3D position stored in `exclusion_map_service`
2. Costmap inflated around that position (0.5m radius)
3. Nav2 won't plan paths through it
4. YOLO detections within radius ignored

### Q: What's the advantage over old nav_controller.py?

**A**: 
- ✅ Professional-grade obstacle avoidance (nvblox 3D costmap)
- ✅ Dynamic replanning if path blocked
- ✅ Recovery behaviors if stuck
- ✅ Microsecond latency (no Python API overhead)
- ✅ Battle-tested Nav2 stack
- ✅ Less custom code to debug

---

## Success Criteria

Task 2 autonomous navigation is considered **successful** if:

- [ ] Drone navigates autonomously without hitting obstacles
- [ ] Detects and engages 3+ targets without human intervention
- [ ] WASD controls respond < 100ms for manual override
- [ ] VIO remains stable throughout 5-minute flight
- [ ] Exclusion map prevents re-engaging targets
- [ ] No crashes, no GPS required indoors
- [ ] System runs reliably in competition

---

## Conclusion

This Nav2 integration eliminates custom navigation code in favor of industry-standard ROS2 autonomy. The key insight is: **let ROS2 handle navigation, mavros2 handle MAVLink, and Edge Core focus on mission orchestration only.**

**Next Steps**:
1. Review and approve this plan
2. Begin Phase 1: ROS2 environment setup
3. Allocate 4-5 weeks for full integration
4. Test in simulation before live flight

**Estimated Total Time**: 4-5 weeks  
**Risk Level**: Medium (standard tools, well-documented)  
**Competition Readiness**: High (professional autonomy stack)

---

**Document Status**: Ready for Review  
**Author**: AI Assistant + MAD Team  
**Date**: January 29, 2026
