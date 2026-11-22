# TF Transform Explorer: details

Nodes, message, costmap plugin, parameters, recovery behaviours and troubleshooting behind the short [README](../README.md).

## Custom Nodes

| Node | Purpose |
|------|---------|
| `tf_monitor_node` | Logs TF transforms every 2 seconds |
| `frame_broadcaster_node` | Publishes custom static/dynamic frames |
| `tf_anomaly_detector` | Detects stale/jumping transforms, publishes TFDiagnostics |
| `patrol_node` | Sends random Nav2 goals continuously |
| `simple_wiggle_recovery` | Standalone wiggle behavior (optional) |


## Custom Message

**msg/TFDiagnostics.msg**
```
string parent_frame
string child_frame
bool is_healthy
string status_message
float64 transform_age_seconds
float64 position_jump_meters
bool jump_detected
```


## Nav2 Costmap Plugin

**Keepout Layer** - Marks rectangular zones as obstacles via pluginlib.

Registered in `costmap_plugins.xml`:
```xml
<class type="tf_explorer::KeepoutLayer" base_class_type="nav2_costmap_2d::Layer"/>
```

The layer is built and exported but not enabled in `config/nav2_params.yaml`. To use it, add it to a costmap's plugin list (here the global costmap):
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "keepout_layer", "inflation_layer"]
      keepout_layer:
        plugin: "tf_explorer::KeepoutLayer"
        zone_min_x: 1.0
        zone_max_x: 2.0
        zone_min_y: 1.0
        zone_max_y: 2.0
```
A zone that only partly overlaps the costmap is clipped to it, and the layer writes only inside the update window it is given (unit tests: `test/test_keepout_layer.cpp`).


## Project Structure

```
tf_explorer/
├── description/
│   └── turtlebot3_burger.urdf.xacro   # gz-sim DiffDrive + gpu_lidar + IMU plugins
├── worlds/
│   └── explorer_world.sdf             # 10 x 10 m arena, 3 obstacles (SDF 1.8)
├── msg/
│   └── TFDiagnostics.msg
├── src/
│   ├── tf_monitor_node.cpp
│   ├── frame_broadcaster_node.cpp
│   ├── tf_anomaly_detector.cpp
│   ├── patrol_node.cpp
│   ├── simple_wiggle_recovery.cpp
│   └── keepout_layer.cpp
├── include/tf_explorer/
│   └── [header files]
├── test/
│   └── [one gtest file per node and one for the keepout layer]
├── launch/
│   └── tf_explorer.launch.py
├── config/
│   ├── nav2_params.yaml
│   ├── slam_params.yaml
│   └── gz_bridge.yaml                 # ros_gz_bridge topic map
├── rviz/
│   └── tf_explorer.rviz
├── costmap_plugins.xml
├── CMakeLists.txt
└── package.xml
```


## Key Parameters to Modify

### Patrol Node (`include/tf_explorer/patrol_node.hpp`)

| Parameter | Line | Default | Description |
|-----------|------|---------|-------------|
| Startup delay | ROS parameter `startup_delay` | 15s | Wait time before first goal |
| Goal range X | `dis_x(-1.0, 1.0)` | -1.0 to 1.0 | Random X coordinate range |
| Goal range Y | `dis_y(-1.0, 1.0)` | -1.0 to 1.0 | Random Y coordinate range |
| Goal delay | ROS parameter `retry_delay` | 2s | Pause between goals, and before a retry after a rejected goal or a missing Nav2 server (unit tests: `test/test_patrol_node.cpp`) |

### TF Anomaly Detector (`src/tf_anomaly_detector.cpp`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_transform_age` | 1.0s | Transform considered stale after this |
| `max_position_jump` | 0.5m | Jump threshold for anomaly detection |

### TF Monitor (`src/tf_monitor_node.cpp`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| Timer period | 2000ms | How often to log TF reports |

### Keepout Layer (`src/keepout_layer.cpp`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | true | Turn the layer on or off |
| `zone_min_x` | 1.0 | Keepout zone X start (m) |
| `zone_max_x` | 2.0 | Keepout zone X end (m) |
| `zone_min_y` | 1.0 | Keepout zone Y start (m) |
| `zone_max_y` | 2.0 | Keepout zone Y end (m) |

### Nav2 (`config/nav2_params.yaml`)

| Parameter | Section | Description |
|-----------|---------|-------------|
| `max_vel_x` | controller_server | Max forward speed |
| `max_vel_theta` | controller_server | Max rotation speed |
| `inflation_radius` | inflation_layer | Safety buffer around obstacles |
| `robot_radius` | behavior_server | Robot footprint for recovery |

After modifying any C++ file, rebuild:
```bash
colcon build --packages-select tf_explorer
source install/setup.bash
```


## Recovery Behaviors

Nav2 automatically triggers when robot gets stuck:

| Behavior | Action |
|----------|--------|
| **Spin** | Rotates 90° in place |
| **Backup** | Reverses ~15cm |
| **Wait** | Pauses 5 seconds |


## Useful Commands

```bash
# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Echo transform
ros2 run tf2_ros tf2_echo map base_link

# Check diagnostics topic
ros2 topic echo /tf_diagnostics

# Manual navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}"
```


## Troubleshooting

**Gazebo crashes or robot stands still with a racing clock:**
```bash
pkill -9 -f "gz sim"
```
A stray `gz sim` server from a previous run shares the gz-transport bus with the new one and corrupts `/clock`, `/odom` and `/tf`.

**RViz: `RobotModel` shows a red error and `TF` a warning on `wheel_left_link` / `wheel_right_link`:**
```bash
ros2 node list | grep joint_state_publisher   # must print nothing, this launch starts none
pkill -9 -f joint_state_publisher
```
A stray `joint_state_publisher` (another project's launch left running on the same `ROS_DOMAIN_ID`) re-publishes the wheel joints with wall-clock stamps while the sim runs on sim time. `robot_state_publisher` forwards them, so tf2 reports "extrapolation into the past" for the wheel frames. After killing it press **Reset** (bottom left of RViz) or restart RViz: its tf buffer keeps the bad stamps until then. A clean start shows no RViz error (checked at 16, 24, 34 and 50 s after launch).

**No `/camera/depth/points` log lines from `tf_monitor_node`:**
Expected. The Burger model has no depth camera; the subscription stays idle. Add an RGB-D sensor to the xacro and a `sensor_msgs/PointCloud2` entry to `config/gz_bridge.yaml` if you want it.

**Robot not moving:**
- Wait 15+ seconds for Nav2 to fully activate
- Check patrol_node is running: `ros2 node list | grep patrol`
