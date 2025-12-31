# TF Transform Explorer

A ROS2 project demonstrating TF2 transforms, SLAM mapping, Nav2 autonomous navigation, custom message types, and Nav2 plugin development using TurtleBot3 Waffle in Gazebo.

---

## Project Overview

| Feature | Description |
|---------|-------------|
| **TF2 Transforms** | Real-time monitoring of frame relationships |
| **SLAM** | slam_toolbox for mapping |
| **Nav2 Navigation** | Autonomous path planning with recovery behaviors |
| **Autonomous Patrol** | Random goal generation for continuous exploration |
| **Custom Messages** | TFDiagnostics.msg for transform health monitoring |
| **Nav2 Plugin** | Keepout layer costmap plugin via pluginlib |
| **TF Anomaly Detection** | Monitors stale transforms and position jumps |

---

## Run

Single command launches everything (Gazebo + SLAM + Nav2 + Custom Nodes + RViz):

```bash
cd ~/tf_explorer_ws
colcon build --packages-select tf_explorer
source install/setup.bash
ros2 launch tf_explorer tf_explorer.launch.py
```

Robot starts patrolling automatically after 15 seconds.

---

## Custom Nodes

| Node | Purpose |
|------|---------|
| `tf_monitor_node` | Logs TF transforms every 2 seconds |
| `frame_broadcaster_node` | Publishes custom static/dynamic frames |
| `tf_anomaly_detector` | Detects stale/jumping transforms, publishes TFDiagnostics |
| `patrol_node` | Sends random Nav2 goals continuously |
| `simple_wiggle_recovery` | Standalone wiggle behavior (optional) |

---

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

---

## Nav2 Costmap Plugin

**Keepout Layer** - Marks rectangular zones as obstacles via pluginlib.

Registered in `costmap_plugins.xml`:
```xml
<class name="tf_explorer/KeepoutLayer" type="tf_explorer::KeepoutLayer" base_class_type="nav2_costmap_2d::Layer"/>
```

---

## Project Structure

```
tf_explorer/
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
├── launch/
│   └── tf_explorer.launch.py
├── config/
│   ├── nav2_params.yaml
│   └── slam_params.yaml
├── rviz/
│   └── tf_explorer.rviz
├── costmap_plugins.xml
├── CMakeLists.txt
└── package.xml
```

---

## Clone & Build

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/tf_explorer_ws.git
cd tf_explorer_ws
colcon build --packages-select tf_explorer
source install/setup.bash
```

---

## Key Parameters to Modify

### Patrol Node (`src/patrol_node.cpp`)

| Parameter | Line | Default | Description |
|-----------|------|---------|-------------|
| Startup delay | `std::chrono::seconds(15)` | 15s | Wait time before first goal |
| Goal range X | `dis_x(-1.0, 1.0)` | -1.0 to 1.0 | Random X coordinate range |
| Goal range Y | `dis_y(-1.0, 1.0)` | -1.0 to 1.0 | Random Y coordinate range |
| Goal delay | `std::chrono::seconds(2)` | 2s | Pause between goals |

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
| `zone_min_x` | -0.5 | Keepout zone X start |
| `zone_max_x` | 0.5 | Keepout zone X end |
| `zone_min_y` | -0.5 | Keepout zone Y start |
| `zone_max_y` | 0.5 | Keepout zone Y end |

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

---

## Prerequisites

```bash
sudo apt install -y \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox

echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
source ~/.bashrc
```

---

## Recovery Behaviors

Nav2 automatically triggers when robot gets stuck:

| Behavior | Action |
|----------|--------|
| **Spin** | Rotates 90° in place |
| **Backup** | Reverses ~15cm |
| **Wait** | Pauses 5 seconds |

---

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

---

## Troubleshooting

**Gazebo crashes:**
```bash
killall -9 gzserver gzclient
```

**Robot not moving:**
- Wait 15+ seconds for Nav2 to fully activate
- Check patrol_node is running: `ros2 node list | grep patrol`
