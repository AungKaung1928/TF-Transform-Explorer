# TF Transform Explorer

A ROS2 robotics project demonstrating TF2 transforms, SLAM mapping, Nav2 autonomous navigation, custom message types, and Nav2 plugin development using TurtleBot3 Burger in **Gazebo Harmonic** (ROS 2 Humble, `ros_gz`).

> Ported from Gazebo Classic (EOL) on 2026-09-05. The robot is now spawned from `description/turtlebot3_burger.urdf.xacro` via `ros_gz_sim create`, and `config/gz_bridge.yaml` bridges `/scan /odom /tf /joint_states /imu /clock /cmd_vel`. The `turtlebot3_gazebo` package is no longer required.

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

Single command launches everything (Gazebo Harmonic + SLAM + Nav2 + Custom Nodes + RViz):

```bash
cd ~/tf_explorer_ws
colcon build --symlink-install --packages-select tf_explorer
source install/setup.bash
ros2 launch tf_explorer tf_explorer.launch.py
```

Robot starts patrolling automatically about 25 seconds after launch (sim up at ~3 s, SLAM at 8 s, Nav2 at 10 s, patrol node waits a further 15 s).

Kill any leftover simulator before relaunching, otherwise two `gz sim` servers share one bus and corrupt `/clock`, `/odom` and `/tf`:

```bash
pkill -9 -f "gz sim"
```

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

---

## Clone & Build

```bash
mkdir -p ~/tf_explorer_ws/src && cd ~/tf_explorer_ws/src
git clone https://github.com/AungKaung1928/TF-Transform-Explorer.git tf_explorer
cd ~/tf_explorer_ws
colcon build --symlink-install --packages-select tf_explorer
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

ROS 2 Humble + Gazebo Harmonic (`gz-harmonic` from the OSRF `gazebo-stable` apt repo). Gazebo Classic must **not** be installed alongside it (both ship `/usr/bin/gz`).

```bash
sudo apt install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-turtlebot3-description \
    ros-humble-xacro \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox
```

No `TURTLEBOT3_MODEL` environment variable is needed; the robot model is the xacro in `description/`.

---

## Measured Results (Gazebo Harmonic, 2026-09-05)

Laptop: Ubuntu 22.04, ROS 2 Humble, gz-harmonic 8.15, Intel Iris Xe (no discrete GPU). Two runs of ~2 min each, all stages up, RTF ~1.

| Signal | Value |
|--------|-------|
| `/scan` | 4.5–4.8 Hz (gpu_lidar configured at 5 Hz) |
| `/odom` | 25–27 Hz |
| `/cmd_vel` (Nav2 → sim) | 20.0 Hz |
| Nav2 lifecycle | bt_navigator, controller, planner, behavior servers all `active` |
| Patrol | 12 random goals reached in two 50 s sample windows, 0 aborts observed |
| `/tf_diagnostics` | `map → odom`, `odom → base_link`, `map → base_link` all `is_healthy: true` |

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
