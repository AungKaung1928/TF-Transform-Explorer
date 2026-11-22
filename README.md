# TF Transform Explorer

[![ci](https://github.com/AungKaung1928/TF-Transform-Explorer/actions/workflows/ci.yml/badge.svg)](https://github.com/AungKaung1928/TF-Transform-Explorer/actions/workflows/ci.yml)

ROS 2 Humble package for a TurtleBot3 Burger in Gazebo Harmonic: SLAM, Nav2 patrol, a TF anomaly detector that publishes its own `TFDiagnostics` message, and a Nav2 keepout costmap plugin. One launch file starts all of it; 12 patrol goals reached in two 50 s windows, 0 aborts.

## Measured (Gazebo Harmonic, 2026-09-05)

Laptop: Ubuntu 22.04, ROS 2 Humble, gz-harmonic 8.15, Intel Iris Xe (no discrete GPU). Two runs of ~2 min each, all stages up, RTF ~1.

| Signal | Value |
|--------|-------|
| `/scan` | 4.5–4.8 Hz (gpu_lidar configured at 5 Hz) |
| `/odom` | 25–27 Hz |
| `/cmd_vel` (Nav2 → sim) | 20.0 Hz |
| Nav2 lifecycle | bt_navigator, controller, planner, behavior servers all `active` |
| Patrol | 12 random goals reached in two 50 s sample windows, 0 aborts observed |
| `/tf_diagnostics` | `map → odom`, `odom → base_link`, `map → base_link` all `is_healthy: true` |


## Quick start

Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic (`gz-harmonic` from the OSRF `gazebo-stable` apt repo; Gazebo Classic must not be installed alongside it, both ship `/usr/bin/gz`). No GPU needed.

```bash
sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic ros-humble-turtlebot3-description \
    ros-humble-xacro ros-humble-nav2-bringup ros-humble-slam-toolbox   # first time only
mkdir -p ~/tf_explorer_ws/src && cd ~/tf_explorer_ws/src
git clone <this repo URL> tf_explorer
cd ~/tf_explorer_ws
colcon build --symlink-install --packages-select tf_explorer
source install/setup.bash
ros2 launch tf_explorer tf_explorer.launch.py
# Gazebo + RViz open; the robot starts patrolling about 25 s after launch
# (sim ~3 s, SLAM 8 s, Nav2 10 s, patrol waits 15 s more).
```

Before relaunching, `pkill -9 -f "gz sim"`: two `gz sim` servers on one bus corrupt `/clock`, `/odom` and `/tf`.

## Tests

```bash
colcon test --packages-select tf_explorer && colcon test-result --verbose
```

CI runs the build and these gtests in a `ros:humble` container on every push:

- KeepoutLayer: cost marking (partly off-map zone, zone fully off the map, update-window bounds); enabled through Nav2's own flag; stays current after a costmap clear
- TF anomaly detector on live `/tf`: missing, fresh, 1 m jump and 5 s stale `map -> odom`
- Patrol node against a fake `navigate_to_pose` server: goes on after a rejected goal, a finished goal and a late server
- TF monitor: `/pose_history` carries the odometry frame
- Wiggle recovery on `/cmd_vel`: alternates, ends on the last right turn, then stops
- Frame broadcaster through a tf2 buffer: static mount offset, rotating sensor is a unit yaw that turns

## How it works

| Node | Purpose |
|------|---------|
| `tf_monitor_node` | Logs TF transforms every 2 seconds |
| `frame_broadcaster_node` | Publishes custom static/dynamic frames |
| `tf_anomaly_detector` | Detects stale/jumping transforms, publishes TFDiagnostics |
| `patrol_node` | Sends random Nav2 goals continuously |
| `simple_wiggle_recovery` | Standalone wiggle behavior (optional) |

The robot is spawned from `description/turtlebot3_burger.urdf.xacro`; `config/gz_bridge.yaml` bridges `/scan /odom /tf /joint_states /imu /clock /cmd_vel`. Message fields, the keepout plugin config, tunable parameters and troubleshooting: [docs/DETAILS.md](docs/DETAILS.md).

## License

MIT, see LICENSE.
