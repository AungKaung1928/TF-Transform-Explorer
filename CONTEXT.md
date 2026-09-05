# CONTEXT.md — TF-Transform-Explorer

## Current
Ported to Gazebo Harmonic 2026-09-05, verified end to end on the laptop (pshell, out-of-tree build
in ~/personal/.build/tf_explorer): SLAM + Nav2 active, patrol_node reaches random goals, TF
diagnostics healthy. Nothing pending.
OPERATIONAL FOOTGUN: `pkill -9 -f "gz sim"` before every relaunch; two gz servers on one bus
corrupt /clock /odom /tf.

## Solved
- RViz RobotModel error + TF warning on wheel links during the first two verification runs was NOT
  the port: three stale joint_state_publisher processes (Autoware-Simulation-Demo test, same domain
  42) published wall-clock /joint_states. Reproduced on purpose with `ros2 run joint_state_publisher
  joint_state_publisher`, cleared after kill + RViz Reset; fresh start clean at 16/24/34/50 s.
  Recipe in README Troubleshooting.
- HARMONIC PORT: turtlebot3_gazebo (Classic, EOL) replaced by ros_gz_sim gz_sim.launch.py +
  `create -topic /robot_description` + ros_gz_bridge. Robot = Burger xacro copied from
  ROS2-Nav2-Random-Explorer-Bot (DiffDrive, gpu_lidar 5 Hz, IMU). World = repo's own
  explorer_world (10x10 m arena, 3 obstacles) rewritten as SDF 1.8 with system plugins,
  inline ground plane + sun. Spawn at origin (free of obstacles).
- tf_monitor_node: canTransform on the Waffle-only camera_link spammed tf2 warnings every 2 s
  on Burger; now gated by _frameExists.
- Stage delays: spawn 3 s, SLAM 8 s, custom nodes 9 s, Nav2 10 s, RViz 12 s after launch.
