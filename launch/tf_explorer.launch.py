#!/usr/bin/env python3
"""Gazebo Harmonic TurtleBot3 sim + SLAM + Nav2 + the tf_explorer nodes + RViz.

Gazebo Classic (turtlebot3_gazebo) is EOL and replaced by ros_gz_sim: the robot is
spawned from /robot_description and ros_gz_bridge relays /scan /odom /tf /clock /cmd_vel.
SLAM, Nav2 and the custom nodes are delayed so sensor data is flowing before they start.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tf_explorer')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file = os.path.join(pkg_dir, 'worlds', 'explorer_world.sdf')
    xacro_file = os.path.join(pkg_dir, 'description', 'turtlebot3_burger.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    # model://turtlebot3_description/... must resolve so the robot meshes load in the GUI
    tb3_desc_parent = os.path.dirname(get_package_share_directory('turtlebot3_description'))
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([tb3_desc_parent, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    # 1. Gazebo Harmonic + robot
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', xacro_file])}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_burger',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'burger',
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}],
    )

    # 2. SLAM
    slam = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(pkg_dir, 'config', 'slam_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # 3. Nav2
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    # 4. Custom Nodes
    custom_nodes = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='tf_explorer',
                executable=exe,
                name=exe,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
            for exe in ('tf_monitor_node', 'frame_broadcaster_node',
                        'tf_anomaly_detector', 'patrol_node')
        ]
    )

    # 5. RViz
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_dir, 'rviz', 'tf_explorer.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        bridge,
        TimerAction(period=3.0, actions=[spawn]),
        slam,
        nav2,
        custom_nodes,
        rviz,
    ])
