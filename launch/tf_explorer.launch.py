import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('tf_explorer')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM
    slam = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(pkg_dir, 'config', 'slam_params.yaml'),
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    # 3. Nav2
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    # 4. Custom Nodes
    custom_nodes = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tf_explorer',
                executable='tf_monitor_node',
                name='tf_monitor_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='tf_explorer',
                executable='frame_broadcaster_node',
                name='frame_broadcaster_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='tf_explorer',
                executable='tf_anomaly_detector',
                name='tf_anomaly_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='tf_explorer',
                executable='patrol_node',
                name='patrol_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
        ]
    )

    # 5. RViz
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_dir, 'rviz', 'tf_explorer.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_tb3_model,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        slam,
        nav2,
        custom_nodes,
        rviz,
    ])