import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('tf_explorer')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Set TurtleBot3 model to waffle (has depth camera)
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo with TurtleBot3 World (spawns robot at correct position)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # SLAM Toolbox (delayed to let Gazebo start)
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
    
    # TF Monitor Node (delayed)
    tf_monitor = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tf_explorer',
                executable='tf_monitor_node',
                name='tf_monitor_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Frame Broadcaster Node (delayed)
    frame_broadcaster = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tf_explorer',
                executable='frame_broadcaster_node',
                name='frame_broadcaster_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # RViz (delayed)
    rviz = TimerAction(
        period=7.0,
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
        tf_monitor,
        frame_broadcaster,
        rviz,
    ])