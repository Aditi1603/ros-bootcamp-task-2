import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    simulation_dir = get_package_share_directory('simulation')
    robot1_slam_dir = get_package_share_directory('robot1_slam')
    robot2_slam_dir = get_package_share_directory('robot2_slam')

    # Simulation and spawn
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, 'launch', 'gazebo_and_spawn.launch.py')
        ),
        launch_arguments={
            'robot1_x': LaunchConfiguration('robot1_x', default='0.0'),
            'robot1_y': LaunchConfiguration('robot1_y', default='1.0'),
            'robot1_yaw': LaunchConfiguration('robot1_yaw', default='0.0'),
            'robot2_x': LaunchConfiguration('robot2_x', default='0.0'),
            'robot2_y': LaunchConfiguration('robot2_y', default='-1.0'),
            'robot2_yaw': LaunchConfiguration('robot2_yaw', default='0.0'),
        }.items()
    )

    # Robot 1 SLAM
    robot1_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot1_slam_dir, 'launch', 'slam.launch.py')
        )
    )

    # Robot 2 SLAM
    robot2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot2_slam_dir, 'launch', 'slam.launch.py')
        )
    )

    # Robot 1 Nav2
    robot1_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot1_slam_dir, 'launch', 'robot1_nav2.launch.py')
        )
    )

    # Robot 2 Nav2
    robot2_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot2_slam_dir, 'launch', 'robot2_nav2.launch.py')
        )
    )

    # Exploration nodes
    robot1_exploration = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace='robot1',
        output='screen'
    )

    robot2_exploration = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace='robot2',
        output='screen'
    )

    # Map Merger node
    map_merger = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='0.0', description='Robot 1 X position'),
        DeclareLaunchArgument('robot1_y', default_value='1.0', description='Robot 1 Y position'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0', description='Robot 1 yaw'),
        DeclareLaunchArgument('robot2_x', default_value='0.0', description='Robot 2 X position'),
        DeclareLaunchArgument('robot2_y', default_value='-1.0', description='Robot 2 Y position'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0', description='Robot 2 yaw'),
        sim_launch,
        robot1_slam_launch,
        robot2_slam_launch,
        robot1_nav2_launch,
        robot2_nav2_launch,
        robot1_exploration,
        robot2_exploration,
        map_merger
    ])
