import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_multi_robot = get_package_share_directory('robot1_slam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pkg_multi_robot, 'config', 'nav2_robot1.yaml')

    # Include official Nav2 navigation launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'use_namespace': 'true',
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])