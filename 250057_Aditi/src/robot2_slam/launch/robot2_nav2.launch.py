import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('robot2_slam')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    namespace = 'robot2'
    params_file = os.path.join(pkg_share, 'config', 'nav2_robot2.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])