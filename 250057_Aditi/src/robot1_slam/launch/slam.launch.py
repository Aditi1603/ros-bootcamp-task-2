import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot1_slam') # Replace with your package name
    slam_config_file = os.path.join(pkg_dir, 'config', 'robot1_slam.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot1',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': True,
                'odom_frame': 'robot1/odom',
                'map_frame': 'robot1/map',
                'base_frame': 'robot1/base_footprint',
                'scan_topic': '/robot1/scan',
            }
        ],
        remappings=[
            ('/scan', '/robot1/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    return LaunchDescription([slam_node])