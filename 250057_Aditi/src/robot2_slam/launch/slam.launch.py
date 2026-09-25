import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot2_slam')
    slam_config_file = os.path.join(pkg_dir, 'config', 'robot2_slam.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': True,
                'odom_frame': 'robot2/odom',
                'map_frame': 'robot2/map',
                'base_frame': 'robot2/base_footprint',
                'scan_topic': '/robot2/scan',
            }
        ],
        remappings=[
            ('/scan', '/robot2/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    return LaunchDescription([slam_node])