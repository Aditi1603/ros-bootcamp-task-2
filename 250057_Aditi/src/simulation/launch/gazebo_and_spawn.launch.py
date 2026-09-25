#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # ---------------------------------------------------------
    # TurtleBot3 model
    # ---------------------------------------------------------
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']

    # ---------------------------------------------------------
    # User-configurable spawn positions
    # ---------------------------------------------------------
    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot1_yaw = LaunchConfiguration('robot1_yaw')

    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')
    robot2_yaw = LaunchConfiguration('robot2_yaw')

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    gazebo_ros_dir = get_package_share_directory(
        'gazebo_ros'
    )

    launch_dir = os.path.join(
        turtlebot3_gazebo_dir,
        'launch'
    )

    model_folder = 'turtlebot3_' + turtlebot3_model

    model_path = os.path.join(
        turtlebot3_gazebo_dir,
        'models',
        model_folder,
        'model.sdf'
    )

    # Temporary modified SDF files
    # Temporary modified SDF files
    # Use /tmp because /opt/ros/humble is system-owned
    robot1_sdf = '/tmp/multi_robot_sim_robot1.sdf'
    robot2_sdf = '/tmp/multi_robot_sim_robot2.sdf'

    # ---------------------------------------------------------
    # ---------------------------------------------------------
    # Gazebo world
    # ---------------------------------------------------------
    world = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_world.world'
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gzclient.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Create modified SDF for each robot
    # ---------------------------------------------------------

    tree = ET.parse(model_path)
    root = tree.getroot()

    # Robot 1
    root1 = ET.fromstring(
        ET.tostring(root, encoding='unicode')
    )

    for tag in root1.iter('odometry_frame'):
        tag.text = 'robot1/odom'

    for tag in root1.iter('robot_base_frame'):
        tag.text = 'robot1/base_footprint'

    for tag in root1.iter('frame_name'):
        tag.text = 'robot1/base_scan'

    robot1_xml = ET.tostring(
        root1,
        encoding='unicode'
    )

    with open(robot1_sdf, 'w') as file:
        file.write('<?xml version="1.0" ?>\n')
        file.write(robot1_xml)

    # Robot 2
    root2 = ET.fromstring(
        ET.tostring(root, encoding='unicode')
    )

    for tag in root2.iter('odometry_frame'):
        tag.text = 'robot2/odom'

    for tag in root2.iter('robot_base_frame'):
        tag.text = 'robot2/base_footprint'

    for tag in root2.iter('frame_name'):
        tag.text = 'robot2/base_scan'

    robot2_xml = ET.tostring(
        root2,
        encoding='unicode'
    )

    with open(robot2_sdf, 'w') as file:
        file.write('<?xml version="1.0" ?>\n')
        file.write(robot2_xml)

    # ---------------------------------------------------------
    # Robot 1 state publisher
    # ---------------------------------------------------------
    robot1_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'frame_prefix': 'robot1'
        }.items()
    )

    # ---------------------------------------------------------
    # Robot 2 state publisher
    # ---------------------------------------------------------
    robot2_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'frame_prefix': 'robot2'
        }.items()
    )

    # ---------------------------------------------------------
    # Robot spawning
    # ---------------------------------------------------------
    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'multi_spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': robot1_x,
            'y_pose': robot1_y,
            'robot_name': 'robot1',
            'namespace': 'robot1',
            'sdf_path': robot1_sdf,
        }.items()
    )

    robot2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'multi_spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': robot2_x,
            'y_pose': robot2_y,
            'robot_name': 'robot2',
            'namespace': 'robot2',
            'sdf_path': robot2_sdf,
        }.items()
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------
    ld = LaunchDescription()

    # Spawn arguments
    ld.add_action(
        DeclareLaunchArgument(
            'robot1_x',
            default_value='0.0',
            description='Robot 1 X position'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'robot1_y',
            default_value='1.0',
            description='Robot 1 Y position'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'robot1_yaw',
            default_value='0.0',
            description='Robot 1 yaw'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'robot2_x',
            default_value='0.0',
            description='Robot 2 X position'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'robot2_y',
            default_value='-1.0',
            description='Robot 2 Y position'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'robot2_yaw',
            default_value='0.0',
            description='Robot 2 yaw'
        )
    )

    # Gazebo
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Robot 1
    ld.add_action(
        GroupAction([
            PushRosNamespace('robot1'),
            robot1_state_publisher,
            robot1_spawn,
        ])
    )

    # Robot 2
    ld.add_action(
        GroupAction([
            PushRosNamespace('robot2'),
            robot2_state_publisher,
            robot2_spawn,
        ])
    )

    # Cleanup temporary SDF files
    ld.add_action(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: [
                    os.remove(path)
                    for path in [robot1_sdf, robot2_sdf]
                    if os.path.exists(path)
                ]
            )
        )
    )

    return ld
