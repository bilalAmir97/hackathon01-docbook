#!/usr/bin/env python3
"""
Integrated System Launch File for ROS 2

This example demonstrates a complete system launch with multiple components,
including robot control, perception, and visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch a complete integrated system."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot1')

    # Robot control nodes
    robot_driver = Node(
        package='your_robot_driver',
        executable='robot_driver',
        name='robot_driver',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'navigation/cmd_vel'),
            ('odom', 'localization/odom')
        ]
    )

    # Perception nodes in a container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace=robot_namespace,
        package='rclcpp_components',
        executable='component_container',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Add composable perception nodes
    perception_container.add_composable_node(
        ComposableNode(
            package='your_perception_package',
            plugin='your_perception_package::LidarProcessor',
            name='lidar_processor',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('input_scan', 'sensor/lidar_scan'),
                ('processed_cloud', 'perception/lidar_cloud')
            ]
        )
    )

    perception_container.add_composable_node(
        ComposableNode(
            package='your_perception_package',
            plugin='your_perception_package::ImageProcessor',
            name='image_processor',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('input_image', 'sensor/camera_image'),
                ('processed_features', 'perception/image_features')
            ]
        )
    )

    # Navigation node
    navigation = Node(
        package='your_navigation_package',
        executable='navigation_node',
        name='navigation_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'navigation/cmd_vel'),
            ('map', 'map_server/map'),
            ('goal_pose', 'navigation/goal_pose')
        ]
    )

    # Localization node
    localization = Node(
        package='your_localization_package',
        executable='localization_node',
        name='localization_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', 'localization/odom'),
            ('imu', 'sensor/imu'),
            ('map', 'map_server/map')
        ]
    )

    # RViz2 for visualization (conditional)
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('your_robot_viz'),
        'rviz',
        'robot_config.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    ))

    ld.add_action(DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace for multi-robot systems'
    ))

    # Add nodes to launch
    ld.add_action(robot_driver)
    ld.add_action(perception_container)
    ld.add_action(navigation)
    ld.add_action(localization)
    ld.add_action(rviz)

    return ld