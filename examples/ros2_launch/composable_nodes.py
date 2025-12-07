#!/usr/bin/env python3
"""
Composable Nodes Example for ROS 2

This example demonstrates how to create and use composable nodes within a single container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch composable nodes in a container."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create a container for composable nodes
    container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Define composable nodes
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('image_rect', '/camera/image_rect')
        ]
    )

    resize_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::ResizeNode',
        name='resize_node',
        parameters=[{'scale_height': 0.5, 'scale_width': 0.5}],
        remappings=[
            ('image', '/camera/image_rect'),
            ('camera_info', '/camera/camera_info'),
            ('resized/image', '/camera/image_resized')
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch argument
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))

    # Add container with composable nodes
    container.add_composable_node(rectify_node)
    container.add_composable_node(resize_node)

    ld.add_action(container)

    # Add log info
    ld.add_action(LogInfo(
        msg=['Launching composable nodes in container']
    ))

    return ld