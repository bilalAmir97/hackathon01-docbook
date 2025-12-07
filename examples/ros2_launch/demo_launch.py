#!/usr/bin/env python3
"""
Demo Launch File for ROS 2

This example demonstrates how to create a launch file that starts multiple nodes
with specific configurations and parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    log_level = LaunchConfiguration('log_level', default='info')

    # Create nodes
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('chatter', 'demo_chatter')
        ]
    )

    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('chatter', 'demo_chatter')
        ]
    )

    # Parameter file node (if parameters file exists)
    param_file_path = os.path.join(
        get_package_share_directory('demo_nodes_cpp'),
        'param',
        'talker_params.yaml'
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
        'log_level',
        default_value='info',
        description='Logging level'
    ))

    # Add nodes
    ld.add_action(talker_node)
    ld.add_action(listener_node)

    # Add log info
    ld.add_action(LogInfo(
        msg=['Launching demo system with use_sim_time: ', use_sim_time]
    ))

    return ld