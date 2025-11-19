#!/usr/bin/env python3
"""Launch file for multi-camera rig GUI"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for GUI node"""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('multi_camera_rig_gui'),
            'config',
            'gui_params.yaml'
        ]),
        description='Path to GUI configuration YAML file'
    )
    
    # GUI node
    gui_node = Node(
        package='multi_camera_rig_gui',
        executable='gui_node',
        name='camera_rig_gui_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        gui_node,
    ])
