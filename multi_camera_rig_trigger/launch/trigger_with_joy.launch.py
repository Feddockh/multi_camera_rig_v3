#!/usr/bin/env python3
"""
Launch file for trigger control with optional joystick support.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for trigger hardware'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='9600',
            description='Serial port baud rate'
        ),
        DeclareLaunchArgument(
            'flash_duration_ms',
            default_value='100',
            description='Flash duration in milliseconds (0-300)'
        ),
        DeclareLaunchArgument(
            'frame_rate_hz',
            default_value='10.0',
            description='Trigger frame rate in Hz (1-20)'
        ),
        DeclareLaunchArgument(
            'auto_connect',
            default_value='true',
            description='Automatically test connection on startup'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Automatically start video triggering on launch'
        ),
        DeclareLaunchArgument(
            'use_joy',
            default_value='false',
            description='Enable joystick control for triggering'
        ),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Joystick topic name'
        ),
        DeclareLaunchArgument(
            'trigger_button',
            default_value='0',
            description='Joystick button index for triggering (0=A button on Xbox)'
        ),

        # Trigger node
        Node(
            package='multi_camera_rig_trigger',
            executable='trigger_node',
            name='trigger_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'flash_duration_ms': LaunchConfiguration('flash_duration_ms'),
                'frame_rate_hz': LaunchConfiguration('frame_rate_hz'),
                'auto_connect': LaunchConfiguration('auto_connect'),
                'auto_start': LaunchConfiguration('auto_start'),
            }]
        ),

        # Joy trigger node (conditional)
        Node(
            package='multi_camera_rig_trigger',
            executable='joy_trigger_node',
            name='joy_trigger_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joy')),
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'trigger_button': LaunchConfiguration('trigger_button'),
            }]
        ),
    ])
