#!/usr/bin/env python3
"""
Full application launch: firefly camera pipeline + rig model + GUI + RViz.

Closing the GUI window or the RViz window shuts down every other process
launched here, so the whole system starts and stops as a single unit.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Process names whose exit should trigger shutdown of the whole app.
# These match the explicit `name=`s given to the GUI node (in gui.launch.py)
# and the rviz2 node below.
SHUTDOWN_TRIGGER_PROCESS_NAMES = ('camera_rig_gui_node', 'rviz2')


def on_process_exit(event, context):
    # launch appends a '-N' counter suffix to the final process name, so match
    # on the base name rather than exact equality.
    base_name = event.process_name.rsplit('-', 1)[0]
    if base_name in SHUTDOWN_TRIGGER_PROCESS_NAMES:
        return [EmitEvent(event=Shutdown(reason=f'{event.process_name} closed'))]
    return None


def generate_launch_description():
    """Generate launch description for the full application."""

    bringup_pkg = get_package_share_directory('multi_camera_rig_bringup')
    firefly_bringup_launch = os.path.join(
        bringup_pkg, 'launch', 'firefly_bringup.launch.py')

    camera_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(firefly_bringup_launch),
        launch_arguments={
            'use_gazebo': LaunchConfiguration('use_gazebo'),
            'enable_detection': LaunchConfiguration('enable_detection'),
            'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            'trigger_auto_start': LaunchConfiguration('trigger_auto_start'),
            'use_background': LaunchConfiguration('use_background'),
        }.items()
    )

    # Ximea camera. Its own launch file's `ffc_dir` default (~/fireblight/ffc)
    # doesn't match gui.launch.py's hardcoded ffc_dir (~/fireblight_data/ffc,
    # see gui.launch.py's 'ffc_dir' parameter) so override it here to the value
    # the GUI actually expects.
    ximea_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PJoin([FindPackageShare('ximea-ros2-wrapper-bringup'), 'launch', 'bringup_real.launch.py'])
        ),
        launch_arguments={
            'ffc_dir': '~/fireblight_data/ffc',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_ximea')),
    )

    # Robot model: same robot_state_publisher/joint_state_publisher setup as
    # multi_camera_rig_description/launch/description.launch.py, reconstructed
    # here (rather than included) so we don't also pull in that launch file's
    # own rviz2 node, which would open a second, redundant RViz window.
    description_pkg = get_package_share_directory('multi_camera_rig_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'multi_camera_rig_description.urdf.xacro')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
        }],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PJoin([FindPackageShare('multi_camera_rig_gui'), 'launch', 'gui.launch.py'])
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    shutdown_on_gui_or_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=None, on_exit=on_process_exit)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Use Gazebo simulation (true) or real hardware (false)',
        ),
        DeclareLaunchArgument(
            'enable_detection',
            default_value='true',
            description='Enable YOLO detection node',
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='true',
            description='Enable semantic point cloud generation node',
        ),
        DeclareLaunchArgument(
            'trigger_auto_start',
            default_value='false',
            description='Automatically start video triggering on launch',
        ),
        DeclareLaunchArgument(
            'use_background',
            default_value='false',
            description='Include background points (clamped to max_range_m) in the semantic point cloud',
        ),
        DeclareLaunchArgument(
            'enable_ximea',
            default_value='true',
            description='Enable the Ximea camera',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PJoin([FindPackageShare('multi_camera_rig_bringup'), 'rviz', 'app.rviz']),
            description='Path to the RViz config file showing the rig model and semantic point cloud',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for the robot state/joint state publishers',
        ),
        camera_pipeline,
        ximea_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gui_launch,
        rviz_node,
        shutdown_on_gui_or_rviz_exit,
    ])
