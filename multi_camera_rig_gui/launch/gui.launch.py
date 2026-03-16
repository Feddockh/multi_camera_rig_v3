#!/usr/bin/env python3
"""Launch file for multi-camera rig GUI"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from  launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    config_file_path = LaunchConfiguration('config_file').perform(context)

    # GUI node
    gui_node = Node(
        package='multi_camera_rig_gui',
        executable='gui_node',
        name='camera_rig_gui_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # GUI settings
                'window_title': 'Multi-Camera Rig Control',
                'update_rate_hz': 60.0,
                # Trigger services
                'trigger_start_service': '/trigger/start_video',
                'trigger_stop_service': '/trigger/stop_video',
                'trigger_is_running_service': '/trigger/is_video_running',
                # Image topics for display
                'image_topics.img1': '/firefly_left/image_raw',
                'image_topics.img2': '/ximea/image_raw',
                # Recording settings
                'recording.topics': [
                    '/firefly_left/image_raw',
                    '/ximea/image_raw',
                    '/trigger/status',
                ],
                'recording.storage_path': '~/fireblight_data/recordings',
                'recording.storage_id': 'sqlite3',
                # FFC calibration output directory (must match ximea camera node's ffc_dir)
                'ffc_dir': '~/fireblight_data/ffc',
                # XIMEA reload service — camera_name + "/reload_ffc"
                'ximea_reload_ffc_service': '/ximea/reload_ffc',
                # Push slider defaults to nodes on startup instead of reading existing values
                'force_default_params': True,
                # Path to gui_params.yaml so defaults can be persisted on slider change
                'gui_config_file_path': config_file_path,
            },
        ],
        emulate_tty=True,
    )
    
    return [
        gui_node,
    ]

def generate_launch_description():
    """Generate launch description with configurable arguments."""
    
    return LaunchDescription([
        DeclareLaunchArgument('config_file', 
                              default_value=PJoin([FindPackageShare('multi_camera_rig_gui'), 'config', 'gui_params.yaml']),
                              description='Path to GUI configuration YAML file'),
        OpaqueFunction(function=launch_setup)
    ])
