from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'flash_duration_ms',
            default_value='100',
            description='Flash duration in milliseconds (0-300)'
        ),
        DeclareLaunchArgument(
            'frame_rate_hz',
            default_value='10',
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
        Node(
            package='multi_camera_rig_trigger',
            executable='trigger_node_fake',
            name='trigger_node_fake',
            output='screen',
            parameters=[{
                'flash_duration_ms': LaunchConfiguration('flash_duration_ms'),
                'frame_rate_hz': LaunchConfiguration('frame_rate_hz'),
                'auto_connect': LaunchConfiguration('auto_connect'),
                'auto_start': LaunchConfiguration('auto_start'),
                'image_sub_topics': ['firefly_left/image_raw', 'firefly_right/image_raw'],
                'info_sub_topics': ['firefly_left/camera_info', 'firefly_right/camera_info'],
                'image_pub_topics': ['firefly_left/image_raw/triggered', 'firefly_right/image_raw/triggered'],
                'info_pub_topics': ['firefly_left/camera_info/triggered', 'firefly_right/camera_info/triggered']
            }]
        )
    ])
