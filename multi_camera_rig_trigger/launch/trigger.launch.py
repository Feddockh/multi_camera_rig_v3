from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for trigger hardware'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='9600',
            description='Serial baudrate'
        ),
        DeclareLaunchArgument(
            'flash_duration_ms',
            default_value='100',
            description='Flash duration in milliseconds (0-300)'
        ),
        DeclareLaunchArgument(
            'frame_rate_hz',
            default_value='5',
            description='Trigger frame rate in Hz (1-5)'
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
        )
    ])
