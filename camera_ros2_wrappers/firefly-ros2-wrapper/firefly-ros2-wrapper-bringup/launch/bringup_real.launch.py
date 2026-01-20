from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
import yaml


def launch_setup(context, *args, **kwargs):

    # Get the calibration directory for all cameras
    calib_dir = LaunchConfig('calib_dir').perform(context)

    # Load in the Spinnaker camera configuration
    spinnaker_config_file = LaunchConfig('spinnaker_config_file').perform(context)
    spinnaker_param_file = LaunchConfig('spinnaker_param_file').perform(context)
    with open(spinnaker_config_file, 'r') as f:
        spinnaker_config = yaml.safe_load(f)

    # Adjust the paths in the configuration
    camera_names = spinnaker_config['cameras']
    for cam in camera_names:
        spinnaker_config[cam]['parameter_file'] = spinnaker_param_file
        spinnaker_config[cam]['camerainfo_url'] = 'file://' + calib_dir + '/' + cam + '.yaml'

    # FLIR Spinnaker stereo cameras in a composable container
    spinnaker_sync_container = ComposableNodeContainer(
        name='spinnaker_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='spinnaker_sync_node',
                namespace='',
                parameters=[spinnaker_config],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('~/firefly_left/camera_info', '/firefly_left/camera_info'),
                    ('~/firefly_left/image_raw', '/firefly_left/image_raw'),
                    ('~/firefly_left/image_raw_compressed', '/firefly_left/image_raw_compressed'),
                    ('~/firefly_left/meta', '/firefly_left/meta'),
                    ('~/firefly_right/camera_info', '/firefly_right/camera_info'),
                    ('~/firefly_right/image_raw', '/firefly_right/image_raw'),
                    ('~/firefly_right/image_raw_compressed', '/firefly_right/image_raw_compressed'),
                    ('~/firefly_right/meta', '/firefly_right/meta'),
                ],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],  # Adjust log level as needed
    )
    
    # Add trigger node if enabled
    enable_trigger = LaunchConfig('enable_trigger').perform(context).lower() == 'true'
    if enable_trigger:
        trigger_node = Node(
            package='multi_camera_rig_trigger',
            executable='trigger_node',
            name='trigger_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfig('trigger_serial_port'),
                'baudrate': LaunchConfig('trigger_baudrate'),
                'flash_duration_ms': LaunchConfig('trigger_flash_duration_ms'),
                'frame_rate_hz': LaunchConfig('trigger_frame_rate_hz'),
                'auto_connect': LaunchConfig('trigger_auto_connect'),
                'auto_start': LaunchConfig('trigger_auto_start'),
            }]
        )
        return [spinnaker_sync_container, trigger_node]
    else:
        return [spinnaker_sync_container]

def generate_launch_description():
    return LaunchDescription([
        LaunchArg(
            'calib_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'calibs']),
            description='Directory containing camera calibration YAML files',
        ),
        LaunchArg(
            'spinnaker_config_file',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'configs', 'firefly.yaml']),
            description='Path to the Spinnaker camera configuration YAML file.',
        ),
        LaunchArg(
            'spinnaker_param_file',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'params', 'firefly.yaml']),
            description='Path to the Spinnaker camera parameter definitions YAML file.',
        ),
        LaunchArg(
            'enable_trigger',
            default_value='true',
            description='Enable hardware trigger node'
        ),
        LaunchArg(
            'trigger_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for trigger hardware'
        ),
        LaunchArg(
            'trigger_baudrate',
            default_value='9600',
            description='Trigger serial baudrate'
        ),
        LaunchArg(
            'trigger_flash_duration_ms',
            default_value='100',
            description='Flash duration in milliseconds (0-300)'
        ),
        LaunchArg(
            'trigger_frame_rate_hz',
            default_value='1',
            description='Trigger frame rate in Hz (1-20)'
        ),
        LaunchArg(
            'trigger_auto_connect',
            default_value='true',
            description='Automatically test trigger connection on startup'
        ),
        LaunchArg(
            'trigger_auto_start',
            default_value='true',
            description='Automatically start video triggering on launch'
        ),
        OpaqueFunction(function=launch_setup)
    ])