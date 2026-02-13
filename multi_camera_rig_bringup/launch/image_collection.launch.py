import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Load in the Spinnaker camera configuration
    spinnaker_config_file = LaunchConfiguration('spinnaker_config_file').perform(context)
    spinnaker_param_file = LaunchConfiguration('spinnaker_param_file').perform(context)
    with open(spinnaker_config_file, 'r') as f:
        spinnaker_config = yaml.safe_load(f)

    # Adjust the paths in the configuration
    calib_dir = LaunchConfiguration('calib_dir').perform(context)
    camera_names = spinnaker_config['cameras']
    for cam in camera_names:
        spinnaker_config[cam]['parameter_file'] = spinnaker_param_file
        spinnaker_config[cam]['camerainfo_url'] = 'file://' + calib_dir + '/' + cam + '.yaml'

    launch_nodes = []

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
                    # Left camera
                    ('~/firefly_left/camera_info', '/firefly_left/camera_info'),
                    ('~/firefly_left/image_raw', '/firefly_left/image_raw'),
                    ('~/firefly_left/image_raw/compressed', '/firefly_left/image_raw/compressed'),
                    ('~/firefly_left/image_raw/compressedDepth', '/firefly_left/image_raw/compressedDepth'),
                    ('~/firefly_left/image_raw/ffmpeg', '/firefly_left/image_raw/ffmpeg'),
                    ('~/firefly_left/image_raw/theora', '/firefly_left/image_raw/theora'),
                    ('~/firefly_left/meta', '/firefly_left/meta'),
                    # Right camera
                    ('~/firefly_right/camera_info', '/firefly_right/camera_info'),
                    ('~/firefly_right/image_raw', '/firefly_right/image_raw'),
                    ('~/firefly_right/image_raw/compressed', '/firefly_right/image_raw/compressed'),
                    ('~/firefly_right/image_raw/compressedDepth', '/firefly_right/image_raw/compressedDepth'),
                    ('~/firefly_right/image_raw/ffmpeg', '/firefly_right/image_raw/ffmpeg'),
                    ('~/firefly_right/image_raw/theora', '/firefly_right/image_raw/theora'),
                    ('~/firefly_right/meta', '/firefly_right/meta'),
                ],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],  # Adjust log level as needed
    )
    launch_nodes.append(spinnaker_sync_container)
    
    # Hardware trigger node
    trigger_node = Node(
        package='multi_camera_rig_trigger',
        executable='trigger_node',
        name='trigger_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('trigger_serial_port'),
            'baudrate': LaunchConfiguration('trigger_baudrate'),
            'flash_duration_ms': LaunchConfiguration('trigger_flash_duration_ms'),
            'frame_rate_hz': LaunchConfiguration('trigger_frame_rate_hz'),
            'auto_connect': LaunchConfiguration('trigger_auto_connect'),
            'auto_start': LaunchConfiguration('trigger_auto_start'),
        }]
    )
    launch_nodes.append(trigger_node)
    
    for cam_name in camera_names:
        rectify_scale_node = Node(
            package='firefly-ros2-wrapper-reconstruction',
            executable='stereo_rectify_scale_node',
            name=f'{cam_name}_rectify_scale',
            output='screen',
            parameters=[{
                # Topics
                'in_image_topic': f'/{cam_name}/image_raw',
                'in_info_topic': f'/{cam_name}/camera_info',
                'out_rect_image_topic': f'/{cam_name}/image_rect',
                'out_rect_info_topic': f'/{cam_name}/camera_info_rect',
                'publish_scaled': False,
                # Subscriber QoS
                'sub_qos.reliability': 'reliable',
                'sub_qos.durability': 'volatile',
                'sub_qos.history': 'keep_last',
                'sub_qos.depth': 5,
                # Publisher QoS
                'pub_qos.reliability': 'best_effort',
                'pub_qos.durability': 'volatile',
                'pub_qos.history': 'keep_last',
                'pub_qos.depth': 5,
            }]
        )
        launch_nodes.append(rectify_scale_node)

    # Image saver nodes for each camera
    save_dir = LaunchConfiguration('save_directory').perform(context)
    if save_dir:  # Only launch if save_directory is provided
        for cam_name in camera_names:
            image_saver_node = Node(
                package='firefly-ros2-wrapper-bringup',
                executable='image_saver_node',
                name=f'{cam_name}_image_saver',
                output='screen',
                parameters=[{
                    'image_topic': f'/{cam_name}/image_rect',
                    'save_directory': f'{save_dir}/{cam_name}',
                    'image_prefix': cam_name,
                    'image_format': LaunchConfiguration('image_format'),
                    'qos_depth': 5,
                    'qos_reliability': 'best_effort',
                    'qos_durability': 'volatile',
                    'qos_history': 'keep_last',
                }]
            )
            launch_nodes.append(image_saver_node)

    joy_trigger = LaunchConfiguration('joy_trigger').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    if joy_trigger.lower() == 'true':
        joy_node = Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}],
        )
        launch_nodes.append(joy_node)
        
        joy_trigger_node = Node(
            package='firefly-ros2-wrapper-bringup',
            executable='joy_trigger_node',
            name='joy_trigger_node',
            output='screen',
            parameters=[{
                'joy_topic': '/joy',
            }]
        )
        launch_nodes.append(joy_trigger_node)

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time',
        ),
        DeclareLaunchArgument(
            'calib_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'calibs']),
            description='Directory containing camera calibration YAML files',
        ),
        DeclareLaunchArgument(
            'spinnaker_config_file',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'configs', 'firefly.yaml']),
            description='Path to the Spinnaker camera configuration YAML file.',
        ),
        DeclareLaunchArgument(
            'spinnaker_param_file',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'params', 'firefly.yaml']),
            description='Path to the Spinnaker camera parameter definitions YAML file.',
        ),
        DeclareLaunchArgument(
            'trigger_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for trigger connection'
        ),
        DeclareLaunchArgument(
            'trigger_baudrate',
            default_value='9600',
            description='Baudrate for trigger serial connection'
        ),
        DeclareLaunchArgument(
            'trigger_flash_duration_ms',
            default_value='200',
            description='Flash duration in milliseconds (0-300)'
        ),
        DeclareLaunchArgument(
            'trigger_frame_rate_hz',
            default_value='1',
            description='Trigger frame rate in Hz (1-20)'
        ),
        DeclareLaunchArgument(
            'trigger_auto_connect',
            default_value='true',
            description='Automatically test trigger connection on startup'
        ),
        DeclareLaunchArgument(
            'trigger_auto_start',
            default_value='false',
            description='Automatically start video triggering on launch'
        ),
        DeclareLaunchArgument(
            'save_directory',
            default_value='/home/hayden/tmp/saved_images',
            description='Directory to save images (empty string disables image saving)'
        ),
        DeclareLaunchArgument(
            'image_format',
            default_value='png',
            description='Image format for saved images (png, jpg, etc.)'
        ),
        DeclareLaunchArgument(
            'joy_trigger',
            default_value='true',
            description='Whether to launch the joystick trigger node'
        ),
        OpaqueFunction(function=launch_setup)
    ])