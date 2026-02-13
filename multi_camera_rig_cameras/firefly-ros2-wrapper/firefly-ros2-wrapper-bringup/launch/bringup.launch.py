"""
Simplified firefly camera bringup launch file.
This launch file only brings up the camera images (raw images + camera info).
Use use_gazebo parameter to switch between real hardware and simulation.
"""
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to configure camera bringup based on use_gazebo parameter."""
    
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context).lower() == 'true'
    
    # Load camera configuration
    spinnaker_config_file = LaunchConfiguration('spinnaker_config_file').perform(context)
    spinnaker_param_file = LaunchConfiguration('spinnaker_param_file').perform(context)
    with open(spinnaker_config_file, 'r') as f:
        spinnaker_config = yaml.safe_load(f)

    # Adjust calibration paths
    calib_dir = LaunchConfiguration('calib_dir').perform(context)
    camera_names = spinnaker_config['cameras']
    for cam in camera_names:
        spinnaker_config[cam]['parameter_file'] = spinnaker_param_file
        spinnaker_config[cam]['camerainfo_url'] = 'file://' + calib_dir + '/' + cam + '.yaml'

    launch_nodes = []

    if use_gazebo:
        # ============================
        # Gazebo/Simulation Pipeline
        # ============================
        
        # Gazebo sensor bridge
        sensor_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='firefly_depth_bridge',
            arguments=[
                '/firefly_left/sim/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/firefly_right/sim/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/firefly_left/sim/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/firefly_right/sim/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/firefly_left/sim/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',  # For flash simulator
                '/firefly_right/sim/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',  # For flash simulator
                '/firefly_left/sim/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/firefly_right/sim/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'qos': 'reliable'},
            ],
        )
        launch_nodes.append(sensor_bridge)

        # Hardware trigger node (fake version for sim)
        trigger_node = Node(
            package='multi_camera_rig_trigger',
            executable='trigger_node_fake',
            name='trigger_node_fake',
            output='screen',
            parameters=[{
                'flash_duration_ms': LaunchConfiguration('trigger_flash_duration_ms'),
                'frame_rate_hz': LaunchConfiguration('trigger_frame_rate_hz'),
                'auto_connect': LaunchConfiguration('trigger_auto_connect'),
                'auto_start': LaunchConfiguration('trigger_auto_start'),
                'image_sub_topics': ['/firefly_left/sim/image_raw', '/firefly_right/sim/image_raw', 
                                   '/firefly_left/sim/depth/image_raw', '/firefly_right/sim/depth/image_raw'],
                'info_sub_topics': ['/firefly_left/sim/camera_info', '/firefly_right/sim/camera_info', 
                                  '/firefly_left/sim/depth/camera_info', '/firefly_right/sim/depth/camera_info'],
                'image_pub_topics': ['/firefly_left/image_raw/triggered', '/firefly_right/image_raw/triggered', 
                                   '/firefly_left/depth/image_raw/triggered', '/firefly_right/depth/image_raw/triggered'],
                'info_pub_topics': ['/firefly_left/camera_info', '/firefly_right/camera_info', 
                                  '/firefly_left/depth/camera_info', '/firefly_right/depth/camera_info'],
            }]
        )
        launch_nodes.append(trigger_node)

        # Flash simulator nodes
        for cam in camera_names:
            flash_node = Node(
                package='firefly-ros2-wrapper-bringup',
                executable='flash_simulator_node',
                name=f'{cam}_flash_simulator',
                parameters=[{
                    'use_sim_time': True,
                    'flash_intensity': 2.5,
                    'shutter_speed': 0.1,
                    'max_flash_distance': 1.5,
                    'color_topic': f'/{cam}/image_raw/triggered',
                    'depth_topic': f'/{cam}/depth/image_raw/triggered',
                    'output_topic': f'/{cam}/image_raw',
                }],
                output='screen'
            )
            launch_nodes.append(flash_node)

    else:
        # ============================
        # Real Hardware Pipeline
        # ============================
        
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
            arguments=['--ros-args', '--log-level', 'warn'],
        )
        launch_nodes.append(spinnaker_sync_container)
        
        # Hardware trigger node (real version)
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

    return launch_nodes


def generate_launch_description():
    """Generate launch description with configurable arguments."""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Use Gazebo simulation (true) or real hardware (false)'
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
            description='Serial port for trigger connection (real hardware only)'
        ),
        DeclareLaunchArgument(
            'trigger_baudrate',
            default_value='9600',
            description='Baudrate for trigger serial connection (real hardware only)'
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
            default_value='true',
            description='Automatically start video triggering on launch'
        ),
        OpaqueFunction(function=launch_setup)
    ])
