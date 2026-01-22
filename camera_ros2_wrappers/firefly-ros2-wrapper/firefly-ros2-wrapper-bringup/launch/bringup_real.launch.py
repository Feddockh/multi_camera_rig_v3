import os
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
                    ('~/firefly_left/image_raw/compressed', '/firefly_left/image_raw/compressed'),
                    ('~/firefly_left/image_raw/compressedDepth', '/firefly_left/image_raw/compressedDepth'),
                    ('~/firefly_left/image_raw/ffmpeg', '/firefly_left/image_raw/ffmpeg'),
                    ('~/firefly_left/image_raw/theora', '/firefly_left/image_raw/theora'),
                    ('~/firefly_left/meta', '/firefly_left/meta'),

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
    nodes = [spinnaker_sync_container]
    # Current delay is about 0.03 seconds to get the images
    
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
        nodes.append(trigger_node)
    
    # Add rectification nodes for each camera
    enable_rectification = LaunchConfig('enable_rectification').perform(context).lower() == 'true'
    if enable_rectification:

        # QoS republishers: RELIABLE (camera driver) -> BEST_EFFORT
        qos_repub_nodes = []
        for cam in camera_names:
            for msg_type, suffix in [
                ('sensor_msgs/msg/Image', 'image_raw'),
                ('sensor_msgs/msg/CameraInfo', 'camera_info'),
            ]:
                qos_repub_nodes.append(
                    Node(
                        package='firefly-ros2-wrapper-bringup',
                        executable='qos_republisher_node',
                        name=f'qos_repub_{cam}_{suffix}',
                        output='screen',
                        parameters=[{
                            'type': msg_type,
                            'in_topic': f'/{cam}/{suffix}',
                            'out_topic': f'/{cam}/{suffix}_be',

                            # subscription QoS (match camera driver)
                            'sub_qos.reliability': 'reliable',
                            'sub_qos.history': 'keep_last',
                            'sub_qos.depth': 5,

                            # publish QoS (match image_proc / sensor pipeline)
                            'pub_qos.reliability': 'best_effort',
                            'pub_qos.history': 'keep_last',
                            'pub_qos.depth': 5,
                        }]
                    )
                )
        nodes.extend(qos_repub_nodes)
        # Delay to about 0.05 seconds now

        for cam_name in camera_names:
            rectify_node = Node(
                package='image_proc',
                executable='rectify_node',
                name=f'{cam_name}_rectify',
                namespace=cam_name,
                remappings=[
                    ('image', 'image_raw_be'),
                    ('camera_info', 'camera_info_be'),
                    ('image_rect', 'image_rect'),
                ],
            )
            nodes.append(rectify_node)
        # Current delay up to this point is about 0.06 seconds
        
        # # Add disparity computation node for the stereo pair
        # enable_disparity = LaunchConfig('enable_disparity').perform(context).lower() == 'true'
        # if enable_disparity and enable_rectification:
        #     custom_disparity_node = Node(
        #         package='firefly-ros2-wrapper-bringup',
        #         executable='fs_disparity_node.py',
        #         name='firefly_custom_disparity_node_fs',
        #         parameters=[
        #             {'use_sim_time': False},
        #             {'height': 1080},
        #             {'width': 1440},
        #             {'baseline': 0.06},
        #             {'scale_factor': 0.7},
        #             {'vit_size': 'small'},
        #         ],
        #         output='screen'
        #     )
        #     nodes.append(custom_disparity_node)

        # Add foundation stereo point cloud node
        enable_point_cloud = LaunchConfig('enable_point_cloud').perform(context).lower() == 'true'
        if enable_point_cloud:
            model_dir = LaunchConfig('model_dir').perform(context)
            model = LaunchConfig('tensorrt_file').perform(context)
            engine_path = os.path.join(model_dir, model)
            foundation_point_cloud_node = Node(
                package='firefly-ros2-wrapper-bringup',
                executable='foundation_stereo_points_node',
                name='foundation_stereo_points_node',
                parameters=[
                    {'engine_path': engine_path},
                    {'baseline': 0.06},
                    {'stride': 2},
                    {'max_range_m': 3.0},
                ],
                output='screen'
            )
            nodes.append(foundation_point_cloud_node)
            # Delay here is about 0.16 seconds

    return nodes

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
            default_value='5',
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
        LaunchArg(
            'enable_rectification',
            default_value='true',
            description='Enable image rectification nodes for each camera'
        ),
        LaunchArg(
            'enable_point_cloud',
            default_value='true',
            description='Enable point cloud computation node for the stereo pair'
        ),
        LaunchArg(
            'model_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        LaunchArg(
            'tensorrt_file',
            default_value='fs_448x672_vit-small_iters5.plan',
            description='TensorRT engine file for the foundation stereo model',
        ),
        OpaqueFunction(function=launch_setup)
    ])