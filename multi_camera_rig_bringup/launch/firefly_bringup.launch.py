"""
Multi-camera rig bringup launch file for firefly stereo cameras.
Brings up cameras, detection, and reconstruction in a unified pipeline.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to configure complete multi-camera rig pipeline."""
    
    launch_nodes = []

    # ============================
    # Camera Bringup
    # ============================
    firefly_bringup_pkg = get_package_share_directory('firefly-ros2-wrapper-bringup')
    firefly_bringup_launch = os.path.join(firefly_bringup_pkg, 'launch', 'bringup.launch.py')
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(firefly_bringup_launch),
        launch_arguments={
            'use_gazebo': LaunchConfiguration('use_gazebo'),
            'calib_dir': LaunchConfiguration('calib_dir'),
            'spinnaker_config_file': LaunchConfiguration('spinnaker_config_file'),
            'spinnaker_param_file': LaunchConfiguration('spinnaker_param_file'),
            'trigger_serial_port': LaunchConfiguration('trigger_serial_port'),
            'trigger_baudrate': LaunchConfiguration('trigger_baudrate'),
            'trigger_flash_duration_ms': LaunchConfiguration('trigger_flash_duration_ms'),
            'trigger_frame_rate_hz': LaunchConfiguration('trigger_frame_rate_hz'),
            'trigger_auto_connect': 'true',
            'trigger_auto_start': LaunchConfiguration('trigger_auto_start'),
        }.items()
    )
    launch_nodes.append(camera_launch)

    # ============================
    # Reconstruction Pipeline
    # ============================
    output_width = int(LaunchConfiguration('output_width').perform(context))
    output_height = int(LaunchConfiguration('output_height').perform(context))
    
    # Get camera names from config
    camera_names = ['firefly_left', 'firefly_right']  # TODO: Load from config
    
    # Stereo rectify + scale nodes
    for cam_name in camera_names:
        rectify_scale_node = Node(
            package='multi_camera_rig_reconstruction',
            executable='stereo_rectify_scale_node',
            name=f'{cam_name}_rectify_scale',
            output='screen',
            parameters=[{
                # Topics
                'in_image_topic': f'/{cam_name}/image_raw',
                'in_info_topic': f'/{cam_name}/camera_info',
                'out_rect_image_topic': f'/{cam_name}/image_rect',
                'out_rect_info_topic': f'/{cam_name}/camera_info_rect',
                'out_rect_scaled_image_topic': f'/{cam_name}/image_rect_scaled',
                'out_rect_scaled_info_topic': f'/{cam_name}/camera_info_rect_scaled',
                # Scale settings
                'output_width': output_width,
                'output_height': output_height,
                'interpolation': 'linear',
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

    # Foundation stereo matcher
    stereo_matcher_model_dir = LaunchConfiguration('stereo_matcher_model_dir').perform(context)
    stereo_matcher_model_trt = LaunchConfiguration('stereo_matcher_model_trt').perform(context)
    engine_path = os.path.join(stereo_matcher_model_dir, stereo_matcher_model_trt)
    
    foundation_stereo_matcher_node = Node(
        package='multi_camera_rig_reconstruction',
        executable='foundation_stereo_matcher_node',
        name='foundation_stereo_matcher_node',
        parameters=[{
            # TensorRT engine
            'engine_path': engine_path,
            # Input topics
            'left_image_topic': '/firefly_left/image_rect_scaled',
            'right_image_topic': '/firefly_right/image_rect_scaled',
            'left_info_topic': '/firefly_left/camera_info_rect_scaled',
            # Output topic
            'disparity_topic': '/firefly_left/disparity',
            # Subscriber QoS settings
            'sub_qos.reliability': 'best_effort',
            'sub_qos.durability': 'volatile',
            'sub_qos.history': 'keep_last',
            'sub_qos.depth': 5,
            # Publisher QoS settings
            'pub_qos.reliability': 'best_effort',
            'pub_qos.durability': 'volatile',
            'pub_qos.history': 'keep_last',
            'pub_qos.depth': 5,
            # Disparity filter: none | speckle
            'disp_filter.mode': 'speckle',
            # speckle params
            'disp_filter.speckle_max_size': 120,
            'disp_filter.speckle_range': 1.0,
            'disp_filter.speckle_scale': 16.0,
        }],
        output='screen'
    )
    launch_nodes.append(foundation_stereo_matcher_node)

    # ============================
    # Detection Pipeline (Optional)
    # ============================
    enable_detection = LaunchConfiguration('enable_detection').perform(context).lower() == 'true'
    if enable_detection:
        detection_model_dir = LaunchConfiguration('detection_model_dir').perform(context)
        detection_model_trt = LaunchConfiguration('detection_model_trt').perform(context)
        engine_path = os.path.join(detection_model_dir, detection_model_trt)
        
        yolo_node = Node(
            package='multi_camera_rig_detection',
            executable='yolo_trt_node',
            name='yolo_trt_node',
            output='screen',
            parameters=[{
                # Core
                'engine_path': engine_path,
                'image_topic': '/firefly_left/image_rect',
                'detection_topic': '/firefly_left/detections',
                'seg_detection_topic': '/firefly_left/instance_segmentation',
                # Tensors
                'input_tensor': 'images',
                'output_tensor': 'output0',
                'proto_tensor': 'output1',
                # Task type (auto-detect based on engine, or force 'det' or 'seg')
                'task': 'seg' if LaunchConfiguration('use_seg_detection').perform(context).lower() == 'true' else 'det',
                # Model input (engine expects 1088x1440)
                'input_width': 1440,
                'input_height': 1088,
                'stride': 32,
                'scaleup': True,
                # Scale settings
                'scale_output': True,
                'output_width': output_width,
                'output_height': output_height,
                'detection_topic_scaled': '/firefly_left/detections_scaled',
                'seg_detection_topic_scaled': '/firefly_left/instance_segmentation_scaled',
                # Postprocess
                'conf_thresh': 0.5,
                'iou_thresh': 0.45,
                'max_det': 300,
                # Segmentation support
                'mask_alpha': 0.45,
                'mask_thresh': 0.50,
                # QoS subscriber
                'sub_qos.reliability': 'reliable',
                'sub_qos.durability': 'volatile',
                'sub_qos.history': 'keep_last',
                'sub_qos.depth': 5,
                # QoS publisher
                'pub_qos.reliability': 'best_effort',
                'pub_qos.durability': 'volatile',
                'pub_qos.history': 'keep_last',
                'pub_qos.depth': 5,
                # Debug
                'debug': True,
                'debug_masks': True,
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        )
        launch_nodes.append(yolo_node)

    # ============================
    # Semantic Point Cloud Generation
    # ============================
    semantic_pointcloud_node = Node(
        package='multi_camera_rig_reconstruction',
        executable='semantic_pointcloud_node',
        name='semantic_pointcloud_node',
        parameters=[{
            # Mode selection
            'use_semantics': LaunchConfiguration('use_semantics').perform(context).lower() == 'true',
            'use_seg_detection': LaunchConfiguration('use_seg_detection').perform(context).lower() == 'true',
            # Stereo parameters
            'baseline': 0.06,
            # Point cloud generation
            'stride': 1,
            'max_range_m': 5.0,
            'use_background': True,
            # Input topics
            'disparity_topic': '/firefly_left/disparity',
            'camera_info_topic': '/firefly_left/camera_info_rect_scaled',
            'image_topic': '/firefly_left/image_rect_scaled',
            'detection_topic': '/firefly_left/detections_scaled',
            'seg_detection_topic': '/firefly_left/instance_segmentation_scaled',
            # Output control
            'publish_cloud': True,
            'publish_depth': False,
            # Output topics
            'cloud_topic': "/firefly_left/points2",
            'depth_topic': '/firefly_left/depth',
            # Semantic parameters (for semantic mode)
            'background_class_id': -1,
            'background_confidence': 0.5,
            'color_by_class': True,
            # Subscriber QoS settings
            'sub_qos.reliability': 'best_effort',
            'sub_qos.durability': 'volatile',
            'sub_qos.history': 'keep_last',
            'sub_qos.depth': 5,
            # Publisher QoS settings
            'pub_qos.reliability': 'best_effort',
            'pub_qos.durability': 'volatile',
            'pub_qos.history': 'keep_last',
            'pub_qos.depth': 5,
            # Debug
            'debug': False,
        }],
        output='screen'
    )
    launch_nodes.append(semantic_pointcloud_node)

    return launch_nodes


def generate_launch_description():
    """Generate launch description with configurable arguments."""
    
    return LaunchDescription([
        # ============================
        # Camera Parameters
        # ============================
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
        
        # ============================
        # Trigger Parameters
        # ============================
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
            'trigger_auto_start',
            default_value='true',
            description='Automatically start video triggering on launch'
        ),
        
        # ============================
        # Reconstruction Parameters
        # ============================
        DeclareLaunchArgument(
            'output_width',
            default_value='448',
            description='Output width for rectified and scaled images'
        ),
        DeclareLaunchArgument(
            'output_height',
            default_value='224',
            description='Output height for rectified and scaled images'
        ),
        DeclareLaunchArgument(
            'stereo_matcher_model_dir',
            default_value=PJoin([FindPackageShare('multi_camera_rig_reconstruction'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument(
            'stereo_matcher_model_trt',
            default_value='fs_224x448_vit-small_iters5.plan',
            description='TensorRT engine file for the foundation stereo model (must match the output resolution)',
        ),
        DeclareLaunchArgument(
            'use_semantics',
            default_value='true',
            description='Enable semantic mode with detections for point cloud'
        ),
        DeclareLaunchArgument(
            'use_seg_detection',
            default_value='true',
            description='Use segmentation detections (true) or just bounding boxes (false) for semantic point cloud coloring'
        ),
        
        # ============================
        # Detection Parameters
        # ============================
        DeclareLaunchArgument(
            'enable_detection', 
            default_value='true',
            description='Enable YOLO detection node'
        ),
        DeclareLaunchArgument(
            'detection_model_dir',
            default_value=PJoin([FindPackageShare('multi_camera_rig_detection'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument(
            'detection_model_trt', 
            default_value='best_lab.plan',
            description='TensorRT engine file for YOLO detection model',
        ),
        
        OpaqueFunction(function=launch_setup)
    ])
