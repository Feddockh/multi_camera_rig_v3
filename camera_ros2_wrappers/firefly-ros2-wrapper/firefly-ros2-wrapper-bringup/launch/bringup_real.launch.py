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
    use_rviz_value = LaunchConfiguration('use_rviz').perform(context)
    if use_rviz_value.lower() == 'true':
        # Include the firefly description launch file
        firefly_description_pkg = get_package_share_directory('firefly-ros2-wrapper-description')
        description_launch_file = os.path.join(firefly_description_pkg, 'launch', 'description.launch.py')
        
        description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'use_rviz': 'false',
            }.items()
        )
        launch_nodes.append(description_launch)
        # Use our own RViz config
        firefly_bringup_pkg = get_package_share_directory('firefly-ros2-wrapper-bringup')
        rviz_config_file = PJoin([firefly_bringup_pkg, 'rviz', 'view.rviz'])
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        launch_nodes.append(rviz_node)

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
    
    # Stereo rectify + scale nodes (replaces QoS republisher + image_proc rectify)
    output_width = int(LaunchConfiguration('output_width').perform(context))
    output_height = int(LaunchConfiguration('output_height').perform(context))
    
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

    # Add ArUco marker detection node for GT generation
    detect_markers = LaunchConfiguration('detect_markers').perform(context).lower() == 'true'
    if detect_markers:
        marker_dict = LaunchConfiguration('marker_dict').perform(context)
        marker_size = float(LaunchConfiguration('marker_length_m').perform(context))
        
        # Parse marker IDs and their class assignments
        marker_ids_str = LaunchConfiguration('marker_ids').perform(context)
        marker_class_ids_str = LaunchConfiguration('marker_class_ids').perform(context)
        
        marker_ids = [int(x) for x in marker_ids_str.split(',') if x.strip()]
        marker_class_ids = [int(x) for x in marker_class_ids_str.split(',') if x.strip()]
        
        marker_output_file = LaunchConfiguration('marker_output_file').perform(context)
        
        aruco_node = Node(
            package='firefly-ros2-wrapper-bringup',
            executable='aruco_detection_node',
            name='aruco_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'image_topic': '/firefly_left/image_rect',
                'camera_info_topic': '/firefly_left/camera_info_rect',
                'det_topic': '/firefly_left/aruco_det',
                'map_frame': LaunchConfiguration('map_frame').perform(context),
                'marker_size': marker_size,
                'dictionary': marker_dict,
                'max_process_rate_hz': 2.0,
                'draw_rejected': True,
                'marker_ids': marker_ids,
                'marker_class_ids': marker_class_ids,
                'marker_output_file': marker_output_file,
                # Detection parameters for fine-tuning
                # 'corner_refinement_method': 0, # 0=NONE, 1=SUBPIX, 2=CONTOUR
                # 'corner_refinement_win_size': 5,
                # 'corner_refinement_max_iterations': 30,
                # 'corner_refinement_min_accuracy': 0.1,
                # 'adaptive_thresh_win_size_min': 3,
                # 'adaptive_thresh_win_size_max': 23,
                # 'adaptive_thresh_win_size_step': 10,
                # 'adaptive_thresh_constant': 7.0,
                # 'min_marker_perimeter_rate': 0.03,
                # 'max_marker_perimeter_rate': 4.0,
                # 'polygonal_approx_accuracy_rate': 0.03,
                # 'min_corner_distance_rate': 0.05,
                # 'min_distance_to_border': 3,
                # 'min_marker_distance_rate': 0.05,
                # 'max_erroneous_bits_in_border_rate': 0.35,
                # 'error_correction_rate': 0.6,
                # 'min_otsu_std_dev': 5.0,
                # 'perspective_remove_pixel_per_cell': 4,
                # 'perspective_remove_ignored_margin_per_cell': 0.13,
            }],
        )
        launch_nodes.append(aruco_node)
        # Return early since we don't need other nodes when generating GT
        return launch_nodes

    # Add foundation stereo point cloud node
    stereo_matcher_model_dir = LaunchConfiguration('stereo_matcher_model_dir').perform(context)
    stereo_matcher_model_trt = LaunchConfiguration('stereo_matcher_model_trt').perform(context)
    engine_path = os.path.join(stereo_matcher_model_dir, stereo_matcher_model_trt)
    
    foundation_stereo_matcher_node = Node(
        package='firefly-ros2-wrapper-reconstruction',
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

    # Add detection node
    enable_detection = LaunchConfiguration('enable_detection').perform(context).lower() == 'true'
    if enable_detection:
        detection_model_dir = LaunchConfiguration('detection_model_dir').perform(context)
        detection_model_trt = LaunchConfiguration('detection_model_trt').perform(context)
        engine_path = os.path.join(detection_model_dir, detection_model_trt)
        
        yolo_node = Node(
            package='firefly-ros2-wrapper-detection',
            executable='yolov8_trt_node',
            name='yolov8_trt_node',
            output='screen',
            parameters=[{
                # Core
                'engine_path': engine_path,
                'image_topic': '/firefly_left/image_rect',
                'detection_topic': '/firefly_left/detections',
                # Model input (engine expects 1088x1440)
                'input_width': LaunchConfiguration('yolo_input_width'),
                'input_height': LaunchConfiguration('yolo_input_height'),
                # Postprocess
                'conf_thresh': LaunchConfiguration('yolo_conf_thresh'),
                'iou_thresh': LaunchConfiguration('yolo_iou_thresh'),
                'max_det': LaunchConfiguration('yolo_max_det'),
                # Scale settings
                'scale_output': True,
                'output_width': output_width,
                'output_height': output_height,
                'detection_topic_scaled': '/firefly_left/detections_scaled',
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
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        )
        launch_nodes.append(yolo_node)

    # Add semantic pointcloud node to generate point clouds from disparity
    semantic_pointcloud_node = Node(
        package='firefly-ros2-wrapper-reconstruction',
        executable='semantic_pointcloud_node',
        name='semantic_pointcloud_node',
        parameters=[{
            # Mode selection
            'use_semantics': LaunchConfiguration('use_semantics').perform(context).lower() == 'true',
            # Stereo parameters
            'baseline': 0.06,
            # Point cloud generation
            'stride': 1,
            'max_range_m': float(LaunchConfiguration('max_range_m').perform(context)),
            'use_background': LaunchConfiguration('use_background').perform(context).lower() == 'true',
            # Input topics
            'disparity_topic': '/firefly_left/disparity',
            'camera_info_topic': '/firefly_left/camera_info_rect_scaled',
            'image_topic': '/firefly_left/image_rect_scaled',
            'detection_topic': '/firefly_left/detections_scaled',
            # Output control
            'publish_cloud': LaunchConfiguration('publish_cloud').perform(context).lower() == 'true',
            'publish_depth': LaunchConfiguration('publish_depth').perform(context).lower() == 'true',
            # Output topics
            'cloud_topic': "/firefly_left/points2",
            'depth_topic': '/firefly_left/depth',
            # Semantic parameters (for semantic mode)
            'background_class_id': -1,
            'background_confidence': 0.5,
            'color_by_class': LaunchConfiguration('color_by_class').perform(context).lower() == 'true',
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
    return LaunchDescription([
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
            default_value='true',
            description='Automatically start video triggering on launch'
        ),
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
            'max_range_m',
            default_value='4.0',
            description='Maximum range for point cloud generation in meters'
        ),
        DeclareLaunchArgument(
            'use_background',
            default_value='true',
            description='Create background at max range value for missing depth points'
        ),
        DeclareLaunchArgument(
            'stereo_matcher_model_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-reconstruction'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument(
            'stereo_matcher_model_trt',
            default_value='fs_224x448_vit-small_iters5.plan',
            description='TensorRT engine file for the foundation stereo model (must match the output resolution)',
        ),
        DeclareLaunchArgument(
            'publish_cloud',
            default_value='true',
            description='Publish point cloud output'
        ),
        DeclareLaunchArgument(
            'use_semantics',
            default_value='true',
            description='Enable semantic mode with detections for point cloud'
        ),
        DeclareLaunchArgument(
            'color_by_class',
            default_value='true',
            description='In semantic mode, color points by class ID instead of RGB image'
        ),
        DeclareLaunchArgument(
            'publish_depth',
            default_value='false',
            description='Publish depth image output'
        ),
        DeclareLaunchArgument(
            'enable_detection', 
            default_value='true',
            description='Enable YOLOv8 detection node'
        ),
        DeclareLaunchArgument(
            'detection_model_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-detection'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument(
            'detection_model_trt', 
            default_value='best_real.plan',
            description='TensorRT engine file for YOLOv8 detection model',
        ),
        DeclareLaunchArgument(
            'yolo_input_width', 
            default_value='1440',
            description='Input width for YOLOv8 model'
        ),
        DeclareLaunchArgument(
            'yolo_input_height', 
            default_value='1088',
            description='Input height for YOLOv8 model'
        ),
        DeclareLaunchArgument(
            'yolo_conf_thresh', 
            default_value='0.25',
            description='Confidence threshold for YOLOv8 detection'
        ),
        DeclareLaunchArgument(
            'yolo_iou_thresh', 
            default_value='0.45',
            description='IOU threshold for YOLOv8 detection'
        ),
        DeclareLaunchArgument(
            'yolo_max_det', 
            default_value='300',
            description='Maximum detections for YOLOv8'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 to visualize camera and point cloud data'
        ),
        DeclareLaunchArgument(
            'detect_markers',
            default_value='false',
            description='Enable ArUco marker detection for GT generation (disables YOLO and pointcloud)'
        ),
        DeclareLaunchArgument(
            'marker_dict',
            default_value='DICT_4X4_50',
            description='ArUco dictionary for marker detection'
        ),
        DeclareLaunchArgument(
            'marker_length_m',
            default_value='0.05',
            description='Physical size of ArUco markers in meters'
        ),
        DeclareLaunchArgument(
            'marker_ids',
            default_value='0,1,2,3,4,5,6',
            description='Comma-separated list of marker IDs (parallel to marker_class_ids)'
        ),
        DeclareLaunchArgument(
            'marker_class_ids',
            default_value='0,0,0,0,0,0,0',
            description='Comma-separated list of class IDs for each marker (parallel to marker_ids). Supports 0-9+ classes.'
        ),
        DeclareLaunchArgument(
            'marker_output_file',
            default_value='aruco_gt_points_real.yaml',
            description='Output file path for GT marker positions'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame for ArUco marker poses'
        ),
        OpaqueFunction(function=launch_setup)
    ])