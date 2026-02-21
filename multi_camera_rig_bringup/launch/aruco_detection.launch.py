"""
ArUco marker detection launch file for ground truth generation.
This launch file brings up camera/gazebo bridging, rectification, and ArUco detection.
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
    """Setup function for ArUco marker detection pipeline."""
    
    launch_nodes = []
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context).lower() == 'true'
    
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
            'trigger_auto_connect': LaunchConfiguration('trigger_auto_connect'),
            'trigger_auto_start': LaunchConfiguration('trigger_auto_start'),
        }.items()
    )
    launch_nodes.append(camera_launch)

    # ============================
    # Rectification (left camera only for ArUco)
    # ============================
    rectify_scale_node = Node(
        package='multi_camera_rig_reconstruction',
        executable='stereo_rectify_scale_node',
        name='firefly_left_rectify_scale',
        output='screen',
        parameters=[{
            # Topics
            'in_image_topic': '/firefly_left/image_raw',
            'in_info_topic': '/firefly_left/camera_info',
            'out_rect_image_topic': '/firefly_left/image_rect',
            'out_rect_info_topic': '/firefly_left/camera_info_rect',
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

    # ============================
    # ArUco Marker Detection
    # ============================
    marker_dict = LaunchConfiguration('marker_dict').perform(context)
    marker_size = float(LaunchConfiguration('marker_length_m').perform(context))
    
    # Parse marker IDs and their class assignments
    marker_ids_str = LaunchConfiguration('marker_ids').perform(context)
    marker_class_ids_str = LaunchConfiguration('marker_class_ids').perform(context)
    
    marker_ids = [int(x) for x in marker_ids_str.split(',') if x.strip()]
    marker_class_ids = [int(x) for x in marker_class_ids_str.split(',') if x.strip()]
    
    marker_output_file = LaunchConfiguration('marker_output_file').perform(context)
    
    aruco_node = Node(
        package='multi_camera_rig_bringup',
        executable='aruco_detection_node',
        name='aruco_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_gazebo,
            'image_topic': '/firefly_left/image_rect',
            'camera_info_topic': '/firefly_left/camera_info_rect',
            'det_topic': '/firefly_left/aruco_det',
            'map_frame': LaunchConfiguration('map_frame').perform(context),
            'marker_size': marker_size,
            'dictionary': marker_dict,
            'max_process_rate_hz': 2.0,
            'draw_rejected': not use_gazebo,
            'marker_ids': marker_ids,
            'marker_class_ids': marker_class_ids,
            'marker_output_file': marker_output_file,
        }],
    )
    launch_nodes.append(aruco_node)

    return launch_nodes


def generate_launch_description():
    """Generate launch description for ArUco marker detection."""
    
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
            'trigger_auto_connect',
            default_value='true',
            description='Automatically test trigger connection on startup'
        ),
        DeclareLaunchArgument(
            'trigger_auto_start',
            default_value='true',
            description='Automatically start video triggering on launch'
        ),
        
        # ============================
        # ArUco Detection Parameters
        # ============================
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
            default_value='0,1,2,3,4,5,6,7,8,9,10,11',
            description='Comma-separated list of marker IDs (parallel to marker_class_ids)'
        ),
        DeclareLaunchArgument(
            'marker_class_ids',
            default_value='0,0,0,0,1,1,1,1,1,1,1,1',
            description='Comma-separated list of class IDs for each marker (parallel to marker_ids). Supports 0-9+ classes.'
        ),
        DeclareLaunchArgument(
            'marker_output_file',
            default_value='aruco_gt_points.yaml',
            description='Output file path for GT marker positions'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='husky/a200_base_footprint',
            description='Map frame for ArUco marker poses'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])
