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

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='firefly_depth_bridge',
        arguments=[
            '/firefly_left/image_raw_sim@sensor_msgs/msg/Image[gz.msgs.Image',
            '/firefly_left/depth/image_sim@sensor_msgs/msg/Image[gz.msgs.Image', # Needed for flash simulator
            '/firefly_right/image_raw_sim@sensor_msgs/msg/Image[gz.msgs.Image',
            '/firefly_right/depth/image_sim@sensor_msgs/msg/Image[gz.msgs.Image', # Needed for flash simulator
            '/firefly_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/firefly_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'qos': 'reliable'},
        ],
    )
    launch_nodes = [sensor_bridge]

    # Flash simulator node - applies flash effect to left camera
    for cam in camera_names:
        flash_node = Node(
            package='firefly-ros2-wrapper-bringup',
            executable='flash_simulator_node',
            name='firefly_flash_simulator',
            parameters=[
                {'use_sim_time': True},
                {'flash_intensity': 2.5},        # Flash brightness multiplier
                {'shutter_speed': 0.1},          # Base image brightness (simulates fast shutter)
                {'max_flash_distance': 1.5},     # Maximum effective flash distance (meters)
                {'color_topic': f'/{cam}/image_raw_sim'},
                {'depth_topic': f'/{cam}/depth/image_sim'},
                {'output_topic': f'/{cam}/image_raw'},
            ],
            output='screen'
        )
        launch_nodes.append(flash_node)
    
    # Add rectification and scaling nodes for each camera
    enable_rectification = LaunchConfiguration('enable_rectification').perform(context).lower() == 'true'
    if enable_rectification:
        # Stereo rectify + scale nodes (replaces QoS republisher + image_proc rectify)
        output_width = int(LaunchConfiguration('output_width').perform(context))
        output_height = int(LaunchConfiguration('output_height').perform(context))
        
        for cam_name in camera_names:
            rectify_scale_node = Node(
                package='firefly-ros2-wrapper-bringup',
                executable='stereo_rectify_scale_node',
                name=f'{cam_name}_rectify_scale',
                output='screen',
                parameters=[{
                    # Topics
                    'in_image_topic': f'/{cam_name}/image_raw',
                    'in_info_topic': f'/{cam_name}/camera_info',
                    'out_image_topic': f'/{cam_name}/image_rect_scaled',
                    'out_info_topic': f'/{cam_name}/camera_info_rect_scaled',
                    # Scale settings
                    'output_width': output_width,
                    'output_height': output_height,
                    'interpolation': 'linear',
                    # Subscriber QoS
                    'sub_qos.reliability': 'reliable',
                    'sub_qos.depth': 5,
                    # Publisher QoS
                    'pub_qos.reliability': 'best_effort',
                    'pub_qos.depth': 5,
                }]
            )
            launch_nodes.append(rectify_scale_node)
            # Added delay here is about 0.02 seconds

        # Add foundation stereo point cloud node
        enable_point_cloud = LaunchConfiguration('enable_point_cloud').perform(context).lower() == 'true'
        if enable_point_cloud:
            model_dir = LaunchConfiguration('model_dir').perform(context)
            model = LaunchConfiguration('tensorrt_file').perform(context)
            engine_path = os.path.join(model_dir, model)
            
            foundation_point_cloud_node = Node(
                package='firefly-ros2-wrapper-bringup',
                executable='foundation_stereo_matcher_node',
                name='foundation_stereo_matcher_node',
                parameters=[
                    # TensorRT engine
                    {'engine_path': engine_path},
                    # Stereo parameters
                    {'baseline': float(LaunchConfiguration('baseline').perform(context))},
                    # Point cloud generation
                    {'stride': int(LaunchConfiguration('stride').perform(context))},
                    {'max_range_m': float(LaunchConfiguration('max_range_m').perform(context))},
                    {'use_background': LaunchConfiguration('use_background').perform(context).lower() == 'true'},
                    # Input topics
                    {'left_image_topic': '/firefly_left/image_rect_scaled'},
                    {'right_image_topic': '/firefly_right/image_rect_scaled'},
                    {'left_info_topic': '/firefly_left/camera_info_rect_scaled'},
                    # Output control
                    {'publish_cloud': LaunchConfiguration('publish_cloud').perform(context).lower() == 'true'},
                    {'publish_depth': LaunchConfiguration('publish_depth').perform(context).lower() == 'true'},
                    {'publish_disparity': LaunchConfiguration('publish_disparity').perform(context).lower() == 'true'},
                    # Output topics
                    {'cloud_topic': LaunchConfiguration('cloud_topic').perform(context)},
                    {'depth_topic': LaunchConfiguration('depth_topic').perform(context)},
                    {'disparity_topic': LaunchConfiguration('disparity_topic').perform(context)},
                    # Subscriber QoS settings
                    {'sub_qos.reliability': 'best_effort'},
                    {'sub_qos.durability': 'volatile'},
                    {'sub_qos.history': 'keep_last'},
                    {'sub_qos.depth': 5},
                    # Publisher QoS settings
                    {'pub_qos.reliability': 'best_effort'},
                    {'pub_qos.durability': 'volatile'},
                    {'pub_qos.history': 'keep_last'},
                    {'pub_qos.depth': 5},
                    ### Filter modes (sample configuration - speckle seems to work best)
                    ### Disparity filter: none | median | bilateral | speckle | edge_flying_kill
                    {'disp_filter.mode': 'speckle'},
                    # median params
                    {'disp_filter.median_ksize': 5},
                    # bilateral params
                    {'disp_filter.bilateral_d': 7},
                    {'disp_filter.bilateral_sigma_color': 3.0},
                    {'disp_filter.bilateral_sigma_space': 7.0},
                    # speckle params
                    {'disp_filter.speckle_max_size': 120},
                    {'disp_filter.speckle_range': 1.0},
                    {'disp_filter.speckle_scale': 16.0},
                    # edge_flying_kill params
                    {'disp_filter.edge_ksize': 5},
                    {'disp_filter.edge_tau': 0.20},
                    {'disp_filter.edge_min_neighbors': 6},
                    ### Depth filter: none | flying_pixel | median
                    {'depth_filter.mode': 'none'},
                    # flying_pixel params
                    {'depth_filter.flying_ksize': 5},
                    {'depth_filter.flying_tau': 0.25},
                    {'depth_filter.flying_min_neighbors': 6},
                    # median params
                    {'depth_filter.median_ksize': 5},
                    ### Point cloud filter: none | grid_outlier | knn_outlier
                    {'pc_filter.mode': 'none'},
                    # grid_outlier params
                    {'pc_filter.grid_ksize': 5},
                    {'pc_filter.grid_tau': 0.25},
                    {'pc_filter.grid_min_neighbors': 4},
                    # knn_outlier params
                    {'pc_filter.knn_k': 20},
                    {'pc_filter.knn_stddev_multiplier': 2.0},
                ],
                output='screen'
            )
            launch_nodes.append(foundation_point_cloud_node)

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
            'enable_trigger',
            default_value='true',
            description='Enable hardware trigger node'
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
            default_value='5',
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
            'enable_rectification',
            default_value='true',
            description='Enable image rectification nodes for each camera'
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
            'enable_point_cloud',
            default_value='true',
            description='Enable point cloud computation node for the stereo pair'
        ),
        DeclareLaunchArgument(
            'baseline',
            default_value='0.06',
            description='Stereo baseline in meters'
        ),
        DeclareLaunchArgument(
            'stride',
            default_value='1',
            description='Point cloud stride (1=full resolution, 2=quarter points, 4=1/16 points)'
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
            'model_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-bringup'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument(
            'tensorrt_file',
            default_value='fs_224x448_vit-small_iters5.plan',
            description='TensorRT engine file for the foundation stereo model (must match the output resolution)',
        ),
        DeclareLaunchArgument(
            'publish_cloud',
            default_value='true',
            description='Publish point cloud output'
        ),
        DeclareLaunchArgument(
            'publish_depth',
            default_value='false',
            description='Publish depth image output'
        ),
        DeclareLaunchArgument(
            'publish_disparity',
            default_value='false',
            description='Publish disparity image output'
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/firefly_left/points2',
            description='Topic name for point cloud output'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/firefly_left/depth',
            description='Topic name for depth image output'
        ),
        DeclareLaunchArgument(
            'disparity_topic',
            default_value='/firefly_left/disparity',
            description='Topic name for disparity image output'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 to visualize camera and point cloud data'
        ),
        OpaqueFunction(function=launch_setup)
    ])