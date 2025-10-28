"""
ros2 run image_view disparity_view --ros-args --remap image:=/firefly/disparity_custom
"""

"""
Notes
- Run with performance power settings
- the image transport 'raw' and other settings didn't do anything

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Launch Firefly camera processing pipeline for cameras already integrated in robot
    Assumes Gazebo is running and camera is part of the robot description in husky_xarm6_mcr_bringup
    """
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Package directories
    firefly_description_pkg = get_package_share_directory('firefly-ros2-wrapper-description')
    
    launch_actions = []

    launch_actions.append(SetEnvironmentVariable('ROS_IMAGE_TRANSPORT', 'raw'))
    launch_actions.append(SetEnvironmentVariable('IMAGE_TRANSPORT', 'raw'))
    
    # Image bridges for Gazebo to ROS2 communication
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='firefly_image_bridge',
        arguments=[
            '/firefly_left/image',
            '/firefly_right/image',
        ],
        output='screen',
        # QoS override to keep latency low
        parameters=[
            {'use_sim_time': True},
            {'qos': 'sensor_data'},  # Options: 'default', 'sensor_data', 'system_default'
        ],
    )
    launch_actions.append(image_bridge)

    # Bridge the camera info topics from Gazebo to ROS2
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='firefly_camera_info_bridge',
        arguments=[
            '/firefly_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/firefly_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'qos': 'sensor_data'},
        ],
    )
    launch_actions.append(camera_info_bridge)

    # 2. Stereo processing pipeline using composable nodes
    # This approach uses a multi-threaded component container for better performance
    # with zero-copy intra-process communication between components
    
    # Define rectification nodes as composable components
    rect_left = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='left_rectify',
        namespace='firefly_left',
        parameters=[
            {'use_sim_time': True},
            {'queue_size': 5},  # Buffer frames to handle processing variations
            {'interpolation': 0},  # INTER_NEAREST (fastest)
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    rect_right = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='right_rectify',
        namespace='firefly_right',
        parameters=[
            {'use_sim_time': True},
            {'queue_size': 5},
            {'interpolation': 0},
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    # Create multi-threaded component container for parallel processing
    stereo_container = ComposableNodeContainer(
        name='stereo_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded executor
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'WARN'],
        composable_node_descriptions=[rect_left, rect_right],
    )
    launch_actions.append(stereo_container)
    
    # Stage 3: Disparity computation (optimized for performance)
    # disparity_node = Node(
    #     package='stereo_image_proc',
    #     executable='disparity_node',
    #     name='firefly_disparity_node',
    #     remappings=[
    #         ('left/image_rect', '/firefly_left/image_rect_mono'),
    #         ('left/camera_info', '/firefly_left/camera_info'),
    #         ('right/image_rect', '/firefly_right/image_rect_mono'),
    #         ('right/camera_info', '/firefly_right/camera_info'),
    #         ('disparity', '/firefly/disparity'),
    #     ],
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'approximate_sync': True},
    #         {'queue_size': 10},
    #         # Better quality stereo parameters
    #         {'stereo_algorithm': 1},  # StereoSGBM for better quality
    #         {'min_disparity': 0},     
    #         {'disparity_range': 96},  # Increase range for better coverage
    #         {'correlation_window_size': 11},  # Smaller for finer details
    #         {'texture_threshold': 15}, 
    #         {'uniqueness_ratio': 10.0},  # Lower for more matches
    #         {'speckle_size': 200},     # More aggressive speckle filtering
    #         {'speckle_range': 2},      # Tighter speckle filtering
    #         {'prefilter_size': 9},     # Add prefiltering
    #         {'prefilter_cap': 31}
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(disparity_node)
    
    # Custom C++ disparity node for better control
    # custom_disparity_node = Node(
    #     package='firefly-ros2-wrapper-bringup',
    #     executable='stereo_disparity_node',
    #     name='firefly_custom_disparity_node_sgbm',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'stereo_algorithm': 1},  # StereoSGBM
    #         {'min_disparity': 0},
    #         {'num_disparities': 160}, # Must be multiple of 16
    #         {'block_size': 5},
    #         {'P1': 0},  # 0 = auto-compute from block_size
    #         {'P2': 0},  # 0 = auto-compute from block_size
    #         {'disp12_max_diff': 1},
    #         {'pre_filter_cap': 31},
    #         {'uniqueness_ratio': 10},
    #         {'speckle_window_size': 200},
    #         {'speckle_range': 2},
    #         {'mode': 2},  # StereoSGBM::MODE_SGBM_3WAY (best quality)
    #         # Camera calibration parameters
    #         {'focal_length': 858.0},  # Focal length in pixels (from camera intrinsics)
    #         {'baseline': 0.06}        # Baseline in meters (6cm from URDF X-offset)
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(custom_disparity_node)

    # Custom C++ disparity node for better control
    # custom_disparity_node = Node(
    #     package='firefly-ros2-wrapper-bringup',
    #     executable='fs_disparity_node',
    #     name='firefly_custom_disparity_node_fs',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'height': 1080},
    #         {'width': 1440},
    #         {'baseline': 0.06},
    #         {'scale_factor': 0.7},
    #         {'vit_size': 'small'},
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(custom_disparity_node)
    
    # Stage 4: Point cloud generation (optimized for performance and RViz compatibility)
    # point_cloud_node = Node(
    #     package='stereo_image_proc',
    #     executable='point_cloud_node',
    #     name='firefly_point_cloud_node',
    #     remappings=[
    #         ('left/camera_info', '/firefly_left/camera_info'),
    #         ('right/camera_info', '/firefly_right/camera_info'),
    #         ('left/image_rect_color', '/firefly_left/image_rect_mono'),  # Use rectified for better results
    #         ('disparity', '/firefly/disparity_custom'),
    #         ('points2', '/firefly/points2'),
    #     ],
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'approximate_sync': True},
    #         {'queue_size': 10},
    #         # Point cloud parameters
    #         {'avoid_point_cloud_padding': True},  # More efficient point cloud
    #         {'use_color': True},  # Enable color information
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(point_cloud_node)

    # Custom C++ point cloud node for better performance
    # point_cloud_node = Node(
    #     package='firefly-ros2-wrapper-bringup',
    #     executable='point_cloud_node',
    #     name='firefly_point_cloud_node',
    #     remappings=[
    #         ('disparity', '/firefly/disparity_custom'),
    #         ('left/camera_info', '/firefly_left/camera_info'),
    #         ('right/camera_info', '/firefly_right/camera_info'),
    #         ('left/image_rect_color', '/firefly_left/image_rect_mono'),
    #         ('points2', '/firefly/points2'),
    #     ],
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'use_color': True},     # Disable color for now to use 3-way sync
    #         {'queue_size': 50},       # Increased queue size
    #         {'min_depth': 0.1},       # Minimum depth in meters
    #         {'max_depth': 2.0},      # Maximum depth in meters
    #         {'organized': False},     # Dense point cloud (no NaN values)
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(point_cloud_node)

    use_rviz_value = use_rviz.perform(context)
    if use_rviz_value.lower() == 'true':
        rviz_config_file = PathJoinSubstitution([
            firefly_description_pkg,
            'rviz',
            'view.rviz'
        ])
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        launch_actions.append(rviz_node)
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz for visualization'
        ),
        OpaqueFunction(function=launch_setup)
    ])
