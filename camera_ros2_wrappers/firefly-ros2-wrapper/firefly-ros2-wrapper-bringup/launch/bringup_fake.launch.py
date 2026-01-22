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
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution as PJoin
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

    # The issue with the image bridge is that it adds some delay to the raw images because of the 
    # Overhead added to create compressed and other image formats. Compressed is faster however,
    # and may be better for some applications.
    # Image bridges for Gazebo to ROS2 communication
    # image_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='firefly_image_bridge',
    #     arguments=[
    #         '/firefly_left/image',
    #         '/firefly_right/image',
    #     ],
    #     output='screen',
    #     # QoS override to keep latency low
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'qos': 'sensor_data'},  # Options: 'default', 'sensor_data', 'system_default'
    #     ],
    # )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='firefly_depth_bridge',
        arguments=[
            '/firefly_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/firefly_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/firefly_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/firefly_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/firefly_left/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/firefly_left/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'qos': 'sensor_data'},
        ],
    )
    launch_nodes = [sensor_bridge]

    # Flash simulator node - applies flash effect to left camera
    flash_simulator = Node(
        package='firefly-ros2-wrapper-bringup',
        executable='flash_simulator_node',
        name='firefly_flash_simulator',
        parameters=[
            {'use_sim_time': True},
            {'flash_intensity': 2.5},        # Flash brightness multiplier
            {'shutter_speed': 0.1},          # Base image brightness (simulates fast shutter)
            {'max_flash_distance': 1.5},     # Maximum effective flash distance (meters)
            {'color_topic': '/firefly_left/image_raw'},
            {'depth_topic': '/firefly_left/depth/image'},
            {'output_topic': '/firefly_left/image'},
        ],
        output='screen'
    )
    launch_nodes.append(flash_simulator)

    # Apply point very far away for pixels with no depth
    # This is important for establishing free space in the occupancy map
    # Invalid pixels are downsampled to reduce redundant ray casting
    depth_cleaner = Node(
        package='firefly-ros2-wrapper-bringup',
        executable='depth_cleaner_node',
        name='firefly_depth_cleaner',
        parameters=[
            {'use_sim_time': True},
            {'input_depth_topic': '/firefly_left/depth/image'},
            {'output_depth_topic': '/firefly_left/depth/image_cleaned'},
            {'min_depth': 0.01},                  # Min valid depth (filters sensor origin artifacts)
            {'max_depth': 5.0},                  # Max valid depth in meters
            {'invalid_depth_value': 5.0},        # Depth value for invalid pixels (match octomap max_range)
            {'invalid_pixel_stride': 4},         # Keep every Nth invalid pixel (1=all, 2=half, 4=quarter, 8=eighth)
            {'preserve_valid_pixels': True},     # Always keep valid depth measurements
        ],
        output='screen'
    )
    launch_nodes.append(depth_cleaner)

    # Registering not necessary because there is no rectification necessary (no intrinsics applied in simulation)
    rgbd_to_cloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='firefly_rgbd_to_cloud',
        namespace='firefly_left',
        remappings=[
            ('rgb/image_rect_color',        '/firefly_left/image'),
            ('rgb/camera_info',             '/firefly_left/camera_info'),
            ('depth_registered/image_rect', '/firefly_left/depth/image_cleaned'),
            ('points', 'points2'),
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )
    launch_nodes.append(rgbd_to_cloud)
    
    use_rviz_value = LaunchConfiguration('use_rviz').perform(context)
    if use_rviz_value.lower() == 'true':
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
            'use_rviz',
            default_value='false',
            description='Whether to start RViz for visualization'
        ),
        OpaqueFunction(function=launch_setup)
    ])
