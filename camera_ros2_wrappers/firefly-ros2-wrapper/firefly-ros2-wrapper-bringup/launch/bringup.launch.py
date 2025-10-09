from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
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
                parameters=[spinnaker_config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],  # Adjust log level as needed
    )
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
        OpaqueFunction(function=launch_setup)
    ])