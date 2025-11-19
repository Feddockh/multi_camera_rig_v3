from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.substitutions import FindPackageShare

import os
import yaml


def launch_setup(context, *args, **kwargs):

    calib_dir = LaunchConfig('calib_dir').perform(context)
    ffc_dir = LaunchConfig('ffc_dir').perform(context)

    # Load in the Ximea camera configuration
    ximea_config_file = LaunchConfig('ximea_config_file').perform(context)
    with open(ximea_config_file, 'r') as f:
        ximea_config = yaml.safe_load(f)

    camera_name = ximea_config.get('camera_name', 'ximea_camera')

    # Replace ffc_dir from data_dir and camera_name
    ximea_config['ffc_dir'] = ffc_dir

    # Replace camera_info_url from calib_dir and camera_name
    camera_info_url = f"file://{os.path.join(calib_dir, camera_name + '.yaml')}"
    ximea_config['camera_info_url'] = camera_info_url

    ximea_node = Node(
        package='ximea-ros2-wrapper-bringup',  # adjust if your package name differs
        executable='ximea_camera_node',
        name=camera_name,
        parameters=[ximea_config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        # namespace='ximea',  # uncomment if you want a namespace
    )

    return [ximea_node]


def generate_launch_description():
    return LaunchDescription([
        LaunchArg(
            'calib_dir',
            default_value=PJoin([
                FindPackageShare('ximea-ros2-wrapper-bringup'),
                'calibs'
            ]),
            description='Directory containing camera calibration YAML files',
        ),
        LaunchArg(
            'ximea_config_file',
            default_value=PJoin([
                FindPackageShare('ximea-ros2-wrapper-bringup'),
                'configs',
                'ximea.yaml'
            ]),
            description='Path to the Ximea camera configuration YAML file.',
        ),
        LaunchArg(
            'ffc_dir',
            default_value=os.path.expanduser('~/tmp/ffc'),
            description='Directory containing FFC dark/mid images. '
                        'If set, overrides data_dir parameter.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
