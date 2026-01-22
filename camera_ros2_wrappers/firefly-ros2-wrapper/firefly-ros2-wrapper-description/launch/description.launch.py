from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os


def launch_setup(context, *args, **kwargs):
    description_pkg = get_package_share_directory('firefly-ros2-wrapper-description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'firefly_stereo_description.urdf.xacro')
    
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str
                )
            }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ]
    
    # Add RViz if enabled
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    if use_rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(description_pkg, 'rviz', 'view.rviz')],
            output='screen'
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 for visualization'),
        OpaqueFunction(function=launch_setup)
    ])