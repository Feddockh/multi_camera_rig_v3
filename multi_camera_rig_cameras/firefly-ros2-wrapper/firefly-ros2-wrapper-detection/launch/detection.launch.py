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

    model_dir = LaunchConfiguration('model_dir').perform(context)
    model = LaunchConfiguration('tensorrt_file').perform(context)
    engine_path = os.path.join(model_dir, model)

    yolo_node = Node(
        package='firefly-ros2-wrapper-detection',
        executable='yolov8_trt_node',
        name='yolov8_trt_node',
        output='screen',
        parameters=[{
            # Core
            'engine_path': engine_path,
            'image_topic': LaunchConfiguration('yolo_image_topic'),
            'detections_topic': LaunchConfiguration('yolo_detections_topic'),

            # Tensors
            'input_tensor': 'images',
            'output_tensor': 'output0',

            # Model input (engine expects 1088x1440)
            'input_width': LaunchConfiguration('yolo_input_width'),
            'input_height': LaunchConfiguration('yolo_input_height'),
            'stride': 32,
            'scaleup': True,

            # Postprocess
            'conf_thresh': LaunchConfiguration('yolo_conf_thresh'),
            'iou_thresh': LaunchConfiguration('yolo_iou_thresh'),
            'max_det': LaunchConfiguration('yolo_max_det'),

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
            'debug': LaunchConfiguration('debug'),
        }],
        # Optional: if you want topic remaps rather than params
        # remappings=[
        #     ('/image', LaunchConfiguration('yolo_image_topic')),
        #     ('/detections', LaunchConfiguration('yolo_detections_topic')),
        # ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return [yolo_node]

def generate_launch_description():
    return LaunchDescription([
        # Model files
        DeclareLaunchArgument(
            'model_dir',
            default_value=PJoin([FindPackageShare('firefly-ros2-wrapper-detection'), 'models']),
            description='Directory containing TensorRT engine (.plan) files',
        ),
        DeclareLaunchArgument('tensorrt_file', default_value='best_sim.plan'),
        # Topic names
        DeclareLaunchArgument('yolo_image_topic', default_value='/firefly_left/image_raw'),
        DeclareLaunchArgument('yolo_detections_topic', default_value='/detections'),

        # YOLO input size (matches your export / engine)
        DeclareLaunchArgument('yolo_input_width', default_value='1440'),
        DeclareLaunchArgument('yolo_input_height', default_value='1088'),

        # Postprocess
        DeclareLaunchArgument('yolo_conf_thresh', default_value='0.25'),
        DeclareLaunchArgument('yolo_iou_thresh', default_value='0.45'),
        DeclareLaunchArgument('yolo_max_det', default_value='300'),

        DeclareLaunchArgument('debug', default_value='false'),

        OpaqueFunction(function=launch_setup)
    ])