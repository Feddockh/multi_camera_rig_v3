#!/usr/bin/env python3
"""
Fake trigger node for simulation.
Controls image flow by relaying topics when video is "recording".

ros2 service call /trigger/test_connection std_srvs/srv/Trigger
ros2 service call /trigger/send_trigger std_srvs/srv/Trigger
ros2 service call /trigger/start_video std_srvs/srv/Trigger
ros2 service call /trigger/stop_video std_srvs/srv/Trigger
ros2 service call /trigger/set_flash_duration std_srvs/srv/Trigger "{ }"
ros2 service call /trigger/set_frame_rate std_srvs/srv/Trigger "{ }"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class TriggerNodeFake(Node):
    """Simulated trigger node that gates image topics."""
    
    def __init__(self):
        super().__init__('trigger_node')
        
        # Declare parameters
        self.declare_parameter(
            'flash_duration_ms', 100,
            ParameterDescriptor(description='Flash duration in milliseconds (0-300)',
                              type=ParameterType.PARAMETER_INTEGER)
        )
        self.declare_parameter(
            'frame_rate_hz', 10,
            ParameterDescriptor(description='Trigger frame rate in Hz (1-20)',
                              type=ParameterType.PARAMETER_INTEGER)
        )
        self.declare_parameter(
            'auto_connect', True,
            ParameterDescriptor(description='Automatically test connection on startup',
                              type=ParameterType.PARAMETER_BOOL
            )
        )
        self.declare_parameter(
            'auto_start', False,
            ParameterDescriptor(description='Automatically start video triggering on startup',
                              type=ParameterType.PARAMETER_BOOL)
        )
        
        # Camera topics to relay - specify full topic names
        self.declare_parameter(
            'image_sub_topics', 
            ['cam0/image_raw', 'cam1/image_raw', 'cam2/image_raw', 'cam3/image_raw'],
            ParameterDescriptor(description='List of image topics to subscribe to',
                              type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'image_pub_topics',
            ['cam0/image_raw/triggered', 'cam1/image_raw/triggered', 
             'cam2/image_raw/triggered', 'cam3/image_raw/triggered'],
            ParameterDescriptor(description='List of image topics to publish to (triggered)',
                              type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'info_sub_topics',
            ['cam0/camera_info', 'cam1/camera_info', 'cam2/camera_info', 'cam3/camera_info'],
            ParameterDescriptor(description='List of camera info topics to subscribe to',
                              type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'info_pub_topics',
            ['cam0/camera_info/triggered', 'cam1/camera_info/triggered',
             'cam2/camera_info/triggered', 'cam3/camera_info/triggered'],
            ParameterDescriptor(description='List of camera info topics to publish to (triggered)',
                              type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        
        # Get parameters
        self.flash_duration = self.get_parameter('flash_duration_ms').value
        self.frame_rate = self.get_parameter('frame_rate_hz').value
        auto_connect = self.get_parameter('auto_connect').value
        auto_start = self.get_parameter('auto_start').value
        image_sub_topics = self.get_parameter('image_sub_topics').value
        image_pub_topics = self.get_parameter('image_pub_topics').value
        info_sub_topics = self.get_parameter('info_sub_topics').value
        info_pub_topics = self.get_parameter('info_pub_topics').value
        
        # Validate that all topic lists have the same length
        num_cameras = len(image_sub_topics)
        if not (len(image_pub_topics) == num_cameras and 
                len(info_sub_topics) == num_cameras and 
                len(info_pub_topics) == num_cameras):
            raise ValueError(
                f"All topic lists must have the same length. Got: "
                f"image_sub={len(image_sub_topics)}, image_pub={len(image_pub_topics)}, "
                f"info_sub={len(info_sub_topics)}, info_pub={len(info_pub_topics)}"
            )
        
        # State tracking
        self.video_running = False
        self.single_trigger_pending = [False] * num_cameras  # Track single trigger per camera
        
        # QoS profile for image topics
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create relay subscriptions and publishers for each camera
        self.relays = []
        for i in range(num_cameras):
            relay = {
                'image_sub': self.create_subscription(
                    Image,
                    image_sub_topics[i],
                    lambda msg, idx=i: self.image_callback(msg, idx),
                    image_qos
                ),
                'image_pub': self.create_publisher(
                    Image,
                    image_pub_topics[i],
                    image_qos
                ),
                'info_sub': self.create_subscription(
                    CameraInfo,
                    info_sub_topics[i],
                    lambda msg, idx=i: self.info_callback(msg, idx),
                    image_qos
                ),
                'info_pub': self.create_publisher(
                    CameraInfo,
                    info_pub_topics[i],
                    image_qos
                ),
                'image_sub_topic': image_sub_topics[i],
                'image_pub_topic': image_pub_topics[i]
            }
            self.relays.append(relay)
        
        # Create services
        self.srv_test_connection = self.create_service(
            Trigger, 'trigger/test_connection', self.test_connection_callback)
        self.srv_send_trigger = self.create_service(
            Trigger, 'trigger/send_trigger', self.send_trigger_callback)
        self.srv_start_video = self.create_service(
            Trigger, 'trigger/start_video', self.start_video_callback)
        self.srv_stop_video = self.create_service(
            Trigger, 'trigger/stop_video', self.stop_video_callback)
        self.srv_set_flash_duration = self.create_service(
            Trigger, 'trigger/set_flash_duration', self.set_flash_duration_callback)
        self.srv_set_frame_rate = self.create_service(
            Trigger, 'trigger/set_frame_rate', self.set_frame_rate_callback)
        
        # Status publisher
        self.status_publisher = self.create_publisher(String, 'trigger/status', 10)
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        if auto_connect:
            self.test_connection()
        
        self.get_logger().info(f'Fake trigger node initialized - relaying {num_cameras} cameras')
        for relay in self.relays:
            self.get_logger().info(f"  {relay['image_sub_topic']} -> {relay['image_pub_topic']}")
        
        # Auto-start if requested
        if auto_start:
            self.video_running = True
            self.publish_status(f"Auto-started video at {self.frame_rate} Hz (simulated)")
    
    def image_callback(self, msg: Image, camera_idx: int):
        """Relay image only if video is running or single trigger pending."""
        if self.video_running:
            self.relays[camera_idx]['image_pub'].publish(msg)
        elif self.single_trigger_pending[camera_idx]:
            self.relays[camera_idx]['image_pub'].publish(msg)
            self.single_trigger_pending[camera_idx] = False
            self.get_logger().debug(f"Single trigger image relayed for camera {camera_idx}")
    
    def info_callback(self, msg: CameraInfo, camera_idx: int):
        """Relay camera info only if video is running or single trigger pending."""
        if self.video_running:
            self.relays[camera_idx]['info_pub'].publish(msg)
        elif self.single_trigger_pending[camera_idx]:
            self.relays[camera_idx]['info_pub'].publish(msg)
            # Don't clear flag here - let image callback clear it
            self.get_logger().debug(f"Single trigger camera_info relayed for camera {camera_idx}")
    
    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)

    def test_connection(self):
        """Simulate testing connection."""
        self.publish_status("Fake trigger connection OK (simulation)")
    
    def test_connection_callback(self, request, response):
        """Service callback for testing connection."""
        response.success = True
        response.message = "Fake trigger connection OK (simulation)"
        self.publish_status("Connection test passed (simulated)")
        return response
    
    def send_trigger_callback(self, request, response):
        """Service callback for sending a single trigger."""
        if self.video_running:
            response.success = False
            response.message = "Cannot send single trigger while video recording is active"
            return response
        
        # Enable single trigger for all cameras
        for i in range(len(self.relays)):
            self.single_trigger_pending[i] = True
        
        response.success = True
        response.message = f"Single trigger armed for {len(self.relays)} cameras (will relay next image)"
        self.publish_status(f"Single trigger armed for {len(self.relays)} cameras")
        return response
    
    def start_video_callback(self, request, response):
        """Service callback for starting video recording."""
        if self.video_running:
            response.success = False
            response.message = "Video recording already active"
            return response
        
        self.video_running = True
        response.success = True
        response.message = f"Video started at {self.frame_rate} Hz (simulated)"
        self.publish_status(f"Video started - now relaying images to /triggered topics")
        return response
    
    def stop_video_callback(self, request, response):
        """Service callback for stopping video recording."""
        if not self.video_running:
            response.success = False
            response.message = "Video recording not active"
            return response
        
        self.video_running = False
        response.success = True
        response.message = "Video stopped (simulated)"
        self.publish_status("Video stopped - images no longer relayed")
        return response
    
    def set_flash_duration_callback(self, request, response):
        """Service callback for setting flash duration."""
        duration = self.flash_duration
        if not 0 <= duration <= 300:
            response.success = False
            response.message = f"Flash duration {duration} out of range (0-300 ms)"
            return response
        
        response.success = True
        response.message = f"Flash duration set to {duration} ms (simulated)"
        self.publish_status(f"Flash duration set to {duration} ms (simulated)")
        return response
    
    def set_frame_rate_callback(self, request, response):
        """Service callback for setting frame rate."""
        rate = self.frame_rate
        if not 1 <= rate <= 20:
            response.success = False
            response.message = f"Frame rate {rate} out of range (1-20 Hz)"
            return response
        
        response.success = True
        response.message = f"Frame rate set to {rate} Hz (simulated)"
        self.publish_status(f"Frame rate set to {rate} Hz (simulated)")
        return response
    
    def parameters_callback(self, params):
        """Callback for parameter changes."""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'flash_duration_ms':
                if 0 <= param.value <= 300:
                    self.flash_duration = param.value
                    self.get_logger().info(f"Flash duration parameter updated to {param.value} ms")
                else:
                    result.successful = False
                    result.reason = f"Flash duration {param.value} out of range (0-300 ms)"
            elif param.name == 'frame_rate_hz':
                if 1 <= param.value <= 20:
                    self.frame_rate = param.value
                    self.get_logger().info(f"Frame rate parameter updated to {param.value} Hz")
                else:
                    result.successful = False
                    result.reason = f"Frame rate {param.value} out of range (1-20 Hz)"
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TriggerNodeFake()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()