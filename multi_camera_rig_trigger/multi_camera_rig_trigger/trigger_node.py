#!/usr/bin/env python3
"""
ROS 2 node for hardware trigger controller.
Provides services and parameters for controlling camera triggers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from .hardware_interface import TriggerHardwareInterface


class TriggerNode(Node):
    """ROS 2 node for hardware trigger control."""
    
    @staticmethod
    def escape_control_chars(message: str) -> str:
        """
        Escape control characters in message for display.
        Converts newlines, carriage returns, etc. to their string representations.
        
        Args:
            message: The message string to escape
            
        Returns:
            Message with control characters escaped
        """
        return message.replace('\r', '\\r').replace('\n', '\\n').replace('\t', '\\t')
    
    def __init__(self):
        super().__init__('trigger_node')
        
        # Declare parameters
        self.declare_parameter(
            'serial_port',
            '/dev/ttyUSB0',
            ParameterDescriptor(
                description='Serial port for trigger hardware',
                type=ParameterType.PARAMETER_STRING
            )
        )
        
        self.declare_parameter(
            'baudrate',
            9600,
            ParameterDescriptor(
                description='Serial baudrate',
                type=ParameterType.PARAMETER_INTEGER
            )
        )
        
        self.declare_parameter(
            'flash_duration_ms',
            100,
            ParameterDescriptor(
                description='Flash duration in milliseconds (0-300)',
                type=ParameterType.PARAMETER_INTEGER
            )
        )
        
        self.declare_parameter(
            'frame_rate_hz',
            10,
            ParameterDescriptor(
                description='Trigger frame rate in Hz (1-20)',
                type=ParameterType.PARAMETER_INTEGER
            )
        )
        
        self.declare_parameter(
            'auto_connect',
            True,
            ParameterDescriptor(
                description='Automatically test connection on startup',
                type=ParameterType.PARAMETER_BOOL
            )
        )
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.flash_duration = self.get_parameter('flash_duration_ms').value
        self.frame_rate = self.get_parameter('frame_rate_hz').value
        auto_connect = self.get_parameter('auto_connect').value
        
        # Initialize hardware interface
        self.hardware = TriggerHardwareInterface(port=port, baudrate=baudrate)
        
        # Create services
        self.srv_test_connection = self.create_service(
            Trigger,
            'trigger/test_connection',
            self.test_connection_callback
        )
        
        self.srv_send_trigger = self.create_service(
            Trigger,
            'trigger/send_trigger',
            self.send_trigger_callback
        )
        
        self.srv_start_video = self.create_service(
            Trigger,
            'trigger/start_video',
            self.start_video_callback
        )
        
        self.srv_stop_video = self.create_service(
            Trigger,
            'trigger/stop_video',
            self.stop_video_callback
        )
        
        self.srv_set_flash_duration = self.create_service(
            Trigger,
            'trigger/set_flash_duration',
            self.set_flash_duration_callback
        )
        
        self.srv_set_frame_rate = self.create_service(
            Trigger,
            'trigger/set_frame_rate',
            self.set_frame_rate_callback
        )
        
        # Publisher for status messages
        self.status_publisher = self.create_publisher(
            String,
            'trigger/status',
            10
        )
        
        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'Trigger node initialized on port {port}')
        
        # Test connection on startup if requested
        if auto_connect:
            self.test_connection()
    
    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)
    
    def test_connection(self):
        """Test connection to trigger hardware."""
        success, message = self.hardware.test_connection()
        escaped_msg = self.escape_control_chars(message)
        if success:
            self.publish_status(f"Connection test passed: '{escaped_msg}'")
        else:
            self.publish_status(f"Connection test failed: '{escaped_msg}'")

    def test_connection_callback(self, request, response):
        """Service callback for testing connection."""
        success, message = self.hardware.test_connection()
        response.success = success
        response.message = message
        return response
    
    def send_trigger_callback(self, request, response):
        """Service callback for sending a single trigger."""
        # Check if video recording is active
        if self.hardware.video_running:
            response.success = False
            response.message = "Cannot send single trigger while video recording is active"
            return response
        
        # Send trigger
        success, message = self.hardware.send_trigger()
        response.success = success
        response.message = message
        
        escaped_msg = self.escape_control_chars(message)
        if success:
            self.publish_status(f"Single trigger success: '{escaped_msg}'")
        else:
            self.publish_status(f"Single trigger failed: '{escaped_msg}'")

        return response
    
    def start_video_callback(self, request, response):
        """Service callback for starting video recording."""
        if self.hardware.video_running:
            response.success = False
            response.message = "Video recording already active"
            return response
        
        success, message = self.hardware.start_video()
        response.success = success
        response.message = message
        
        if success:
            self.publish_status(f"Video recording started at {self.frame_rate} Hz and flash duration {self.flash_duration} ms")
        else:
            self.publish_status(f"Failed to start video recording: '{self.escape_control_chars(message)}'")
        
        return response
    
    def stop_video_callback(self, request, response):
        """Service callback for stopping video recording."""
        if not self.hardware.video_running:
            response.success = False
            response.message = "Video recording not active"
            return response
        
        success, message = self.hardware.stop_video()
        response.success = success
        response.message = message
        
        if success:
            self.publish_status("Video recording stopped")
        else:
            self.publish_status(f"Failed to stop video recording: '{self.escape_control_chars(message)}'")

        return response
    
    def set_flash_duration_callback(self, request, response):
        """Service callback for setting flash duration."""
        duration = self.flash_duration
        success, message = self.hardware.set_flash_duration(duration)
        response.success = success
        response.message = message
        
        if success:
            self.publish_status(f"Flash duration set to {duration} ms")
        
        return response
    
    def set_frame_rate_callback(self, request, response):
        """Service callback for setting frame rate."""
        rate = self.frame_rate
        success, message = self.hardware.set_frame_rate(rate)
        response.success = success
        response.message = message
        
        if success:
            self.publish_status(f"Frame rate set to {rate} Hz")
        
        return response
    
    def parameters_callback(self, params):
        """
        Callback for parameter changes.
        This only updates the internal variables; actual hardware changes are done via services.
        """
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
    
    def shutdown(self):
        """Cleanup on shutdown."""
        if self.hardware.video_running:
            self.hardware.stop_video()
        self.hardware.close_connection()


def main(args=None):
    rclpy.init(args=args)
    
    node = TriggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
