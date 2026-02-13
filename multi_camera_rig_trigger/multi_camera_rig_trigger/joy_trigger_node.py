#!/usr/bin/env python3
"""
Joy Trigger Node

Maps joystick/gamepad buttons to trigger video start/stop commands.
Default mapping uses Xbox controller A button to toggle video triggering.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class XboxButton:
    """Xbox controller button indices (typical on Linux joy_node)"""
    A = 0
    B = 1
    X = 2
    Y = 3
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    CHANGE_VIEW = 6
    MENU = 7
    HOME = 8
    LEFT_STICK_CLICK = 9
    RIGHT_STICK_CLICK = 10


class XboxAxis:
    """Xbox controller axis indices (typical on Linux joy_node)"""
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 2  # usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    RIGHT_TRIGGER = 5  # usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
    D_PAD_X = 6
    D_PAD_Y = 7


class JoyTriggerNode(Node):
    """Node that maps joystick button presses to trigger service calls."""

    def __init__(self):
        super().__init__('joy_trigger_node')

        # Declare parameters
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('start_service', '/trigger/start_video')
        self.declare_parameter('stop_service', '/trigger/stop_video')
        self.declare_parameter('trigger_button', XboxButton.A)

        # Get parameters
        joy_topic = self.get_parameter('joy_topic').value
        start_service = self.get_parameter('start_service').value
        stop_service = self.get_parameter('stop_service').value
        self.trigger_button = self.get_parameter('trigger_button').value

        # State
        self.video_on = False

        # Create subscription
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )

        # Create service clients
        self.start_client = self.create_client(Trigger, start_service)
        self.stop_client = self.create_client(Trigger, stop_service)

        # Wait for services to be available
        self.get_logger().info(f'Waiting for services {start_service} and {stop_service}...')
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().info(f'Waiting for {start_service}...')
        
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().info(f'Waiting for {stop_service}...')

        self.get_logger().info(
            f'Joy Trigger Node started. Press button {self.trigger_button} to start/stop video.'
        )
        self.get_logger().info(f'Listening to: {joy_topic}')

    def joy_callback(self, msg: Joy):
        """Handle joystick messages."""
        # Check if button index is valid
        if len(msg.buttons) <= self.trigger_button:
            return

        button_state = msg.buttons[self.trigger_button]

        # Button pressed (rising edge)
        if button_state == 1 and not self.video_on:
            self.video_on = True
            self.get_logger().info(f'Button {self.trigger_button} pressed - starting video')
            self.call_service(self.start_client)

        # Button released (falling edge)
        elif button_state == 0 and self.video_on:
            self.video_on = False
            self.get_logger().info(f'Button {self.trigger_button} released - stopping video')
            self.call_service(self.stop_client)

    def call_service(self, client):
        """Call a trigger service asynchronously."""
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Handle service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call succeeded: {response.message}')
            else:
                self.get_logger().warn(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {str(e)}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = JoyTriggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception in joy_trigger_node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
