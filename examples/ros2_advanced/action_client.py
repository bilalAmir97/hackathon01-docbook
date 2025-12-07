#!/usr/bin/env python3
"""
Action Client Example for Robot Navigation

This example demonstrates how to create a ROS 2 action client that sends
navigation goals to an action server and handles feedback and results.
"""

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64


class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('navigation_action_client')

        # In a real implementation, you would connect to a proper action server
        # For this example, we'll simulate the interaction
        self.get_logger().info('Navigation action client initialized')

        # Subscriber for progress updates (simulating action feedback)
        self.feedback_subscription = self.create_subscription(
            Float64,
            'navigation_progress',
            self.feedback_callback,
            10
        )

        self.current_progress = 0.0
        self.goal_active = False

        self.get_logger().info('Navigation action client ready')

    def feedback_callback(self, msg):
        """Handle feedback from the action server"""
        self.current_progress = msg.data
        self.get_logger().info(f'Received progress update: {self.current_progress * 100:.1f}%')

    def send_navigation_goal(self, target_x, target_y):
        """Send a navigation goal to the action server"""
        self.get_logger().info(f'Sending navigation goal to ({target_x}, {target_y})')

        # In a real implementation, you would:
        # 1. Create a goal message
        # 2. Send it to the action server
        # 3. Handle the result
        # For this example, we'll simulate the process

        self.goal_active = True
        self.current_progress = 0.0

        # Wait for the simulated action to complete
        # In a real implementation, you would use:
        # goal_handle = self._action_client.send_goal_async(
        #     goal_msg,
        #     feedback_callback=self.feedback_callback
        # )

        # Simulate waiting for result
        for i in range(10):
            if self.current_progress >= 1.0:
                break
            time.sleep(0.5)

        self.goal_active = False
        self.get_logger().info(f'Navigation to ({target_x}, {target_y}) completed')

        return True

    def send_simple_goal(self):
        """Send a simple navigation goal as an example"""
        return self.send_navigation_goal(2.5, 3.0)


def main(args=None):
    rclpy.init(args=args)
    client = NavigationActionClient()

    try:
        # Send a navigation goal
        client.send_simple_goal()

        # Keep spinning to receive feedback
        timeout = time.time() + 10.0  # 10 second timeout
        while rclpy.ok() and time.time() < timeout:
            rclpy.spin_once(client, timeout_sec=0.1)

    except KeyboardInterrupt:
        client.get_logger().info('Action client interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()