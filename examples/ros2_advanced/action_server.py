#!/usr/bin/env python3
"""
Action Server Example for Robot Navigation

This example demonstrates how to create a ROS 2 action server that handles
navigation goals with feedback and result reporting.
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
import threading

# For this example, we'll create a simple custom action-like interface
# since we're focusing on the concept rather than specific action types
class NavigationGoal:
    def __init__(self, x=0.0, y=0.0):
        self.target_x = x
        self.target_y = y

class NavigationResult:
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

class NavigationFeedback:
    def __init__(self, progress=0.0, distance_remaining=0.0):
        self.progress = progress
        self.distance_remaining = distance_remaining


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # In a real implementation, you would use a proper action interface
        # For this example, we'll simulate the action behavior
        self.get_logger().info('Navigation action server initialized')

        # Publisher for progress updates (simulating action feedback)
        self.feedback_publisher = self.create_publisher(Float64, 'navigation_progress', 10)

        # Track active goals
        self.active_goals = {}

        self.get_logger().info('Navigation action server ready - use with companion client')

    def execute_navigation_goal(self, goal_x, goal_y):
        """Execute a navigation goal with simulated progress"""
        self.get_logger().info(f'Executing navigation to ({goal_x}, {goal_y})')

        # Simulate navigation with progress updates
        result = NavigationResult()

        for step in range(1, 11):  # 10 progress updates
            # Simulate cancellation check (in a real action server, this would be part of the goal handle)
            progress = step / 10.0
            distance_remaining = (1.0 - progress) * 5.0  # Simulated distance

            # Publish progress feedback
            feedback_msg = Float64()
            feedback_msg.data = progress
            self.feedback_publisher.publish(feedback_msg)

            self.get_logger().info(f'Navigation progress: {progress * 100:.0f}%')

            # Simulate navigation time
            time.sleep(0.3)

        result.success = True
        result.message = f'Navigation to ({goal_x}, {goal_y}) completed successfully'
        self.get_logger().info(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()

    try:
        # This is a simplified example - in a real implementation you would have:
        # self._action_server = ActionServer(
        #     self,
        #     NavigateToPose,  # Action interface type
        #     'navigate_to_pose',  # Action name
        #     self.execute_callback  # Execution callback
        # )

        node.get_logger().info('Action server running (simulated). Use companion client to test.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action server shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()