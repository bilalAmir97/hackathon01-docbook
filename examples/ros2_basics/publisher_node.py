#!/usr/bin/env python3

"""
Basic publisher node example for ROS 2

This node demonstrates the fundamental concepts of ROS 2 by publishing
a simple string message to a topic at a regular interval.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that publishes messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher for String messages on the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter to include in the message
        self.i = 0

        # Log that the publisher has started
        self.get_logger().info('Minimal publisher node has started')

    def timer_callback(self):
        """
        Callback function that is called by the timer at regular intervals.
        """
        # Create a message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up and destroy the node
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()