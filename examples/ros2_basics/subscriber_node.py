#!/usr/bin/env python3

"""
Basic subscriber node example for ROS 2

This node demonstrates the fundamental concepts of ROS 2 by subscribing
to a topic and printing received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that subscribes to messages from a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription to String messages on the 'topic' topic
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Make sure the subscription is properly created
        self.subscription  # prevent unused variable warning

        # Log that the subscriber has started
        self.get_logger().info('Minimal subscriber node has started')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.

        Args:
            msg: The received message of type std_msgs.msg.String
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up and destroy the node
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()