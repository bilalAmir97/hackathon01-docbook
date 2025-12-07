#!/usr/bin/env python3

"""
Advanced publisher/subscriber example for ROS 2

This example demonstrates advanced pub/sub patterns including custom messages,
QoS policies, and performance considerations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import random
import time


class AdvancedPublisher(Node):
    """
    Advanced publisher demonstrating different QoS configurations and message types.
    """

    def __init__(self):
        super().__init__('advanced_publisher')

        # Different QoS profiles for different use cases
        # Reliable, keep last 10 messages
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Best effort for sensor-like data
        self.best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Transient local for configuration data
        self.transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers with different QoS
        self.reliable_pub = self.create_publisher(String, 'reliable_topic', self.reliable_qos)
        self.best_effort_pub = self.create_publisher(String, 'best_effort_topic', self.best_effort_qos)
        self.config_pub = self.create_publisher(String, 'config_topic', self.transient_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.best_effort_qos)

        # Timer for publishing
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Advanced publisher initialized with multiple QoS profiles')

    def timer_callback(self):
        """Publish messages with different QoS profiles."""
        # Publish reliable message
        reliable_msg = String()
        reliable_msg.data = f'Reliable message {self.counter}'
        self.reliable_pub.publish(reliable_msg)

        # Publish best-effort message
        best_effort_msg = String()
        best_effort_msg.data = f'Best effort message {self.counter}'
        self.best_effort_pub.publish(best_effort_msg)

        # Publish configuration message periodically
        if self.counter % 10 == 0:
            config_msg = String()
            config_msg.data = f'Configuration update {self.counter // 10}'
            self.config_pub.publish(config_msg)

        # Publish velocity command
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5 + random.uniform(-0.1, 0.1)  # Forward with slight variation
        cmd_vel_msg.angular.z = random.uniform(-0.2, 0.2)  # Random angular velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

        self.counter += 1


class AdvancedSubscriber(Node):
    """
    Advanced subscriber demonstrating different QoS configurations.
    """

    def __init__(self):
        super().__init__('advanced_subscriber')

        # Different QoS profiles for different subscriptions
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions with matching QoS profiles
        self.reliable_sub = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            self.reliable_qos
        )

        self.best_effort_sub = self.create_subscription(
            String,
            'best_effort_topic',
            self.best_effort_callback,
            self.best_effort_qos
        )

        self.config_sub = self.create_subscription(
            String,
            'config_topic',
            self.config_callback,
            self.transient_qos
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            self.best_effort_qos
        )

        self.get_logger().info('Advanced subscriber initialized with multiple QoS profiles')

    def reliable_callback(self, msg):
        """Handle reliable messages."""
        self.get_logger().info(f'Received reliable message: {msg.data}')

    def best_effort_callback(self, msg):
        """Handle best-effort messages."""
        # Don't log every message to avoid spam
        if random.random() < 0.1:  # Log about 10% of messages
            self.get_logger().info(f'Received best-effort message: {msg.data}')

    def config_callback(self, msg):
        """Handle configuration messages."""
        self.get_logger().info(f'Received configuration: {msg.data}')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands."""
        self.get_logger().info(
            f'Received velocity command: linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f}'
        )


def main(args=None):
    """
    Main function demonstrating both publisher and subscriber in the same process.
    In practice, these would typically run in separate nodes/processes.
    """
    rclpy.init(args=args)

    # Create both publisher and subscriber nodes
    publisher = AdvancedPublisher()
    subscriber = AdvancedSubscriber()

    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    try:
        # Spin both nodes
        executor.spin()
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down nodes...')
    finally:
        # Clean up
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()