#!/usr/bin/env python3

"""
Service client example for ROS 2

This node implements a service client that calls the add_two_ints service.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):
    """
    A minimal service client that calls the add_two_ints service.
    """

    def __init__(self):
        super().__init__('minimal_client_async')
        # Create a client for the AddTwoInts service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client initialized and connected to service')

    def send_request(self, a, b):
        """
        Send a request to the service server.

        Args:
            a: First integer to add
            b: Second integer to add

        Returns:
            A future object that will contain the response when ready
        """
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(request)
        return self.future


def main(args=None):
    """
    Main function that initializes the client, sends a request, and waits for response.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the service client node
    minimal_client = MinimalClientAsync()

    # Check if command line arguments are provided
    if len(sys.argv) != 3:
        print('Usage: python3 service_client.py <int1> <int2>')
        print('Example: python3 service_client.py 1 2')
        sys.exit(1)

    # Parse the command line arguments
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Please provide two integers as arguments')
        sys.exit(1)

    # Send the request
    future = minimal_client.send_request(a, b)

    try:
        # Keep spinning until the response is received
        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if future.done():
                try:
                    response = future.result()
                    minimal_client.get_logger().info(
                        f'Result of {a} + {b} = {response.sum}')
                except Exception as e:
                    minimal_client.get_logger().error(f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        minimal_client.get_logger().info('Service client shutting down...')
    finally:
        # Clean up and destroy the node
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()