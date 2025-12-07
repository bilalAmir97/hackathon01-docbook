#!/usr/bin/env python3

"""
Service server example for ROS 2

This node implements a service server that provides a simple calculation service.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    """
    A minimal service server that adds two integers.
    """

    def __init__(self):
        super().__init__('minimal_service')
        # Create a service that takes AddTwoInts messages
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server started, waiting for requests...')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that processes service requests.

        Args:
            request: The service request containing two integers (a and b)
            response: The service response that will contain the sum

        Returns:
            The response with the calculated sum
        """
        # Perform the calculation
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')

        # Return the response
        return response


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the service server node
    minimal_service = MinimalService()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_service.get_logger().info('Service server shutting down...')
    finally:
        # Clean up and destroy the node
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()