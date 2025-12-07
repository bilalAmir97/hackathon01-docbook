#!/usr/bin/env python3
"""
Parameter Management Example

This example demonstrates how to use ROS 2 parameters for configuration management,
including declaration, retrieval, updating, and validation.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterDescriptor
from rclpy.node import ParameterType, SetParametersResult
from rclpy.qos import qos_profile_system_default


class ParameterManagementNode(Node):
    def __init__(self):
        super().__init__('parameter_management_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'robot_name',
            'turtlebot4',
            ParameterDescriptor(
                description='Name of the robot for identification'
            )
        )

        self.declare_parameter(
            'max_velocity',
            0.5,
            ParameterDescriptor(
                description='Maximum linear velocity in m/s',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        self.declare_parameter(
            'enable_logging',
            True,
            ParameterDescriptor(
                description='Enable or disable detailed logging'
            )
        )

        self.declare_parameter(
            'sensors_enabled',
            [True, True, False, True],  # Example: [lidar, camera, imu, gps]
            ParameterDescriptor(
                description='Array indicating which sensors are enabled'
            )
        )

        # Retrieve parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.enable_logging = self.get_parameter('enable_logging').value
        self.sensors_enabled = self.get_parameter('sensors_enabled').value

        self.get_logger().info(f'Node initialized with robot: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'Logging enabled: {self.enable_logging}')

        # Set up parameter callback for validation and response to changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a timer to periodically check for parameter changes
        self.timer = self.create_timer(2.0, self.timer_callback)

    def parameter_callback(self, params):
        """
        Callback function to validate and respond to parameter changes
        """
        successful = True
        reason = ''

        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                # Validate velocity constraints
                if param.value <= 0 or param.value > 5.0:
                    successful = False
                    reason = f'Max velocity must be between 0 and 5.0, got {param.value}'
                    self.get_logger().error(reason)
                    break
                else:
                    self.get_logger().info(f'Max velocity updated to: {param.value} m/s')
            elif param.name == 'robot_name' and param.type_ == Parameter.Type.STRING:
                if not param.value or len(param.value) < 2:
                    successful = False
                    reason = 'Robot name must be at least 2 characters'
                    self.get_logger().error(reason)
                    break
                else:
                    self.get_logger().info(f'Robot name updated to: {param.value}')

        return SetParametersResult(successful=successful, reason=reason)

    def timer_callback(self):
        """Periodically check for parameter changes and log current values"""
        # Get current parameter values (in case they changed)
        current_velocity = self.get_parameter('max_velocity').value
        current_logging = self.get_parameter('enable_logging').value
        current_sensors = self.get_parameter('sensors_enabled').value

        if self.enable_logging:
            self.get_logger().info(
                f'Current max velocity: {current_velocity} m/s, '
                f'Logging: {current_logging}, Sensors: {current_sensors}'
            )

        # Update internal values
        self.max_velocity = current_velocity
        self.enable_logging = current_logging
        self.sensors_enabled = current_sensors

    def get_robot_config(self):
        """Return current robot configuration as a dictionary"""
        return {
            'robot_name': self.get_parameter('robot_name').value,
            'max_velocity': self.get_parameter('max_velocity').value,
            'enable_logging': self.get_parameter('enable_logging').value,
            'sensors_enabled': self.get_parameter('sensors_enabled').value
        }

    def update_velocity_safely(self, new_velocity):
        """Safely update velocity with validation"""
        if 0 < new_velocity <= 5.0:
            self.set_parameters([Parameter('max_velocity', Parameter.Type.DOUBLE, new_velocity)])
            self.get_logger().info(f'Velocity safely updated to: {new_velocity} m/s')
            return True
        else:
            self.get_logger().warn(f'Invalid velocity value: {new_velocity}. Must be 0 < value <= 5.0')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ParameterManagementNode()

    try:
        # Print initial configuration
        config = node.get_robot_config()
        node.get_logger().info(f'Initial robot configuration: {config}')

        # Example of updating a parameter programmatically
        node.update_velocity_safely(1.0)

        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parameter management node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()