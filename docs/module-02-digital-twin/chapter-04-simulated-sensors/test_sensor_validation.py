#!/usr/bin/env python3

"""
Test script for validating sensor data generation in Gazebo Garden
This script connects to ROS 2 topics and validates the sensor data from
LiDAR, depth camera, and IMU sensors on the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from std_msgs.msg import Header
import numpy as np
import time
from datetime import datetime

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Initialize validation flags
        self.lidar_received = False
        self.depth_received = False
        self.rgb_received = False
        self.imu_received = False

        # Initialize validation statistics
        self.lidar_stats = {'count': 0, 'min_range': float('inf'), 'max_range': 0}
        self.imu_stats = {'count': 0, 'orientation_valid': True}
        self.depth_stats = {'count': 0, 'min_depth': float('inf'), 'max_depth': 0}

        # Create subscribers for sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )

        self.rgb_subscription = self.create_subscription(
            Image,
            '/humanoid_robot/camera/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/humanoid_robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # Timer for periodic validation
        self.timer = self.create_timer(5.0, self.validate_sensors)

        self.get_logger().info('Sensor Validator Node Started')

    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        self.lidar_received = True
        self.lidar_stats['count'] += 1

        # Validate ranges
        valid_ranges = [r for r in msg.ranges if not np.isnan(r) and r > 0]
        if valid_ranges:
            self.lidar_stats['min_range'] = min(self.lidar_stats['min_range'], min(valid_ranges))
            self.lidar_stats['max_range'] = max(self.lidar_stats['max_range'], max(valid_ranges))

        self.get_logger().debug(f'LiDAR: {len(msg.ranges)} points, ranges: {msg.range_min:.2f}-{msg.range_max:.2f}m')

    def rgb_callback(self, msg):
        """Process RGB image data"""
        self.rgb_received = True

        # Calculate expected image size
        expected_size = msg.width * msg.height * 3  # 3 channels for RGB

        if len(msg.data) == expected_size:
            self.get_logger().debug(f'RGB Image: {msg.width}x{msg.height}, encoding: {msg.encoding}')
        else:
            self.get_logger().warn(f'RGB Image size mismatch: expected {expected_size}, got {len(msg.data)}')

    def depth_callback(self, msg):
        """Process depth image data"""
        self.depth_received = True
        self.depth_stats['count'] += 1

        # Convert byte data to float values (assuming 32FC1 encoding)
        if msg.encoding == '32FC1':
            # Calculate expected size for float32 data
            expected_size = msg.width * msg.height * 4  # 4 bytes per float32

            if len(msg.data) == expected_size:
                # Extract depth values (simplified - in real implementation, you'd properly decode the float values)
                num_floats = len(msg.data) // 4
                depth_values = []

                for i in range(0, len(msg.data), 4):
                    # This is a simplified conversion - in practice, you'd properly decode the bytes to float
                    byte_val = msg.data[i:i+4]
                    # For validation, just check that we have proper data
                    pass

                self.get_logger().debug(f'Depth Image: {msg.width}x{msg.height}, encoding: {msg.encoding}')
            else:
                self.get_logger().warn(f'Depth Image size mismatch: expected {expected_size}, got {len(msg.data)}')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_received = True
        self.imu_stats['count'] += 1

        # Validate quaternion (should be normalized)
        quat_norm = np.sqrt(
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )

        if abs(quat_norm - 1.0) > 0.01:
            self.imu_stats['orientation_valid'] = False
            self.get_logger().warn(f'IMU quaternion not normalized: {quat_norm:.3f}')

        # Validate that angular velocity and linear acceleration are reasonable
        ang_vel_norm = np.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )

        lin_acc_norm = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        # Typical thresholds for humanoid robot (adjust as needed)
        if ang_vel_norm > 100:  # rad/s
            self.get_logger().warn(f'IMU angular velocity unusually high: {ang_vel_norm:.3f}')

        if lin_acc_norm > 100:  # m/s^2 (about 10g)
            self.get_logger().warn(f'IMU linear acceleration unusually high: {lin_acc_norm:.3f}')

        self.get_logger().debug(f'IMU: orientation valid={self.imu_stats["orientation_valid"]}, '
                               f'angular_vel={ang_vel_norm:.3f}, linear_acc={lin_acc_norm:.3f}')

    def validate_sensors(self):
        """Periodic validation of all sensors"""
        self.get_logger().info('--- Sensor Validation Report ---')
        self.get_logger().info(f'LiDAR: {"✓ Received" if self.lidar_received else "✗ No data"}')
        self.get_logger().info(f'RGB: {"✓ Received" if self.rgb_received else "✗ No data"}')
        self.get_logger().info(f'Depth: {"✓ Received" if self.depth_received else "✗ No data"}')
        self.get_logger().info(f'IMU: {"✓ Received" if self.imu_received else "✗ No data"}')

        if self.lidar_received:
            self.get_logger().info(f'  LiDAR Stats: {self.lidar_stats["count"]} messages, '
                                 f'range: {self.lidar_stats["min_range"]:.2f}-{self.lidar_stats["max_range"]:.2f}m')

        if self.imu_received:
            self.get_logger().info(f'  IMU Stats: {self.imu_stats["count"]} messages, '
                                 f'orientation valid: {self.imu_stats["orientation_valid"]}')

        if self.depth_received:
            self.get_logger().info(f'  Depth Stats: {self.depth_stats["count"]} messages')

        # Overall validation
        all_received = all([self.lidar_received, self.rgb_received, self.depth_received, self.imu_received])
        if all_received:
            self.get_logger().info('✓ All sensors are publishing data!')
        else:
            self.get_logger().info('✗ Some sensors are not publishing data')

        self.get_logger().info('--- End Validation Report ---')

        # Reset flags for next validation cycle
        self.lidar_received = False
        self.rgb_received = False
        self.depth_received = False
        self.imu_received = False

def main(args=None):
    rclpy.init(args=args)

    sensor_validator = SensorValidator()

    try:
        rclpy.spin(sensor_validator)
    except KeyboardInterrupt:
        sensor_validator.get_logger().info('Sensor Validator stopped by user')
    finally:
        sensor_validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()