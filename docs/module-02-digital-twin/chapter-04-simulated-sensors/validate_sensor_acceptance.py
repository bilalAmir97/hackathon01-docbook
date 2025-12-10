#!/usr/bin/env python3

"""
Acceptance validation script for sensor simulation in Gazebo Garden
This script validates that the simulated sensors meet the acceptance criteria
for the humanoid robot digital twin project.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import Vector3
import numpy as np
import time
import sys
from datetime import datetime

class SensorAcceptanceValidator(Node):
    def __init__(self):
        super().__init__('sensor_acceptance_validator')

        # Configuration parameters
        self.declare_parameter('test_duration', 30)  # seconds
        self.declare_parameter('lidar_range_min', 0.1)  # meters
        self.declare_parameter('lidar_range_max', 10.0)  # meters
        self.declare_parameter('lidar_min_points', 100)  # minimum valid points per scan
        self.declare_parameter('imu_orientation_threshold', 0.01)  # normalized quaternion diff
        self.declare_parameter('image_min_width', 320)  # pixels
        self.declare_parameter('image_min_height', 240)  # pixels

        self.test_duration = self.get_parameter('test_duration').value
        self.lidar_range_min = self.get_parameter('lidar_range_min').value
        self.lidar_range_max = self.get_parameter('lidar_range_max').value
        self.lidar_min_points = self.get_parameter('lidar_min_points').value
        self.imu_orientation_threshold = self.get_parameter('imu_orientation_threshold').value
        self.image_min_width = self.get_parameter('image_min_width').value
        self.image_min_height = self.get_parameter('image_min_height').value

        # Test results
        self.test_results = {
            'lidar_range_validation': False,
            'lidar_point_count': False,
            'imu_orientation_validation': False,
            'imu_angular_velocity_validation': False,
            'imu_linear_acceleration_validation': False,
            'image_resolution_validation': False,
            'image_encoding_validation': False,
            'sensor_synchronization': False
        }

        # Data collection
        self.lidar_data_samples = []
        self.imu_data_samples = []
        self.image_data_samples = []
        self.test_start_time = time.time()

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
            self.image_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # Timer for periodic validation
        self.timer = self.create_timer(1.0, self.periodic_validation)

        self.get_logger().info('Sensor Acceptance Validator Started')
        self.get_logger().info(f'Running acceptance tests for {self.test_duration} seconds...')

    def lidar_callback(self, msg):
        """Process LiDAR scan data for acceptance testing"""
        current_time = time.time()
        self.lidar_data_samples.append({
            'timestamp': current_time,
            'msg': msg,
            'valid_ranges': [r for r in msg.ranges if not np.isnan(r) and r > 0]
        })

        # Keep only recent samples (last 5 seconds)
        self.lidar_data_samples = [s for s in self.lidar_data_samples
                                   if current_time - s['timestamp'] <= 5.0]

    def image_callback(self, msg):
        """Process image data for acceptance testing"""
        current_time = time.time()
        self.image_data_samples.append({
            'timestamp': current_time,
            'msg': msg
        })

        # Keep only recent samples (last 5 seconds)
        self.image_data_samples = [s for s in self.image_data_samples
                                   if current_time - s['timestamp'] <= 5.0]

    def imu_callback(self, msg):
        """Process IMU data for acceptance testing"""
        current_time = time.time()
        self.imu_data_samples.append({
            'timestamp': current_time,
            'msg': msg
        })

        # Keep only recent samples (last 5 seconds)
        self.imu_data_samples = [s for s in self.imu_data_samples
                                 if current_time - s['timestamp'] <= 5.0]

    def periodic_validation(self):
        """Perform periodic validation of sensor data"""
        current_time = time.time()

        # Check if test duration has been reached
        if current_time - self.test_start_time >= self.test_duration:
            self.run_final_validation()
            self.timer.cancel()
            return

        # Validate LiDAR data
        self.validate_lidar_data()

        # Validate IMU data
        self.validate_imu_data()

        # Validate image data
        self.validate_image_data()

    def validate_lidar_data(self):
        """Validate LiDAR sensor data against acceptance criteria"""
        if not self.lidar_data_samples:
            return

        latest_sample = self.lidar_data_samples[-1]
        msg = latest_sample['msg']
        valid_ranges = latest_sample['valid_ranges']

        # Check range parameters
        if (msg.range_min >= self.lidar_range_min * 0.9 and
            msg.range_max <= self.lidar_range_max * 1.1):
            self.test_results['lidar_range_validation'] = True
        else:
            self.test_results['lidar_range_validation'] = False
            self.get_logger().warn(f'LiDAR range validation failed: '
                                 f'min={msg.range_min}, max={msg.range_max}')

        # Check point count
        if len(valid_ranges) >= self.lidar_min_points:
            self.test_results['lidar_point_count'] = True
        else:
            self.test_results['lidar_point_count'] = False
            self.get_logger().warn(f'LiDAR point count validation failed: '
                                 f'only {len(valid_ranges)} valid points, '
                                 f'expected at least {self.lidar_min_points}')

    def validate_imu_data(self):
        """Validate IMU sensor data against acceptance criteria"""
        if not self.imu_data_samples:
            return

        latest_sample = self.imu_data_samples[-1]
        msg = latest_sample['msg']

        # Validate quaternion normalization
        quat_norm = np.sqrt(
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )

        if abs(quat_norm - 1.0) <= self.imu_orientation_threshold:
            self.test_results['imu_orientation_validation'] = True
        else:
            self.test_results['imu_orientation_validation'] = False
            self.get_logger().warn(f'IMU orientation validation failed: '
                                 f'norm={quat_norm:.3f}, expected=1.0')

        # Validate angular velocity (should be reasonable for humanoid robot)
        ang_vel_norm = np.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )

        # Threshold: 50 rad/s is very high for a humanoid (about 500 RPM)
        if ang_vel_norm <= 50.0:
            self.test_results['imu_angular_velocity_validation'] = True
        else:
            self.test_results['imu_angular_velocity_validation'] = False
            self.get_logger().warn(f'IMU angular velocity validation failed: '
                                 f'{ang_vel_norm:.3f} rad/s (too high)')

        # Validate linear acceleration (should be reasonable)
        lin_acc_norm = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        # Threshold: 50 m/s^2 is very high (about 5g)
        if lin_acc_norm <= 50.0:
            self.test_results['imu_linear_acceleration_validation'] = True
        else:
            self.test_results['imu_linear_acceleration_validation'] = False
            self.get_logger().warn(f'IMU linear acceleration validation failed: '
                                 f'{lin_acc_norm:.3f} m/s^2 (too high)')

    def validate_image_data(self):
        """Validate image sensor data against acceptance criteria"""
        if not self.image_data_samples:
            return

        latest_sample = self.image_data_samples[-1]
        msg = latest_sample['msg']

        # Check image resolution
        if msg.width >= self.image_min_width and msg.height >= self.image_min_height:
            self.test_results['image_resolution_validation'] = True
        else:
            self.test_results['image_resolution_validation'] = False
            self.get_logger().warn(f'Image resolution validation failed: '
                                 f'{msg.width}x{msg.height}, '
                                 f'expected at least {self.image_min_width}x{self.image_min_height}')

        # Check image encoding (should be standard format)
        valid_encodings = ['rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8']
        if msg.encoding in valid_encodings:
            self.test_results['image_encoding_validation'] = True
        else:
            self.test_results['image_encoding_validation'] = False
            self.get_logger().warn(f'Image encoding validation failed: '
                                 f'{msg.encoding}, expected one of {valid_encodings}')

    def run_final_validation(self):
        """Run final validation and report results"""
        self.get_logger().info('--- Final Sensor Acceptance Validation Results ---')

        all_passed = True
        for test_name, result in self.test_results.items():
            status = 'PASS' if result else 'FAIL'
            self.get_logger().info(f'{test_name}: {status}')
            if not result:
                all_passed = False

        self.get_logger().info(f'Overall result: {"PASS" if all_passed else "FAIL"}')

        if all_passed:
            self.get_logger().info('🎉 All sensor acceptance tests passed!')
            exit_code = 0
        else:
            self.get_logger().info('❌ Some sensor acceptance tests failed!')
            exit_code = 1

        # Print summary
        self.print_validation_summary()

        # Exit with appropriate code
        sys.exit(exit_code)

    def print_validation_summary(self):
        """Print a detailed validation summary"""
        self.get_logger().info('\n--- Detailed Validation Summary ---')

        # LiDAR summary
        lidar_samples = len(self.lidar_data_samples)
        if lidar_samples > 0:
            latest_lidar = self.lidar_data_samples[-1]
            self.get_logger().info(f'LiDAR samples collected: {lidar_samples}')
            self.get_logger().info(f'Latest scan points: {len(latest_lidar["msg"].ranges)}')
            self.get_logger().info(f'Valid ranges: {len(latest_lidar["valid_ranges"])}')

        # IMU summary
        imu_samples = len(self.imu_data_samples)
        if imu_samples > 0:
            latest_imu = self.imu_data_samples[-1]
            self.get_logger().info(f'IMU samples collected: {imu_samples}')
            quat = latest_imu['msg'].orientation
            self.get_logger().info(f'Latest orientation: ({quat.x:.3f}, {quat.y:.3f}, {quat.z:.3f}, {quat.w:.3f})')

        # Image summary
        image_samples = len(self.image_data_samples)
        if image_samples > 0:
            latest_image = self.image_data_samples[-1]
            self.get_logger().info(f'Image samples collected: {image_samples}')
            self.get_logger().info(f'Latest image: {latest_image["msg"].width}x{latest_image["msg"].height} {latest_image["msg"].encoding}')

def main(args=None):
    rclpy.init(args=args)

    validator = SensorAcceptanceValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Sensor Acceptance Validator stopped by user')
        validator.print_validation_summary()
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()