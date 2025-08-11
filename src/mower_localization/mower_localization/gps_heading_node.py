#!/usr/bin/env python3

"""
GPS Heading Node

This node subscribes to dual GPS sensor data and calculates absolute heading
based on the baseline vector between the two GPS sensors.

Subscriptions:
    /gps/left/fix (sensor_msgs/NavSatFix): Left GPS position data
    /gps/right/fix (sensor_msgs/NavSatFix): Right GPS position data

Publications:
    /gps/heading (sensor_msgs/Imu): Calculated heading as orientation quaternion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import math
import tf_transformations


class GPSHeadingNode(Node):
    def __init__(self):
        super().__init__('gps_heading_node')
        
        # Subscribers for dual GPS data
        self.gps_left_sub = self.create_subscription(
            NavSatFix, 
            '/gps/left/fix', 
            self.gps_left_callback, 
            10
        )
        
        self.gps_right_sub = self.create_subscription(
            NavSatFix, 
            '/gps/right/fix', 
            self.gps_right_callback, 
            10
        )
        
        # Publisher for calculated heading
        self.heading_pub = self.create_publisher(Imu, '/gps/heading', 10)
        
        # Storage for latest GPS messages
        self.gps_left_msg = None
        self.gps_right_msg = None
        
        # Timer for heading calculation and publication (20 Hz)
        self.timer = self.create_timer(0.05, self.calculate_and_publish_heading)
        
        self.get_logger().info('GPS Heading Node initialized')

    def gps_left_callback(self, msg):
        """Store latest left GPS message"""
        self.gps_left_msg = msg

    def gps_right_callback(self, msg):
        """Store latest right GPS message"""
        self.gps_right_msg = msg

    def calculate_and_publish_heading(self):
        """Calculate heading from GPS baseline and publish as IMU message"""
        # Check if we have data from both GPS sensors
        if self.gps_left_msg is None or self.gps_right_msg is None:
            self.get_logger().debug('Waiting for GPS data from both sensors')
            return
        
        # Calculate vector between GPS positions
        # Convert lat/lon to radians for calculation
        lat1 = math.radians(self.gps_left_msg.latitude)
        lon1 = math.radians(self.gps_left_msg.longitude)
        lat2 = math.radians(self.gps_right_msg.latitude)
        lon2 = math.radians(self.gps_right_msg.longitude)
        
        # Calculate the difference in longitude
        dlon = lon2 - lon1
        
        # Calculate bearing using atan2
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        # Add mandatory 90-degree yaw offset correction (1.5707963 radians)
        corrected_bearing = bearing + 1.5707963
        
        # Normalize angle to [-pi, pi]
        corrected_bearing = math.atan2(math.sin(corrected_bearing), math.cos(corrected_bearing))
        
        # Convert to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, corrected_bearing)
        
        # Create and populate IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Set orientation quaternion
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Set orientation covariance (reasonable fixed values)
        # Only set the yaw covariance (position [8] in 3x3 matrix)
        imu_msg.orientation_covariance = [
            100.0, 0.0, 0.0,
            0.0, 100.0, 0.0,
            0.0, 0.0, 0.01  # Yaw variance: 0.01 rad^2
        ]
        
        # Set unused angular velocity and acceleration to zero with high covariance
        imu_msg.angular_velocity_covariance[0] = -1.0  # Mark as unused
        imu_msg.linear_acceleration_covariance[0] = -1.0  # Mark as unused
        
        # Publish the heading message
        self.heading_pub.publish(imu_msg)
        
        self.get_logger().debug(f'Published GPS heading: {math.degrees(corrected_bearing):.2f} degrees')


def main(args=None):
    rclpy.init(args=args)
    node = GPSHeadingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
