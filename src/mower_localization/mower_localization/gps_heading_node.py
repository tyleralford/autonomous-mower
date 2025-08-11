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
        
        # TODO: Implement heading calculation logic in next task
        self.get_logger().debug('GPS heading calculation placeholder')


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
