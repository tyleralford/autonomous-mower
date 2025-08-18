#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class GroundTruthHeadingNode(Node):
    """
    Subscribes to the simulated IMU orientation and republishes a noisy heading on
    the /gps/heading topic as sensor_msgs/Imu. This mimics a future GPS-based
    heading sensor while leveraging the simulator's IMU for orientation.
    """
    def __init__(self):
        super().__init__('ground_truth_heading_node')
        self.get_logger().info('Ground Truth Heading Node initialized')

        # Parameters
        self.declare_parameter('noise_stddev_rad', 0.005)  # Standard deviation for orientation noise in radians
        self.noise_stddev = self.get_parameter('noise_stddev_rad').get_parameter_value().double_value

    # Publisher for the heading topic
        self.heading_pub = self.create_publisher(Imu, '/gps/heading', 10)

        # Subscriber to the simulated IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            50
        )

        # Desired publish rate (Hz)
        self.declare_parameter('publish_hz', 20.0)
        self.publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value

        # Latest IMU orientation cache
        self._last_orientation = None

        # Timer to publish at a fixed rate (e.g., 20 Hz)
        self.timer = self.create_timer(1.0 / float(self.publish_hz), self.publish_heading)

    def imu_callback(self, msg: Imu):
        """Cache the latest IMU orientation."""
        self._last_orientation = msg.orientation

    def publish_heading(self):
        """Publish the heading at a fixed rate using the latest IMU orientation."""
        if self._last_orientation is None:
            return

        q = self._last_orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_noisy = yaw + np.random.normal(0.0, self.noise_stddev)
        q_noisy = quaternion_from_euler(roll, pitch, yaw_noisy)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = Quaternion(x=q_noisy[0], y=q_noisy[1], z=q_noisy[2], w=q_noisy[3])
        imu_msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, (self.noise_stddev ** 2)
        ]
        self.heading_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
