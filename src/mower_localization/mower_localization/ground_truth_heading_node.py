#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class GroundTruthHeadingNode(Node):
    """
    This node subscribes to the ground truth pose of the robot from the simulator,
    adds noise to the orientation, and publishes it as a sensor_msgs/Imu message.
    This provides a realistic, high-quality heading source for the EKF in simulation.
    """
    def __init__(self):
        super().__init__('ground_truth_heading_node')
        self.get_logger().info('Ground Truth Heading Node initialized')

        # Parameters
        self.declare_parameter('noise_stddev_rad', 0.01)  # Standard deviation for orientation noise in radians
        self.noise_stddev = self.get_parameter('noise_stddev_rad').get_parameter_value().double_value

        # Publisher for the heading topic
        self.heading_pub = self.create_publisher(Imu, '/gps/heading', 10)

        # Subscriber to the ground truth pose (geometry_msgs/Pose)
        self.pose_sub = self.create_subscription(
            Pose,
            '/model/mower/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: Pose):
        """
        Callback for receiving the ground truth pose.
        """
        orientation_q = msg.orientation

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Add Gaussian noise to yaw
        yaw_noisy = yaw + np.random.normal(0.0, self.noise_stddev)

        # Convert back to quaternion
        q_noisy = quaternion_from_euler(roll, pitch, yaw_noisy)

        # Create and publish the Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'  # Or an appropriate frame
        
        imu_msg.orientation = Quaternion(x=q_noisy[0], y=q_noisy[1], z=q_noisy[2], w=q_noisy[3])
        
        # Set a realistic covariance for a high-quality heading sensor
        # Low variance for yaw, high for roll and pitch as they are not measured
        imu_msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, (self.noise_stddev**2)
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
