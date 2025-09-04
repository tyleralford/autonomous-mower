#!/usr/bin/env python3
"""
Cmd Vel Stamper Node

Nav2's controller server outputs geometry_msgs/msg/Twist on the /cmd_vel topic.
The mower's diff_drive_controller is configured with use_stamped_vel=true and
therefore requires geometry_msgs/msg/TwistStamped on /diff_drive_controller/cmd_vel.

This node bridges that interface mismatch by subscribing to /cmd_vel (Twist)
and republishing the same velocity data as TwistStamped on the controller's
expected topic. Header stamp is populated with the current ROS time and the
frame_id is set to base_link to align with the robot's base frame.

If in the future Nav2 gains native TwistStamped publishing support, this node
can be removed and the launch remappings simplified.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub_ = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub_ = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.get_logger().info(f'CmdVelStamper bridging {in_topic} (Twist) -> {out_topic} (TwistStamped)')

    def cb(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub_.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
