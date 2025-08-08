#!/usr/bin/env python3
"""
Simple Teleoperation Node for Autonomous Mower

This script provides keyboard teleoperation for the mower robot.
It publishes geometry_msgs/TwistStamped messages to control the robot.

Controls:
    w - Move forward
    s - Move backward  
    a - Turn left
    d - Turn right
    q - Forward + left
    e - Forward + right
    z - Backward + left
    c - Backward + right
    x - Stop
    
    SPACE - Emergency stop
    ESC or Ctrl+C - Exit

Linear velocity: 0.5 m/s
Angular velocity: 1.0 rad/s
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class SimpleTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.publisher_ = self.create_publisher(
            TwistStamped, 
            '/diff_drive_controller/cmd_vel', 
            10
        )
        
        # Velocity settings
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s
        
        # Key bindings
        self.key_bindings = {
            'w': (1, 0),    # forward
            's': (-1, 0),   # backward
            'a': (0, 1),    # left
            'd': (0, -1),   # right
            'q': (1, 1),    # forward + left
            'e': (1, -1),   # forward + right
            'z': (-1, 1),   # backward + left
            'c': (-1, -1),  # backward + right
            'x': (0, 0),    # stop
        }
        
        self.get_logger().info('Simple Teleop Node Started')
        self.get_logger().info('Use w/a/s/d for movement, x to stop, SPACE for emergency stop')
        self.get_logger().info('Press ESC or Ctrl+C to exit')
        
    def get_key(self):
        """Get a single keypress from stdin"""
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return None
        
    def publish_twist(self, linear_x, angular_z):
        """Publish a TwistStamped message"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x * self.linear_speed
        msg.twist.angular.z = angular_z * self.angular_speed
        
        self.publisher_.publish(msg)
        
    def run(self):
        """Main teleoperation loop"""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode for single character input
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = self.get_key()
                
                if key is not None:
                    if key == '\x1b':  # ESC key
                        break
                    elif key == '\x03':  # Ctrl+C
                        break
                    elif key == ' ':  # Space - emergency stop
                        self.publish_twist(0.0, 0.0)
                        self.get_logger().info('Emergency Stop!')
                    elif key.lower() in self.key_bindings:
                        linear, angular = self.key_bindings[key.lower()]
                        self.publish_twist(float(linear), float(angular))
                        
                        if linear == 0 and angular == 0:
                            self.get_logger().info('Stop')
                        else:
                            direction = []
                            if linear > 0:
                                direction.append('Forward')
                            elif linear < 0:
                                direction.append('Backward')
                            if angular > 0:
                                direction.append('Left')
                            elif angular < 0:
                                direction.append('Right')
                            self.get_logger().info(f'Moving: {" + ".join(direction)}')
                    
                # Spin once to process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error in teleop loop: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # Send final stop command
            self.publish_twist(0.0, 0.0)
            self.get_logger().info('Teleoperation stopped')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = SimpleTeleopNode()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
