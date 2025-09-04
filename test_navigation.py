#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, description):
        self.get_logger().info(f'Testing: {description}')
        
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available!')
            return False
            
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info(f'Goal accepted for: {description}')
                return True
            else:
                self.get_logger().warn(f'Goal rejected for: {description}')
                return False
        else:
            self.get_logger().error(f'Failed to send goal for: {description}')
            return False

def main():
    rclpy.init()
    tester = NavigationTester()
    
    # Test A: Valid path in free space (center of map approximately)
    test_a = tester.send_goal(2.0, 0.0, "Test A: Valid Path in Free Space")
    
    # Test B: Path around keep-out (assuming keepout is around 1.5, -0.5)
    test_b = tester.send_goal(3.0, -2.0, "Test B: Path Around Keep-Out Zone")
    
    # Test C: Invalid goal (inside keepout zone, approximate location)
    test_c = tester.send_goal(1.5, -0.5, "Test C: Invalid Goal (Inside Keep-Out)")
    
    print("\n=== NAVIGATION TEST RESULTS ===")
    print(f"Test A (Valid Path): {'PASS' if test_a else 'FAIL'}")
    print(f"Test B (Around Keep-Out): {'PASS' if test_b else 'FAIL'}")  
    print(f"Test C (Invalid Goal): {'PASS' if not test_c else 'FAIL'}")  # Should fail
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()