#!/usr/bin/env python3

"""
Test script to demonstrate position and orientation hold functionality.
This script publishes velocity commands to test the PID controller's hold capabilities.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class PositionOrientationHoldTester(Node):
    """Test node for position and orientation hold functionality."""
    
    def __init__(self):
        super().__init__('position_orientation_hold_tester')
        
        # Publisher for raw velocity commands
        self.cmd_vel_raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Test sequence timer
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info('Position and Orientation Hold Tester initialized')
        self.get_logger().info('Test sequence will start in 5 seconds...')
        
    def run_test_sequence(self):
        """Run a sequence of tests to demonstrate hold functionality."""
        cmd_vel = Twist()
        
        if self.test_step == 0:
            # Step 1: Move forward
            self.get_logger().info('Step 1: Moving forward (0.5 m/s)')
            cmd_vel.linear.x = 0.5
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        elif self.test_step == 1:
            # Step 2: Stop and activate position hold
            self.get_logger().info('Step 2: Stopping and activating position hold')
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        elif self.test_step == 2:
            # Step 3: Rotate
            self.get_logger().info('Step 3: Rotating (0.3 rad/s)')
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.3
            
        elif self.test_step == 3:
            # Step 4: Stop and activate orientation hold
            self.get_logger().info('Step 4: Stopping rotation and activating orientation hold')
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        elif self.test_step == 4:
            # Step 5: Move up
            self.get_logger().info('Step 5: Moving up (0.3 m/s)')
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.3
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        elif self.test_step == 5:
            # Step 6: Final stop - both position and orientation hold should be active
            self.get_logger().info('Step 6: Final stop - both position and orientation hold active')
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        else:
            # Test complete
            self.get_logger().info('Test sequence complete!')
            self.test_timer.cancel()
            return
            
        # Publish command
        self.cmd_vel_raw_pub.publish(cmd_vel)
        self.test_step += 1


def main(args=None):
    rclpy.init(args=args)
    
    tester = PositionOrientationHoldTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down tester')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 