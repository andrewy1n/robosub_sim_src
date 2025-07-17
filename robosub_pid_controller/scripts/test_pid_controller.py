#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class PIDTester(Node):
    """Test script for the PID controller."""
    
    def __init__(self):
        super().__init__('pid_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Test parameters
        self.test_amplitude = 0.5
        self.test_duration = 5.0
        
        self.get_logger().info('PID Controller Tester initialized')
        
    def run_linear_tests(self):
        """Test linear motion in all axes."""
        axes = ['x', 'y', 'z']
        
        for axis in axes:
            self.get_logger().info(f'Testing linear {axis} axis...')
            
            cmd = Twist()
            if axis == 'x':
                cmd.linear.x = self.test_amplitude
            elif axis == 'y':
                cmd.linear.y = self.test_amplitude
            elif axis == 'z':
                cmd.linear.z = self.test_amplitude
                
            # Send command
            self.cmd_vel_pub.publish(cmd)
            time.sleep(self.test_duration)
            
            # Stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(2.0)
            
    def run_angular_tests(self):
        """Test angular motion in all axes."""
        axes = ['x', 'y', 'z']
        
        for axis in axes:
            self.get_logger().info(f'Testing angular {axis} axis...')
            
            cmd = Twist()
            if axis == 'x':
                cmd.angular.x = self.test_amplitude
            elif axis == 'y':
                cmd.angular.y = self.test_amplitude
            elif axis == 'z':
                cmd.angular.z = self.test_amplitude
                
            # Send command
            self.cmd_vel_pub.publish(cmd)
            time.sleep(self.test_duration)
            
            # Stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(2.0)
            
    def run_combined_test(self):
        """Test combined motion."""
        self.get_logger().info('Testing combined motion...')
        
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.linear.y = 0.2
        cmd.angular.z = 0.3
        
        self.cmd_vel_pub.publish(cmd)
        time.sleep(self.test_duration)
        
        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
    def run_sinusoidal_test(self):
        """Test sinusoidal input for frequency response."""
        self.get_logger().info('Running sinusoidal test...')
        
        frequency = 0.1  # Hz
        duration = 20.0  # seconds
        dt = 0.1
        
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            cmd = Twist()
            cmd.linear.x = self.test_amplitude * math.sin(2 * math.pi * frequency * t)
            
            self.cmd_vel_pub.publish(cmd)
            time.sleep(dt)
            
        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    
    tester = PIDTester()
    
    try:
        print("\nPID Controller Test Menu:")
        print("1. Linear axis tests")
        print("2. Angular axis tests") 
        print("3. Combined motion test")
        print("4. Sinusoidal test")
        print("5. All tests")
        
        choice = input("Enter your choice (1-5): ")
        
        if choice == '1':
            tester.run_linear_tests()
        elif choice == '2':
            tester.run_angular_tests()
        elif choice == '3':
            tester.run_combined_test()
        elif choice == '4':
            tester.run_sinusoidal_test()
        elif choice == '5':
            tester.run_linear_tests()
            tester.run_angular_tests()
            tester.run_combined_test()
        else:
            print("Invalid choice")
            
        tester.get_logger().info('Test completed')
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
    finally:
        # Ensure stop command is sent
        stop_cmd = Twist()
        tester.cmd_vel_pub.publish(stop_cmd)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 