#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# from dave_interfaces.msg import DVL
import time
import math
from typing import List


class FloatingDemo(Node):
    """
    Demonstration script showing submarine floating in water and holding z = -1m.
    
    This script:
    1. Publishes zero velocity commands to hold position
    2. Monitors IMU and DVL feedback
    3. Shows the submarine maintaining its position and orientation
    4. Provides real-time status updates
    """
    
    def __init__(self):
        super().__init__('floating_demo')
        
        # Initialize state variables
        self.current_velocity = Twist()
        self.current_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.target_velocity = Twist()
        self.dvl_received = False
        self.imu_received = False
        self.odom_received = False
        self.target_depth = -1.0  # meters (negative = underwater)
        self.current_depth = 0.0  # meters
        self.depth_kp = 1.0       # Proportional gain for depth hold
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Create subscribers for feedback
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/imu', self.imu_callback, 10)
        # self.dvl_sub = self.create_subscription(
        #     DVL, '/sensors/dvl/velocity', self.dvl_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/high_level_robosub/odometry', self.odom_callback, 10)
            
        # Create timer for status updates (1 Hz)
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        # Create timer for zero velocity commands (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info('Floating Demo initialized - Submarine will hold position at z=-1m')
        self.get_logger().info('Starting depth control immediately...')
        
    def imu_callback(self, msg: Imu):
        """Handle IMU feedback for orientation."""
        self.imu_received = True
        # Convert quaternion to euler angles
        self.current_orientation = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w)
            
    # def dvl_callback(self, msg: DVL):
    #     """Handle DVL feedback for velocity."""
    #     self.dvl_received = True
    #     self.current_velocity.linear.x = msg.velocity.twist.linear.x
    #     self.current_velocity.linear.y = msg.velocity.twist.linear.y
    #     self.current_velocity.linear.z = msg.velocity.twist.linear.z
        
    def odom_callback(self, msg: Odometry):
        """Handle odometry feedback for depth and velocity."""
        self.odom_received = True
        # Extract depth from odometry pose
        self.current_depth = msg.pose.pose.position.z
        # Extract velocity from odometry twist
        self.current_velocity.linear.x = msg.twist.twist.linear.x
        self.current_velocity.linear.y = msg.twist.twist.linear.y
        self.current_velocity.linear.z = msg.twist.twist.linear.z
        
    def quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> List[float]:
        """Convert quaternion to euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
        
    def control_callback(self):
        """Send velocity commands to hold z = 1m and zero elsewhere."""
        # Proportional control for depth
        depth_error = self.target_depth - self.current_depth
        self.target_velocity.linear.z = self.depth_kp * depth_error
        
        # Debug output (only if we have odometry data)
        if self.odom_received:
            self.get_logger().info(f'Depth control: target={self.target_depth:.2f}, current={self.current_depth:.2f}, error={depth_error:.2f}, command={self.target_velocity.linear.z:.2f}')
        else:
            self.get_logger().info(f'Sending initial command: z={self.target_velocity.linear.z:.2f} (no odometry data yet)')
        
        # Zero other velocities
        self.target_velocity.linear.x = 0.0
        self.target_velocity.linear.y = 0.0
        self.target_velocity.angular.x = 0.0
        self.target_velocity.angular.y = 0.0
        self.target_velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(self.target_velocity)
        
    def status_callback(self):
        """Print status updates."""
        if not self.odom_received:
            self.get_logger().warn('Waiting for odometry data... (sending commands anyway)')
            return
            
        # Convert radians to degrees for display
        roll_deg = math.degrees(self.current_orientation[0])
        pitch_deg = math.degrees(self.current_orientation[1])
        yaw_deg = math.degrees(self.current_orientation[2])
        
        # Get velocity magnitude
        vel_magnitude = math.sqrt(
            self.current_velocity.linear.x**2 + 
            self.current_velocity.linear.y**2 + 
            self.current_velocity.linear.z**2
        )
        
        self.get_logger().info(
            f'Submarine Status:\n'
            f'  Orientation: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°\n'
            f'  Velocity: X={self.current_velocity.linear.x:.3f}, Y={self.current_velocity.linear.y:.3f}, Z={self.current_velocity.linear.z:.3f} m/s\n'
            f'  Depth: {self.current_depth:.2f} m (target: {self.target_depth:.2f} m)\n'
            f'  Speed: {vel_magnitude:.3f} m/s\n'
            f'  Status: Holding depth at z = -1m'
        )


def main(args=None):
    rclpy.init(args=args)
    
    demo = FloatingDemo()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('Floating demo stopped by user')
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 