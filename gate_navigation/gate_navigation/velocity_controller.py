#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Twist


class VelocityController:
    def __init__(self):
        """Initialize the velocity controller"""
        # Control parameters
        self.max_linear_velocity = 2.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.min_linear_velocity = 0.1   # m/s
        
        # PID gains for velocity control
        self.kp_linear = 1.0
        self.ki_linear = 0.1
        self.kd_linear = 0.05
        
        self.kp_angular = 2.0
        self.ki_angular = 0.2
        self.kd_angular = 0.1
        
        # Integral and derivative terms
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        
        # Time step for PID
        self.dt = 0.1
        
    def set_velocity_limits(self, max_linear, max_angular):
        """
        Set velocity limits
        
        Args:
            max_linear: Maximum linear velocity (m/s)
            max_angular: Maximum angular velocity (rad/s)
        """
        self.max_linear_velocity = max_linear
        self.max_angular_velocity = max_angular
    
    def set_pid_gains(self, kp_linear, ki_linear, kd_linear, 
                     kp_angular, ki_angular, kd_angular):
        """
        Set PID gains for velocity control
        
        Args:
            kp_linear: Proportional gain for linear velocity
            ki_linear: Integral gain for linear velocity
            kd_linear: Derivative gain for linear velocity
            kp_angular: Proportional gain for angular velocity
            ki_angular: Integral gain for angular velocity
            kd_angular: Derivative gain for angular velocity
        """
        self.kp_linear = kp_linear
        self.ki_linear = ki_linear
        self.kd_linear = kd_linear
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
    
    def calculate_velocity_command(self, desired_linear, desired_angular, 
                                 current_linear=0.0, current_angular=0.0):
        """
        Calculate velocity command using PID control
        
        Args:
            desired_linear: Desired linear velocity (m/s)
            desired_angular: Desired angular velocity (rad/s)
            current_linear: Current linear velocity (m/s)
            current_angular: Current angular velocity (rad/s)
            
        Returns:
            Twist message with velocity commands
        """
        # Calculate errors
        linear_error = desired_linear - current_linear
        angular_error = desired_angular - current_angular
        
        # PID control for linear velocity
        linear_cmd = self._pid_control(
            linear_error, 
            self.kp_linear, self.ki_linear, self.kd_linear,
            self.linear_integral, self.prev_linear_error
        )
        
        # PID control for angular velocity
        angular_cmd = self._pid_control(
            angular_error,
            self.kp_angular, self.ki_angular, self.kd_angular,
            self.angular_integral, self.prev_angular_error
        )
        
        # Update integral and derivative terms
        self.linear_integral += linear_error * self.dt
        self.angular_integral += angular_error * self.dt
        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error
        
        # Apply velocity limits
        linear_cmd = np.clip(linear_cmd, -self.max_linear_velocity, self.max_linear_velocity)
        angular_cmd = np.clip(angular_cmd, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        
        return twist
    
    def _pid_control(self, error, kp, ki, kd, integral, prev_error):
        """
        Calculate PID control output
        
        Args:
            error: Current error
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            integral: Current integral term
            prev_error: Previous error
            
        Returns:
            Control output
        """
        # Anti-windup for integral term
        integral = np.clip(integral, -1.0, 1.0)
        
        # PID formula
        derivative = (error - prev_error) / self.dt
        output = kp * error + ki * integral + kd * derivative
        
        return output
    
    def simple_velocity_command(self, desired_linear, desired_angular):
        """
        Simple velocity command without PID control
        
        Args:
            desired_linear: Desired linear velocity (m/s)
            desired_angular: Desired angular velocity (rad/s)
            
        Returns:
            Twist message with velocity commands
        """
        # Apply velocity limits
        linear_cmd = np.clip(desired_linear, -self.max_linear_velocity, self.max_linear_velocity)
        angular_cmd = np.clip(desired_angular, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Ensure minimum linear velocity for movement
        if abs(linear_cmd) > 0 and abs(linear_cmd) < self.min_linear_velocity:
            linear_cmd = np.sign(linear_cmd) * self.min_linear_velocity
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        
        return twist
    
    def stop_robot(self):
        """
        Generate stop command
        
        Returns:
            Twist message with zero velocities
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        return twist
    
    def reset_pid(self):
        """Reset PID integral and derivative terms"""
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
    
    def convert_to_robot_frame(self, world_linear, world_angular, robot_orientation):
        """
        Convert world frame velocities to robot frame
        
        Args:
            world_linear: Linear velocity in world frame [x, y, z]
            world_angular: Angular velocity in world frame [x, y, z]
            robot_orientation: Robot orientation as quaternion [w, x, y, z]
            
        Returns:
            Tuple of (robot_linear_x, robot_angular_z)
        """
        # Convert quaternion to rotation matrix (simplified for 2D)
        w, x, y, z = robot_orientation
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        # Rotation matrix for 2D
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # Transform linear velocity
        robot_linear_x = world_linear[0] * cos_yaw + world_linear[1] * sin_yaw
        robot_linear_y = -world_linear[0] * sin_yaw + world_linear[1] * cos_yaw
        
        # Angular velocity remains the same in 2D
        robot_angular_z = world_angular[2]
        
        return robot_linear_x, robot_angular_z
    
    def convert_to_robot_frame_3d(self, world_linear, world_angular, robot_orientation):
        """
        Convert world frame velocities to robot frame (3D version with y-direction)
        
        Args:
            world_linear: Linear velocity in world frame [x, y, z]
            world_angular: Angular velocity in world frame [x, y, z]
            robot_orientation: Robot orientation as quaternion [w, x, y, z] or euler angles [roll, pitch, yaw]
            
        Returns:
            Tuple of (robot_linear_x, robot_linear_y, robot_angular_z)
        """
        # Convert orientation to yaw angle
        if len(robot_orientation) == 4:  # Quaternion
            w, x, y, z = robot_orientation
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        elif len(robot_orientation) == 3:  # Euler angles
            roll, pitch, yaw = robot_orientation
        else:
            raise ValueError("Invalid orientation format")
        
        # Rotation matrix for 2D
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # Transform linear velocity
        robot_linear_x = world_linear[0] * cos_yaw + world_linear[1] * sin_yaw
        robot_linear_y = -world_linear[0] * sin_yaw + world_linear[1] * cos_yaw
        
        # Angular velocity remains the same in 2D
        robot_angular_z = world_angular[2]
        
        return robot_linear_x, robot_linear_y, robot_angular_z
    
    def apply_velocity_ramp(self, current_vel, target_vel, max_accel):
        """
        Apply velocity ramp to limit acceleration
        
        Args:
            current_vel: Current velocity
            target_vel: Target velocity
            max_accel: Maximum acceleration
            
        Returns:
            Limited velocity command
        """
        vel_diff = target_vel - current_vel
        max_vel_change = max_accel * self.dt
        
        if abs(vel_diff) > max_vel_change:
            vel_change = np.sign(vel_diff) * max_vel_change
            limited_vel = current_vel + vel_change
        else:
            limited_vel = target_vel
        
        return limited_vel 