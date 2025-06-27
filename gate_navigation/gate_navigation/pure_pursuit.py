#!/usr/bin/env python3

import numpy as np
import math


class PurePursuitController:
    def __init__(self, lookahead_distance=2.0, max_steering_angle=0.5):
        """
        Initialize Pure Pursuit Controller
        
        Args:
            lookahead_distance: Distance to look ahead on the path (meters)
            max_steering_angle: Maximum steering angle (radians)
        """
        self.lookahead_distance = lookahead_distance
        self.max_steering_angle = max_steering_angle
        
        # Control gains
        self.k_linear = 1.0
        self.k_angular = 2.0
        
        # Path storage
        self.path = []
        self.current_waypoint_index = 0
    
    def generate_path(self, start_point, end_point, num_points=10):
        """
        Generate a simple straight-line path from start to end
        
        Args:
            start_point: Starting position [x, y, z]
            end_point: Ending position [x, y, z]
            num_points: Number of waypoints to generate
            
        Returns:
            List of waypoints [[x, y, z], ...]
        """
        start = np.array(start_point)
        end = np.array(end_point)
        
        # Generate waypoints along the line
        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)
            waypoint = start + t * (end - start)
            waypoints.append(waypoint)
        
        self.path = waypoints
        return waypoints
    
    def generate_curved_path(self, start_point, end_point, control_point=None, num_points=20):
        """
        Generate a curved path using quadratic Bezier curve
        
        Args:
            start_point: Starting position [x, y, z]
            end_point: Ending position [x, y, z]
            control_point: Control point for the curve (optional)
            num_points: Number of waypoints to generate
            
        Returns:
            List of waypoints [[x, y, z], ...]
        """
        start = np.array(start_point)
        end = np.array(end_point)
        
        # If no control point provided, create one perpendicular to the line
        if control_point is None:
            direction = end - start
            perpendicular = np.array([-direction[1], direction[0], 0])
            control_point = start + 0.5 * direction + 2.0 * perpendicular
        
        control = np.array(control_point)
        
        # Generate Bezier curve waypoints
        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)
            waypoint = self._bezier_curve(start, control, end, t)
            waypoints.append(waypoint)
        
        self.path = waypoints
        return waypoints
    
    def _bezier_curve(self, p0, p1, p2, t):
        """
        Calculate point on quadratic Bezier curve
        
        Args:
            p0: Start point
            p1: Control point
            p2: End point
            t: Parameter (0 to 1)
            
        Returns:
            Point on the curve
        """
        return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2
    
    def calculate_control(self, current_position, current_orientation, path=None):
        """
        Calculate control commands using pure pursuit
        
        Args:
            current_position: Current robot position [x, y, z]
            current_orientation: Current robot orientation as quaternion [w, x, y, z]
            path: Optional path to use (if None, uses stored path)
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        if path is not None:
            self.path = path
        
        if not self.path:
            return 0.0, 0.0
        
        # Convert orientation to yaw angle
        yaw = self._orientation_to_yaw(current_orientation)
        
        # Find the lookahead point
        lookahead_point = self._find_lookahead_point(current_position)
        
        if lookahead_point is None:
            return 0.0, 0.0
        
        # Calculate pure pursuit control
        linear_vel, angular_vel = self._pure_pursuit_control(
            current_position, yaw, lookahead_point
        )
        
        return linear_vel, angular_vel
    
    def _find_lookahead_point(self, current_position):
        """
        Find the lookahead point on the path
        
        Args:
            current_position: Current robot position
            
        Returns:
            Lookahead point or None if not found
        """
        current_pos = np.array(current_position)
        
        # Find the closest point on the path within lookahead distance
        for i, waypoint in enumerate(self.path):
            waypoint_pos = np.array(waypoint)
            distance = np.linalg.norm(waypoint_pos - current_pos)
            
            if distance >= self.lookahead_distance:
                # Found a point at or beyond lookahead distance
                return waypoint
        
        # If no point found, return the last waypoint
        if self.path:
            return self.path[-1]
        
        return None
    
    def _pure_pursuit_control(self, current_position, yaw, lookahead_point):
        """
        Calculate pure pursuit control commands
        
        Args:
            current_position: Current robot position
            yaw: Current robot yaw angle
            lookahead_point: Target lookahead point
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        current_pos = np.array(current_position)
        target_pos = np.array(lookahead_point)
        
        # Calculate distance and angle to target
        distance = np.linalg.norm(target_pos[:2] - current_pos[:2])
        target_angle = math.atan2(
            target_pos[1] - current_pos[1],
            target_pos[0] - current_pos[0]
        )
        
        # Calculate angle difference
        angle_diff = self._normalize_angle(target_angle - yaw)
        
        # Pure pursuit formula
        # Angular velocity = 2 * v * sin(alpha) / L
        # where alpha is the angle to target, L is lookahead distance
        linear_velocity = self.k_linear * min(distance, 2.0)  # Limit forward speed
        
        if abs(angle_diff) > 0.1:  # If significantly off course
            angular_velocity = self.k_angular * angle_diff
            # Reduce linear velocity when turning
            linear_velocity *= 0.5
        else:
            angular_velocity = 0.0
        
        # Limit angular velocity
        angular_velocity = np.clip(angular_velocity, -self.max_steering_angle, self.max_steering_angle)
        
        return linear_velocity, angular_velocity
    
    def _orientation_to_yaw(self, orientation):
        """
        Convert orientation to yaw angle
        
        Args:
            orientation: Current robot orientation as quaternion [w, x, y, z] or euler angles [roll, pitch, yaw]
            
        Returns:
            Yaw angle in radians
        """
        if len(orientation) == 4:  # Quaternion
            w, x, y, z = orientation
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        elif len(orientation) == 3:  # Euler angles
            roll, pitch, yaw = orientation
        else:
            raise ValueError("Invalid orientation format")
        
        return yaw
    
    def _normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi]
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def update_path(self, new_path):
        """
        Update the current path
        
        Args:
            new_path: New path as list of waypoints
        """
        self.path = new_path
        self.current_waypoint_index = 0
    
    def get_path_progress(self):
        """
        Get the progress along the current path
        
        Returns:
            Progress as a percentage (0-100)
        """
        if not self.path:
            return 0.0
        
        return (self.current_waypoint_index / len(self.path)) * 100.0
    
    def is_path_complete(self, current_position, tolerance=1.0):
        """
        Check if the robot has reached the end of the path
        
        Args:
            current_position: Current robot position
            tolerance: Distance tolerance in meters
            
        Returns:
            True if path is complete
        """
        if not self.path:
            return True
        
        current_pos = np.array(current_position)
        end_pos = np.array(self.path[-1])
        
        distance = np.linalg.norm(current_pos - end_pos)
        return distance < tolerance 