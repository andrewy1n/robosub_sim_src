#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from typing import List, Optional, Any, Dict
# import gz.transport  # Commented out due to import issues
# import gz.msgs       # Commented out due to import issues
from .pure_pursuit import PurePursuitController
from .gate_detector import GateDetector
from .velocity_controller import VelocityController
from std_msgs.msg import Int32MultiArray


class GateNavigator(Node):
    def __init__(self):
        super().__init__('gate_navigator')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize components
        self.gate_detector = GateDetector()
        self.pure_pursuit = PurePursuitController()
        self.velocity_controller = VelocityController()
        
        # Robot state
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.current_image = None
        self.gate_detections = []
        self.last_velocity_command = [0, 0, 0, 0, 0, 0]
        self.original_image_frame_id = "camera_link"  # Default frame ID
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('gate_side', 'center')  # 'left', 'center', 'right'
        self.declare_parameter('lateral_gain', 0.5)
        
        confidence_param = self.get_parameter('confidence_threshold').value
        self.confidence_threshold = float(confidence_param) if confidence_param is not None else 0.5
        self.gate_side = self.get_parameter('gate_side').value
        self.lateral_gain = self.get_parameter('lateral_gain').value
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/robosub/camera/simulated_image',
            self.image_callback,
            10
        )
        
        self.velocity_pub = self.create_publisher(
            Int32MultiArray,
            '/vector_topic',
            10
        )
        
        self.debug_pub = self.create_publisher(
            Image,
            '/debug_image',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/navigation_path',
            10
        )
        
        # Timer for navigation loop
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz
        
        self.get_logger().info(f"Gate Navigator initialized with gate_side={self.gate_side}, lateral_gain={self.lateral_gain}")

    def __del__(self):
        """Cleanup when node is destroyed"""
        cv2.destroyAllWindows()

    def pose_callback(self, msg):  # Simplified callback without type hints
        """Callback for robot pose updates"""
        # For now, use default values since pose topic is not available
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.get_logger().debug("Pose callback called with default values")

    def image_callback(self, msg):
        """Callback for camera images"""
        try:
            # Store the original frame ID
            self.original_image_frame_id = msg.header.frame_id
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Detect gates in the image
            self.gate_detections = self.gate_detector.detect_gates(
                self.current_image, 
                self.confidence_threshold
            )
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def navigation_loop(self):
        """Main navigation loop"""
        # Always publish debug information first
        self.publish_debug_info()
        
        if self.current_image is None:
            return
            
        # Update pure pursuit with current detections
        if self.gate_detections:
            # Find the best gate to navigate through based on gate_side parameter
            best_gate = self.select_best_gate(self.gate_detections)
            
            if best_gate:
                # Get gate center position in image
                gate_center_x, gate_center_y = best_gate['center']
                
                # Calculate lateral movement based on gate position in image
                lateral_velocity = self.calculate_lateral_velocity(gate_center_x, gate_center_y)
                
                # Convert gate detection to a path for pure pursuit
                gate_path = self.convert_gate_to_path(best_gate)
                
                # Update pure pursuit controller with the path
                self.pure_pursuit.update_path(gate_path)
                
                # Get desired velocity from pure pursuit (forward movement and yaw)
                desired_linear, desired_angular = self.pure_pursuit.calculate_control(
                    self.robot_position, 
                    self.robot_orientation
                )
                
                # Convert to robot frame velocities
                robot_linear_x, robot_linear_y, robot_angular_z = self.velocity_controller.convert_to_robot_frame_3d(
                    [desired_linear, 0.0, 0.0],
                    [0.0, 0.0, desired_angular],
                    self.robot_orientation
                )
                
                # Add lateral movement to the y-direction velocity
                robot_linear_y += lateral_velocity
                
                # Publish velocity command in the correct format for wrench_movement
                self.publish_velocity_command(robot_linear_x, robot_linear_y, 0.0, 0.0, 0.0, robot_angular_z)
            else:
                # No suitable gate found, stop the robot
                self.publish_velocity_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            # No gate detected, stop the robot
            self.publish_velocity_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def select_best_gate(self, detections):
        """
        Select the best gate to navigate through based on gate_side parameter
        
        Args:
            detections: List of gate detections
            
        Returns:
            Selected gate detection or None if no suitable gate found
        """
        if not detections:
            return None
        
        # Get gate components from detections
        gate_components = self.gate_detector.get_gate_components(detections)
        
        center_pipe = gate_components['center_pipe']
        left_pipe = gate_components['left_pipe']
        right_pipe = gate_components['right_pipe']
        
        # Check if we have the necessary pipes for the selected gate side
        if self.gate_side == 'left':
            # Need center pipe and left pipe for left gate
            if center_pipe is not None and left_pipe is not None:
                # Create a virtual gate between center and left pipes
                gate_center = self.gate_detector.calculate_gate_center(center_pipe, left_pipe)
                gate_width = self.gate_detector.calculate_gate_width(center_pipe, left_pipe)
                
                if gate_center and gate_width:
                    return {
                        'bbox': [
                            min(center_pipe['bbox'][0], left_pipe['bbox'][0]),
                            min(center_pipe['bbox'][1], left_pipe['bbox'][1]),
                            max(center_pipe['bbox'][2], left_pipe['bbox'][2]),
                            max(center_pipe['bbox'][3], left_pipe['bbox'][3])
                        ],
                        'class': 'left_gate',
                        'confidence': min(center_pipe['confidence'], left_pipe['confidence']),
                        'center': gate_center,
                        'width': gate_width
                    }
        
        elif self.gate_side == 'right':
            # Need center pipe and right pipe for right gate
            if center_pipe is not None and right_pipe is not None:
                # Create a virtual gate between center and right pipes
                gate_center = self.gate_detector.calculate_gate_center(center_pipe, right_pipe)
                gate_width = self.gate_detector.calculate_gate_width(center_pipe, right_pipe)
                
                if gate_center and gate_width:
                    return {
                        'bbox': [
                            min(center_pipe['bbox'][0], right_pipe['bbox'][0]),
                            min(center_pipe['bbox'][1], right_pipe['bbox'][1]),
                            max(center_pipe['bbox'][2], right_pipe['bbox'][2]),
                            max(center_pipe['bbox'][3], right_pipe['bbox'][3])
                        ],
                        'class': 'right_gate',
                        'confidence': min(center_pipe['confidence'], right_pipe['confidence']),
                        'center': gate_center,
                        'width': gate_width
                    }
        
        # If no suitable gate found, return the first detection as fallback
        if detections:
            self.get_logger().warn(f"No suitable {self.gate_side} gate found, using fallback")
            return detections[0]
        
        return None

    def convert_gate_to_path(self, gate_detection):
        """
        Convert gate detection to a navigation path
        
        Args:
            gate_detection: Gate detection dictionary
            
        Returns:
            List of waypoints [[x, y, z], ...]
        """
        # Get gate center from bounding box
        x1, y1, x2, y2 = gate_detection['bbox']
        gate_center_x = (x1 + x2) / 2
        gate_center_y = (y1 + y2) / 2
        
        # Convert image coordinates to world coordinates (simplified)
        # Assuming camera is at origin, looking forward
        # This is a very simplified conversion - in reality you'd need proper camera calibration
        
        # Normalize image coordinates to [-1, 1]
        if self.current_image is None:
            # Fallback values if no image available
            image_width = 640
            image_height = 480
        else:
            image_width = self.current_image.shape[1]
            image_height = self.current_image.shape[0]
        
        normalized_x = (gate_center_x - image_width/2) / (image_width/2)
        normalized_y = (gate_center_y - image_height/2) / (image_height/2)
        
        # Convert to world coordinates (simplified)
        # Assuming gate is 5 meters away and camera FOV is 60 degrees
        distance_to_gate = 5.0
        fov_rad = np.radians(60)
        
        world_x = distance_to_gate * np.tan(normalized_x * fov_rad/2)
        world_y = distance_to_gate * np.tan(normalized_y * fov_rad/2)
        world_z = 0.0  # Assume gate is at same depth as robot
        
        # Create a simple path: current position -> gate position -> beyond gate
        current_pos = self.robot_position
        gate_pos = np.array([world_x, world_y, world_z])
        
        # Path: approach gate, go through gate, continue beyond
        approach_distance = 2.0  # meters from gate
        through_distance = 3.0   # meters beyond gate
        
        # Calculate approach point (closer to gate)
        approach_pos = gate_pos - np.array([approach_distance, 0, 0])
        
        # Calculate exit point (beyond gate)
        exit_pos = gate_pos + np.array([through_distance, 0, 0])
        
        # Create path waypoints
        path = [
            current_pos.tolist(),
            approach_pos.tolist(),
            gate_pos.tolist(),
            exit_pos.tolist()
        ]
        
        return path

    def publish_velocity_command(self, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z):
        """
        Publish velocity command as Int32MultiArray for wrench_movement node
        
        Args:
            lin_x: Linear velocity in x direction (forward/backward)
            lin_y: Linear velocity in y direction (left/right)
            lin_z: Linear velocity in z direction (up/down)
            ang_x: Angular velocity around x axis (roll)
            ang_y: Angular velocity around y axis (pitch)
            ang_z: Angular velocity around z axis (yaw)
        """
        # Convert velocities to integer commands
        # The wrench_movement node expects integers that will be multiplied by force/torque magnitudes
        # We'll scale the velocities appropriately
        
        # Scale factors (significantly reduced for slower movement)
        linear_scale = 1  # Reduced from 10 to 1
        angular_scale = 0.5  # Reduced from 5 to 0.5
        
        # Convert to integers and clamp to reasonable range
        cmd_x = int(np.clip(lin_x * linear_scale, -2, 2))  # Reduced range from [-10, 10] to [-2, 2]
        cmd_y = int(np.clip(lin_y * linear_scale, -2, 2))
        cmd_z = int(np.clip(lin_z * linear_scale, -2, 2))
        cmd_roll = int(np.clip(ang_x * angular_scale, -1, 1))  # Reduced range from [-5, 5] to [-1, 1]
        cmd_pitch = int(np.clip(ang_y * angular_scale, -1, 1))
        cmd_yaw = int(np.clip(ang_z * angular_scale, -1, 1))
        
        # Store for debug visualization
        self.last_velocity_command = [cmd_x, cmd_y, cmd_z, cmd_roll, cmd_pitch, cmd_yaw]
        
        # Create Int32MultiArray message
        msg = Int32MultiArray()
        msg.data = [cmd_x, cmd_y, cmd_z, cmd_roll, cmd_pitch, cmd_yaw]
        
        self.velocity_pub.publish(msg)
        
        # Log the command for debugging
        self.get_logger().info(f"Velocity command: {msg.data}")
        
        # Also publish path for visualization
        self.publish_path_visualization()

    def publish_path_visualization(self):
        """Publish the current path for visualization in RViz"""
        if not hasattr(self.pure_pursuit, 'path') or not self.pure_pursuit.path:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = []  # Initialize empty list
        
        for waypoint in self.pure_pursuit.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0  # Default orientation
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def publish_debug_info(self):
        """Publish debug information"""
        self.get_logger().debug("publish_debug_info called")
        
        if self.current_image is not None:
            self.get_logger().debug(f"Current image shape: {self.current_image.shape}")
            # Always create debug image, even if no detections
            if self.gate_detections:
                self.get_logger().debug(f"Gate detections found: {len(self.gate_detections)}")
                # Create debug image with detections
                debug_image = self.gate_detector.visualize_detections(
                    self.current_image, 
                    self.gate_detections
                )
            else:
                self.get_logger().debug("No gate detections, creating fallback image")
                # Create debug image without detections (just the original image)
                debug_image = self.current_image.copy()
                
                # Add text overlay to show no detections
                cv2.putText(debug_image, "No gate detections", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(debug_image, f"Gate side: {self.gate_side}", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(debug_image, f"Lateral gain: {self.lateral_gain}", (10, 100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish debug image to ROS topic
            try:
                ros_image = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link"
                self.debug_pub.publish(ros_image)
                self.get_logger().debug(f"Published debug image to /debug_image topic: {debug_image.shape}")
            except Exception as e:
                self.get_logger().error(f"Error publishing debug image: {e}")
        
        else:
            self.get_logger().debug("No current image available, creating test image")
            # Create a test image if no camera image is available
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            test_image[:] = (100, 100, 100)  # Gray background
            
            # Add test text
            cv2.putText(test_image, "TEST IMAGE - No Camera", (50, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(test_image, "Debug image test", (50, 280), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Publish test image to ROS topic
            try:
                ros_image = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link"
                self.debug_pub.publish(ros_image)
                self.get_logger().debug("Published test debug image to /debug_image topic")
            except Exception as e:
                self.get_logger().error(f"Error publishing test debug image: {e}")

    def calculate_lateral_velocity(self, gate_center_x, gate_center_y):
        """
        Calculate lateral velocity based on gate position in image
        
        Args:
            gate_center_x: Gate center x position in image (pixels)
            gate_center_y: Gate center y position in image (pixels)
            
        Returns:
            Lateral velocity (positive = right, negative = left)
        """
        if self.current_image is None:
            return 0.0
        
        image_width = self.current_image.shape[1]
        image_height = self.current_image.shape[0]
        
        # Calculate horizontal error (how far the gate is from image center)
        image_center_x = image_width / 2
        horizontal_error = gate_center_x - image_center_x
        
        # Normalize error to [-1, 1] range
        normalized_error = horizontal_error / (image_width / 2)
        
        # Calculate lateral velocity using proportional control
        # Positive error means gate is to the right, so move right (positive velocity)
        # Negative error means gate is to the left, so move left (negative velocity)
        lateral_velocity = normalized_error * self.lateral_gain
        
        # Clamp lateral velocity to reasonable limits
        max_lateral_velocity = 1.0  # m/s
        lateral_velocity = np.clip(lateral_velocity, -max_lateral_velocity, max_lateral_velocity)
        
        # Log lateral movement for debugging
        self.get_logger().info(f"Gate center: ({gate_center_x:.1f}, {gate_center_y:.1f}), "
                              f"Error: {horizontal_error:.1f}px, "
                              f"Lateral vel: {lateral_velocity:.3f} m/s")
        
        return lateral_velocity


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 