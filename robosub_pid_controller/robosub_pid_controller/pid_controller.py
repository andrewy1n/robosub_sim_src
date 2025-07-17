import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from dave_interfaces.msg import DVL
import math
import time
from typing import Dict, List, Optional


class PIDController:    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, 
                 output_limit: float = 1.0, integral_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        
    def update(self, error: float, current_time: Optional[float] = None) -> float:
        if current_time is None:
            current_time = time.time()
            
        dt = current_time - self.previous_time
        if dt <= 0:
            dt = 0.001  # Prevent division by zero
            
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        output = max(min(output, self.output_limit), -self.output_limit)
        
        # Store values for next iteration
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        
    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd


class RobosubPIDController(Node):
    
    def __init__(self):
        super().__init__('robosub_pid_controller')
        
        # Declare parameters for PID tuning
        self.declare_pid_parameters()
        
        # Initialize PID controllers for each DOF
        self.pid_controllers = self.initialize_pid_controllers()
        
        # Feedback state
        self.current_velocity = Twist()
        self.current_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.target_velocity = Twist()
        
        # Position tracking for position hold
        self.current_position = [0.0, 0.0, 0.0]  # x, y, z
        self.target_position = [0.0, 0.0, 0.0]   # x, y, z
        self.last_position_update = time.time()
        
        # Orientation tracking for orientation hold
        self.target_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.orientation_hold_active = False
        
        # Subscribers
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_raw_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/imu', self.imu_callback, 10)
        self.dvl_sub = self.create_subscription(
            DVL, '/sensors/dvl/velocity', self.dvl_callback, 10)
            
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('6-DOF PID Controller initialized')
        
    def declare_pid_parameters(self):
        # Linear velocity PID gains
        self.declare_parameter('linear_x.kp', 2.0)
        self.declare_parameter('linear_x.ki', 0.1)
        self.declare_parameter('linear_x.kd', 0.5)
        
        self.declare_parameter('linear_y.kp', 2.0)
        self.declare_parameter('linear_y.ki', 0.1)
        self.declare_parameter('linear_y.kd', 0.5)
        
        self.declare_parameter('linear_z.kp', 2.5)
        self.declare_parameter('linear_z.ki', 0.15)
        self.declare_parameter('linear_z.kd', 0.6)
        
        # Angular velocity PID gains
        self.declare_parameter('angular_x.kp', 1.5)
        self.declare_parameter('angular_x.ki', 0.05)
        self.declare_parameter('angular_x.kd', 0.3)
        
        self.declare_parameter('angular_y.kp', 1.5)
        self.declare_parameter('angular_y.ki', 0.05)
        self.declare_parameter('angular_y.kd', 0.3)
        
        self.declare_parameter('angular_z.kp', 1.8)
        self.declare_parameter('angular_z.ki', 0.08)
        self.declare_parameter('angular_z.kd', 0.4)
        
        # Output limits
        self.declare_parameter('linear_output_limit', 2.0)
        self.declare_parameter('angular_output_limit', 1.5)
        
        # Integral limits (anti-windup)
        self.declare_parameter('linear_integral_limit', 1.0)
        self.declare_parameter('angular_integral_limit', 0.5)
        
        # Control enable flags
        self.declare_parameter('enable_position_hold', True)
        self.declare_parameter('enable_orientation_hold', True)
        self.declare_parameter('velocity_feedback_weight', 0.8)
        
        # Position and orientation hold parameters
        self.declare_parameter('position_hold_threshold', 0.01)  # m/s threshold for position hold activation
        self.declare_parameter('orientation_hold_threshold', 0.01)  # rad/s threshold for orientation hold activation
        self.declare_parameter('position_tolerance', 0.1)  # m tolerance for position hold
        self.declare_parameter('orientation_tolerance', 0.05)  # rad tolerance for orientation hold
        
    def initialize_pid_controllers(self) -> Dict[str, PIDController]:
        controllers = {}
        
        # Get current parameter values
        linear_limit = self.get_parameter('linear_output_limit').value
        angular_limit = self.get_parameter('angular_output_limit').value
        linear_int_limit = self.get_parameter('linear_integral_limit').value
        angular_int_limit = self.get_parameter('angular_integral_limit').value
        
        # Linear controllers
        for axis in ['x', 'y', 'z']:
            kp = float(self.get_parameter(f'linear_{axis}.kp').value or 0.0)
            ki = float(self.get_parameter(f'linear_{axis}.ki').value or 0.0)
            kd = float(self.get_parameter(f'linear_{axis}.kd').value or 0.0)
            controllers[f'linear_{axis}'] = PIDController(
                kp, ki, kd, linear_limit or 1.0, linear_int_limit or 1.0)
                
        # Angular controllers
        for axis in ['x', 'y', 'z']:
            kp = float(self.get_parameter(f'angular_{axis}.kp').value or 0.0)
            ki = float(self.get_parameter(f'angular_{axis}.ki').value or 0.0)
            kd = float(self.get_parameter(f'angular_{axis}.kd').value or 0.0)
            controllers[f'angular_{axis}'] = PIDController(
                kp, ki, kd, angular_limit or 1.0, angular_int_limit or 1.0)
                
        return controllers
        
    def parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Handle parameter updates for real-time tuning."""
        for param in params:
            name = param.name
            value = param.value
            
            # Update PID gains
            if '.' in name:
                axis, gain = name.split('.')
                if axis in self.pid_controllers and gain in ['kp', 'ki', 'kd']:
                    if gain == 'kp':
                        self.pid_controllers[axis].kp = float(value or 0.0)
                    elif gain == 'ki':
                        self.pid_controllers[axis].ki = float(value or 0.0)
                    elif gain == 'kd':
                        self.pid_controllers[axis].kd = float(value or 0.0)
                    self.get_logger().info(f'Updated {name} to {value}')
                    
            # Update limits
            elif name.endswith('_limit'):
                for controller in self.pid_controllers.values():
                    if 'output' in name:
                        controller.output_limit = float(value or 1.0)
                    elif 'integral' in name:
                        controller.integral_limit = float(value or 1.0)
                        
        return SetParametersResult(successful=True)
        
    def cmd_vel_raw_callback(self, msg: Twist):
        """Handle raw velocity commands."""
        self.target_velocity = msg
        
        # Check if position hold should be activated (zero velocity command)
        enable_position_hold = self.get_parameter('enable_position_hold').value
        position_threshold = float(self.get_parameter('position_hold_threshold').value or 0.01)
        if enable_position_hold:
            if (abs(msg.linear.x) < position_threshold and 
                abs(msg.linear.y) < position_threshold and 
                abs(msg.linear.z) < position_threshold):
                # Store current position as target for position hold
                self.target_position = self.current_position.copy()
                self.get_logger().debug('Position hold activated')
            else:
                # Update target position based on velocity command
                dt = time.time() - self.last_position_update
                self.target_position[0] += msg.linear.x * dt
                self.target_position[1] += msg.linear.y * dt
                self.target_position[2] += msg.linear.z * dt
                self.last_position_update = time.time()
        
        # Check if orientation hold should be activated (zero angular velocity command)
        enable_orientation_hold = self.get_parameter('enable_orientation_hold').value
        orientation_threshold = float(self.get_parameter('orientation_hold_threshold').value or 0.01)
        if enable_orientation_hold:
            if (abs(msg.angular.x) < orientation_threshold and 
                abs(msg.angular.y) < orientation_threshold and 
                abs(msg.angular.z) < orientation_threshold):
                # Store current orientation as target for orientation hold
                self.target_orientation = self.current_orientation.copy()
                self.orientation_hold_active = True
                self.get_logger().debug('Orientation hold activated')
            else:
                # Update target orientation based on angular velocity command
                dt = time.time() - self.last_position_update
                self.target_orientation[0] += msg.angular.x * dt  # roll
                self.target_orientation[1] += msg.angular.y * dt  # pitch
                self.target_orientation[2] += msg.angular.z * dt  # yaw
                self.orientation_hold_active = False
        
    def imu_callback(self, msg: Imu):
        """Handle IMU feedback for orientation."""
        # Convert quaternion to euler angles
        self.current_orientation = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w)
            
    def dvl_callback(self, msg: DVL):
        """Handle DVL feedback for velocity."""
        # Use DVL velocity data for feedback
        self.current_velocity.linear.x = msg.velocity.twist.linear.x
        self.current_velocity.linear.y = msg.velocity.twist.linear.y
        self.current_velocity.linear.z = msg.velocity.twist.linear.z
        
        # Integrate velocity to update current position
        current_time = time.time()
        dt = current_time - self.last_position_update
        if dt > 0:
            self.current_position[0] += self.current_velocity.linear.x * dt
            self.current_position[1] += self.current_velocity.linear.y * dt
            self.current_position[2] += self.current_velocity.linear.z * dt
            self.last_position_update = current_time
        
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
        
    def wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def control_loop(self):
        """Main control loop running at 50 Hz."""
        current_time = time.time()
        feedback_weight = self.get_parameter('velocity_feedback_weight').value
        enable_position_hold = self.get_parameter('enable_position_hold').value
        enable_orientation_hold = self.get_parameter('enable_orientation_hold').value
        
        # Create output message
        cmd_vel = Twist()
        
        # Linear control with position hold
        feedback_weight = float(feedback_weight or 0.8)
        
        if enable_position_hold:
            # Position hold mode - calculate position errors
            position_errors = [
                self.target_position[0] - self.current_position[0],
                self.target_position[1] - self.current_position[1],
                self.target_position[2] - self.current_position[2]
            ]
            
            # Convert position errors to velocity commands using PID
            cmd_vel.linear.x = self.pid_controllers['linear_x'].update(position_errors[0], current_time)
            cmd_vel.linear.y = self.pid_controllers['linear_y'].update(position_errors[1], current_time)
            cmd_vel.linear.z = self.pid_controllers['linear_z'].update(position_errors[2], current_time)
        else:
            # Velocity control mode - calculate velocity errors
            linear_errors = [
                self.target_velocity.linear.x - (self.current_velocity.linear.x * feedback_weight),
                self.target_velocity.linear.y - (self.current_velocity.linear.y * feedback_weight),
                self.target_velocity.linear.z - (self.current_velocity.linear.z * feedback_weight)
            ]
            
            cmd_vel.linear.x = self.pid_controllers['linear_x'].update(linear_errors[0], current_time)
            cmd_vel.linear.y = self.pid_controllers['linear_y'].update(linear_errors[1], current_time)
            cmd_vel.linear.z = self.pid_controllers['linear_z'].update(linear_errors[2], current_time)
        
        # Angular control with orientation hold
        if enable_orientation_hold and self.orientation_hold_active:
            # Orientation hold mode - calculate orientation errors
            # Handle angle wrapping for yaw (z-axis)
            orientation_errors = [
                self.target_orientation[0] - self.current_orientation[0],  # roll
                self.target_orientation[1] - self.current_orientation[1],  # pitch
                self.wrap_angle(self.target_orientation[2] - self.current_orientation[2])  # yaw
            ]
            
            # Convert orientation errors to angular velocity commands using PID
            cmd_vel.angular.x = self.pid_controllers['angular_x'].update(orientation_errors[0], current_time)
            cmd_vel.angular.y = self.pid_controllers['angular_y'].update(orientation_errors[1], current_time)
            cmd_vel.angular.z = self.pid_controllers['angular_z'].update(orientation_errors[2], current_time)
        else:
            # Angular velocity control mode
            angular_errors = [
                self.target_velocity.angular.x,
                self.target_velocity.angular.y,
                self.target_velocity.angular.z
            ]
            
            cmd_vel.angular.x = self.pid_controllers['angular_x'].update(angular_errors[0], current_time)
            cmd_vel.angular.y = self.pid_controllers['angular_y'].update(angular_errors[1], current_time)
            cmd_vel.angular.z = self.pid_controllers['angular_z'].update(angular_errors[2], current_time)
        
        # Debug logging
        if enable_position_hold:
            self.get_logger().debug(
                f'Position Hold - Target: {self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f} | '
                f'Current: {self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f} | '
                f'Errors: {position_errors[0]:.3f}, {position_errors[1]:.3f}, {position_errors[2]:.3f} | '
                f'Output: {cmd_vel.linear.x:.3f}, {cmd_vel.linear.y:.3f}, {cmd_vel.linear.z:.3f}'
            )
        else:
            self.get_logger().debug(
                f'Velocity Control - Target: {self.target_velocity.linear.x:.3f}, {self.target_velocity.linear.y:.3f}, {self.target_velocity.linear.z:.3f} | '
                f'Current: {self.current_velocity.linear.x:.3f}, {self.current_velocity.linear.y:.3f}, {self.current_velocity.linear.z:.3f} | '
                f'Output: {cmd_vel.linear.x:.3f}, {cmd_vel.linear.y:.3f}, {cmd_vel.linear.z:.3f}'
            )
        
        # Publish controlled velocity
        self.cmd_vel_pub.publish(cmd_vel)
        
    def reset_controllers(self):
        """Reset all PID controllers."""
        for controller in self.pid_controllers.values():
            controller.reset()
        self.get_logger().info('All PID controllers reset')
        
    def reset_position_orientation(self):
        """Reset position and orientation tracking."""
        self.current_position = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0]
        self.target_orientation = [0.0, 0.0, 0.0]
        self.orientation_hold_active = False
        self.last_position_update = time.time()
        self.get_logger().info('Position and orientation tracking reset')


def main(args=None):
    rclpy.init(args=args)
    
    controller = RobosubPIDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down PID controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 