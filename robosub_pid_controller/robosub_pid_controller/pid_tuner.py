import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from dave_interfaces.msg import DVL
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String, Float64
from std_srvs.srv import Empty, Trigger
import time
import math
from typing import List
import signal
from nav_msgs.msg import Odometry


class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner')
        
        self.destroyed = False
        
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu', self.imu_callback, 10)
        self.dvl_sub = self.create_subscription(DVL, '/sensors/dvl/velocity', self.dvl_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/high_level_robosub/odometry',
            self.odom_callback,
            10
        )
        
        self.viz_pub = self.create_publisher(MarkerArray, '/pid_tuner/visualization', 10)

        self.target_pub = self.create_publisher(Float64, '/pid_tuner/plot/target', 10)
        self.current_pub = self.create_publisher(Float64, '/pid_tuner/plot/current', 10)
        self.error_pub = self.create_publisher(Float64, '/pid_tuner/plot/error', 10)
        self.kp_pub = self.create_publisher(Float64, '/pid_tuner/plot/kp', 10)
        self.ki_pub = self.create_publisher(Float64, '/pid_tuner/plot/ki', 10)
        self.kd_pub = self.create_publisher(Float64, '/pid_tuner/plot/kd', 10)
        
        self.mode_service = self.create_service(Trigger, '/pid_tuner/set_mode', self.set_mode_service_callback)
        self.gain_service = self.create_service(Trigger, '/pid_tuner/set_gain', self.set_gain_service_callback)
        self.stop_service = self.create_service(Empty, '/pid_tuner/stop', self.stop_service_callback)
        self.state_service = self.create_service(Trigger, '/pid_tuner/get_state', self.get_state_service_callback)
        self.toggle_depth_service = self.create_service(Trigger, '/pid_tuner/toggle_depth_hold', self.toggle_depth_service_callback)
        
        self.mode_sub = self.create_subscription(String, '/pid_tuner/command/mode', self.mode_topic_callback, 10)
        self.gain_sub = self.create_subscription(String, '/pid_tuner/command/gain', self.gain_topic_callback, 10)
        
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0]
        self.current_position = [0.0, 0.0, 0.0]
        self.last_position_update = time.time()
        
        self.pid_gains = {
            'linear_x': {'kp': 2.0, 'ki': 0, 'kd': 2.0},
            'linear_y': {'kp': 3.0, 'ki': 0, 'kd': 3.0},
            'linear_z': {'kp': 1.7, 'ki': 0, 'kd': 1.5},
            'angular_x': {'kp': 0, 'ki': 0, 'kd': 0},
            'angular_y': {'kp': 0, 'ki': 0, 'kd': 0},
            'angular_z': {'kp': 0, 'ki': 0, 'kd': 0}
        }
        
        self.pid_integrals = {axis: 0.0 for axis in self.pid_gains.keys()}
        self.pid_prev_errors = {axis: 0.0 for axis in self.pid_gains.keys()}
        self.pid_last_times = {axis: time.time() for axis in self.pid_gains.keys()}
        
        self.oscillation_amplitude = 0.25  # m/s for linear, rad/s for angular
        self.oscillation_frequency = 0.1  # Hz
        self.target_depth = -1.0  # meters (negative = below surface)
        self.oscillation_period = 10.0
        
        # Initialize Z position to target depth
        self.current_position[2] = self.target_depth
        
        self.current_mode = 'idle'  # 'idle', 'oscillate_x', 'oscillate_y', 'oscillate_z', 'depth_hold', 'oscillate_roll', 'oscillate_pitch', 'oscillate_yaw'
        self.oscillation_start_time = None
        self.depth_hold_enabled = False
        
        self.performance_data = {
            'time': [],
            'position': [],
            'velocity': [],
            'orientation': [],
            'target': [],
            'error': []
        }
        
        self.viz_time_window = 30.0
        self.viz_origin = [0.0, 0.0, 0.0]
        self.graph_width = 10.0
        self.graph_height = 4.0
        
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.viz_timer = self.create_timer(0.1, self.update_visualization)
        
        self.get_logger().info('PID Tuner initialized with RViz visualization and ROS2 services')
        
    def mode_topic_callback(self, msg: String):
        mode = msg.data.strip().lower()
        self.set_mode(mode)
        self.get_logger().info(f"Topic command: Mode set to {mode}")
        
    def gain_topic_callback(self, msg: String):
        try:
            parts = msg.data.split()
            if len(parts) < 3:
                self.get_logger().warn("Invalid gain command format. Use: 'axis gain_type value' (e.g., 'linear_x kp 0.5')")
                return
                
            axis = parts[0]  # linear_x, linear_y, etc.
            gain_type = parts[1]  # kp, ki, kd
            value_str = parts[2]  # 0.5, etc.
            
            if axis in self.pid_gains and gain_type in ['kp', 'ki', 'kd']:
                value = float(value_str)
                self.update_gain(axis, gain_type, value)
                self.get_logger().info(f"Topic command: Updated {axis} {gain_type} to {value}")
            else:
                self.get_logger().warn(f"Invalid axis or gain type. Axis must be one of: {list(self.pid_gains.keys())}, gain must be kp/ki/kd")
        except ValueError:
            self.get_logger().warn(f"Invalid value in gain command: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing gain command: {e}")
        
    def signal_handler(self, signum, frame):
        self.get_logger().info('Received interrupt signal, shutting down...')
        self.destroyed = True
        rclpy.shutdown()
        
    def imu_callback(self, msg: Imu):
        self.current_orientation = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w)
            
    def dvl_callback(self, msg: DVL):
        self.current_velocity = [
            msg.velocity.twist.linear.x,
            msg.velocity.twist.linear.y,
            msg.velocity.twist.linear.z
        ]
        # Remove position integration from DVL
        # self.current_position[0] += self.current_velocity[0] * dt
        # self.current_position[1] += self.current_velocity[1] * dt
        # self.current_position[2] += self.current_velocity[2] * dt
        # self.last_position_update = current_time
            
    def odom_callback(self, msg: Odometry):
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        self.current_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
            
    def quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> List[float]:
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
        
    def calculate_pid_output(self, axis: str, error: float) -> float:
        current_time = time.time()
        dt = current_time - self.pid_last_times[axis]
        
        if dt <= 0:
            dt = 0.001  # Prevent division by zero
            
        gains = self.pid_gains[axis]
        
        proportional = gains['kp'] * error
        
        self.pid_integrals[axis] += error * dt
        self.pid_integrals[axis] = max(min(self.pid_integrals[axis], 1.0), -1.0)  # Limit integral windup
        integral = gains['ki'] * self.pid_integrals[axis]
        
        derivative = gains['kd'] * (error - self.pid_prev_errors[axis]) / dt
        
        output = max(min(proportional + integral + derivative, 1.5), -1.5)
        
        self.pid_prev_errors[axis] = error
        self.pid_last_times[axis] = current_time
        
        return output
        
    def control_loop(self):
        cmd_vel = Twist()
        period = 5.0  # seconds for angular oscillation
        angle_amplitude = math.pi / 2  # 90 degrees in radians
        if self.current_mode == 'oscillate_x':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
                
            elapsed_time = time.time() - self.oscillation_start_time
            target_x = self.oscillation_amplitude * math.sin(2 * math.pi * self.oscillation_frequency * elapsed_time)
            
            error_x = target_x - self.current_position[0]
            cmd_vel.linear.x = self.calculate_pid_output('linear_x', error_x)
            
            if self.depth_hold_enabled:
                depth_error = self.target_depth - self.current_position[2]
                cmd_vel.linear.z = self.calculate_pid_output('linear_z', depth_error)
                
        elif self.current_mode == 'oscillate_y':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
                
            elapsed_time = time.time() - self.oscillation_start_time
            target_y = self.oscillation_amplitude * math.sin(2 * math.pi * self.oscillation_frequency * elapsed_time)
            
            error_y = target_y - self.current_position[1]
            cmd_vel.linear.y = self.calculate_pid_output('linear_y', error_y)
            
            if self.depth_hold_enabled:
                depth_error = self.target_depth - self.current_position[2]
                cmd_vel.linear.z = self.calculate_pid_output('linear_z', depth_error)
                
        elif self.current_mode == 'oscillate_z':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
                
            elapsed_time = time.time() - self.oscillation_start_time
            
            if (elapsed_time % self.oscillation_period) < (self.oscillation_period / 2):
                target_z = self.target_depth + 0.5  # -0.5m (shallow)
            else:
                target_z = self.target_depth  # -1.0m (deep)
            
            error_z = target_z - self.current_position[2]
            cmd_vel.linear.z = self.calculate_pid_output('linear_z', error_z)
            
        elif self.current_mode == 'depth_hold':
            depth_error = self.target_depth - self.current_position[2]
            cmd_vel.linear.z = self.calculate_pid_output('linear_z', depth_error)
                
        elif self.current_mode == 'oscillate_roll':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
            elapsed_time = time.time() - self.oscillation_start_time
            if int(elapsed_time / period) % 2 == 0:
                target_roll = 0.0
            else:
                target_roll = angle_amplitude
            error_roll = target_roll - self.current_orientation[0]
            cmd_vel.angular.x = self.calculate_pid_output('angular_x', error_roll)
        elif self.current_mode == 'oscillate_pitch':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
            elapsed_time = time.time() - self.oscillation_start_time
            if int(elapsed_time / period) % 2 == 0:
                target_pitch = 0.0
            else:
                target_pitch = angle_amplitude
            error_pitch = target_pitch - self.current_orientation[1]
            cmd_vel.angular.y = self.calculate_pid_output('angular_y', error_pitch)
        elif self.current_mode == 'oscillate_yaw':
            if self.oscillation_start_time is None:
                self.oscillation_start_time = time.time()
            elapsed_time = time.time() - self.oscillation_start_time
            if int(elapsed_time / period) % 2 == 0:
                target_yaw = 0.0
            else:
                target_yaw = angle_amplitude
            error_yaw = target_yaw - self.current_orientation[2]
            cmd_vel.angular.z = self.calculate_pid_output('angular_z', error_yaw)
        else:  # idle mode
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.store_performance_data()
        
    def update_visualization(self):
        if not self.performance_data['time'] or self.current_mode == 'idle':
            marker_array = MarkerArray()
            clear_marker = Marker()
            clear_marker.header.frame_id = "map"
            clear_marker.header.stamp = self.get_clock().now().to_msg()
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            self.viz_pub.publish(marker_array)
            return
            
        current_time = time.time()
        marker_array = MarkerArray()
        
        filtered_indices = []
        for i, t in enumerate(self.performance_data['time']):
            if current_time - t <= self.viz_time_window:
                filtered_indices.append(i)
                
        if not filtered_indices:
            return
            
        if self.current_mode == 'oscillate_x':
            self.create_axis_markers(marker_array, filtered_indices, 0, "X-Axis Position", current_time)
        elif self.current_mode == 'oscillate_y':
            self.create_axis_markers(marker_array, filtered_indices, 1, "Y-Axis Position", current_time)
        elif self.current_mode == 'oscillate_z':
            self.create_axis_markers(marker_array, filtered_indices, 2, "Z-Axis Position", current_time)
        elif self.current_mode == 'depth_hold':
            self.create_axis_markers(marker_array, filtered_indices, 2, "Depth Hold", current_time)
        elif self.current_mode == 'oscillate_roll':
            self.create_orientation_markers(marker_array, filtered_indices, 0, "Roll Angle (deg)", current_time)
        elif self.current_mode == 'oscillate_pitch':
            self.create_orientation_markers(marker_array, filtered_indices, 1, "Pitch Angle (deg)", current_time)
        elif self.current_mode == 'oscillate_yaw':
            self.create_orientation_markers(marker_array, filtered_indices, 2, "Yaw Angle (deg)", current_time)
        self.create_error_markers(marker_array, filtered_indices, current_time)
        
        self.create_info_markers(marker_array, current_time)
        
        self.viz_pub.publish(marker_array)
        
    def store_performance_data(self):
        current_time = time.time()
        
        if self.current_mode != 'idle':
            self.performance_data['time'].append(current_time)
            self.performance_data['position'].append(self.current_position.copy())
            self.performance_data['velocity'].append(self.current_velocity.copy())
            self.performance_data['orientation'].append(self.current_orientation.copy())
            
            if self.current_mode == 'oscillate_x':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    target = self.oscillation_amplitude * math.sin(2 * math.pi * self.oscillation_frequency * elapsed_time)
                    error = target - self.current_position[0]
                else:
                    target = error = 0.0
            elif self.current_mode == 'oscillate_y':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    target = self.oscillation_amplitude * math.sin(2 * math.pi * self.oscillation_frequency * elapsed_time)
                    error = target - self.current_position[1]
                else:
                    target = error = 0.0
            elif self.current_mode == 'oscillate_z':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    if (elapsed_time % self.oscillation_period) < (self.oscillation_period / 2):
                        target = self.target_depth + 0.5  # -0.5m (shallow)
                    else:
                        target = self.target_depth  # -1.0m (deep)
                    error = target - self.current_position[2]
                else:
                    target = error = 0.0
            elif self.current_mode == 'depth_hold':
                target = self.target_depth
                error = target - self.current_position[2]
            elif self.current_mode == 'oscillate_roll':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    period = 5.0
                    angle_amplitude = math.pi / 2
                    if int(elapsed_time / period) % 2 == 0:
                        target = 0.0
                    else:
                        target = angle_amplitude
                    error = target - self.current_orientation[0]
                else:
                    target = error = 0.0
            elif self.current_mode == 'oscillate_pitch':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    period = 5.0
                    angle_amplitude = math.pi / 2
                    if int(elapsed_time / period) % 2 == 0:
                        target = 0.0
                    else:
                        target = angle_amplitude
                    error = target - self.current_orientation[1]
                else:
                    target = error = 0.0
            elif self.current_mode == 'oscillate_yaw':
                if self.oscillation_start_time is not None:
                    elapsed_time = current_time - self.oscillation_start_time
                    period = 5.0
                    angle_amplitude = math.pi / 2
                    if int(elapsed_time / period) % 2 == 0:
                        target = 0.0
                    else:
                        target = angle_amplitude
                    error = target - self.current_orientation[2]
                else:
                    target = error = 0.0
            else:
                target = error = 0.0
                
            self.performance_data['target'].append(target)
            self.performance_data['error'].append(error)
            
            if len(self.performance_data['time']) > 1000:
                for key in self.performance_data:
                    self.performance_data[key] = self.performance_data[key][-1000:]
                    
        # Publish data for rqt_plot
        self.publish_plot_data()
        
    def create_axis_markers(self, marker_array: MarkerArray, indices: List[int], axis: int, title: str, current_time: float):
        marker_id = 0
        
        self.create_2d_graph_background(marker_array, marker_id, title)
        marker_id += 10  # Reserve IDs for background

        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_trajectory"
        target_marker.id = marker_id
        marker_id += 1
        target_marker.type = Marker.LINE_STRIP
        target_marker.action = Marker.ADD
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.05  # Thicker line for 2D visibility
        target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green for target
        
        current_marker = Marker()
        current_marker.header.frame_id = "map"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.ns = "current_trajectory"
        current_marker.id = marker_id
        marker_id += 1
        current_marker.type = Marker.LINE_STRIP
        current_marker.action = Marker.ADD
        current_marker.pose.orientation.w = 1.0
        current_marker.scale.x = 0.05  # Thicker line for 2D visibility
        current_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red for current
        
        # Populate points for 2D graph (time on X-axis, value on Y-axis)
        start_time = self.performance_data['time'][indices[0]]
        time_range = self.performance_data['time'][indices[-1]] - start_time
        
        for i in indices:
            # Normalize time to graph width
            time_progress = (self.performance_data['time'][i] - start_time) / max(time_range, 1.0)
            graph_x = self.viz_origin[0] - self.graph_width/2 + time_progress * self.graph_width
            
            # Target point (flat 2D graph)
            target_point = Point()
            target_point.x = graph_x
            target_point.y = self.viz_origin[1] + self.performance_data['target'][i]
            target_point.z = self.viz_origin[2]  # Keep at Z=0 for 2D
            target_marker.points.append(target_point)
            
            # Current point (flat 2D graph)  
            current_point = Point()
            current_point.x = graph_x
            current_point.y = self.viz_origin[1] + self.performance_data['position'][i][axis]
            current_point.z = self.viz_origin[2]  # Keep at Z=0 for 2D
            current_marker.points.append(current_point)
            
        marker_array.markers.append(target_marker)
        marker_array.markers.append(current_marker)
        
        self.create_text_marker(marker_array, marker_id, "Target", 
                               [self.viz_origin[0] - self.graph_width/2, self.viz_origin[1] + self.graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, "Current", 
                               [self.viz_origin[0], self.viz_origin[1] + self.graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, f"{title}", 
                               [self.viz_origin[0] + self.graph_width/2, self.viz_origin[1] + self.graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        
    def create_orientation_markers(self, marker_array: MarkerArray, indices: List[int], axis: int, title: str, current_time: float):
        marker_id = 50
        
        # Use smaller graph dimensions for angular tests
        angular_graph_width = 6.0  # Smaller width for angular tests
        angular_graph_height = 2.5  # Smaller height for angular tests
        
        # Create background with smaller dimensions
        self.create_2d_graph_background_angular(marker_array, marker_id, title, angular_graph_width, angular_graph_height)
        marker_id += 10
        
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_orientation"
        target_marker.id = marker_id
        marker_id += 1
        target_marker.type = Marker.LINE_STRIP
        target_marker.action = Marker.ADD
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.05
        target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        current_marker = Marker()
        current_marker.header.frame_id = "map"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.ns = "current_orientation"
        current_marker.id = marker_id
        marker_id += 1
        current_marker.type = Marker.LINE_STRIP
        current_marker.action = Marker.ADD
        current_marker.pose.orientation.w = 1.0
        current_marker.scale.x = 0.05
        current_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        start_time = self.performance_data['time'][indices[0]]
        time_range = self.performance_data['time'][indices[-1]] - start_time
        
        for i in indices:
            time_progress = (self.performance_data['time'][i] - start_time) / max(time_range, 1.0)
            graph_x = self.viz_origin[0] - angular_graph_width/2 + time_progress * angular_graph_width
            
            # Target orientation (convert to degrees for visualization)
            target_point = Point()
            target_point.x = graph_x
            target_point.y = self.viz_origin[1] + math.degrees(self.performance_data['target'][i])
            target_point.z = self.viz_origin[2]
            target_marker.points.append(target_point)
            
            # Current orientation (convert to degrees)
            current_point = Point()
            current_point.x = graph_x
            current_point.y = self.viz_origin[1] + math.degrees(self.performance_data['orientation'][i][axis])
            current_point.z = self.viz_origin[2]
            current_marker.points.append(current_point)
            
        marker_array.markers.append(target_marker)
        marker_array.markers.append(current_marker)
        
        self.create_text_marker(marker_array, marker_id, "Target", 
                               [self.viz_origin[0] - angular_graph_width/2, self.viz_origin[1] + angular_graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, "Current", 
                               [self.viz_origin[0], self.viz_origin[1] + angular_graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, f"{title}", 
                               [self.viz_origin[0] + angular_graph_width/2, self.viz_origin[1] + angular_graph_height/2 + 0.5, self.viz_origin[2]],
                               ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        
    def publish_plot_data(self):
        try:
            if not self.performance_data['time']:
                return
                
            latest_idx = -1
            
            if (latest_idx >= len(self.performance_data['target']) or 
                latest_idx >= len(self.performance_data['error']) or
                latest_idx >= len(self.performance_data['position'])):
                return
                
            target_val = self.performance_data['target'][latest_idx]
            current_val = 0.0
            error_val = self.performance_data['error'][latest_idx]
            
            if not isinstance(target_val, (int, float)):
                target_val = 0.0
            if not isinstance(error_val, (int, float)):
                error_val = 0.0
                
            # Get current position or orientation based on active axis
            if self.current_mode == 'oscillate_x':
                if len(self.performance_data['position'][latest_idx]) > 0:
                    current_val = float(self.performance_data['position'][latest_idx][0])
            elif self.current_mode == 'oscillate_y':
                if len(self.performance_data['position'][latest_idx]) > 1:
                    current_val = float(self.performance_data['position'][latest_idx][1])
            elif self.current_mode == 'oscillate_z':
                if len(self.performance_data['position'][latest_idx]) > 2:
                    current_val = float(self.performance_data['position'][latest_idx][2])
            elif self.current_mode == 'depth_hold':
                if len(self.performance_data['position'][latest_idx]) > 2:
                    current_val = float(self.performance_data['position'][latest_idx][2])
            elif self.current_mode == 'oscillate_roll':
                if len(self.performance_data['orientation'][latest_idx]) > 0:
                    current_val = float(self.performance_data['orientation'][latest_idx][0])
            elif self.current_mode == 'oscillate_pitch':
                if len(self.performance_data['orientation'][latest_idx]) > 1:
                    current_val = float(self.performance_data['orientation'][latest_idx][1])
            elif self.current_mode == 'oscillate_yaw':
                if len(self.performance_data['orientation'][latest_idx]) > 2:
                    current_val = float(self.performance_data['orientation'][latest_idx][2])
            if not isinstance(current_val, (int, float)):
                current_val = 0.0
            
            # Get current PID gains for active axis
            if self.current_mode.startswith('oscillate_'):
                axis = self.current_mode.split('_')[1]
                if axis == 'x':
                    gains = self.pid_gains['linear_x']
                elif axis == 'y':
                    gains = self.pid_gains['linear_y']
                elif axis == 'z':
                    gains = self.pid_gains['linear_z']
                elif axis == 'roll':
                    gains = self.pid_gains['angular_x']
                elif axis == 'pitch':
                    gains = self.pid_gains['angular_y']
                elif axis == 'yaw':
                    gains = self.pid_gains['angular_z']
                else:
                    gains = {'kp': 0, 'ki': 0, 'kd': 0}
            elif self.current_mode == 'depth_hold':
                gains = self.pid_gains['linear_z']
            else:
                gains = {'kp': 0, 'ki': 0, 'kd': 0}
            
            # Publish Float64 messages for rqt_plot
            target_msg = Float64()
            target_msg.data = float(target_val)
            self.target_pub.publish(target_msg)
            
            current_msg = Float64()
            current_msg.data = float(current_val)
            self.current_pub.publish(current_msg)
            
            error_msg = Float64()
            error_msg.data = float(error_val)
            self.error_pub.publish(error_msg)
            
            kp_msg = Float64()
            kp_msg.data = float(gains['kp'])
            self.kp_pub.publish(kp_msg)
            
            ki_msg = Float64()
            ki_msg.data = float(gains['ki'])
            self.ki_pub.publish(ki_msg)
            
            kd_msg = Float64()
            kd_msg.data = float(gains['kd'])
            self.kd_pub.publish(kd_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error in publish_plot_data: {e}")
        
    def create_2d_graph_background(self, marker_array: MarkerArray, marker_id_offset: int, title: str):
        frame_marker = Marker()
        frame_marker.header.frame_id = "map"
        frame_marker.header.stamp = self.get_clock().now().to_msg()
        frame_marker.ns = "graph_frame"
        frame_marker.id = marker_id_offset
        frame_marker.type = Marker.LINE_STRIP
        frame_marker.action = Marker.ADD
        frame_marker.pose.orientation.w = 1.0
        frame_marker.scale.x = 0.02
        frame_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray frame
        
        frame_points = [
            Point(x=self.viz_origin[0] - self.graph_width/2, y=self.viz_origin[1] - self.graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + self.graph_width/2, y=self.viz_origin[1] - self.graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + self.graph_width/2, y=self.viz_origin[1] + self.graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - self.graph_width/2, y=self.viz_origin[1] + self.graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - self.graph_width/2, y=self.viz_origin[1] - self.graph_height/2, z=self.viz_origin[2])  # Close the rectangle
        ]
        
        for point in frame_points:
            frame_marker.points.append(point)
        marker_array.markers.append(frame_marker)
        
        center_line = Marker()
        center_line.header.frame_id = "map"
        center_line.header.stamp = self.get_clock().now().to_msg()
        center_line.ns = "graph_center"
        center_line.id = marker_id_offset + 1
        center_line.type = Marker.LINE_STRIP
        center_line.action = Marker.ADD
        center_line.pose.orientation.w = 1.0
        center_line.scale.x = 0.01
        center_line.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.8)  # Lighter gray for reference
        
        center_line.points.append(Point(x=self.viz_origin[0] - self.graph_width/2, y=self.viz_origin[1], z=self.viz_origin[2]))
        center_line.points.append(Point(x=self.viz_origin[0] + self.graph_width/2, y=self.viz_origin[1], z=self.viz_origin[2]))
        marker_array.markers.append(center_line)
        
        self.create_text_marker(marker_array, marker_id_offset + 2, "Time →", 
                               [self.viz_origin[0], self.viz_origin[1] - self.graph_height/2 - 0.3, self.viz_origin[2]],
                               ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        
        self.create_text_marker(marker_array, marker_id_offset + 3, "Value ↑", 
                               [self.viz_origin[0] - self.graph_width/2 - 0.5, self.viz_origin[1], self.viz_origin[2]],
                               ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        
    def create_2d_graph_background_angular(self, marker_array: MarkerArray, marker_id_offset: int, title: str, graph_width: float, graph_height: float):
        frame_marker = Marker()
        frame_marker.header.frame_id = "map"
        frame_marker.header.stamp = self.get_clock().now().to_msg()
        frame_marker.ns = "graph_frame"
        frame_marker.id = marker_id_offset
        frame_marker.type = Marker.LINE_STRIP
        frame_marker.action = Marker.ADD
        frame_marker.pose.orientation.w = 1.0
        frame_marker.scale.x = 0.02
        frame_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray frame
        
        frame_points = [
            Point(x=self.viz_origin[0] - graph_width/2, y=self.viz_origin[1] - graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + graph_width/2, y=self.viz_origin[1] - graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + graph_width/2, y=self.viz_origin[1] + graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - graph_width/2, y=self.viz_origin[1] + graph_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - graph_width/2, y=self.viz_origin[1] - graph_height/2, z=self.viz_origin[2])  # Close the rectangle
        ]
        
        for point in frame_points:
            frame_marker.points.append(point)
        marker_array.markers.append(frame_marker)
        
        center_line = Marker()
        center_line.header.frame_id = "map"
        center_line.header.stamp = self.get_clock().now().to_msg()
        center_line.ns = "graph_center"
        center_line.id = marker_id_offset + 1
        center_line.type = Marker.LINE_STRIP
        center_line.action = Marker.ADD
        center_line.pose.orientation.w = 1.0
        center_line.scale.x = 0.01
        center_line.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.8)  # Lighter gray for reference
        
        center_line.points.append(Point(x=self.viz_origin[0] - graph_width/2, y=self.viz_origin[1], z=self.viz_origin[2]))
        center_line.points.append(Point(x=self.viz_origin[0] + graph_width/2, y=self.viz_origin[1], z=self.viz_origin[2]))
        marker_array.markers.append(center_line)
        
        self.create_text_marker(marker_array, marker_id_offset + 2, "Time →", 
                               [self.viz_origin[0], self.viz_origin[1] - graph_height/2 - 0.3, self.viz_origin[2]],
                               ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        
        self.create_text_marker(marker_array, marker_id_offset + 3, "Angle (deg) ↑", 
                               [self.viz_origin[0] - graph_width/2 - 0.5, self.viz_origin[1], self.viz_origin[2]],
                               ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        
    def create_error_markers(self, marker_array: MarkerArray, indices: List[int], current_time: float):
        marker_id = 100
        
        error_graph_y = self.viz_origin[1]
        
        error_frame = Marker()
        error_frame.header.frame_id = "map"
        error_frame.header.stamp = self.get_clock().now().to_msg()
        error_frame.ns = "error_frame"
        error_frame.id = marker_id
        error_frame.type = Marker.LINE_STRIP
        error_frame.action = Marker.ADD
        error_frame.pose.orientation.w = 1.0
        error_frame.scale.x = 0.02
        error_frame.color = ColorRGBA(r=0.8, g=0.8, b=0.0, a=0.5)  # Yellow frame for error graph
        
        error_height = self.graph_height * 0.4  # 40% of main graph height
        error_frame_points = [
            Point(x=self.viz_origin[0] - self.graph_width/2, y=error_graph_y - error_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + self.graph_width/2, y=error_graph_y - error_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] + self.graph_width/2, y=error_graph_y + error_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - self.graph_width/2, y=error_graph_y + error_height/2, z=self.viz_origin[2]),
            Point(x=self.viz_origin[0] - self.graph_width/2, y=error_graph_y - error_height/2, z=self.viz_origin[2])
        ]
        
        for point in error_frame_points:
            error_frame.points.append(point)
        marker_array.markers.append(error_frame)
        
        error_center = Marker()
        error_center.header.frame_id = "map"
        error_center.header.stamp = self.get_clock().now().to_msg()
        error_center.ns = "error_center"
        error_center.id = marker_id + 1
        error_center.type = Marker.LINE_STRIP
        error_center.action = Marker.ADD
        error_center.pose.orientation.w = 1.0
        error_center.scale.x = 0.01
        error_center.color = ColorRGBA(r=0.5, g=0.5, b=0.0, a=0.8)
        
        error_center.points.append(Point(x=self.viz_origin[0] - self.graph_width/2, y=error_graph_y, z=self.viz_origin[2]))
        error_center.points.append(Point(x=self.viz_origin[0] + self.graph_width/2, y=error_graph_y, z=self.viz_origin[2]))
        marker_array.markers.append(error_center)
        
        error_marker = Marker()
        error_marker.header.frame_id = "map"
        error_marker.header.stamp = self.get_clock().now().to_msg()
        error_marker.ns = "error_trajectory"
        error_marker.id = marker_id + 2
        error_marker.type = Marker.LINE_STRIP
        error_marker.action = Marker.ADD
        error_marker.pose.orientation.w = 1.0
        error_marker.scale.x = 0.05  # Thicker line
        error_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Bright yellow for error
        
        start_time = self.performance_data['time'][indices[0]]
        time_range = self.performance_data['time'][indices[-1]] - start_time
        
        for i in indices:
            time_progress = (self.performance_data['time'][i] - start_time) / max(time_range, 1.0)
            graph_x = self.viz_origin[0] - self.graph_width/2 + time_progress * self.graph_width
            
            error_point = Point()
            error_point.x = graph_x
            error_point.y = error_graph_y + self.performance_data['error'][i]
            error_point.z = self.viz_origin[2]
            error_marker.points.append(error_point)
            
        marker_array.markers.append(error_marker)
        
        self.create_text_marker(marker_array, marker_id + 3, "🟡 Error", 
                               [self.viz_origin[0] - self.graph_width/2, error_graph_y + error_height/2 + 0.3, self.viz_origin[2]],
                               ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0))
        
    def create_info_markers(self, marker_array: MarkerArray, current_time: float):
        marker_id = 200
        
        panel_x = self.viz_origin[0] + self.graph_width/2 + 2.0  # Position to the right of the graph
        panel_y = self.viz_origin[1] + self.graph_height/2
        panel_z = self.viz_origin[2]
        
        panel_bg = Marker()
        panel_bg.header.frame_id = "map"
        panel_bg.header.stamp = self.get_clock().now().to_msg()
        panel_bg.ns = "info_panel"
        panel_bg.id = marker_id
        panel_bg.type = Marker.CUBE
        panel_bg.action = Marker.ADD
        panel_bg.pose.position.x = panel_x
        panel_bg.pose.position.y = panel_y
        panel_bg.pose.position.z = panel_z
        panel_bg.pose.orientation.w = 1.0
        panel_bg.scale.x = 0.1
        panel_bg.scale.y = 3.0
        panel_bg.scale.z = 0.1
        panel_bg.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.3)  # Semi-transparent background
        marker_array.markers.append(panel_bg)
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, "PID COEFFICIENTS", 
                               [panel_x, panel_y + 1.2, panel_z],
                               ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        marker_id += 1
        
        mode_text = f"Mode: {self.current_mode.upper()}"
        self.create_text_marker(marker_array, marker_id, mode_text,
                               [panel_x, panel_y + 0.8, panel_z],
                               ColorRGBA(r=0.8, g=1.0, b=0.8, a=1.0))
        marker_id += 1
        
        # FIX: Ensure axis_name is always set
        if self.current_mode.startswith('oscillate_'):
            axis = self.current_mode.split('_')[1]
            if axis == 'x':
                gains = self.pid_gains['linear_x']
                axis_name = "X-AXIS"
            elif axis == 'y':
                gains = self.pid_gains['linear_y']
                axis_name = "Y-AXIS"
            elif axis == 'z':
                gains = self.pid_gains['linear_z']
                axis_name = "Z-AXIS"
            elif axis == 'roll':
                gains = self.pid_gains['angular_x']
                axis_name = "ROLL"
            elif axis == 'pitch':
                gains = self.pid_gains['angular_y']
                axis_name = "PITCH"
            elif axis == 'yaw':
                gains = self.pid_gains['angular_z']
                axis_name = "YAW"
            else:
                gains = {'kp': 0, 'ki': 0, 'kd': 0}
                axis_name = "UNKNOWN"
        elif self.current_mode == 'depth_hold':
            gains = self.pid_gains['linear_z']
            axis_name = "DEPTH"
        else:
            gains = {'kp': 0, 'ki': 0, 'kd': 0}
            axis_name = "IDLE"
        
        self.create_text_marker(marker_array, marker_id, f"{axis_name} (ACTIVE)", 
                               [panel_x, panel_y + 0.4, panel_z],
                               ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0))  # Gold for active
        marker_id += 1
        
        kp_text = f"Kp: {gains['kp']:.3f}"
        ki_text = f"Ki: {gains['ki']:.3f}"
        kd_text = f"Kd: {gains['kd']:.3f}"
        
        self.create_text_marker(marker_array, marker_id, kp_text,
                               [panel_x, panel_y + 0.0, panel_z],
                               ColorRGBA(r=1.0, g=0.5, b=0.5, a=1.0))  # Red for Kp
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, ki_text,
                               [panel_x, panel_y - 0.2, panel_z],
                               ColorRGBA(r=0.5, g=1.0, b=0.5, a=1.0))  # Green for Ki
        marker_id += 1
        
        self.create_text_marker(marker_array, marker_id, kd_text,
                               [panel_x, panel_y - 0.4, panel_z],
                               ColorRGBA(r=0.5, g=0.5, b=1.0, a=1.0))  # Blue for Kd
        marker_id += 1
        
        # Only show the current axis being tested
        self.create_text_marker(marker_array, marker_id, f"CURRENT AXIS ({axis_name}):", 
                               [panel_x, panel_y - 0.8, panel_z],
                               ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        marker_id += 1
        
        # Show only the current axis gains
        if self.current_mode.startswith('oscillate_'):
            axis = self.current_mode.split('_')[1]
            if axis in ['x', 'y', 'z']:
                axis_key = f'linear_{axis}'
                gains_text = f"Linear {axis.upper()}: Kp={gains['kp']:.2f} Ki={gains['ki']:.2f} Kd={gains['kd']:.2f}"
            elif axis in ['roll', 'pitch', 'yaw']:
                if axis == 'roll':
                    axis_key = 'angular_x'
                elif axis == 'pitch':
                    axis_key = 'angular_y'
                elif axis == 'yaw':
                    axis_key = 'angular_z'
                gains_text = f"Angular {axis.upper()}: Kp={gains['kp']:.2f} Ki={gains['ki']:.2f} Kd={gains['kd']:.2f}"
            else:
                gains_text = f"Unknown axis: Kp={gains['kp']:.2f} Ki={gains['ki']:.2f} Kd={gains['kd']:.2f}"
        else:
            gains_text = f"Idle: Kp={gains['kp']:.2f} Ki={gains['ki']:.2f} Kd={gains['kd']:.2f}"
        
        self.create_text_marker(marker_array, marker_id, gains_text,
                               [panel_x, panel_y - 1.0, panel_z],
                               ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0))
        
    def create_text_marker(self, marker_array: MarkerArray, marker_id: int, text: str, 
                          position: List[float], color: ColorRGBA):
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "info_text"
        text_marker.id = marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = position[2]
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color = color
        text_marker.text = text
        marker_array.markers.append(text_marker)
        
    def set_mode(self, mode: str):
        self.current_mode = mode
        self.oscillation_start_time = None
        
        # Reset position or orientation when starting a new test
        if mode != 'idle':
            if mode.startswith('oscillate_z') or mode == 'depth_hold':
                self.current_position[2] = self.target_depth
                self.get_logger().info(f'Reset Z position to {self.target_depth}m')
            elif mode == 'oscillate_x':
                self.current_position[0] = 0.0
                self.get_logger().info('Reset X position to 0m')
            elif mode == 'oscillate_y':
                self.current_position[1] = 0.0
                self.get_logger().info('Reset Y position to 0m')
            elif mode == 'oscillate_roll':
                self.current_orientation[0] = 0.0
                self.get_logger().info('Reset roll to 0 rad')
            elif mode == 'oscillate_pitch':
                self.current_orientation[1] = 0.0
                self.get_logger().info('Reset pitch to 0 rad')
            elif mode == 'oscillate_yaw':
                self.current_orientation[2] = 0.0
                self.get_logger().info('Reset yaw to 0 rad')
        if mode == 'idle':
            self.get_logger().info('Stopped - entering idle mode')
        elif mode.startswith('oscillate'):
            axis = mode.split('_')[1]
            self.get_logger().info(f'Starting oscillation test for {axis}-axis')
        elif mode == 'depth_hold':
            self.get_logger().info(f'Starting depth hold at {self.target_depth}m')
            
    def update_gain(self, axis: str, gain_type: str, value: float):
        if axis in self.pid_gains and gain_type in ['kp', 'ki', 'kd']:
            self.pid_gains[axis][gain_type] = value
            self.get_logger().info(f'Updated {axis} {gain_type} = {value}')
            
            # Reset PID state for this axis
            self.pid_integrals[axis] = 0.0
            self.pid_prev_errors[axis] = 0.0
            self.pid_last_times[axis] = time.time()
            
    def print_current_state(self):
        self.get_logger().info('\n=== Current State ===')
        self.get_logger().info(f'Mode: {self.current_mode}')
        self.get_logger().info(f'Position: X={self.current_position[0]:.3f}, Y={self.current_position[1]:.3f}, Z={self.current_position[2]:.3f}')
        self.get_logger().info(f'Velocity: X={self.current_velocity[0]:.3f}, Y={self.current_velocity[1]:.3f}, Z={self.current_velocity[2]:.3f}')
        self.get_logger().info(f'Target Depth: {self.target_depth}m')
        self.get_logger().info(f'Depth Hold Enabled: {self.depth_hold_enabled}')
        
        self.get_logger().info('\n=== Current PID Gains ===')
        for axis, gains in self.pid_gains.items():
            self.get_logger().info(f'{axis}: Kp={gains["kp"]:.3f}, Ki={gains["ki"]:.3f}, Kd={gains["kd"]:.3f}')
        
    def set_mode_service_callback(self, request, response):
        mode = request.data
        self.set_mode(mode)
        response.data = f"Mode set to: {mode}"
        self.get_logger().info(f"Service call: Mode set to {mode}")
        return response
        
    def set_gain_service_callback(self, request, response):
        try:
            parts = request.data.split()
            if len(parts) < 3:
                response.data = "Invalid command format. Use: 'axis gain_type value' (e.g., 'linear_x kp 0.5')"
                return response
                
            axis = parts[0]  # linear_x, linear_y, etc.
            gain_type = parts[1]  # kp, ki, kd
            value_str = parts[2]  # 0.5, etc.
            
            if axis in self.pid_gains and gain_type in ['kp', 'ki', 'kd']:
                value = float(value_str)
                self.update_gain(axis, gain_type, value)
                response.data = f"Updated {axis} {gain_type} to {value}"
                self.get_logger().info(f"Service call: Updated {axis} {gain_type} to {value}")
            else:
                response.data = f"Invalid axis or gain type. Axis must be one of: {list(self.pid_gains.keys())}, gain must be kp/ki/kd"
        except ValueError:
            response.data = f"Invalid value in command: {request.data}"
        except Exception as e:
            response.data = f"Error updating gain: {e}"
        return response
        
    def stop_service_callback(self, request, response):
        self.set_mode('idle')
        self.get_logger().info("Service call: Stopped PID tuning")
        return response
        
    def get_state_service_callback(self, request, response):
        self.print_current_state()
        response.data = "Current state printed to console."
        return response
        
    def toggle_depth_service_callback(self, request, response):
        self.depth_hold_enabled = not self.depth_hold_enabled
        status = "enabled" if self.depth_hold_enabled else "disabled"
        self.get_logger().info(f"Service call: Depth holding {status}")
        return response


def main(args=None):
    rclpy.init(args=args)
    
    tuner = PIDTuner()
    
    try:
        # Use a shorter timeout in spin to allow Ctrl+C to be caught
        while not tuner.destroyed:
            rclpy.spin_once(tuner, timeout_sec=0.1)
    except KeyboardInterrupt:
        tuner.get_logger().info('Tuning interrupted')
    finally:
        tuner.destroyed = True
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 