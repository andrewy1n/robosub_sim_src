import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rclpy.duration import Duration
import time
# from .depth_perception import find_gate_distance
# from .gate_detection_yolo import contains_gate, find_gate_center

"""
Based on ROS2 Python Basics module 6.5 multithreading example and adapted code from "plant_detector_multithreading_callbackgroups.py"
"""

class GateDetectorNode(Node):
	def __init__(self, image1_topic = 'camera1/image_raw', image2_topic = 'camera2/image_raw'):
		super().__init__('gate_detector_node')
		self.cmd_vel_publisher = self.create_publisher(Int32MultiArray, '/vector_topic', 10)

		self.reentrant_group_1 = ReentrantCallbackGroup()

		# Not sure which image subscription is for which camera, change var names to 'left' and 'right' images later
		self.image1_subscription = self.create_subscription(Image, image1_topic, self.image1_listener_callback, 10)
		# self.image2_subscription = self.create_subscription(Image, image2_topic, self.image2_listener_callback, 10)
		self.image1 = None
		# self.image2 = None
		self.image1_subscription
		# self.image2_subscription
		self.bridge = CvBridge()
		self.img_width = None
		self.img_height = None

		self.timer = self.create_timer(1, self.timer_callback)

		self.TIME_STEP = 0.1

		# PID constants with default values
		self.Kp = 0.0
		self.Ki = 0.0
		self.Kd = 0.01

		# Create custom service with PID parameters
		self.srv = self.create_service(Trigger, 'detect_gate', self.detect_gate_callback, callback_group=self.reentrant_group_1)

		self.movement_arr = [0, 0, 0, 0, 0, 0]
		
	def image1_listener_callback(self, data):
		self.get_logger().info(f"Got image 1 message: {data.header}")
		self.img_width = data.width
		self.img_height = data.height
		self.image1 = self.bridge.imgmsg_to_cv2(data, 'bgr8')

	# def image2_listener_callback(self, data):
	# 	self.get_logger().info(f"Got image 2 message: {data.header}")
	# 	self.image2 = self.bridge.imgmsg_to_cv2(data, 'bgr8')
	
	def detect_gate_callback(self, request, response):
		# Rotate until gate is visible
		self.rotate_until_gate_callback()

		# Stop rotating
		self.get_logger().info("Gate found! Moving toward gate")
		self.publish_velocity(linX=0, angX=0)

		# Move toward gate
		self.move_toward_gate_callback()

        # # Stop the robot
		self.publish_velocity(linX=0, linY=0, angY=0)
		self.get_logger().info("Gate detection process completed")
		response.success = True
		response.message = 'Gate detection process completed'
		return response
	
	def rotate_until_gate_callback(self):
		start_time = self.get_clock().now()
		max_duration = Duration(seconds=30)
        
		gate_found = False

        # Start moving the robot
		self.publish_velocity(angX=-1)
		delta_time = self.get_clock().now() - start_time
		
		# Rotate robot until the gate is found
		while delta_time < max_duration and not gate_found:
			delta_time = self.get_clock().now() - start_time
			
			if self.image1 is not None:
				self.get_logger().info("Processing images...")

				# if contains_gate(self.image1):
				# 	gate_found = True
				# else:
				# 	self.get_logger().info("Gate Not Found")
				
			
			time.sleep(0.1)
	
	def move_toward_gate_callback(self):
		start_time = self.get_clock().now()
		max_duration = Duration(seconds=10)
		delta_time = self.get_clock().now() - start_time
		# gate_distance = float('inf')
		forward_speed = 1

		# Initialize PID variables
		prev_error_x = 0
		prev_error_y = 0
		integral_x = 0
		integral_y = 0

		# Start moving forwards
		self.publish_velocity(linX = forward_speed, angY = 0)

		while delta_time < max_duration:
			delta_time = self.get_clock().now() - start_time
			
			if self.image1 is not None:
				self.get_logger().info("Processing images...")
				
				# center = find_gate_center(self.image1, test=False)
				
				# # if center is None:
				# # 	self.get_logger().info("Only one gate detected, waiting for second gate...")
				# # 	time.sleep(self.TIME_STEP)
				# # 	continue
					
				# cx1, cy1 = center
				
				# # compute horizontal error (pixels)
				# # avg_cx = 0.5 * (cx1 + cx2)
				# error_x = cx1 - (self.img_width / 2)
				
				# # PID to get yaw correction
				# control_yaw, prev_error_x, integral_x = self.pid(
				# 	error_x, prev_error_x, integral_x
				# )
				
				# # compute vertical error
				# # avg_cy = 0.5 * (cy1 + cy2)
				# error_y = (self.img_height / 2) - cy1 

				# # PID to get vertical speed correction
				# control_vert, prev_error_y, integral_y = self.pid(
				# 	error_y, prev_error_y, integral_y
				# )

				# publish velocities: forward + yaw + vertical
				self.publish_velocity(
					linX=forward_speed,
					# linY=control_vert,
					# angY=-control_yaw
				)

				# self.get_logger().info(
				# 	f"ErrX: {error_x:.1f}px  CtrlYaw: {control_yaw:.3f}  "
				# 	f"ErrY: {error_y:.1f}px  CtrlVert: {control_vert:.3f}"
				# )
				
			# self.get_logger().info(f"Gate Distance: {gate_distance}")
			
			time.sleep(self.TIME_STEP)
	
	def timer_callback(self):
		self.get_logger().info(f'Movement status: {self.movement_arr}')
	
	def publish_velocity(self, linX = 0, linY = 0, linZ = 0, angX = 0, angY = 0, angZ = 0):
		vel_msg = [0, 0, 0, 0, 0, 0]
		vel_msg[0] = linX
		vel_msg[1] = linY
		vel_msg[2] = linZ
		vel_msg[3] = angX
		vel_msg[4] = angY
		vel_msg[5] = angZ

		msg = Int32MultiArray()
		msg.data = vel_msg
		self.movement_arr = vel_msg
		self.cmd_vel_publisher.publish(msg)
	
	def pid(self, error, prev_error, prev_integral):
		p = self.Kp * error
		i = self.Ki * ((error * self.TIME_STEP) + prev_integral)
		d = self.Kd * ((error - prev_error) / self.TIME_STEP)
		# Convert to integer and ensure it's within reasonable bounds
		control_output = int(p + i + d)
		# Clamp the output to reasonable values (e.g., -100 to 100)
		control_output = max(min(control_output, 5), -5)
		return control_output, error, i

def main(args=None):
	rclpy.init(args=args)
	node = GateDetectorNode(image1_topic = '/robosub/camera/simulated_image')

	executor = MultiThreadedExecutor(num_threads=2)
	executor.add_node(node)

	try:
		executor.spin()
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()