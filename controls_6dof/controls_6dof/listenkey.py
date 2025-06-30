import rclpy
from rclpy.node import Node

from pynput import keyboard
from pynput.keyboard import Key, KeyCode

from geometry_msgs.msg import Twist

KEYCODE_FORWARD = KeyCode.from_char('w')
KEYCODE_BACKWARD = KeyCode.from_char('s')
KEYCODE_STRAFE_L = KeyCode.from_char('a')
KEYCODE_STRAFE_R = KeyCode.from_char('d')
KEYCODE_SURFACE = Key.up
KEYCODE_DIVE = Key.down
KEYCODE_YAW_CCW = KeyCode.from_char('q')
KEYCODE_YAW_CW = KeyCode.from_char('e')
KEYCODE_PITCH_L = Key.left
KEYCODE_PITCH_R = Key.right

# KEYCODE_ROLL_FW1 and KEYCODE_ROLL_BW1 are for key 'left shift'
# KEYCODE_ROLL_FW2 and KEYCODE_ROLL_BW2 are for key 'right shift'
KEYCODE_ROLL_FW1 = [Key.shift_l, Key.up]
KEYCODE_ROLL_FW2 = [Key.shift_r, Key.up]
KEYCODE_ROLL_BW1 = [Key.shift_l, Key.down]
KEYCODE_ROLL_BW2 = [Key.shift_r, Key.down]

# Queue size: A required QoS (quality of service) setting that limits the amout of queued messages 
#			 if a subscriber is not receiving them fast enough
QUEUE_SIZE = 10

# Seconds to wait in spin_once() -> 0 means no wait
TIMEOUT_SECOND = 0

class ListenKey(Node):
	"""
	ListenKey class receives the information regarding the keys currently pressed,
	converts this data into Twist velocity commands each representing 1) Forward/Backward,
	2) Strafe Left/Right, 3) Surface/Dive, 4) Yaw Counterclockwise/Clockwise, 5) Pitch Left/Right
	6) Roll Forward/Backward, and sends this data through 'cmd_vel' topic.

	Attributes:
		publisher_: The node where 'listenkey' node sends the Twist velocity commands
		keypresses: The set of keys currently pressed
		movement_rotation: The 6 dimensional vector which will be sent as Twist message
						   The vector will be updated whenever there is a change in the vector 
						   by comparing with current_movement_rotation in publish method
	"""
	def __init__(self):
		"""
		No Arguments and No Return Values

		Initialize the class
		"""
		super().__init__('listenkey')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', QUEUE_SIZE)
		
		# Keyboard
		self.keypresses = set()
		
		# Create 6-dimensional vector
		self.movement_rotation = [0, 0, 0, 0, 0, 0]

	def spin(self):
		"""
		No Argument and No Return value

		Whenever the key is pressed, it will call the on_press method and 
		whenever the key is released, it will call the on_release method
		"""
		with keyboard.Listener(on_press = self.on_press, on_release = self.on_release) as listener:
			while rclpy.ok() and listener.running:
				rclpy.spin_once(self, timeout_sec=TIMEOUT_SECOND)

	def on_press(self, key):
		"""
		Args:
			key: key that is pressed
		No Return value

		It will add the key in self.keypresses set and call publish method 
		to check if the addition of this key changes the 6 dimensional vector
		"""
		if key not in self.keypresses:
			self.keypresses.add(key)
			self.publish()

	def on_release(self, key):
		"""
		Args:
			key: key that is released
		No Return value

		It will remove the key in self.keypresses and call publish method
		to check if the removal of this key changes the 6 dimensional vector
		"""
		try:
			self.keypresses.remove(key)
			self.publish()
		except KeyError:
			pass

	def publish(self):
		"""
		No Arguments and No Return value

		Check the current 6 dimensional vector (current_movement_rotation) based on the keys currently pressed.
		If this current_movement_rotation vector is different from the self.movement_rotation, 
		replace the self.movement_rotation into current_movement_rotation and send this vector info as Twist message through 'cmd_vel'.
		Otherwise, no action is occurred.
		"""
		current_movement_rotation = [0, 0, 0, 0, 0, 0]
		self.get_logger().info('Keys pressed: {0}'.format(self.keypresses))

		# Fill out each 6 elements in the current_movement_rotation vector based on the keys pressed

		# Forward 
		# Check if key 'w' is pressed
		if KEYCODE_FORWARD in self.keypresses: current_movement_rotation[0] += 1

		# Backward
		# Check if key 's' is pressed
		if KEYCODE_BACKWARD in self.keypresses: current_movement_rotation[0] -= 1

		# Strafe Left
		# Check if key 'a' is pressed
		if KEYCODE_STRAFE_L in self.keypresses: current_movement_rotation[1] += 1

		# Strafe Right
		# Check if key 'd' is pressed
		if KEYCODE_STRAFE_R in self.keypresses: current_movement_rotation[1] -= 1

		# Surface
		# Check if key 'up arrow' but key 'shift' is pressed
		if KEYCODE_SURFACE in self.keypresses and not set(KEYCODE_ROLL_FW1).issubset(set(self.keypresses)) and not set(KEYCODE_ROLL_FW2).issubset(set(self.keypresses)): current_movement_rotation[2] += 1
		
		# Dive
		# Check if key 'down arrow' but key 'shift' is pressed
		if KEYCODE_DIVE in self.keypresses and not set(KEYCODE_ROLL_BW1).issubset(set(self.keypresses)) and not set(KEYCODE_ROLL_BW2).issubset(set(self.keypresses)): current_movement_rotation[2] -= 1
		
		# Yaw Counterclockwise
		# Check if key 'q' is pressed
		if KEYCODE_YAW_CCW in self.keypresses: current_movement_rotation[3] += 1
		
		# Yaw Clockwise
		# Check if key 'e' is pressed
		if KEYCODE_YAW_CW in self.keypresses: current_movement_rotation[3] -= 1

		# Pitch Left
		# Check if key 'left arrow' is pressed
		if KEYCODE_PITCH_L in self.keypresses: current_movement_rotation[4] += 1

		# Pitch Right
		# Check if key 'right arrow' is pressed
		if KEYCODE_PITCH_R in self.keypresses: current_movement_rotation[4] -= 1

		# Roll Forward
		# Check if both key 'up arrow' and key 'shift' are pressed
		if set(KEYCODE_ROLL_FW1).issubset(set(self.keypresses)) or set(KEYCODE_ROLL_FW2).issubset(set(self.keypresses)): current_movement_rotation[5] += 1
		
		# Roll Backward
		# Check if both key 'down arrow' and key 'shift' are pressed
		if set(KEYCODE_ROLL_BW1).issubset(set(self.keypresses)) or set(KEYCODE_ROLL_BW2).issubset(set(self.keypresses)): current_movement_rotation[5] -= 1

		# If there is no change in vector, do not send the message 
		if self.movement_rotation != current_movement_rotation:	 
			msg = Twist()
			# Map the 6-element vector to Twist message
			msg.linear.x = float(current_movement_rotation[0])   # forward/backward
			msg.linear.y = float(current_movement_rotation[1])   # strafe left/right
			msg.linear.z = float(current_movement_rotation[2])   # surface/dive
			msg.angular.x = float(current_movement_rotation[5])  # roll
			msg.angular.y = float(current_movement_rotation[4])  # pitch
			msg.angular.z = float(current_movement_rotation[3])  # yaw
			
			self.movement_rotation = current_movement_rotation
			self.publisher_.publish(msg)

def main(args=None):	
	rclpy.init(args=None) # the rclpy library is initialized
	listenkey = ListenKey() # The node listenkey is created
	listenkey.spin() # The node listenkey is spinned, meaning its callbacks are called
	
	listenkey.destroy_node() # Destroy the node explicitly
	rclpy.shutdown()

if __name__ == '__main__':
	main()