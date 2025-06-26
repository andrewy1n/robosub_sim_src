from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
	robosub_arguments = (
		[
			"/robosub/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
			"/robosub/camera/simulated_image@sensor_msgs/msg/Image@gz.msgs.Image",
			"/robosub/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
		]
	)
	robosub_bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		arguments=robosub_arguments,
		output="screen",
	)

	movement = Node(
		package="robosub_wrench_movement",
		executable="wrench_movement",
		output="screen",
	)

	keyInput = Node(
		package="controls_6dof",
		executable="listenkey",
		output="screen",
	)

	# gate_detector = Node(
	# 	package="gate_detection",
	# 	executable="gate_detector_node",
	# 	output="screen",
	# )

	# camera_viewer = Node(
	# 	package="camera_test",
	# 	executable="show_camera",
	# 	output="screen",
	# )	

	return [robosub_bridge, movement, keyInput]


def generate_launch_description():
	args = [
		DeclareLaunchArgument(
			"namespace",
			default_value="",
			description="Namespace",
		),
	]

	return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
