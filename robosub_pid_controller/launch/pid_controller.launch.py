from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = FindPackageShare('robosub_pid_controller')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'pid_config.yaml']),
        description='Path to PID configuration file'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch rqt_reconfigure for PID tuning'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the PID controller node'
    )
    
    pid_controller_node = Node(
        package='robosub_pid_controller',
        executable='pid_controller',
        name='robosub_pid_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_raw'),
            ('/cmd_vel', '/cmd_vel'),
            ('/sensors/imu', '/sensors/imu'),
            ('/sensors/dvl/velocity', '/sensors/dvl/velocity')
        ]
    )
    
    rqt_reconfigure_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_gui')),
        cmd=['rqt_reconfigure'],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        use_gui_arg,
        log_level_arg,
        pid_controller_node,
        rqt_reconfigure_node
    ]) 