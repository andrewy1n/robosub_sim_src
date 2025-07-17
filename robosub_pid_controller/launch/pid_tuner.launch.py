from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = FindPackageShare('robosub_pid_controller')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for PID tuning visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'pid_tuner.rviz']),
        description='Path to RViz configuration file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the PID tuner node'
    )
    
    pid_tuner_node = Node(
        package='robosub_pid_controller',
        executable='pid_tuner',
        name='pid_tuner',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/sensors/imu', '/sensors/imu'),
            ('/sensors/dvl/velocity', '/sensors/dvl/velocity')
        ]
    )
    
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        log_level_arg,
        pid_tuner_node,
        rviz_node,
        static_transform_publisher
    ]) 