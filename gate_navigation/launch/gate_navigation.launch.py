#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('gate_navigation')
    
    # Declare launch arguments
    gate_side_arg = DeclareLaunchArgument(
        'gate_side',
        default_value='left',
        description='Which side of the gate to navigate through (left or right)'
    )
    
    approach_distance_arg = DeclareLaunchArgument(
        'approach_distance',
        default_value='3.0',
        description='Distance to approach the gate before going through'
    )
    
    through_distance_arg = DeclareLaunchArgument(
        'through_distance',
        default_value='5.0',
        description='Distance to travel through the gate'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='Maximum velocity for navigation'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO detection confidence threshold'
    )
    
    lateral_gain_arg = DeclareLaunchArgument(
        'lateral_gain',
        default_value='0.5',
        description='Lateral movement control gain'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )
    
    # Gate Navigator Node
    gate_navigator_node = Node(
        package='gate_navigation',
        executable='gate_navigator',
        name='gate_navigator',
        output='screen',
        parameters=[{
            'gate_side': LaunchConfiguration('gate_side'),
            'approach_distance': LaunchConfiguration('approach_distance'),
            'through_distance': LaunchConfiguration('through_distance'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'lateral_gain': LaunchConfiguration('lateral_gain'),
        }],
        remappings=[
            ('/robosub/camera/simulated_image', '/robosub/camera/simulated_image'),
            ('/wrench_movement', '/wrench_movement'),
        ]
    )
    
    # Camera bridge node (if needed)
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robosub/camera/simulated_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
    )
    
    # RViz Node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'config', 'gate_navigation.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    
    return LaunchDescription([
        # Launch arguments
        gate_side_arg,
        approach_distance_arg,
        through_distance_arg,
        max_velocity_arg,
        confidence_threshold_arg,
        lateral_gain_arg,
        use_rviz_arg,
        
        # Nodes
        camera_bridge_node,
        gate_navigator_node,
        rviz_node,
    ]) 