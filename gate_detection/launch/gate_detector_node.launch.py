import launch
import launch_ros.actions
from tracetools_launch.action import Trace

def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='gate_detector'
    )
    # Nodes
    gate_detector_node = launch_ros.actions.Node(
        package='gate_detection',
        executable='gate_detector_node.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        gate_detector_node
    ])