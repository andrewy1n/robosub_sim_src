# Gate Navigation Package

A ROS2 package for autonomous gate navigation using YOLO object detection and pure pursuit path planning for underwater robots.

## Features

- **YOLO Gate Detection**: Uses the `gate_yolo_sim` model to detect gate components (leftPipe, rightPipe, centerPipe, divider)
- **Pure Pursuit Path Planning**: Implements pure pursuit algorithm for smooth path following
- **Multi-Mode Navigation**: Supports search, approach, and through navigation modes
- **Configurable Parameters**: Easy-to-tune parameters for different environments
- **Real-time Visualization**: Debug images and path visualization

## Architecture

The package consists of several key components:

### 1. Gate Navigator (`gate_navigator.py`)
Main navigation node that orchestrates the entire navigation process:
- Subscribes to camera images and robot pose
- Processes YOLO detections
- Generates navigation paths
- Controls robot movement

### 2. Gate Detector (`gate_detector.py`)
Handles YOLO model loading and gate detection:
- Loads the `gate_yolo_sim.pt` model
- Processes detections for gate components
- Provides gate center and width calculations

### 3. Pure Pursuit Controller (`pure_pursuit.py`)
Implements the pure pursuit path following algorithm:
- Generates straight-line and curved paths
- Calculates lookahead points
- Provides velocity control commands

### 4. Velocity Controller (`velocity_controller.py`)
Handles velocity command generation:
- PID control for smooth movement
- Velocity limiting and ramping
- Coordinate frame transformations

## Installation

1. Build the package:
```bash
cd ~/dave_ws
colcon build --packages-select gate_navigation
source install/setup.bash
```

2. Ensure the YOLO model is available:
```bash
# The model should be in camera_test/config/gate_yolo_sim.pt
ls ~/dave_ws/src/camera_test/config/gate_yolo_sim.pt
```

## Usage

### Basic Usage

1. Launch the gate navigation system:
```bash
ros2 launch gate_navigation gate_navigation.launch.py
```

2. With custom parameters:
```bash
ros2 launch gate_navigation gate_navigation.launch.py gate_side:=right approach_distance:=2.0 max_velocity:=1.5
```

### Parameters

- `gate_side`: Which side of the gate to navigate through (`left` or `right`)
- `approach_distance`: Distance to approach the gate before going through (meters)
- `through_distance`: Distance to travel through the gate (meters)
- `max_velocity`: Maximum velocity for navigation (m/s)
- `confidence_threshold`: YOLO detection confidence threshold (0.0-1.0)

### Topics

#### Subscribed Topics
- `/robosub/camera/simulated_image` (sensor_msgs/Image): Camera feed
- `/model/high_level_robosub/pose` (gz.msgs.Pose_V): Robot pose from Gazebo

#### Published Topics
- `/vector_topic` (geometry_msgs/Twist): Velocity commands for the robot
- `/gate_navigation/path` (nav_msgs/Path): Planned path for visualization
- `/gate_navigation/debug_image` (sensor_msgs/Image): Debug image with detections

## Navigation Modes

### 1. Search Mode
- Robot moves forward slowly while searching for gates
- Slight turning to scan the environment
- Continues until a gate is detected

### 2. Approach Mode
- Calculates target point based on gate side preference
- Uses pure pursuit to generate and follow a path
- Switches to through mode when close enough

### 3. Through Mode
- Navigates through the gate at increased speed
- Maintains straight trajectory
- Completes navigation when past the gate

## Configuration

Edit `config/gate_navigation_params.yaml` to customize:
- Control gains and limits
- Navigation parameters
- PID controller settings

## Troubleshooting

### Common Issues

1. **YOLO model not found**:
   - Ensure `gate_yolo_sim.pt` is in `camera_test/config/`
   - Check file permissions

2. **No detections**:
   - Adjust `confidence_threshold` parameter
   - Check camera feed quality
   - Verify model compatibility

3. **Poor navigation performance**:
   - Tune `lookahead_distance` for pure pursuit
   - Adjust velocity limits
   - Modify PID gains

### Debug Visualization

Enable debug visualization to see:
- YOLO detections on camera feed
- Navigation mode and status
- Gate components and centers

## Dependencies

- ROS2 Humble
- Python 3.8+
- OpenCV
- NumPy
- Ultralytics (YOLO)
- Gazebo Transport

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This package is licensed under the Apache 2.0 License. 